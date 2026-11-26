package decad

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// documentState is the part of a document an atomic operation must leave
// alone: its live bodies, operation history, producer counter, container
// membership and the sizes of the internal indexes a commit writes.
type documentState struct {
	live       []*Body
	ops        []*Operation
	next       producerID
	containers []*Container
	members    [][]*Body
	index      map[string]int
}

func captureDocumentState(ctx context.Context, t *testing.T, doc *Document) documentState {
	t.Helper()
	state := documentState{
		live:       doc.Bodies(),
		ops:        doc.OperationHistory().Operations(),
		next:       doc.producerAt(ctx),
		containers: append([]*Container{doc.Root()}, doc.Root().Children()...),
		index: map[string]int{
			"producers":          len(doc.producers),
			"consumers":          len(doc.consumers),
			"container entries":  len(doc.groups.ofBody),
			"circle builds":      len(doc.circleBuilds),
			"circle placements":  len(doc.circlePlacements),
			"identity placement": len(doc.identityPlacements),
		},
	}
	for _, c := range state.containers {
		state.members = append(state.members, c.Bodies())
	}
	return state
}

// requireSameBodies compares bodies by pointer and prints only list positions
// and producer numbers, never a body graph.
func requireSameBodies(t *testing.T, want, got []*Body, what string) {
	t.Helper()
	require.Len(t, got, len(want), "%s count", what)
	for i := range want {
		require.True(t, want[i] == got[i], "%s %d: want producer %d, got producer %d",
			what, i, want[i].n.Origin.Producer, got[i].n.Origin.Producer)
	}
}

// requireDocumentState compares by pointer and by count so a failure prints
// bounded data. Every operand must sit in the container it started in.
func requireDocumentState(ctx context.Context, t *testing.T, want documentState, doc *Document, operands ...*Body) {
	t.Helper()
	got := captureDocumentState(ctx, t, doc)
	requireSameBodies(t, want.live, got.live, "live body")
	require.Len(t, got.ops, len(want.ops), "history length")
	for i := range want.ops {
		require.True(t, want.ops[i] == got.ops[i], "history entry %d is not the entry recorded before the call", i)
	}
	require.Equal(t, want.next, got.next, "next producer id")
	require.Equal(t, want.index, got.index, "internal index sizes")
	require.Len(t, got.containers, len(want.containers), "container count")
	for i := range want.containers {
		require.True(t, want.containers[i] == got.containers[i], "container %d", i)
		requireSameBodies(t, want.members[i], got.members[i], "container member")
	}
	for _, b := range operands {
		consumers, err := doc.OperationHistory().Consumers(b)
		require.NoError(t, err)
		require.Empty(t, consumers, "no operation consumed the operand")
	}
}

// cancelPoints are the numbers of Err polls a call lets pass before it is
// canceled: the first few, about ten evenly spaced ones, and the last few
// before the call would have finished (agent-docs/command-api-design.md
// section 14).
func cancelPoints(checks int) []int {
	stride := max(1, checks/10)
	seen := map[int]struct{}{}
	var points []int
	add := func(p int) {
		if _, dup := seen[p]; dup || p < 0 || p >= checks {
			return
		}
		seen[p] = struct{}{}
		points = append(points, p)
	}
	for _, p := range []int{0, 1, 2} {
		add(p)
	}
	for p := stride; p < checks; p += stride {
		add(p)
	}
	add(checks - 2)
	add(checks - 1)
	return points
}

// sweepCancellation builds a fresh document with fresh() and runs the trial it
// returns once without cancellation to count its context polls, then once per
// cancel point on a new document. A trial returns true when its call
// succeeded; it asserts the whole document state itself when the call was
// canceled.
func sweepCancellation(t *testing.T, fresh func() func(ctx context.Context) bool) {
	t.Helper()
	counter := &countdownContext{Context: t.Context(), left: 1 << 30}
	require.True(t, fresh()(counter), "an uncanceled call succeeds")
	checks := (1 << 30) - counter.left
	require.Greater(t, checks, 3, "the call polls the context")
	for _, p := range cancelPoints(checks) {
		require.False(t, fresh()(&countdownContext{Context: t.Context(), left: p}),
			"a cancel after %d of %d polls stops the call", p, checks)
	}
}

// operationKindOf is the operation kind the diagnostics of err name.
func operationKindOf(t *testing.T, err error) OperationKind {
	t.Helper()
	opErr, ok := errors.AsType[*OperationError](err)
	require.True(t, ok, "the error carries operation diagnostics")
	diagnostics := opErr.OperationDiagnostics()
	require.NotEmpty(t, diagnostics)
	return diagnostics[0].Operation
}

func straightPath(t *testing.T) *Path {
	t.Helper()
	path, err := NewPath(r3.Vec{}, LineTo{End: r3.Vec{X: 30}})
	require.NoError(t, err)
	return path
}

func TestPathPatternRefusedCopyLeavesTheDocumentUnchanged(t *testing.T) {
	t.Parallel()
	doc := New()
	block := internalBoxBody(t, doc, 0, 0, 4, 4, 2)
	// The path runs out and turns straight back, so the copies on the first
	// leg are made and the first copy past the turn has no shortest turn.
	path, err := NewPath(r3.Vec{}, LineTo{End: r3.Vec{X: 10}}, LineTo{End: r3.Vec{X: 2}})
	require.NoError(t, err)
	before := captureDocumentState(t.Context(), t, doc)
	copies, err := PathPattern(block, path, PathCount{Count: 6}).Follow().Do(t.Context())
	require.ErrorIs(t, err, ErrDegenerate)
	require.Equal(t, OperationCopy, operationKindOf(t, err))
	require.Nil(t, copies)
	requireDocumentState(t.Context(), t, before, doc, block)
	// The refused call used no producer number, so the next copy takes the
	// number the first copy of the refused call would have taken.
	again, err := Copy(block).Do(t.Context())
	require.NoError(t, err)
	require.Equal(t, before.next, again.n.Origin.Producer)
}

func TestPathPatternCanceledMidwayLeavesTheDocumentUnchanged(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name  string
		build func(t *testing.T, doc *Document) *Body
		child bool
	}{
		{"root box", func(t *testing.T, doc *Document) *Body { return internalBoxBody(t, doc, 0, 0, 4, 4, 2) }, false},
		{"child circle", func(t *testing.T, doc *Document) *Body {
			return internalCircleBody(t, doc, 0, 2, 0, Distance{D: units.Millimeters(2), Dir: Along})
		}, true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			sweepCancellation(t, func() func(ctx context.Context) bool {
				doc := New()
				block := tc.build(t, doc)
				path := straightPath(t)
				if tc.child {
					child, err := doc.Root().NewChild("parts")
					require.NoError(t, err)
					require.NoError(t, child.AddBody(block))
				}
				before := captureDocumentState(t.Context(), t, doc)
				return func(ctx context.Context) bool {
					copies, err := PathPattern(block, path, PathCount{Count: 4}).Do(ctx)
					if err == nil {
						require.Len(t, copies, 3)
						return true
					}
					require.Equal(t, context.Canceled, err)
					requireDocumentState(context.WithoutCancel(ctx), t, before, doc, block)
					if tc.child {
						got, cerr := doc.ContainerOf(block)
						require.NoError(t, cerr)
						require.True(t, got == before.containers[1], "the operand is back in its child container")
						requireSameBodies(t, []*Body{block}, got.Bodies(), "child member")
					}
					return false
				}
			})
		})
	}
}

func TestPathPatternSuccessRecordsOneCopyEntryPerInstance(t *testing.T) {
	t.Parallel()
	doc := New()
	block := internalBoxBody(t, doc, 0, 0, 4, 4, 2)
	first := doc.producerAt(t.Context())
	copies, err := PathPattern(block, straightPath(t), PathCount{Count: 4}).Do(t.Context())
	require.NoError(t, err)
	require.Len(t, copies, 3)
	require.Len(t, doc.Bodies(), 4)
	require.Equal(t, first+3, doc.producerAt(t.Context()))
	ops := doc.OperationHistory().Operations()
	require.Len(t, ops, 4, "the block's own extrusion and three copies")
	for i, c := range copies {
		op := ops[i+1]
		require.Equal(t, OperationCopy, op.Kind())
		in := op.Inputs()
		require.Len(t, in, 1)
		require.Same(t, block, in[0].Body)
		require.Equal(t, InputReferenced, in[0].Use)
		out := op.Outputs()
		require.Len(t, out, 1)
		require.Same(t, c, out[0])
		require.Equal(t, first+producerID(i), c.n.Origin.Producer)
		box, err := c.Bounds()
		require.NoError(t, err)
		x := 10 * float64(i+1)
		require.Equal(t, r3.Vec{X: x}, box.Min)
	}
}
