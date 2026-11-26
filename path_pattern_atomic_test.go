package decad

import (
	"context"
	"errors"
	"sync/atomic"
	"testing"
	"time"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// cancelAfterChecks is a context that reports itself canceled once Err has
// been called more than limit times, so a sweep over limit stops an operation
// at every one of its cancellation checks in turn.
type cancelAfterChecks struct {
	limit int64
	calls atomic.Int64
	done  chan struct{}
}

func newCancelAfterChecks(limit int) *cancelAfterChecks {
	return &cancelAfterChecks{limit: int64(limit), done: make(chan struct{})}
}

func (c *cancelAfterChecks) Deadline() (time.Time, bool) { return time.Time{}, false }
func (c *cancelAfterChecks) Done() <-chan struct{}       { return c.done }
func (c *cancelAfterChecks) Value(any) any               { return nil }
func (c *cancelAfterChecks) Err() error {
	if c.calls.Add(1) > c.limit {
		return context.Canceled
	}
	return nil
}

// documentState is the part of a document an atomic operation must leave
// alone: its live bodies, operation history, producer counter and the boxes of
// the bodies a test names.
type documentState struct {
	live  []*Body
	ops   []*Operation
	next  producerID
	boxes []r3.Vec
}

func captureDocumentState(t *testing.T, doc *Document, operands ...*Body) documentState {
	t.Helper()
	state := documentState{
		live: doc.Bodies(),
		ops:  doc.OperationHistory().Operations(),
		next: doc.producerAt(t.Context()),
	}
	for _, b := range operands {
		box, err := b.Bounds()
		require.NoError(t, err)
		state.boxes = append(state.boxes, box.Min, box.Max)
	}
	return state
}

// requireDocumentState compares by pointer and by count so a failure prints
// bounded data, never a body graph.
func requireDocumentState(t *testing.T, want documentState, doc *Document, operands ...*Body) {
	t.Helper()
	got := captureDocumentState(t, doc, operands...)
	require.Len(t, got.live, len(want.live), "live body count")
	for i := range want.live {
		require.Same(t, want.live[i], got.live[i], "live body %d", i)
	}
	require.Len(t, got.ops, len(want.ops), "history length")
	for i := range want.ops {
		require.Same(t, want.ops[i], got.ops[i], "history entry %d", i)
	}
	require.Equal(t, want.next, got.next, "next producer id")
	require.Equal(t, want.boxes, got.boxes, "operand placement")
	for _, b := range operands {
		consumers, err := doc.OperationHistory().Consumers(b)
		require.NoError(t, err)
		require.Empty(t, consumers, "no operation consumed the operand")
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
	before := captureDocumentState(t, doc, block)
	copies, err := PathPattern(t.Context(), block, path, WithPathCount(6), WithPathFollow())
	require.ErrorIs(t, err, ErrDegenerate)
	require.Equal(t, OperationCopy, operationKindOf(t, err))
	require.Nil(t, copies)
	requireDocumentState(t, before, doc, block)
	// The refused call used no producer number, so the next copy takes the
	// number the first copy of the refused call would have taken.
	again, err := Copy(t.Context(), block)
	require.NoError(t, err)
	require.Equal(t, before.next, again.n.Origin.Producer)
}

func TestPathPatternCanceledMidwayLeavesTheDocumentUnchanged(t *testing.T) {
	t.Parallel()
	succeeded := false
	for limit := 0; limit < 20000 && !succeeded; limit++ {
		doc := New()
		block := internalBoxBody(t, doc, 0, 0, 4, 4, 2)
		before := captureDocumentState(t, doc, block)
		copies, err := PathPattern(newCancelAfterChecks(limit), block, straightPath(t), WithPathCount(4))
		if err == nil {
			require.Len(t, copies, 3)
			succeeded = true
			continue
		}
		require.ErrorIs(t, err, context.Canceled)
		requireDocumentState(t, before, doc, block)
	}
	require.True(t, succeeded, "the sweep reached a call that no cancellation stopped")
}

func TestPathPatternSuccessRecordsOneCopyEntryPerInstance(t *testing.T) {
	t.Parallel()
	doc := New()
	block := internalBoxBody(t, doc, 0, 0, 4, 4, 2)
	first := doc.producerAt(t.Context())
	copies, err := PathPattern(t.Context(), block, straightPath(t), WithPathCount(4))
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
