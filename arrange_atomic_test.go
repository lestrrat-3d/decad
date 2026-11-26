package decad

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func atomicArrangeBlocks(t *testing.T, doc *Document, sizes [][2]float64) []*Body {
	t.Helper()
	out := make([]*Body, len(sizes))
	for i, sz := range sizes {
		x := float64(i) * 1000
		out[i] = internalBoxBody(t, doc, x, -x, x+sz[0], -x+sz[1], 3)
	}
	return out
}

func atomicArrangeBed(width, depth float64) ArrangeBed {
	return ArrangeBed{
		Origin: r3.Vec{X: 100, Y: 200, Z: 5},
		Width:  units.Millimeters(width),
		Depth:  units.Millimeters(depth),
		Gap:    units.Millimeters(1),
	}
}

// TestArrangeCanceledMidwayLeavesTheDocumentUnchanged stops Arrange at a
// spread of its cancellation checks. Every stopped call leaves the document as
// it was, with and without turns, so no body keeps a place it was moved to.
func TestArrangeCanceledMidwayLeavesTheDocumentUnchanged(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name         string
		width, depth float64
		opts         []ArrangeOption
		circles      bool
	}{
		{"plain", 60, 40, nil, false},
		{"turns", 30, 20, []ArrangeOption{WithArrangeTurns()}, false},
		{"angle", 60, 40, []ArrangeOption{WithArrangeAngle(units.Degrees(45))}, false},
		{"child circles", 60, 40, nil, true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			sweepCancellation(t, func() func(ctx context.Context) bool {
				doc := New()
				var blocks []*Body
				var child *Container
				if tc.circles {
					for i := range 2 {
						blocks = append(blocks, internalCircleBody(t, doc, float64(i)*1000, 5, 0,
							Distance{D: units.Millimeters(3), Dir: Along}))
					}
					var err error
					child, err = doc.Root().NewChild("parts")
					require.NoError(t, err)
					for _, b := range blocks {
						require.NoError(t, child.AddBody(b))
					}
				} else {
					blocks = atomicArrangeBlocks(t, doc, [][2]float64{{4, 20}, {4, 12}})
				}
				before := captureDocumentState(t.Context(), t, doc)
				return func(ctx context.Context) bool {
					moved, err := Arrange(ctx, blocks, atomicArrangeBed(tc.width, tc.depth), tc.opts...)
					if err == nil {
						require.Len(t, moved, len(blocks))
						return true
					}
					require.ErrorIs(t, err, context.Canceled)
					requireDocumentState(context.WithoutCancel(ctx), t, before, doc, blocks...)
					if child != nil {
						for _, b := range blocks {
							got, cerr := doc.ContainerOf(b)
							require.NoError(t, cerr)
							require.True(t, got == child, "the operand is back in its child container")
						}
						requireSameBodies(t, blocks, child.Bodies(), "child member")
					}
					return false
				}
			})
		})
	}
}

func TestArrangeSuccessRecordsOneTransformEntryPerBody(t *testing.T) {
	t.Parallel()
	doc := New()
	blocks := atomicArrangeBlocks(t, doc, [][2]float64{{10, 5}, {10, 8}})
	first := doc.producerAt(t.Context())
	moved, err := Arrange(t.Context(), blocks, ArrangeBed{
		Origin: r3.Vec{X: 100, Y: 200, Z: 5},
		Width:  units.Millimeters(30), Depth: units.Millimeters(20), Gap: units.Millimeters(2),
	})
	require.NoError(t, err)
	require.Len(t, moved, 2)
	require.Len(t, doc.Bodies(), 2)
	require.Equal(t, first+2, doc.producerAt(t.Context()))
	ops := doc.OperationHistory().Operations()
	require.Len(t, ops, 4, "two extrusions and two moves")
	for i, b := range blocks {
		op := ops[2+i]
		require.Equal(t, OperationTransform, op.Kind())
		in := op.Inputs()
		require.Len(t, in, 1)
		require.Same(t, b, in[0].Body)
		require.Equal(t, InputConsumed, in[0].Use)
		out := op.Outputs()
		require.Len(t, out, 1)
		require.Same(t, moved[i], out[0])
		require.Equal(t, first+producerID(i), moved[i].n.Origin.Producer)
	}
}
