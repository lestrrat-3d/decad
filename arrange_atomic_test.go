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

// TestArrangeCanceledMidwayLeavesTheDocumentUnchanged stops Arrange at each of
// its cancellation checks in turn. Every stopped call leaves the document as
// it was, with and without turns, so no body keeps a place it was moved to.
func TestArrangeCanceledMidwayLeavesTheDocumentUnchanged(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name         string
		width, depth float64
		opts         []ArrangeOption
	}{
		{"plain", 60, 40, nil},
		{"turns", 30, 20, []ArrangeOption{WithArrangeTurns()}},
		{"angle", 60, 40, []ArrangeOption{WithArrangeAngle(units.Degrees(45))}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			succeeded := false
			stopped := 0
			for limit := 0; limit < 20000 && !succeeded; limit++ {
				doc := New()
				blocks := atomicArrangeBlocks(t, doc, [][2]float64{{4, 20}, {4, 20}, {4, 12}})
				before := captureDocumentState(t, doc, blocks...)
				moved, err := Arrange(newCancelAfterChecks(limit), blocks, atomicArrangeBed(tc.width, tc.depth), tc.opts...)
				if err == nil {
					require.Len(t, moved, len(blocks))
					succeeded = true
					continue
				}
				stopped++
				require.ErrorIs(t, err, context.Canceled)
				requireDocumentState(t, before, doc, blocks...)
			}
			require.True(t, succeeded, "the sweep reached a call that no cancellation stopped")
			require.Positive(t, stopped)
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
