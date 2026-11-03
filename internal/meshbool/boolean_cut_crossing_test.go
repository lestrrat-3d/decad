package meshbool

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestCutTriangleRefusesCrossingCurves cuts the facet (0,0), (4,0), (0,4)
// along two contact segments from its bottom edge to its left edge. Two
// curves that cross inside the facet come from a held surface folding through
// itself, and the cutter refuses them as an unclassifiable contact; two that
// do not cross split the facet into three regions.
//
// Shown to fail: without the crossing check the crossing pair stops in the
// region split with a bare ErrBooleanFailed that Verify reports as a failed
// call rather than an undecided pair.
func TestCutTriangleRefusesCrossingCurves(t *testing.T) {
	t.Parallel()
	pt := func(x, y float64) proof.Xpt { return proof.XptOf(r3.Vec{X: x, Y: y}) }
	tri := [3]proof.Xpt{pt(0, 0), pt(4, 0), pt(0, 4)}
	normal := proof.XptOf(r3.Vec{Z: 1})
	seg := func(x0, y1 float64) Xseg {
		return Xseg{A: pt(x0, 0), B: pt(0, y1), AOnEdge: true, BOnEdge: true,
			Partner: [3]r3.Vec{{X: x0, Z: -1}, {Y: y1, Z: -1}, {X: x0, Y: y1, Z: 1}}}
	}
	t.Run("crossing curves", func(t *testing.T) {
		t.Parallel()
		_, err := CutTriangle(t.Context(), tri, normal, []Xseg{seg(1, 2), seg(2, 1)})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		var expected *BooleanExpectedError
		require.True(t, errors.As(err, &expected), "the refusal is an expected contact, not a failed call")
		require.Equal(t, BooleanExpectedContact, expected.Kind)
	})
	t.Run("nested curves", func(t *testing.T) {
		t.Parallel()
		regions, err := CutTriangle(t.Context(), tri, normal, []Xseg{seg(1, 1), seg(2, 2)})
		require.NoError(t, err)
		require.Len(t, regions, 3)
	})
}
