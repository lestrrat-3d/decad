package meshbool

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestDelaunayFlip holds the flip test to exact coplanarity, strict convexity
// and a strictly inside fourth point: the facets (u, w, b) and (w, u, d) over
// the chord u = (0, 0), w = (2, 0) of the unit circle about (1, 0) flip only
// when d lies inside that circle on the far side of the chord, in their plane.
// A tilted plane reads the same answer through its coordinate projection.
//
// Legs shown to fail: dropping the coplanarity test flips the raised point,
// dropping the convexity test flips the point on b's side, and a non-strict
// circle test flips the cocircular point.
func TestDelaunayFlip(t *testing.T) {
	t.Parallel()
	u, w, b := r3.Vec{}, r3.Vec{X: 2}, r3.Vec{X: 1, Y: 1}
	tilt := func(p r3.Vec) r3.Vec { return r3.Vec{X: p.X, Y: p.Y, Z: p.X} }
	for _, tc := range []struct {
		name string
		lift func(r3.Vec) r3.Vec
		d    r3.Vec
		flip bool
	}{
		{"inside the circle", nil, r3.Vec{X: 1, Y: -0.5}, true},
		{"inside the circle on a tilted plane", tilt, r3.Vec{X: 1, Y: -0.5}, true},
		{"on the circle", nil, r3.Vec{X: 1, Y: -1}, false},
		{"outside the circle", nil, r3.Vec{X: 1, Y: -2}, false},
		{"inside the circle on b's side", nil, r3.Vec{X: 1, Y: 0.5}, false},
		{"off the plane", nil, r3.Vec{X: 1, Y: -0.5, Z: 0.125}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			lift := tc.lift
			if lift == nil {
				lift = func(p r3.Vec) r3.Vec { return p }
			}
			got := delaunayFlip(proof.XptOf(lift(u)), proof.XptOf(lift(w)), proof.XptOf(lift(b)), proof.XptOf(lift(tc.d)))
			require.Equal(t, tc.flip, got)
		})
	}
}

// TestFlipDelaunayFromClearsANearlyStraightChain flips a planar fan whose
// chord A, B and the two points C, D lie within 2^-60 of one line, with E
// well above it: the facet (A, B, C) is a sliver whose circle holds D, and the
// flip that removes it leaves another sliver whose circle holds E. A needle
// pass that refuses to create a needle stops at the first; the Delaunay pass
// takes both and joins the chord to E. The flips keep the region: its signed
// area and its boundary edges are unchanged. A neighbour from another source
// is never flipped, and neither is a pair of facets that are not needles,
// even across an edge that is not Delaunay.
//
// Legs shown to fail: dropping the source test flips the second case, and
// dropping the needle test flips the third.
func TestFlipDelaunayFromClearsANearlyStraightChain(t *testing.T) {
	t.Parallel()
	h := math.Ldexp(1, -60)
	const a, b, c, d, e = 0, 1, 2, 3, 4
	verts := []proof.Xpt{
		proof.XptOf(r3.Vec{X: 8}),
		proof.XptOf(r3.Vec{X: 7}),
		proof.XptOf(r3.Vec{Y: h}),
		proof.XptOf(r3.Vec{Y: h + math.Ldexp(1, -110)}),
		proof.XptOf(r3.Vec{X: 0.01, Y: 0.05}),
	}
	held := make([]r3.Vec, len(verts))
	for i, p := range verts {
		held[i] = p.Vec()
	}
	start := [][3]int{{a, b, c}, {c, d, a}, {a, d, e}}
	t.Run("needle pass", func(t *testing.T) {
		t.Parallel()
		tris := append([][3]int(nil), start...)
		n, err := FlipNeedles(t.Context(), verts, tris, []int{7, 7, 7}, []float64{0, 0, 0})
		require.NoError(t, err)
		require.Zero(t, n)
	})
	t.Run("same source", func(t *testing.T) {
		t.Parallel()
		tris := append([][3]int(nil), start...)
		bounds := []float64{0.25, 0.5, 0.125}
		n, err := flipDelaunayFrom(t.Context(), verts, held, tris, []int{7, 7, 7}, bounds, []int{0})
		require.NoError(t, err)
		require.GreaterOrEqual(t, n, 2)
		chord := -1
		for _, tri := range tris {
			for k := range 3 {
				if tri[k] == a && tri[(k+1)%3] == b {
					chord = tri[(k+2)%3]
				}
			}
		}
		require.Equal(t, e, chord, "the chord A→B must take E as its apex")
		require.Zero(t, signedAreaZ(verts, tris).Cmp(signedAreaZ(verts, start)))
		require.Equal(t, boundaryEdges(start), boundaryEdges(tris))
		for _, bound := range bounds {
			require.Equal(t, 0.5, bound)
		}
	})
	t.Run("another source", func(t *testing.T) {
		t.Parallel()
		tris := append([][3]int(nil), start...)
		n, err := flipDelaunayFrom(t.Context(), verts, held, tris, []int{7, 8, 7}, []float64{0, 0, 0}, []int{0})
		require.NoError(t, err)
		require.Zero(t, n)
		require.Equal(t, start, tris)
	})
	t.Run("no needle", func(t *testing.T) {
		t.Parallel()
		fat := []proof.Xpt{
			proof.XptOf(r3.Vec{}), proof.XptOf(r3.Vec{X: 2}),
			proof.XptOf(r3.Vec{X: 1, Y: 1}), proof.XptOf(r3.Vec{X: 1, Y: -0.5}),
		}
		fatHeld := []r3.Vec{fat[0].Vec(), fat[1].Vec(), fat[2].Vec(), fat[3].Vec()}
		require.True(t, delaunayFlip(fat[0], fat[1], fat[2], fat[3]))
		tris := [][3]int{{0, 1, 2}, {1, 0, 3}}
		n, err := flipDelaunayFrom(t.Context(), fat, fatHeld, tris, []int{7, 7}, []float64{0, 0}, []int{0, 1})
		require.NoError(t, err)
		require.Zero(t, n)
		require.Equal(t, [][3]int{{0, 1, 2}, {1, 0, 3}}, tris)
	})
}

// signedAreaZ is twice the exact signed xy area of tris.
func signedAreaZ(verts []proof.Xpt, tris [][3]int) *big.Rat {
	sum := new(big.Rat)
	for _, tri := range tris {
		n := proof.Xcross(proof.Xsub(verts[tri[1]], verts[tri[0]]), proof.Xsub(verts[tri[2]], verts[tri[0]]))
		sum.Add(sum, RatCoordOf(n, 2))
	}
	return sum
}

// boundaryEdges is the set of directed edges of tris whose reverse is not an
// edge of tris.
func boundaryEdges(tris [][3]int) map[[2]int]struct{} {
	all := map[[2]int]struct{}{}
	for _, tri := range tris {
		for k := range 3 {
			all[[2]int{tri[k], tri[(k+1)%3]}] = struct{}{}
		}
	}
	out := map[[2]int]struct{}{}
	for edge := range all {
		if _, ok := all[[2]int{edge[1], edge[0]}]; !ok {
			out[edge] = struct{}{}
		}
	}
	return out
}
