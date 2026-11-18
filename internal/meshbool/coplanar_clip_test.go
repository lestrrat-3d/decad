package meshbool_test

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

func clipPt(x, y float64) proof.Xpt { return proof.XptOf(r3.Vec{X: x, Y: y}) }

// TestCoplanarClip pins one edge clipped to the closed triangle (0,0) (4,0)
// (0,4): pieces inside are kept with exact ends, and a point, an empty piece or
// a piece on one of the triangle's own edges is dropped.
//
// Leg shown to fail: dropping the SegAlongEdge test keeps the collinear
// pieces.
func TestCoplanarClip(t *testing.T) {
	t.Parallel()
	tri := [3]proof.Xpt{clipPt(0, 0), clipPt(4, 0), clipPt(0, 4)}
	n := proof.XptOf(r3.Vec{Z: 1})
	tests := []struct {
		name            string
		a, b            r3.Vec
		keep            bool
		p0, p1          r3.Vec
		p0OnA, p1OnA    bool
		reversedWinding bool
	}{
		{name: "inside", a: r3.Vec{X: 1, Y: 1}, b: r3.Vec{X: 2, Y: 1}, keep: true,
			p0: r3.Vec{X: 1, Y: 1}, p1: r3.Vec{X: 2, Y: 1}},
		{name: "crossing one side", a: r3.Vec{X: -1, Y: 1}, b: r3.Vec{X: 1, Y: 1}, keep: true,
			p0: r3.Vec{X: 0, Y: 1}, p1: r3.Vec{X: 1, Y: 1}, p0OnA: true},
		{name: "through", a: r3.Vec{X: -1, Y: 2}, b: r3.Vec{X: 5, Y: 2}, keep: true,
			p0: r3.Vec{X: 0, Y: 2}, p1: r3.Vec{X: 2, Y: 2}, p0OnA: true, p1OnA: true},
		{name: "through reversed", a: r3.Vec{X: 5, Y: 2}, b: r3.Vec{X: -1, Y: 2}, keep: true,
			p0: r3.Vec{X: 2, Y: 2}, p1: r3.Vec{X: 0, Y: 2}, p0OnA: true, p1OnA: true},
		{name: "shared edge", a: r3.Vec{X: 0, Y: 0}, b: r3.Vec{X: 4, Y: 0}},
		{name: "collinear overlap", a: r3.Vec{X: -2, Y: 0}, b: r3.Vec{X: 2, Y: 0}},
		{name: "collinear inside the edge", a: r3.Vec{X: 1, Y: 0}, b: r3.Vec{X: 2, Y: 0}},
		{name: "shared vertex", a: r3.Vec{X: -1, Y: -1}, b: r3.Vec{X: 0, Y: 0}},
		{name: "hypotenuse", a: r3.Vec{X: 5, Y: -1}, b: r3.Vec{X: -1, Y: 5}},
		{name: "disjoint", a: r3.Vec{X: 5, Y: 5}, b: r3.Vec{X: 6, Y: 5}},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			piece, ok, err := meshbool.CoplanarClip([2]proof.Xpt{proof.XptOf(tc.a), proof.XptOf(tc.b)}, tri, n)
			require.NoError(t, err)
			require.Equal(t, tc.keep, ok)
			if !ok {
				return
			}
			require.True(t, spanTestPointsEqual(piece.P0, proof.XptOf(tc.p0)), "P0 %v", piece.P0.Vec())
			require.True(t, spanTestPointsEqual(piece.P1, proof.XptOf(tc.p1)), "P1 %v", piece.P1.Vec())
			require.Equal(t, tc.p0OnA, piece.P0OnA)
			require.Equal(t, tc.p1OnA, piece.P1OnA)
		})
	}
}

// TestCoplanarClipExactRational pins a clip end that is not a float: the edge
// crosses the hypotenuse at (1 + 5/6, 0.5 + 5/3), and the piece end is that exact point.
func TestCoplanarClipExactRational(t *testing.T) {
	t.Parallel()
	tri := [3]proof.Xpt{clipPt(0, 0), clipPt(4, 0), clipPt(0, 4)}
	n := proof.XptOf(r3.Vec{Z: 1})
	// The line from (1, 0.5) to (3, 4.5) meets x + y = 4 at t = 5/12.
	piece, ok, err := meshbool.CoplanarClip([2]proof.Xpt{clipPt(1, 0.5), clipPt(3, 4.5)}, tri, n)
	require.NoError(t, err)
	require.True(t, ok)
	require.True(t, piece.P1OnA)
	require.False(t, piece.P0OnA)
	require.Equal(t, 0, meshbool.CompareCoord(piece.P1, proof.Xlerp(clipPt(1, 0.5), clipPt(3, 4.5), bigInt(5), bigInt(12)), 0))
	require.Equal(t, 0, meshbool.CompareCoord(piece.P1, proof.Xlerp(clipPt(1, 0.5), clipPt(3, 4.5), bigInt(5), bigInt(12)), 1))
}

func bigInt(v int64) *big.Int { return big.NewInt(v) }

// TestTriContactCoplanarFlag pins which classifier branch sets the flag: a
// coplanar region and a coplanar point do, a crossing segment does not.
func TestTriContactCoplanarFlag(t *testing.T) {
	t.Parallel()
	classify := func(ta, tb [3]r3.Vec) meshbool.TriContact {
		var xa, xb [3]proof.Xpt
		for i := range 3 {
			xa[i], xb[i] = proof.XptOf(ta[i]), proof.XptOf(tb[i])
		}
		na := proof.Xcross(proof.Xsub(xa[1], xa[0]), proof.Xsub(xa[2], xa[0]))
		nb := proof.Xcross(proof.Xsub(xb[1], xb[0]), proof.Xsub(xb[2], xb[0]))
		c, err := meshbool.TriTriClassify(ta, tb, xa, xb, na, nb)
		require.NoError(t, err)
		return c
	}
	base := [3]r3.Vec{{}, {X: 4}, {Y: 4}}
	region := classify(base, [3]r3.Vec{{X: 1, Y: 1}, {X: 3, Y: 1}, {X: 1, Y: 3}})
	require.Equal(t, meshbool.ContactRegion, region.Kind)
	require.True(t, region.Coplanar)
	point := classify(base, [3]r3.Vec{{X: -1, Y: -1}, {X: 0}, {X: -1, Y: 1}})
	require.Equal(t, meshbool.ContactPoint, point.Kind)
	require.True(t, point.Coplanar)
	crossing := classify(base, [3]r3.Vec{{X: 1, Y: 1, Z: -1}, {X: 1, Y: 1, Z: 1}, {X: 2, Y: 1, Z: 1}})
	require.Equal(t, meshbool.ContactSegment, crossing.Kind)
	require.False(t, crossing.Coplanar)
	none := classify(base, [3]r3.Vec{{X: 9, Y: 9}, {X: 10, Y: 9}, {X: 9, Y: 10}})
	require.Equal(t, meshbool.ContactNone, none.Kind)
	require.False(t, none.Coplanar)
}
