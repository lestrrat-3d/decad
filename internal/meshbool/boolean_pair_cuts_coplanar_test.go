package meshbool

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/coplanarcontact"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// exactBoxMesh is the axis-aligned box with float corners lo and hi, held
// exactly: every vertex bound is zero and every facet is marked exact, as
// SetExactFacets marks the facets of a face with a zero reading.
func exactBoxMesh(t *testing.T, lo, hi r3.Vec) *BoolMesh {
	t.Helper()
	half := r3.Vec{X: (hi.X - lo.X) / 2, Y: (hi.Y - lo.Y) / 2, Z: (hi.Z - lo.Z) / 2}
	at := r3.Vec{X: (hi.X + lo.X) / 2, Y: (hi.Y + lo.Y) / 2, Z: (hi.Z + lo.Z) / 2}
	bm := boxMesh(t, half, func(v r3.Vec) r3.Vec { return v }, at)
	bm.SetVertexBounds(make([]float64, len(bm.Verts)))
	bm.Exact = make([]bool, len(bm.Tris))
	for i := range bm.Exact {
		bm.Exact[i] = true
	}
	return bm
}

func coplanarCtx(t *testing.T) context.Context {
	return coplanarcontact.With(t.Context(), true)
}

// TestPairCutsCoplanarAdmitted pins a box resting on a box, with the upper
// box shifted in x: the coplanar pass admits the pair, inserts only the
// patch-boundary segments that cross the other facet's interior, and records
// the refusal the switch-off pass returns.
//
// Leg shown to fail: inserting a facet's triangulation diagonals adds segments
// off the x = 0.5 and x = 10 lines.
func TestPairCutsCoplanarAdmitted(t *testing.T) {
	a := exactBoxMesh(t, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10})
	b := exactBoxMesh(t, r3.Vec{X: 0.5, Z: 10}, r3.Vec{X: 10.5, Y: 10, Z: 20})
	res, err := pairCuts(coplanarCtx(t), a, b, NewContactMemo(a, b), newKeyMemo())
	require.NoError(t, err)
	require.NotEmpty(t, res.CoplanarA)
	require.NotEmpty(t, res.CoplanarB)
	topPairs := 0
	for i, js := range res.CoplanarA {
		ta := TriCorners(a, i)
		for _, j := range js {
			for _, vj := range b.Tris[j] {
				require.Zero(t, proof.OrientSign(ta[0], ta[1], ta[2], b.Verts[vj]), "pair (%d, %d) is coplanar", i, j)
			}
			if ta[0].Z == 10 && ta[1].Z == 10 && ta[2].Z == 10 {
				topPairs++
			}
		}
	}
	require.Positive(t, topPairs, "the top and bottom facets pair up")
	// width is the summed y extent of the segments of cuts, which must all lie
	// on the plane x = line: the patch rims of the other box.
	width := func(cuts map[int][]Xseg, line float64) float64 {
		total := 0.0
		for _, segs := range cuts {
			for _, s := range segs {
				require.True(t, s.ViaParity)
				require.InDelta(t, line, s.A.Vec().X, 0, "a cut lies on one patch rim")
				require.InDelta(t, line, s.B.Vec().X, 0)
				total += math.Abs(s.B.Vec().Y - s.A.Vec().Y)
			}
		}
		return total
	}
	require.InDelta(t, 10, width(res.CutsA, 0.5), 0, "B's rim x = 0.5 crosses A's whole top")
	require.InDelta(t, 10, width(res.CutsB, 10), 0, "A's rim x = 10 crosses B's whole bottom")
	for _, p := range []r3.Vec{{X: 0.5, Y: 0, Z: 10}, {X: 0.5, Y: 10, Z: 10}, {X: 10, Y: 0, Z: 10}, {X: 10, Y: 10, Z: 10}} {
		_, ok := res.Owned[proof.XptOf(p).Key()]
		require.True(t, ok, "%v is owned", p)
	}

	// The legacy refusal is the switch-off pass's own.
	_, err = pairCuts(t.Context(), a, b, NewContactMemo(a, b), newKeyMemo())
	off, ok := errors.AsType[*TouchContactError](err)
	require.True(t, ok, "the switch-off pass refuses: %v", err)
	require.Equal(t, off.Contacts, mustLegacy(t, res).Contacts)
	require.Equal(t, off.What, mustLegacy(t, res).What)
}

func mustLegacy(t *testing.T, res pairCutsResult) *TouchContactError {
	t.Helper()
	require.Error(t, res.Legacy)
	touch, ok := errors.AsType[*TouchContactError](res.Legacy)
	require.True(t, ok)
	return touch
}

// TestPairCutsCoplanarRefusedWhenInexact pins that one inexact facet refuses
// the pair as the switch-off pass does.
func TestPairCutsCoplanarRefusedWhenInexact(t *testing.T) {
	a := exactBoxMesh(t, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10})
	b := exactBoxMesh(t, r3.Vec{X: 0.5, Z: 10}, r3.Vec{X: 10.5, Y: 10, Z: 20})
	for i := range a.Exact {
		a.Exact[i] = false
	}
	_, err := pairCuts(coplanarCtx(t), a, b, NewContactMemo(a, b), newKeyMemo())
	got, ok := errors.AsType[*TouchContactError](err)
	require.True(t, ok, "%v", err)
	require.Equal(t, touchCoplanar, got.What)
	_, err = pairCuts(t.Context(), a, b, NewContactMemo(a, b), newKeyMemo())
	want, ok := errors.AsType[*TouchContactError](err)
	require.True(t, ok)
	require.Equal(t, want.Contacts, got.Contacts)
	// Exact flags that were never set read as all false.
	a.Exact, b.Exact = nil, nil
	_, err = pairCuts(coplanarCtx(t), a, b, NewContactMemo(a, b), newKeyMemo())
	got, ok = errors.AsType[*TouchContactError](err)
	require.True(t, ok)
	require.Equal(t, want.Contacts, got.Contacts)
}

// TestPairCutsCoplanarSwitchOff pins that with the switch off the exact flags
// change nothing: the pair is refused with the same contacts and no coplanar
// state is built.
func TestPairCutsCoplanarSwitchOff(t *testing.T) {
	a := exactBoxMesh(t, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10})
	b := exactBoxMesh(t, r3.Vec{X: 0.5, Z: 10}, r3.Vec{X: 10.5, Y: 10, Z: 20})
	res, err := pairCuts(t.Context(), a, b, NewContactMemo(a, b), newKeyMemo())
	require.Error(t, err)
	require.Nil(t, res.Owned)
	require.Nil(t, res.CoplanarA)
	touch, ok := errors.AsType[*TouchContactError](err)
	require.True(t, ok)
	require.Equal(t, touchCoplanar, touch.What)
}

// TestEdgeSide pins the three readings of an in-plane edge: a crossing, a
// graze and the rim of a coplanar patch, which names the adjacent facet that
// lies in the plane.
func TestEdgeSide(t *testing.T) {
	a := exactBoxMesh(t, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10})
	// The partner plane z = 10 holds A's top facets: every edge of a wall
	// facet along z = 10 is the rim of the top patch.
	var wall, k = -1, -1
	for f, tri := range a.Tris {
		if a.Verts[tri[0]].Y != 0 || a.Verts[tri[1]].Y != 0 || a.Verts[tri[2]].Y != 0 {
			continue
		}
		for e := range 3 {
			u, v := a.Verts[tri[e]], a.Verts[tri[(e+1)%3]]
			if u.Z == 10 && v.Z == 10 {
				wall, k = f, e
			}
		}
	}
	require.GreaterOrEqual(t, wall, 0)
	top := [3]r3.Vec{{Z: 10}, {X: 1, Z: 10}, {Y: 1, Z: 10}}
	_, side, inPlane, err := EdgeSide(a, wall, k, top)
	require.NoError(t, err)
	require.Equal(t, EdgeCoplanarBounded, side)
	require.GreaterOrEqual(t, inPlane, 0)
	for _, vi := range a.Tris[inPlane] {
		require.InDelta(t, 10, a.Verts[vi].Z, 0, "the in-plane neighbour is a top facet")
	}
	// A plane through the edge that the wall apex and the top apex lie on
	// opposite sides of is a crossing; one they lie on the same side of is a
	// graze. Neither names an in-plane facet.
	crossingPlane := [3]r3.Vec{{Z: 10}, {X: 1, Z: 10}, {Y: 1, Z: 9}}
	_, side, inPlane, err = EdgeSide(a, wall, k, crossingPlane)
	require.NoError(t, err)
	require.Equal(t, EdgeCrossing, side)
	require.Equal(t, -1, inPlane)
	grazePlane := [3]r3.Vec{{Z: 10}, {X: 1, Z: 10}, {Y: 1, Z: 11}}
	_, side, inPlane, err = EdgeSide(a, wall, k, grazePlane)
	require.NoError(t, err)
	require.Equal(t, EdgeGraze, side)
	require.Equal(t, -1, inPlane)
	_, crossing, err := EdgeCrosses(a, wall, k, top)
	require.NoError(t, err)
	require.False(t, crossing, "the wrapper reads a coplanar-bounded edge as a graze")
}
