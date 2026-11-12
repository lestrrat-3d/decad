package explodeblock_test

import (
	"math"
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad/internal/explodeblock"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func place(t *testing.T, m explodeblock.Mesh, dir r3.Vec) explodeblock.Instance {
	t.Helper()
	p, err := explodeblock.Project(m, dir)
	require.NoError(t, err)
	return explodeblock.Instance{P: p}
}

func verdict(t *testing.T, mover, other explodeblock.Mesh, dir r3.Vec) explodeblock.Verdict {
	t.Helper()
	return explodeblock.Block(place(t, mover, dir), place(t, other, dir)).Verdict
}

// boxOnlyBlocked is the rule the design rules out: shadow boxes overlap and the
// other box reaches ahead of the mover's trailing end.
func boxOnlyBlocked(a, b explodeblock.Mesh, up bool) bool {
	lo := func(m explodeblock.Mesh) (r3.Vec, r3.Vec) {
		mn, mx := m.Vertices[0], m.Vertices[0]
		for _, v := range m.Vertices {
			mn = r3.NewVec(math.Min(mn.X, v.X), math.Min(mn.Y, v.Y), math.Min(mn.Z, v.Z))
			mx = r3.NewVec(math.Max(mx.X, v.X), math.Max(mx.Y, v.Y), math.Max(mx.Z, v.Z))
		}
		return mn, mx
	}
	amn, amx := lo(a)
	bmn, bmx := lo(b)
	if amn.X > bmx.X || bmn.X > amx.X || amn.Y > bmx.Y || bmn.Y > amx.Y {
		return false
	}
	if up {
		return bmx.Z > amn.Z
	}
	return bmn.Z < amx.Z
}

// The shaft's collar is wider than the bearing bore, so the shaft can only
// leave away from the bearing's far side: toward the bearing it is blocked.
func TestCollarShaftAndBearing(t *testing.T) {
	const bound = 0.02
	shaftMesh := collarShaft()
	bearing := ring(bound)

	require.Equal(t, explodeblock.Blocked, verdict(t, shaftMesh, bearing, up))
	require.Equal(t, explodeblock.Free, verdict(t, shaftMesh, bearing, down))

	// The bearing is not blocked by the shaft in either sense: it slides off
	// the free end.
	require.Equal(t, explodeblock.Free, verdict(t, bearing, shaftMesh, up))
	require.Equal(t, explodeblock.Blocked, verdict(t, bearing, shaftMesh, down))

	// Boxes alone call the shaft blocked both ways.
	require.True(t, boxOnlyBlocked(shaftMesh, bearing, true))
	require.True(t, boxOnlyBlocked(shaftMesh, bearing, false))
}

func TestCollarWitness(t *testing.T) {
	shaftMesh := collarShaft()
	bearing := ring(0.02)
	res := explodeblock.Block(place(t, shaftMesh, up), place(t, bearing, up))
	require.Equal(t, explodeblock.Blocked, res.Verdict)
	// The witness lies on a collar face under the bearing's annulus
	// (2.5 < r < 4.95), and the point plus the height reaches a bearing face
	// (z = 0 or z = 4) exactly.
	w := res.Witness
	r := math.Hypot(w.Point.X, w.Point.Y)
	require.Greater(t, r, 2.5)
	require.Less(t, r, 4.95)
	require.Greater(t, w.Height.Base(), 1.9)
	top := w.Point.Z + w.Height.Base()
	require.True(t, math.Abs(top) < 1e-9 || math.Abs(top-4) < 1e-9, "reached z=%v", top)
	require.True(t, math.Abs(w.Point.Z+6) < 1e-9 || math.Abs(w.Point.Z+2) < 1e-9, "on z=%v", w.Point.Z)
}

func TestClearanceShaftFreeBothWays(t *testing.T) {
	const bound = 0.02
	through := shaft(2.25, 12, bound)
	pin := shaft(2.25, 2, bound) // ends inside the bore
	bearing := ring(bound)
	for _, dir := range []r3.Vec{up, down} {
		require.Equal(t, explodeblock.Free, verdict(t, through, bearing, dir))
		require.Equal(t, explodeblock.Free, verdict(t, bearing, through, dir))
		require.Equal(t, explodeblock.Free, verdict(t, pin, bearing, dir))
	}
}

// A gap below the sum of the bounds cannot be told from touching: the answer
// is undecided, never a block that would deadlock a removal search.
func TestPressFitIsUndecidedNotBlocked(t *testing.T) {
	const bound = 0.02
	bearing := ring(bound)
	for _, r := range []float64{2.49, 2.505} { // 0.01 gap, 0.005 overlap sliver
		pin := shaft(r, 2, bound)
		for _, dir := range []r3.Vec{up, down} {
			require.Equal(t, explodeblock.Undecided, verdict(t, pin, bearing, dir), "r=%v dir=%v", r, dir)
		}
	}
	// With no bound the same overlap sliver is a real block.
	pin := shaft(2.505, 2, 0)
	require.Equal(t, explodeblock.Blocked, verdict(t, pin, ring(0), up))
}

func TestCoplanarTouchingIsNotABlock(t *testing.T) {
	a := tri(r3.NewVec(0, 0, 1), r3.NewVec(10, 0, 1), r3.NewVec(0, 10, 1))
	at := func(z float64) explodeblock.Mesh {
		return tri(r3.NewVec(1, 1, z), r3.NewVec(5, 1, z), r3.NewVec(1, 5, z))
	}
	require.Equal(t, explodeblock.Free, verdict(t, a, at(1), up))
	require.Equal(t, explodeblock.Free, verdict(t, a, at(1+5e-7), up))
	require.Equal(t, explodeblock.Blocked, verdict(t, a, at(1.001), up))
	require.Equal(t, explodeblock.Free, verdict(t, a, at(0.999), up))

	// Stacked boxes: the top box leaves upward whatever the contact.
	lower := box(r3.NewVec(0, 0, 0), r3.NewVec(4, 4, 1))
	upper := box(r3.NewVec(0, 0, 1), r3.NewVec(4, 4, 2))
	require.Equal(t, explodeblock.Free, verdict(t, upper, lower, up))
	require.Equal(t, explodeblock.Blocked, verdict(t, lower, upper, up))
}

func TestWitnessValues(t *testing.T) {
	a := tri(r3.NewVec(0, 0, 1), r3.NewVec(10, 0, 1), r3.NewVec(0, 10, 1))
	flat := tri(r3.NewVec(1, 1, 3), r3.NewVec(5, 1, 3), r3.NewVec(1, 5, 3))
	res := explodeblock.Block(place(t, a, up), place(t, flat, up))
	require.Equal(t, explodeblock.Blocked, res.Verdict)
	require.InDelta(t, 2.0, res.Witness.Height.Base(), 1e-9)
	require.InDelta(t, 7.0/3, res.Witness.Point.X, 1e-9)
	require.InDelta(t, 7.0/3, res.Witness.Point.Y, 1e-9)
	require.InDelta(t, 1.0, res.Witness.Point.Z, 1e-9)

	// z = 2 + x/2 over the same footprint: 3.1667 at the centroid, 2.1667 above a.
	tilt := tri(r3.NewVec(1, 1, 2.5), r3.NewVec(5, 1, 4.5), r3.NewVec(1, 5, 2.5))
	res = explodeblock.Block(place(t, a, up), place(t, tilt, up))
	require.Equal(t, explodeblock.Blocked, res.Verdict)
	require.InDelta(t, 2+7.0/6-1, res.Witness.Height.Base(), 1e-9)

	// Along the other sense the same pair measures the flipped way: blocked
	// would need the plate below, so the plate above is free.
	require.Equal(t, explodeblock.Free, verdict(t, a, flat, down))
}

// One projection serves several instances: only the offset differs.
func TestSharedProjectionWithOffset(t *testing.T) {
	a := tri(r3.NewVec(0, 0, 1), r3.NewVec(10, 0, 1), r3.NewVec(0, 10, 1))
	b := tri(r3.NewVec(1, 1, 3), r3.NewVec(5, 1, 3), r3.NewVec(1, 5, 3))
	pa, err := explodeblock.Project(a, up)
	require.NoError(t, err)
	pb, err := explodeblock.Project(b, up)
	require.NoError(t, err)
	ia := explodeblock.Instance{P: pa}

	res := explodeblock.Block(ia, explodeblock.Instance{P: pb})
	require.Equal(t, explodeblock.Blocked, res.Verdict)
	// Moved to z = -9 the plate is behind the mover; moved by +20 in x it is clear.
	require.Equal(t, explodeblock.Free, explodeblock.Block(ia, explodeblock.Instance{P: pb, Offset: r3.NewVec(0, 0, -12)}).Verdict)
	require.Equal(t, explodeblock.Free, explodeblock.Block(ia, explodeblock.Instance{P: pb, Offset: r3.NewVec(20, 0, 0)}).Verdict)
	// Moving the mover by the opposite offset gives the same relative answer.
	res2 := explodeblock.Block(explodeblock.Instance{P: pa, Offset: r3.NewVec(3, 0, 5)}, explodeblock.Instance{P: pb, Offset: r3.NewVec(3, 0, 5)})
	require.Equal(t, explodeblock.Blocked, res2.Verdict)
	require.InDelta(t, res.Witness.Height.Base(), res2.Witness.Height.Base(), 1e-9)
	require.InDelta(t, res.Witness.Point.X+3, res2.Witness.Point.X, 1e-9)
	require.InDelta(t, res.Witness.Point.Z+5, res2.Witness.Point.Z, 1e-9)
}

func TestObliqueDirection(t *testing.T) {
	d := r3.NewVec(1, 1, 1)
	flat := box(r3.NewVec(-3, -3, -3), r3.NewVec(3, 3, 3))
	flatAhead := box(r3.NewVec(17, 17, 17), r3.NewVec(23, 23, 23))
	require.Equal(t, explodeblock.Blocked, verdict(t, flat, flatAhead, d))
	require.Equal(t, explodeblock.Free, verdict(t, flatAhead, flat, d))
}

func TestProjectRejectsBadInput(t *testing.T) {
	m := box(r3.NewVec(0, 0, 0), r3.NewVec(1, 1, 1))
	_, err := explodeblock.Project(m, r3.Vec{})
	require.Error(t, err)
	_, err = explodeblock.Project(m, r3.NewVec(math.NaN(), 0, 1))
	require.Error(t, err)
	bad := m
	bad.Triangles = append([][3]int{{0, 1, 99}}, bad.Triangles...)
	_, err = explodeblock.Project(bad, up)
	require.Error(t, err)
}

func TestConcurrentBlockIsSafe(t *testing.T) {
	const bound = 0.02
	s := place(t, collarShaft(), up)
	b := place(t, ring(bound), up)
	var wg sync.WaitGroup
	got := make([]explodeblock.Verdict, 8)
	for i := range got {
		wg.Go(func() { got[i] = explodeblock.Block(s, b).Verdict })
	}
	wg.Wait()
	for _, v := range got {
		require.Equal(t, explodeblock.Blocked, v)
	}
}
