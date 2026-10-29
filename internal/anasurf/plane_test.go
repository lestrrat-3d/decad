package anasurf_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/anasurf"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func frame(t *testing.T, o, u, v r3.Vec) surfacegeom.Plane {
	t.Helper()
	f, err := r3.NewFrame(o, u, v)
	require.NoError(t, err)
	return surfacegeom.Plane{Frame: f}
}

// axisPlane is the plane through the origin with outward normal -axis
// (material on the positive side of the axis), so Reversed carries it.
func axisPlane(t *testing.T, u, v r3.Vec) anasurf.Plane {
	t.Helper()
	p, err := anasurf.PlaneOf(frame(t, r3.NewVec(0, 0, 0), u, v), true)
	require.NoError(t, err)
	return p
}

func TestMeet3AxisPlanes(t *testing.T) {
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	// Normals: U×V = z, x, y, negated by Reversed: the material is on the positive side.
	px, py, pz := axisPlane(t, y, z), axisPlane(t, z, x), axisPlane(t, x, y)
	one := big.NewRat(1, 1)
	var in [3]anasurf.Plane
	for i, p := range []anasurf.Plane{px, py, pz} {
		q, err := p.Inward(one)
		require.NoError(t, err)
		in[i] = q
	}
	pt, err := anasurf.Meet3(in[0], in[1], in[2])
	require.NoError(t, err)
	held, bound := pt.Held()
	require.Equal(t, r3.NewVec(1, 1, 1), held)
	require.Zero(t, bound)
	require.False(t, anasurf.StrictlyInside(pt, in[0]), "a vertex lies on its own planes")
}

// bevelPlane is z - y = 18 with the exact frame U=(1,0,0), V=(0,c,c): U×V = (0,-c,c) is parallel to (0,-1,1) and
// the material is on the -(U×V) side.
func bevelPlane(t *testing.T) anasurf.Plane {
	t.Helper()
	c := 0.7071067811865476
	f, err := r3.NewFrame(r3.NewVec(0, 0, 18), r3.NewVec(1, 0, 0), r3.NewVec(0, c, c))
	require.NoError(t, err)
	require.Equal(t, f.V().Y, f.V().Z)
	p, err := anasurf.PlaneOf(surfacegeom.Plane{Frame: f}, false)
	require.NoError(t, err)
	return p
}

func TestMeet3BevelContainsIrrational(t *testing.T) {
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	one := big.NewRat(1, 1)
	bevel, err := bevelPlane(t).Inward(one)
	require.NoError(t, err)
	px, err := axisPlane(t, y, z).Inward(one)
	require.NoError(t, err)
	py, err := axisPlane(t, z, x).Inward(one)
	require.NoError(t, err)
	pt, err := anasurf.Meet3(px, py, bevel)
	require.NoError(t, err)

	// The exact point is (1, 1, 19 - sqrt 2). x and y are exact; z is checked against sqrt 2 in rationals.
	require.True(t, pt.Box[0].Lo.Cmp(one) <= 0 && pt.Box[0].Hi.Cmp(one) >= 0)
	require.True(t, pt.Box[1].Lo.Cmp(one) <= 0 && pt.Box[1].Hi.Cmp(one) >= 0)
	zb := pt.Box[2]
	// 19 - sqrt2 >= Lo  <=>  (19 - Lo)^2 >= 2.   19 - sqrt2 <= Hi  <=>  (19 - Hi)^2 <= 2.
	nineteen := big.NewRat(19, 1)
	two := big.NewRat(2, 1)
	a := new(big.Rat).Sub(nineteen, zb.Lo)
	b := new(big.Rat).Sub(nineteen, zb.Hi)
	require.True(t, a.Mul(a, a).Cmp(two) >= 0, "the enclosure's low end is above 19 - sqrt 2")
	require.True(t, b.Mul(b, b).Cmp(two) <= 0, "the enclosure's high end is below 19 - sqrt 2")

	held, bound := pt.Held()
	require.InDelta(t, 19-1.4142135623730951, held.Z, 1e-12)
	require.Greater(t, bound, 0.0)
	require.Less(t, bound, 1e-12)
}

func TestMeet3Singular(t *testing.T) {
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	one := big.NewRat(1, 1)
	// Three planes through the z axis: normals x, y and x+y span only a plane.
	px, err := axisPlane(t, y, z).Inward(one)
	require.NoError(t, err)
	py, err := axisPlane(t, z, x).Inward(one)
	require.NoError(t, err)
	pd, err := anasurf.PlaneOf(frame(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), r3.NewVec(-1, 1, 0)), false)
	require.NoError(t, err)
	_, err = anasurf.Meet3(px, py, pd)
	require.ErrorIs(t, err, anasurf.ErrSingular)
}

func TestStrictlyInsideTouchingBox(t *testing.T) {
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	// Material on z < 5: outward normal +z through (0,0,5).
	p, err := anasurf.PlaneOf(frame(t, r3.NewVec(0, 0, 5), x, y), false)
	require.NoError(t, err)
	_ = z
	box := func(zlo, zhi int64) anasurf.Point {
		iv := proofbound.Interval(big.NewRat(zlo, 1), big.NewRat(zhi, 1))
		zero := proofbound.PointInterval(new(big.Rat))
		return anasurf.Point{Box: proofbound.IvVec3{zero, zero, iv}}
	}
	require.True(t, anasurf.StrictlyInside(box(0, 4), p))
	require.False(t, anasurf.StrictlyInside(box(0, 5), p), "a box touching the plane is not strictly inside")
	require.False(t, anasurf.StrictlyInside(box(0, 6), p))
}

func TestInwardRejectsNonPositive(t *testing.T) {
	_, err := bevelPlane(t).Inward(new(big.Rat))
	require.Error(t, err)
}
