package occupancy_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/occupancy"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// placed maps local cylinder coordinates by a rotation about a skew axis and a
// translation far from the origin, so every coordinate the tests read is
// rounded. image is the same map in floats, for building held points.
type placed struct {
	chain occupancy.Chain
	m     [3][3]float64
	t     r3.Vec
}

func newPlaced(axis r3.Vec, angle float64, t r3.Vec) placed {
	k, _ := axis.Normalize()
	c, s := math.Cos(angle), math.Sin(angle)
	m := [3][3]float64{
		{c + k.X*k.X*(1-c), k.X*k.Y*(1-c) - k.Z*s, k.X*k.Z*(1-c) + k.Y*s},
		{k.Y*k.X*(1-c) + k.Z*s, c + k.Y*k.Y*(1-c), k.Y*k.Z*(1-c) - k.X*s},
		{k.Z*k.X*(1-c) - k.Y*s, k.Z*k.Y*(1-c) + k.X*s, c + k.Z*k.Z*(1-c)},
	}
	return placed{occupancy.Chain{{M: m, T: [3]float64{t.X, t.Y, t.Z}}}, m, t}
}

func (p placed) image(v r3.Vec) r3.Vec {
	return r3.Vec{
		X: p.m[0][0]*v.X + p.m[0][1]*v.Y + p.m[0][2]*v.Z + p.t.X,
		Y: p.m[1][0]*v.X + p.m[1][1]*v.Y + p.m[1][2]*v.Z + p.t.Y,
		Z: p.m[2][0]*v.X + p.m[2][1]*v.Y + p.m[2][2]*v.Z + p.t.Z,
	}
}

func (p placed) direction(v r3.Vec) r3.Vec { return p.image(v).Sub(p.t) }

func bigBudget() *occupancy.Budget { return &occupancy.Budget{Limit: 1 << 40} }

func TestCirclePrismContainsCylinder(t *testing.T) {
	t.Parallel()
	const r, sides = 0.35, 64
	pl := newPlaced(r3.Vec{X: 1, Y: 2, Z: 3}, 1.1, r3.Vec{X: -140.25, Y: 270.5, Z: 690.125})
	piece, err := occupancy.CirclePrism(pl.chain, r, 0, 22, sides)
	require.NoError(t, err)

	circumradius := r / math.Cos(math.Pi/sides)
	for k := range 97 {
		th := 2 * math.Pi * float64(k) / 97
		for _, z := range []float64{1e-6, 7.5, 22 - 1e-6} {
			// A point a hair inside the true circle stays inside the piece:
			// no axis may separate it, at any shift of zero.
			in := pl.image(r3.Vec{X: r * (1 - 1e-9) * math.Cos(th), Y: r * (1 - 1e-9) * math.Sin(th), Z: z})
			ok, err := occupancy.Misses([3]r3.Vec{in, in, in}, r3.Vec{}, piece, bigBudget())
			require.NoError(t, err)
			require.False(t, ok, "true cylinder point at angle %d, level %v escaped the piece", k, z)
		}
		// Past the polygon's circumradius every point is outside the piece.
		out := pl.image(r3.Vec{X: circumradius * 1.001 * math.Cos(th), Y: circumradius * 1.001 * math.Sin(th), Z: 11})
		ok, err := occupancy.Misses([3]r3.Vec{out, out, out}, r3.Vec{}, piece, bigBudget())
		require.NoError(t, err)
		require.True(t, ok, "point outside the circumscribed polygon at angle %d was not separated", k)
	}
	// Beyond either cap.
	for _, z := range []float64{-0.001, 22.001} {
		p := pl.image(r3.Vec{Z: z})
		ok, err := occupancy.Misses([3]r3.Vec{p, p, p}, r3.Vec{}, piece, bigBudget())
		require.NoError(t, err)
		require.True(t, ok, "point beyond the cap at %v was not separated", z)
	}
}

func TestFacetBoundChordSagitta(t *testing.T) {
	t.Parallel()
	// A held wall facet on one chord of a 16-gon inscribed in the true circle.
	// Its edge midline sits one sagitta inside the true cylinder, so no
	// translation shorter than that sagitta can leave the cylinder from there.
	const r, chords, length = 0.35, 16, 30.0
	pl := newPlaced(r3.Vec{X: -2, Y: 1, Z: 0.5}, 0.7, r3.Vec{X: 12.5, Y: -40.75, Z: 3})
	piece, err := occupancy.CirclePrism(pl.chain, r, 0, length, 256)
	require.NoError(t, err)

	a0 := r3.Vec{X: r, Y: 0}
	a1 := r3.Vec{X: r * math.Cos(2*math.Pi/chords), Y: r * math.Sin(2*math.Pi/chords)}
	tri := [3]r3.Vec{
		pl.image(r3.Vec{X: a0.X, Y: a0.Y, Z: 2}),
		pl.image(r3.Vec{X: a1.X, Y: a1.Y, Z: 2}),
		pl.image(r3.Vec{X: a1.X, Y: a1.Y, Z: length - 2}),
	}
	mid := a0.Add(a1).Scale(0.5)
	outward := pl.direction(mid)
	sagitta := r - mid.Len()
	slack := r * (1/math.Cos(math.Pi/256) - 1)

	h, err := occupancy.FacetBound(tri, outward, []*occupancy.Piece{piece},
		occupancy.Search{H0: 1e-4, HMax: 0.05, MaxDepth: 4}, bigBudget())
	require.NoError(t, err)
	// tri's edge from a0 to a1 passes through the chord midpoint's line, so a
	// translation must cover at least the sagitta there.
	require.GreaterOrEqual(t, h, sagitta*0.999)
	require.LessOrEqual(t, h, 1.25*(sagitta+slack)+1e-12)
}

func TestFacetBoundEscapesTwoLeaves(t *testing.T) {
	t.Parallel()
	// Two leaves overlap at a right-angle joint. A facet on the first leaf's
	// held wall that runs into the second leaf's true solid must clear both,
	// so its bound reaches the depth it lies inside the second leaf.
	const r = 0.35
	id := newPlaced(r3.Vec{Z: 1}, 0, r3.Vec{})
	turned := newPlaced(r3.Vec{Y: 1}, math.Pi/2, r3.Vec{})
	a, err := occupancy.CirclePrism(id.chain, r, -20, 0, 256)
	require.NoError(t, err)
	b, err := occupancy.CirclePrism(turned.chain, r, 0, 20, 256)
	require.NoError(t, err)

	// A chord facet of a near the top (x = 0 side, +y), inside b near b's
	// start cap. b's axis runs along +x and its wall bounds |(y, z)| ≤ r.
	th0, th1 := math.Pi/2-0.05, math.Pi/2+0.05
	p0 := r3.Vec{X: r * math.Cos(th0), Y: r * math.Sin(th0)}
	p1 := r3.Vec{X: r * math.Cos(th1), Y: r * math.Sin(th1)}
	tri := [3]r3.Vec{
		{X: p0.X, Y: p0.Y, Z: -0.01},
		{X: p1.X, Y: p1.Y, Z: -0.01},
		{X: p1.X, Y: p1.Y, Z: -0.002},
	}
	outward := r3.Vec{Y: 1}
	h, err := occupancy.FacetBound(tri, outward, []*occupancy.Piece{a, b},
		occupancy.Search{H0: 1e-4, HMax: 0.05, MaxDepth: 4}, bigBudget())
	require.NoError(t, err)
	// The facet sits at |(y, z)| below r inside b, and one sagitta inside a.
	// Leaving both needs at least the larger of the two depths.
	far := 0.0
	for _, v := range tri {
		far = math.Max(far, math.Hypot(v.Y, v.Z))
	}
	sagitta := r - p0.Y
	require.Greater(t, h, math.Max(r-far, sagitta)-1e-9)
	require.Less(t, h, 0.05)
}

func TestFacetBoundRefusals(t *testing.T) {
	t.Parallel()
	const r = 0.35
	id := newPlaced(r3.Vec{Z: 1}, 0, r3.Vec{})
	piece, err := occupancy.CirclePrism(id.chain, r, 0, 10, 64)
	require.NoError(t, err)
	// A facet deep inside the cylinder cannot leave it within HMax.
	deep := [3]r3.Vec{{X: 0.01, Z: 5}, {X: 0.02, Z: 5}, {X: 0.01, Y: 0.01, Z: 5}}
	_, err = occupancy.FacetBound(deep, r3.Vec{Z: 1}, []*occupancy.Piece{piece},
		occupancy.Search{H0: 1e-4, HMax: 0.01, MaxDepth: 1}, bigBudget())
	require.ErrorIs(t, err, decaderr.ErrUnsupported)

	// The work ceiling refuses rather than running on.
	wall := [3]r3.Vec{{X: r, Z: 1}, {X: r * math.Cos(0.1), Y: r * math.Sin(0.1), Z: 1}, {X: r, Z: 9}}
	_, err = occupancy.FacetBound(wall, r3.Vec{X: 1, Y: 0.05}, []*occupancy.Piece{piece},
		occupancy.Search{H0: 1e-6, HMax: 0.05, MaxDepth: 2}, &occupancy.Budget{Limit: 10})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)

	for _, bad := range []struct {
		r, z0, z1 float64
		sides     int
	}{{0, 0, 1, 8}, {r, 1, 1, 8}, {r, 0, 1, 2}, {math.NaN(), 0, 1, 8}, {r, 0, math.Inf(1), 8}} {
		_, err := occupancy.CirclePrism(id.chain, bad.r, bad.z0, bad.z1, bad.sides)
		require.ErrorIs(t, err, occupancy.ErrInvalid)
	}
}
