package loftface_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// xPlane is phi(P) = P.x with loft side +1.
func xPlane() loftface.TangentPlane {
	return loftface.TangentPlane{N: ratVecXY(1, 0), D: rat(0, 1), Side: 1}
}

// zUpPlane is the plane z = height with the loft below it, so Side = -1 and
// psi = height - z is positive on the cone's side (below). The prism points up.
func zUpPlane(height float64) loftface.TangentPlane {
	return loftface.TangentPlane{N: loftface.Vec(r3.NewVec(0, 0, 1)), D: loftface.Vec(r3.NewVec(height, 0, 0))[0], Side: -1}
}

// slabOf extends a cube whose cap is at z = 1 and returns the slab.
func slabOf(t *testing.T, mutate func(*loftface.HeldMesh)) loftface.Slab {
	t.Helper()
	held := cubeMesh(0, 0)
	if mutate != nil {
		mutate(&held)
	}
	ext, err := loftface.CapExtension(t.Context(), held, faceCap, zUpPlane(1), rat(1, 1),
		cubeLink([4]float64{0.01, 0.01, 0.01, 0.01}))
	require.NoError(t, err)
	return ext.Slab
}

func flatTri(x0, y0, x1, y1, z float64) ([]r3.Vec, [][3]int) {
	return []r3.Vec{r3.NewVec(x0, y0, z), r3.NewVec(x1, y0, z), r3.NewVec(x0, y1, z)}, [][3]int{{0, 1, 2}}
}

func clears(t *testing.T, slab loftface.Slab, verts []r3.Vec, tris [][3]int) error {
	t.Helper()
	return loftface.ExtensionClearsCone(slab, verts, tris, zUpPlane(1), proofbound.NewWorkBudget(t.Context()))
}

func TestExtensionClearsConeRefusesNormalCapDisplacement(t *testing.T) {
	const eps = 0x1p-40
	level := slabOf(t, nil)
	require.Zero(t, level.Eps.Sign(), "a planar cap has eps_S = 0")

	// Moving one cap vertex toward the cone (down, along -N) raises eps_S to 2^-40.
	toward := slabOf(t, func(m *loftface.HeldMesh) { m.Vertices[4].Z = 1 - eps })
	require.Zero(t, toward.Eps.Cmp(loftface.Vec(r3.NewVec(eps, 0, 0))[0]))
	away := slabOf(t, func(m *loftface.HeldMesh) { m.Vertices[4].Z = 1 + eps })
	require.Zero(t, away.Eps.Sign())

	// A cone facet 2^-41 below the plane is clear of the unmoved slab, because
	// it lies outside B_S, but inside the slab once a cap vertex moved toward it.
	verts, tris := flatTri(0, 0, 1, 1, 1-eps/2)
	require.NoError(t, clears(t, level, verts, tris))
	require.NoError(t, clears(t, away, verts, tris), "the move away from the cone changes nothing")
	require.ErrorIs(t, clears(t, toward, verts, tris), decaderr.ErrUnsupported,
		"psi = 2^-41 is below eps_S = 2^-40 inside B_S")

	// A facet at exactly eps_S refuses, and so does one at exactly 0 on the planar cap.
	verts, tris = flatTri(0, 0, 1, 1, 1-eps)
	require.ErrorIs(t, clears(t, toward, verts, tris), decaderr.ErrUnsupported, "psi == eps_S")
	verts, tris = flatTri(0, 0, 1, 1, 1)
	require.ErrorIs(t, clears(t, level, verts, tris), decaderr.ErrUnsupported, "psi == eps_S == 0")
	verts, tris = flatTri(0, 0, 1, 1, 1-2*eps)
	require.NoError(t, clears(t, toward, verts, tris), "psi = 2^-39 > eps_S")

	// Condition 1: a cone vertex deeper than the extension cap (psi -1.5 against
	// the extension cap's -1) refuses even though the facet is far from B_S.
	verts, tris = flatTri(10, 10, 11, 11, 2.5)
	require.ErrorIs(t, clears(t, level, verts, tris), decaderr.ErrUnsupported)
	verts, tris = flatTri(10, 10, 11, 11, 1.5)
	require.NoError(t, clears(t, level, verts, tris), "psi = -0.5 is above the extension cap's psi = -1")
}

// creaseMesh is the cube with its bottom pulled in so the front wall (y = 0
// side) and the back wall meet the cap plane z = 1/4 at the given angles in
// degrees.
func creaseMesh(frontDeg, backDeg float64) loftface.HeldMesh {
	m := cubeMesh(0, 0)
	const height = 0.25
	front := height / math.Tan(frontDeg*math.Pi/180)
	back := height / math.Tan(backDeg*math.Pi/180)
	for i := range m.Vertices {
		v := &m.Vertices[i]
		if v.Z == 0 {
			v.Y = v.Y*(1-front-back) + front
			continue
		}
		v.Z = height
	}
	return m
}

func TestExtensionClearsConeAtCreaseAngles(t *testing.T) {
	plane := zUpPlane(0.25)
	for _, angles := range [][2]float64{{30, 75}, {75, 30}} {
		ext, err := loftface.CapExtension(t.Context(), creaseMesh(angles[0], angles[1]), faceCap, plane, rat(1, 1),
			cubeLink([4]float64{0.01, 0.01, 0.01, 0.01}))
		require.NoError(t, err)
		check := func(verts []r3.Vec, tris [][3]int) error {
			return loftface.ExtensionClearsCone(ext.Slab, verts, tris, plane, proofbound.NewWorkBudget(t.Context()))
		}
		// A facet that crosses the front extension wall (y = 0) just above the
		// cap plane, and one that crosses the back wall (y = 1).
		z := 0.25 + 0x1p-20
		front := []r3.Vec{r3.NewVec(0.25, -0.5, z), r3.NewVec(0.75, -0.5, z), r3.NewVec(0.5, 0.5, z)}
		back := []r3.Vec{r3.NewVec(0.25, 1.5, z), r3.NewVec(0.75, 1.5, z), r3.NewVec(0.5, 0.5, z)}
		require.ErrorIs(t, check(front, [][3]int{{0, 1, 2}}), decaderr.ErrUnsupported, "%v front", angles)
		require.ErrorIs(t, check(back, [][3]int{{0, 1, 2}}), decaderr.ErrUnsupported, "%v back", angles)
		// The same facets just below the plane stay above the slab.
		z = 0.25 - 0x1p-20
		front = []r3.Vec{r3.NewVec(0.25, -0.5, z), r3.NewVec(0.75, -0.5, z), r3.NewVec(0.5, 0.5, z)}
		back = []r3.Vec{r3.NewVec(0.25, 1.5, z), r3.NewVec(0.75, 1.5, z), r3.NewVec(0.5, 0.5, z)}
		require.NoError(t, check(front, [][3]int{{0, 1, 2}}), "%v front above the slab", angles)
		require.NoError(t, check(back, [][3]int{{0, 1, 2}}), "%v back above the slab", angles)
	}
}

func TestResultBeyondSlab(t *testing.T) {
	plane := zUpPlane(1)
	bound := rat(1, 4)
	inside := []r3.Vec{r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 0.5), r3.NewVec(0, 0, 1.25)}
	require.False(t, loftface.ResultBeyondSlab(inside, plane, bound), "psi = -1/4 is not beyond the bound")
	beyond := append(inside, r3.NewVec(0, 0, 1.25+0x1p-40))
	require.True(t, loftface.ResultBeyondSlab(beyond, plane, bound))
	require.True(t, loftface.ResultBeyondSlab([]r3.Vec{r3.NewVec(0, 0, math.NaN())}, plane, bound))
}

func TestClippedPlaneMarginMatchesVertexMinimum(t *testing.T) {
	box := [2]loftface.RatVec{{rat(0, 1), rat(0, 1), rat(0, 1)}, {rat(2, 1), rat(1, 2), rat(2, 1)}}
	tri := func(a, b, c [3]int64) [3]loftface.RatVec {
		v := func(p [3]int64) loftface.RatVec { return loftface.RatVec{rat(p[0], 1), rat(p[1], 1), rat(p[2], 1)} }
		return [3]loftface.RatVec{v(a), v(b), v(c)}
	}
	plane := xPlane()

	inside := tri([3]int64{1, 0, 1}, [3]int64{2, 0, 1}, [3]int64{1, 0, 2})
	require.Zero(t, loftface.ClippedPlaneMargin(inside, box, plane).Cmp(rat(1, 1)), "fully inside")

	// The vertex at x = -2 is outside the box; the clip raises the minimum to 0.
	partial := tri([3]int64{-2, 0, 1}, [3]int64{4, 0, 1}, [3]int64{1, 0, 2})
	require.Zero(t, loftface.ClippedPlaneMargin(partial, box, plane).Cmp(rat(0, 1)), "partly inside")

	require.Nil(t, loftface.ClippedPlaneMargin(tri([3]int64{5, 0, 0}, [3]int64{6, 0, 0}, [3]int64{5, 0, 1}), box, plane),
		"fully outside")

	// A triangle touching the box in one point keeps that point.
	touch := tri([3]int64{2, 1, 1}, [3]int64{3, 1, 1}, [3]int64{3, 2, 1})
	require.Nil(t, loftface.ClippedPlaneMargin(touch, box, plane), "the y range [0, 1/2] excludes it")
	point := tri([3]int64{2, 0, 1}, [3]int64{3, 0, 1}, [3]int64{3, 1, 1})
	require.Zero(t, loftface.ClippedPlaneMargin(point, box, plane).Cmp(rat(2, 1)), "a single touching vertex")

	// A triangle in the plane y = 0 that rises above the box through the edge
	// z = 2 meets the box in that edge only: a segment with two clip vertices.
	// The margin reads both ends, whichever side of the plane functional.
	segment := tri([3]int64{0, 0, 2}, [3]int64{2, 0, 2}, [3]int64{1, 0, 3})
	require.Zero(t, loftface.ClippedPlaneMargin(segment, box, plane).Cmp(rat(0, 1)), "segment: low end")
	require.Zero(t, loftface.ClippedPlaneMargin(segment, box,
		loftface.TangentPlane{N: ratVecXY(-1, 0), D: rat(0, 1), Side: 1}).Cmp(rat(-2, 1)), "segment: high end")

	// Clipping at a rational point: z = 1 plane, the triangle (0,0),(3,0),(0,3)
	// in (x, y) cut to x <= 2, y <= 1/2 has its largest x + y = 5/2 at (2, 1/2).
	cut := tri([3]int64{0, 0, 1}, [3]int64{3, 0, 1}, [3]int64{0, 3, 1})
	sum := loftface.TangentPlane{N: ratVecXY(-1, -1), D: rat(0, 1), Side: 1}
	require.Zero(t, loftface.ClippedPlaneMargin(cut, box, sum).Cmp(rat(-5, 2)), "rational clip vertex")
	// Side -1 reads the opposite extreme.
	reverse := loftface.TangentPlane{N: ratVecXY(1, 1), D: rat(0, 1), Side: -1}
	require.Zero(t, loftface.ClippedPlaneMargin(cut, box, reverse).Cmp(rat(-5, 2)))
	require.Zero(t, loftface.ClippedPlaneMargin(cut, box, xPlane()).Cmp(rat(0, 1)))
}
