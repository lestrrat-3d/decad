package decad_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/multibody-dynamics-design.md §13 PR 14's public half:
// a stitched tetrahedron against its closed form, a loft between two exact
// octagons against independent integrals, and a cup against its closed form.
// The tolerance ladder's own steps, on revolves read off their meshes, are
// asserted in mass_properties_mesh_internal_test.go.

// meshMassDensity is dyadic, so ρ enters every expected value exactly.
var meshMassDensity = units.KilogramsPerCubicMillimeter(1.0 / 1024)

// meshMassRho is meshMassDensity as an exact rational in kg/mm³.
func meshMassRho() *big.Rat { return new(big.Rat).SetFloat64(meshMassDensity.Base()) }

// stitchTrianglePatch draws the triangle with the given plane-local corners
// on plane and patches it.
func stitchTrianglePatch(t *testing.T, doc *decad.Document, w *sketch.World, plane *sketch.Plane, local [3][2]float64) *decad.Body {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	var corners [3]*sketch.Point
	for i, p := range local {
		corners[i] = s.CreatePoint(p[0], p[1])
		s.Fix(corners[i])
	}
	for i := range corners {
		s.CreateLine(corners[i], corners[(i+1)%3])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	patch, err := doc.Patch(t.Context(), s, s.Profiles()[0])
	require.NoError(t, err)
	return patch
}

// stitchedTetrahedron stitches the tetrahedron with corners a·(1,0,0),
// (0,0,0), a·(0,1,0) and a·(1,0,1), returning it and a. Two faces lie on the
// XY and XZ datum planes. The other two lie on the planes x+y = a and z = x,
// whose frames each hold one cardinal axis and one held diagonal (±s, ±s)
// with s = 1/√2 rounded. a = 8s, and the two diagonal corners sit at
// plane-local coordinate 8 along that diagonal, so every lift product is a
// power-of-two scaling of a held float and every sum cancels or adds zero:
// each corner lands exactly, with or without fused multiply-add.
func stitchedTetrahedron(t *testing.T, doc *decad.Document) (*decad.Body, float64) {
	t.Helper()
	w := sketch.NewWorld()
	slanted, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 0, 1), r3.NewVec(-1, 1, 0))
	require.NoError(t, err)
	s := slanted.V().Y
	require.Equal(t, r3.NewVec(-s, s, 0), slanted.V(), "premise: the diagonal axis is held symmetric")
	a := 8 * s
	slanted, err = r3.NewFrame(r3.NewVec(a, 0, 0), r3.NewVec(0, 0, 1), r3.NewVec(-1, 1, 0))
	require.NoError(t, err)
	diagonal, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(1, 0, 1))
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(s, 0, s), diagonal.V(), "premise: both diagonals hold the same s")
	slantedPlane, err := w.CreatePlaneFromFrame(slanted)
	require.NoError(t, err)
	diagonalPlane, err := w.CreatePlaneFromFrame(diagonal)
	require.NoError(t, err)

	cardinal := func(plane *sketch.Plane, corners ...r3.Vec) [3][2]float64 {
		frame, err := plane.Frame()
		require.NoError(t, err)
		var local [3][2]float64
		for i, c := range corners {
			d := c.Sub(frame.Origin())
			local[i] = [2]float64{d.Dot(frame.U()), d.Dot(frame.V())}
		}
		return local
	}
	v0, v1, v2, v3 := r3.NewVec(a, 0, 0), r3.Vec{}, r3.NewVec(0, a, 0), r3.NewVec(a, 0, a)
	faces := []*decad.Body{
		stitchTrianglePatch(t, doc, w, w.XY(), cardinal(w.XY(), v0, v1, v2)),
		stitchTrianglePatch(t, doc, w, w.XZ(), cardinal(w.XZ(), v0, v1, v3)),
		stitchTrianglePatch(t, doc, w, slantedPlane, [3][2]float64{{0, 0}, {0, 8}, {a, 0}}),
		stitchTrianglePatch(t, doc, w, diagonalPlane, [3][2]float64{{0, 0}, {a, 0}, {0, 8}}),
	}
	want := map[r3.Vec]struct{}{v0: {}, v1: {}, v2: {}, v3: {}}
	for _, face := range faces {
		for _, v := range face.Vertices() {
			require.Contains(t, want, v.Position().Value, "premise: every patch corner lands exactly")
		}
	}
	tetrahedron, err := decad.Stitch(t.Context(), faces...)
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, tetrahedron.Kind())
	return tetrahedron, a
}

// TestMassPropertiesStitchedTetrahedron checks the stitched tetrahedron's
// mass, center and every tensor component against the closed form: for
// corners a·p_k, V = a³/6, C = a(1/2, 1/4, 1/4), and the second moment about
// C is V·a²·[(Σ p pᵀ + s sᵀ)/20 − c cᵀ] with s = Σ p_k, c = s/4. All three
// products of inertia are nonzero.
func TestMassPropertiesStitchedTetrahedron(t *testing.T) {
	doc := decad.New()
	tetrahedron, a := stitchedTetrahedron(t, doc)
	got, err := tetrahedron.MassProperties(t.Context(), meshMassDensity)
	require.NoError(t, err)

	side := new(big.Rat).SetFloat64(a)
	cube := new(big.Rat).Mul(side, new(big.Rat).Mul(side, side))
	mass := new(big.Rat).Mul(meshMassRho(), new(big.Rat).Quo(cube, big.NewRat(6, 1)))
	requireReadingCovers(t, got.Mass, mass)
	scale := new(big.Rat).Mul(mass, new(big.Rat).Mul(side, side))
	for _, component := range []struct {
		reading decad.Measurement
		factor  *big.Rat
	}{
		{got.Inertia.XX, big.NewRat(3, 40)},
		{got.Inertia.YY, big.NewRat(7, 80)},
		{got.Inertia.ZZ, big.NewRat(7, 80)},
		{got.Inertia.XY, big.NewRat(1, 40)},
		{got.Inertia.XZ, big.NewRat(-1, 40)},
		{got.Inertia.YZ, big.NewRat(1, 80)},
	} {
		requireReadingCovers(t, component.reading, new(big.Rat).Mul(scale, component.factor))
	}
	for i, fraction := range []*big.Rat{big.NewRat(1, 2), big.NewRat(1, 4), big.NewRat(1, 4)} {
		held := []float64{got.Center.Value.X, got.Center.Value.Y, got.Center.Value.Z}[i]
		deviation := new(big.Rat).Sub(new(big.Rat).Mul(side, fraction), new(big.Rat).SetFloat64(held))
		require.LessOrEqual(t, deviation.Abs(deviation).Cmp(new(big.Rat).SetFloat64(got.Center.Bound.Base())), 0)
	}
	volume, err := tetrahedron.Volume()
	require.NoError(t, err)
	requireReadingCovers(t, volume, new(big.Rat).Quo(cube, big.NewRat(6, 1)))

	// A placed stitched solid's mesh carries no occupied-volume proof
	// (docs/tessellation-design.md §2's stitchPayload row), so its mass is
	// refused rather than read off a mesh that proves only signed volume.
	shift, err := r3.Translation(r3.NewVec(.1, 0, 0))
	require.NoError(t, err)
	placed, err := tetrahedron.Placed(t.Context(), shift)
	require.NoError(t, err)
	reading, err := placed.MassProperties(t.Context(), meshMassDensity)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, decad.MassProperties{}, reading)
}

// meshPolygonSketch draws a closed polygon through fixed corners on plane.
func meshPolygonSketch(t *testing.T, w *sketch.World, plane *sketch.Plane, corners [][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = s.CreatePoint(c[0], c[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

// TestMassPropertiesLoftOctagons lofts the 8×8 square whose edge midpoints
// are pushed out by 1/4 to a chamfered-square octagon shifted off the axis by
// (1, 1/2). The cap triangulator refuses collinear corners, and a loft pairs
// equal segment counts, so the square side carries its midpoints this way.
// Every corner is dyadic and every wall cell is a twisted quad, so the held
// triangles are the loft itself. Mass and center are checked against the
// loft's own Volume and Centroid; the tensor against the divergence theorem
// over the mesh, ∫x_i x_j dV = ∮ F n_i dA with F = x_i³/3 or x_i² x_j/2,
// integrated per triangle by the cubic-exact rule (vertices 1/20, edge
// midpoints 2/15, centroid 9/20) — a formula independent of the tetrahedron
// sums the reading uses.
func TestMassPropertiesLoftOctagons(t *testing.T) {
	doc := decad.New()
	w := sketch.NewWorld()
	top, err := w.CreateOffsetPlane(w.XY(), 8)
	require.NoError(t, err)
	s0, p0 := meshPolygonSketch(t, w, w.XY(), [][2]float64{
		{4.25, 0}, {4, 4}, {0, 4.25}, {-4, 4}, {-4.25, 0}, {-4, -4}, {0, -4.25}, {4, -4},
	})
	s1, p1 := meshPolygonSketch(t, w, top, [][2]float64{
		{5, -1.5}, {5, 2.5}, {3, 4.5}, {-1, 4.5}, {-3, 2.5}, {-3, -1.5}, {-1, -3.5}, {3, -3.5},
	})
	loft, err := doc.Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)
	got, err := loft.MassProperties(t.Context(), meshMassDensity)
	require.NoError(t, err)

	volume, err := loft.Volume()
	require.NoError(t, err)
	centroid, err := loft.Centroid()
	require.NoError(t, err)
	mesh, err := loft.Tessellate(t.Context(), units.Millimeters(1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())

	exactVolume, first, second := divergenceMoments(mesh)
	// The loft publishes its own volume within one rounding: the exact
	// divergence integral must lie inside it, and the mass must cover ρ times
	// that same exact volume.
	requireReadingCovers(t, volume, exactVolume)
	rho := meshMassRho()
	requireReadingCovers(t, got.Mass, new(big.Rat).Mul(rho, exactVolume))
	center := [3]*big.Rat{}
	for i := range center {
		center[i] = new(big.Rat).Quo(first[i], exactVolume)
		for _, held := range []struct{ value, bound float64 }{
			{[]float64{got.Center.Value.X, got.Center.Value.Y, got.Center.Value.Z}[i], got.Center.Bound.Base()},
			{[]float64{centroid.Value.X, centroid.Value.Y, centroid.Value.Z}[i], centroid.Bound.Base()},
		} {
			deviation := new(big.Rat).Sub(center[i], new(big.Rat).SetFloat64(held.value))
			require.LessOrEqual(t, deviation.Abs(deviation).Cmp(new(big.Rat).SetFloat64(held.bound)), 0)
		}
	}
	var central [3][3]*big.Rat
	for i := range 3 {
		for j := range 3 {
			shift := new(big.Rat).Mul(first[i], first[j])
			central[i][j] = new(big.Rat).Sub(second[i][j], shift.Quo(shift, exactVolume))
		}
	}
	trace := new(big.Rat).Add(new(big.Rat).Add(central[0][0], central[1][1]), central[2][2])
	inertia := func(i, j int) *big.Rat {
		if i == j {
			return new(big.Rat).Mul(rho, new(big.Rat).Sub(trace, central[i][i]))
		}
		return new(big.Rat).Mul(rho, new(big.Rat).Neg(central[i][j]))
	}
	requireReadingCovers(t, got.Inertia.XX, inertia(0, 0))
	requireReadingCovers(t, got.Inertia.YY, inertia(1, 1))
	requireReadingCovers(t, got.Inertia.ZZ, inertia(2, 2))
	requireReadingCovers(t, got.Inertia.XY, inertia(0, 1))
	requireReadingCovers(t, got.Inertia.XZ, inertia(0, 2))
	requireReadingCovers(t, got.Inertia.YZ, inertia(1, 2))
	require.NotZero(t, inertia(0, 1).Sign(), "premise: the shifted top section makes the products nonzero")
	require.NotZero(t, inertia(0, 2).Sign())
	require.NotZero(t, inertia(1, 2).Sign())
}

// divergenceMoments integrates V, ∫x dV and ∫x_i x_j dV over a closed,
// outward mesh by the divergence theorem, exactly.
func divergenceMoments(mesh *decad.Mesh) (*big.Rat, [3]*big.Rat, [3][3]*big.Rat) {
	vertices := mesh.Vertices()
	volume := new(big.Rat)
	var first [3]*big.Rat
	var second [3][3]*big.Rat
	for i := range 3 {
		first[i] = new(big.Rat)
		for j := range 3 {
			second[i][j] = new(big.Rat)
		}
	}
	lift := func(v r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
	}
	combine := func(weights []int64, denominator int64, points ...[3]*big.Rat) [3]*big.Rat {
		var out [3]*big.Rat
		for axis := range out {
			out[axis] = new(big.Rat)
			for k, p := range points {
				out[axis].Add(out[axis], new(big.Rat).Mul(big.NewRat(weights[k], denominator), p[axis]))
			}
		}
		return out
	}
	for _, tri := range mesh.Triangles() {
		a, b, c := lift(vertices[tri[0]]), lift(vertices[tri[1]]), lift(vertices[tri[2]])
		var u, v [3]*big.Rat
		for axis := range 3 {
			u[axis] = new(big.Rat).Sub(b[axis], a[axis])
			v[axis] = new(big.Rat).Sub(c[axis], a[axis])
		}
		// half is n·dA integrated over the triangle: (u × v)/2.
		half := [3]*big.Rat{
			new(big.Rat).Sub(new(big.Rat).Mul(u[1], v[2]), new(big.Rat).Mul(u[2], v[1])),
			new(big.Rat).Sub(new(big.Rat).Mul(u[2], v[0]), new(big.Rat).Mul(u[0], v[2])),
			new(big.Rat).Sub(new(big.Rat).Mul(u[0], v[1]), new(big.Rat).Mul(u[1], v[0])),
		}
		for axis := range half {
			half[axis].Quo(half[axis], big.NewRat(2, 1))
		}
		type weighted struct {
			point  [3]*big.Rat
			weight *big.Rat
		}
		samples := []weighted{
			{a, big.NewRat(1, 20)}, {b, big.NewRat(1, 20)}, {c, big.NewRat(1, 20)},
			{combine([]int64{1, 1}, 2, a, b), big.NewRat(2, 15)},
			{combine([]int64{1, 1}, 2, b, c), big.NewRat(2, 15)},
			{combine([]int64{1, 1}, 2, c, a), big.NewRat(2, 15)},
			{combine([]int64{1, 1, 1}, 3, a, b, c), big.NewRat(9, 20)},
		}
		average := func(f func(p [3]*big.Rat) *big.Rat) *big.Rat {
			sum := new(big.Rat)
			for _, s := range samples {
				sum.Add(sum, new(big.Rat).Mul(s.weight, f(s.point)))
			}
			return sum
		}
		volume.Add(volume, new(big.Rat).Mul(half[0], average(func(p [3]*big.Rat) *big.Rat {
			return new(big.Rat).Set(p[0])
		})))
		for i := range 3 {
			first[i].Add(first[i], new(big.Rat).Mul(half[i], average(func(p [3]*big.Rat) *big.Rat {
				return new(big.Rat).Quo(new(big.Rat).Mul(p[i], p[i]), big.NewRat(2, 1))
			})))
			for j := range 3 {
				second[i][j].Add(second[i][j], new(big.Rat).Mul(half[i], average(func(p [3]*big.Rat) *big.Rat {
					if i == j {
						return new(big.Rat).Quo(new(big.Rat).Mul(p[i], new(big.Rat).Mul(p[i], p[i])), big.NewRat(3, 1))
					}
					return new(big.Rat).Quo(new(big.Rat).Mul(new(big.Rat).Mul(p[i], p[i]), p[j]), big.NewRat(2, 1))
				})))
			}
		}
	}
	return volume, first, second
}

// TestMassPropertiesCupFallback reads a shelled box — the 100×60×20 plate
// with a 5 mm wall and its top open — off its verified mesh and checks it
// against the outer box minus the cavity box, integrated exactly.
func TestMassPropertiesCupFallback(t *testing.T) {
	_, box := shellBox(t)
	cup, err := box.Shell(t.Context(), topCap(box), units.Millimeters(5))
	require.NoError(t, err)
	got, err := cup.MassProperties(t.Context(), meshMassDensity)
	require.NoError(t, err)

	outerV, outerP, outerQ := boxMoments(0, 100, 0, 60, 0, 20)
	innerV, innerP, innerQ := boxMoments(5, 95, 5, 55, 5, 20)
	volume := new(big.Rat).Sub(outerV, innerV)
	rho := meshMassRho()
	requireReadingCovers(t, got.Mass, new(big.Rat).Mul(rho, volume))
	var center [3]*big.Rat
	for i := range center {
		center[i] = new(big.Rat).Quo(new(big.Rat).Sub(outerP[i], innerP[i]), volume)
		held := []float64{got.Center.Value.X, got.Center.Value.Y, got.Center.Value.Z}[i]
		deviation := new(big.Rat).Sub(center[i], new(big.Rat).SetFloat64(held))
		require.LessOrEqual(t, deviation.Abs(deviation).Cmp(new(big.Rat).SetFloat64(got.Center.Bound.Base())), 0)
	}
	var central [3][3]*big.Rat
	for i := range 3 {
		for j := range 3 {
			second := new(big.Rat).Sub(outerQ[i][j], innerQ[i][j])
			central[i][j] = second.Sub(second, new(big.Rat).Mul(volume, new(big.Rat).Mul(center[i], center[j])))
		}
	}
	trace := new(big.Rat).Add(new(big.Rat).Add(central[0][0], central[1][1]), central[2][2])
	for i, reading := range []decad.Measurement{got.Inertia.XX, got.Inertia.YY, got.Inertia.ZZ} {
		requireReadingCovers(t, reading, new(big.Rat).Mul(rho, new(big.Rat).Sub(trace, central[i][i])))
	}
	for _, mixed := range []decad.Measurement{got.Inertia.XY, got.Inertia.XZ, got.Inertia.YZ} {
		requireReadingCovers(t, mixed, new(big.Rat))
	}
}

// boxMoments is V, ∫x dV and ∫x_i x_j dV of an axis-aligned box about the
// origin.
func boxMoments(x0, x1, y0, y1, z0, z1 int64) (*big.Rat, [3]*big.Rat, [3][3]*big.Rat) {
	lo, hi := [3]int64{x0, y0, z0}, [3]int64{x1, y1, z1}
	// Per axis: length, ∫x and ∫x² over [lo, hi].
	var length, linear, square [3]*big.Rat
	for i := range 3 {
		a, b := big.NewRat(lo[i], 1), big.NewRat(hi[i], 1)
		length[i] = new(big.Rat).Sub(b, a)
		linear[i] = new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(b, b), new(big.Rat).Mul(a, a)), big.NewRat(2, 1))
		cube := func(v *big.Rat) *big.Rat { return new(big.Rat).Mul(v, new(big.Rat).Mul(v, v)) }
		square[i] = new(big.Rat).Quo(new(big.Rat).Sub(cube(b), cube(a)), big.NewRat(3, 1))
	}
	volume := new(big.Rat).Mul(length[0], new(big.Rat).Mul(length[1], length[2]))
	var first [3]*big.Rat
	var second [3][3]*big.Rat
	for i := range 3 {
		first[i] = new(big.Rat).Quo(new(big.Rat).Mul(volume, linear[i]), length[i])
		for j := range 3 {
			if i == j {
				second[i][j] = new(big.Rat).Quo(new(big.Rat).Mul(volume, square[i]), length[i])
				continue
			}
			pair := new(big.Rat).Mul(linear[i], linear[j])
			second[i][j] = pair.Quo(new(big.Rat).Mul(pair, volume), new(big.Rat).Mul(length[i], length[j]))
		}
	}
	return volume, first, second
}
