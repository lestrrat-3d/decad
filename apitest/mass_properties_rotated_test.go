package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures check docs/multibody-dynamics-design.md §8.1 and §8.2: a
// prism under a non-cardinal frame or placement basis, or with a displaced
// level, publishes mass and all six world inertia components enclosing the
// independent closed-form box's exact image under the held basis product M
// read exactly, the map the prism's volume and vertices denote through. Side lengths, levels and the density are dyadic; the one
// non-dyadic length is the denoted level of the inch extrusion, which is the
// point of that fixture.
//
// Legs shown to fail (each deleted in internal/massmoment, the fixture
// watched go red, then restored):
//   - The affine image: on the rigid path (ρ·V and the polar factor's Q I Qᵀ
//     widened by the defect) TestMassPropertiesRotatedBox and
//     TestMassPropertiesTiltedFrameBox both miss the mass.
//   - The occupied-volume error E, and its V leg alone: with either zeroed,
//     TestMassPropertiesDisplacedLevelBox misses the denoted mass. The
//     remaining legs are recorded in mass_properties_rotated_internal_test.go,
//     whose displacements are large enough to separate them.

const rotatedDensity = 1.0 / 1024

func TestMassPropertiesRotatedBox(t *testing.T) {
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 20, 10, 30)
	density := units.KilogramsPerCubicMillimeter(rotatedDensity)
	pose := composedRotation(t, r3.NewVec(2, -3, 5), r3.NewVec(1, 1, 1), 30, 8)
	placed, err := box.Placed(t.Context(), pose)
	require.NoError(t, err)
	got, err := placed.MassProperties(t.Context(), density)
	require.NoError(t, err)

	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	requireRotatedBox(t, got, pose, frame, [3]float64{20, 10, 30})
	center := pose.Apply(r3.NewVec(10, 5, 15))
	// The evaluator's own centroid bound encloses the exact placed center;
	// 1e-12 covers this test's float evaluation of pose.Apply.
	slack := got.Center.Bound.Base() + 1e-12
	require.InDelta(t, center.X, got.Center.Value.X, slack)
	require.InDelta(t, center.Y, got.Center.Value.Y, slack)
	require.InDelta(t, center.Z, got.Center.Value.Z, slack)
	// The rotation mixes every pair of axes, so no product of inertia is zero.
	for _, mixed := range []decad.Measurement{got.Inertia.XY, got.Inertia.XZ, got.Inertia.YZ} {
		require.Greater(t, math.Abs(mixed.Value.Base()), 1.0)
	}
	again, err := placed.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Equal(t, got, again)
}

func TestMassPropertiesTiltedFrameBox(t *testing.T) {
	frame, err := r3.NewFrame(r3.NewVec(1, 2, 3), r3.NewVec(1, 1, 0), r3.NewVec(-1, 1, 1))
	require.NoError(t, err)
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 12, 4)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	plane2, err := s.Plane().Frame()
	require.NoError(t, err)
	// The prism records the plane's axes normalized once more, as Extrude does.
	held, err := r3.NewFrame(plane2.Origin(), plane2.U(), plane2.V())
	require.NoError(t, err)
	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(6), Dir: decad.Along})
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(rotatedDensity)
	got, err := body.MassProperties(t.Context(), density)
	require.NoError(t, err)
	requireRotatedBox(t, got, r3.Identity(), held, [3]float64{12, 4, 6})

	pose, err := r3.RotationAround(r3.NewVec(-1, 0, 4), r3.NewVec(0, 1, 2), units.Degrees(50))
	require.NoError(t, err)
	placed, err := body.PlacedCopy(t.Context(), pose)
	require.NoError(t, err)
	turned, err := placed.MassProperties(t.Context(), density)
	require.NoError(t, err)
	requireRotatedBox(t, turned, pose, held, [3]float64{12, 4, 6})
}

// TestMassPropertiesDisplacedLevelBox extrudes by a distance in inches, whose
// rescale to millimetres records a level that is not the exact denoted level
// (a positive z1Delta), and checks the reading encloses the denoted box.
func TestMassPropertiesDisplacedLevelBox(t *testing.T) {
	distance := units.New(1.25, units.Inch)
	denoted := new(big.Rat).Mul(new(big.Rat).SetFloat64(distance.Mag()),
		new(big.Rat).SetFloat64(distance.Unit().Factor()))
	_, representable := denoted.Float64()
	require.False(t, representable, "the denoted level must differ from every float")

	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 16, 8)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: distance, Dir: decad.Along})
	require.NoError(t, err)
	pose := composedRotation(t, r3.Vec{}, r3.NewVec(1, 1, 1), 30, 1)
	placed, err := body.Placed(t.Context(), pose)
	require.NoError(t, err)
	got, err := placed.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(rotatedDensity))
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	requireRotatedBoxExact(t, got, pose, frame, [3]*big.Rat{big.NewRat(16, 1), big.NewRat(8, 1), denoted})
}

// composedRotation rotates by degrees about axis through center in steps
// equal parts, composed with Then, so the held basis carries the rounding a
// chain of placements accumulates.
func composedRotation(t *testing.T, center, axis r3.Vec, degrees float64, steps int) r3.Transform {
	t.Helper()
	step, err := r3.RotationAround(center, axis, units.Degrees(degrees/float64(steps)))
	require.NoError(t, err)
	pose := step
	for range steps - 1 {
		pose, err = pose.Then(step)
		require.NoError(t, err)
	}
	return pose
}

func requireRotatedBox(t *testing.T, got decad.MassProperties, pose r3.Transform, frame r3.Frame, size [3]float64) {
	t.Helper()
	var exact [3]*big.Rat
	for i, length := range size {
		exact[i] = new(big.Rat).SetFloat64(length)
	}
	requireRotatedBoxExact(t, got, pose, frame, exact)
}

// requireRotatedBoxExact checks a box of the given frame-local (u, v, n) side
// lengths against its exact image under the held map M of pose and frame:
// mass ρ·|det M|·V, and each world inertia component against the image's
// (affineInertia). frame must be the frame the payload records.
func requireRotatedBoxExact(t *testing.T, got decad.MassProperties, pose r3.Transform, frame r3.Frame, size [3]*big.Rat) {
	t.Helper()
	rho := new(big.Rat).SetFloat64(rotatedDensity)
	mass := new(big.Rat).Mul(rho, new(big.Rat).Mul(size[0], new(big.Rat).Mul(size[1], size[2])))
	m := heldRotation(pose, frame)
	det := wideDet(m)
	det.Abs(det)
	requireWideCovers(t, got.Mass, newWide().Mul(newWide().SetRat(mass), det))
	var local [3][3]*big.Float
	for i := range local {
		a, b := size[(i+1)%3], size[(i+2)%3]
		sum := new(big.Rat).Add(new(big.Rat).Mul(a, a), new(big.Rat).Mul(b, b))
		for j := range local[i] {
			local[i][j] = newWide()
		}
		local[i][i] = newWide().SetRat(new(big.Rat).Quo(new(big.Rat).Mul(mass, sum), big.NewRat(12, 1)))
	}
	image := affineInertia(m, local, newWide().SetRat(rho))
	world := func(i, j int) *big.Float { return image[i][j] }
	for _, entry := range []struct {
		reading decad.Measurement
		i, j    int
	}{
		{got.Inertia.XX, 0, 0}, {got.Inertia.YY, 1, 1}, {got.Inertia.ZZ, 2, 2},
		{got.Inertia.XY, 0, 1}, {got.Inertia.XZ, 0, 2}, {got.Inertia.YZ, 1, 2},
	} {
		// The tensor entries already carry the products' physical sign.
		want := world(entry.i, entry.j)
		deviation := newWide().Sub(want, newWide().SetFloat64(entry.reading.Value.Base()))
		deviation.Abs(deviation)
		require.LessOrEqual(t, deviation.Cmp(newWide().SetFloat64(entry.reading.Bound.Base())), 0,
			"component (%d,%d) = %s ± %s misses %s", entry.i, entry.j,
			entry.reading.Value, entry.reading.Bound, want.Text('g', 20))
	}
}

// heldRotation is the exact product of the placement basis and the frame
// axes, both read from the held floats: column k is local axis k in world.
func heldRotation(pose r3.Transform, frame r3.Frame) [3][3]*big.Float {
	basis := pose.Basis()
	placement := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	axes := [3]r3.Vec{frame.U(), frame.V(), frame.N()}
	component := func(v r3.Vec, i int) float64 { return [3]float64{v.X, v.Y, v.Z}[i] }
	var out [3][3]*big.Float
	for i := range out {
		for k := range out[i] {
			sum := newWide()
			for l := range placement {
				term := newWide().SetFloat64(component(placement[l], i))
				sum.Add(sum, term.Mul(term, newWide().SetFloat64(component(axes[k], l))))
			}
			out[i][k] = sum
		}
	}
	return out
}

func newWide() *big.Float { return new(big.Float).SetPrec(512) }

// wideDet is the determinant of m.
func wideDet(m [3][3]*big.Float) *big.Float {
	minor := func(i, j, k, l int) *big.Float {
		a := newWide().Mul(m[i][k], m[j][l])
		return a.Sub(a, newWide().Mul(m[i][l], m[j][k]))
	}
	det := newWide().Mul(m[0][0], minor(1, 2, 1, 2))
	det.Sub(det, newWide().Mul(m[0][1], minor(1, 2, 0, 2)))
	return det.Add(det, newWide().Mul(m[0][2], minor(1, 2, 0, 1)))
}

// affineInertia is the inertia of the image of a solid under the linear map
// m, its column k the image of local axis k, at the same density rho: from
// the local inertia I the local second moment is S = (trace(I)/2·1 − I)/rho,
// the image's is S′ = |det m|·m·S·mᵀ, and its inertia rho·(trace(S′)·1 − S′)
// (docs/multibody-dynamics-design.md §8.6).
func affineInertia(m, local [3][3]*big.Float, rho *big.Float) [3][3]*big.Float {
	half := newWide().Add(newWide().Add(local[0][0], local[1][1]), local[2][2])
	half.Quo(half, newWide().SetInt64(2))
	var s [3][3]*big.Float
	for i := range 3 {
		for j := range 3 {
			s[i][j] = newWide().Neg(local[i][j])
			if i == j {
				s[i][j].Add(s[i][j], half)
			}
			s[i][j].Quo(s[i][j], rho)
		}
	}
	det := wideDet(m)
	det.Abs(det)
	var sp [3][3]*big.Float
	for i := range 3 {
		for j := range 3 {
			sum := newWide()
			for k := range 3 {
				for l := range 3 {
					term := newWide().Mul(m[i][k], m[j][l])
					sum.Add(sum, term.Mul(term, s[k][l]))
				}
			}
			sp[i][j] = sum.Mul(sum, det)
		}
	}
	trace := newWide().Add(newWide().Add(sp[0][0], sp[1][1]), sp[2][2])
	var out [3][3]*big.Float
	for i := range 3 {
		for j := range 3 {
			out[i][j] = newWide().Neg(sp[i][j])
			if i == j {
				out[i][j].Add(out[i][j], trace)
			}
			out[i][j].Mul(out[i][j], rho)
		}
	}
	return out
}
