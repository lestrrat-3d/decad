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
// independent closed-form box tensor rotated by the rigid rotation decad
// holds.
//
// The reference rotation is the polar factor Q of the held basis product M,
// the rotation nearest M; it is computed by Newton iteration at 512 bits, so
// its own error is far below any reading bound and the enclosure checks add
// no slack. Side lengths, levels and the density are dyadic; the one
// non-dyadic length is the denoted level of the inch extrusion, which is the
// point of that fixture.
//
// Legs shown to fail (each deleted in mass_properties_rotated.go, the fixture
// watched go red, then restored):
//   - The orthonormality-defect widening 3·d·(2+d)·m: with it zeroed,
//     TestMassPropertiesRotatedBox and TestMassPropertiesTiltedFrameBox both
//     miss the reference XX component.
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
	held, err := s.Plane().Frame()
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
// lengths: the exact mass, and each world inertia component against Q I Qᵀ.
func requireRotatedBoxExact(t *testing.T, got decad.MassProperties, pose r3.Transform, frame r3.Frame, size [3]*big.Rat) {
	t.Helper()
	rho := new(big.Rat).SetFloat64(rotatedDensity)
	mass := new(big.Rat).Mul(rho, new(big.Rat).Mul(size[0], new(big.Rat).Mul(size[1], size[2])))
	requireReadingCovers(t, got.Mass, mass)
	var local [3]*big.Rat
	for i := range local {
		a, b := size[(i+1)%3], size[(i+2)%3]
		sum := new(big.Rat).Add(new(big.Rat).Mul(a, a), new(big.Rat).Mul(b, b))
		local[i] = new(big.Rat).Quo(new(big.Rat).Mul(mass, sum), big.NewRat(12, 1))
	}
	q := polarFactor(heldRotation(pose, frame))
	world := func(i, j int) *big.Float {
		sum := newWide()
		for k := range local {
			term := newWide().Mul(q[i][k], q[j][k])
			sum.Add(sum, term.Mul(term, newWide().SetRat(local[k])))
		}
		return sum
	}
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

// polarFactor iterates X ← (X + X⁻ᵀ)/2, which converges quadratically to the
// orthogonal polar factor from a nearly orthogonal start.
func polarFactor(m [3][3]*big.Float) [3][3]*big.Float {
	x := m
	for range 8 {
		var cofactor [3][3]*big.Float
		for i := range 3 {
			for j := range 3 {
				r0, r1 := (i+1)%3, (i+2)%3
				c0, c1 := (j+1)%3, (j+2)%3
				left := newWide().Mul(x[r0][c0], x[r1][c1])
				cofactor[i][j] = left.Sub(left, newWide().Mul(x[r0][c1], x[r1][c0]))
			}
		}
		det := newWide()
		for j := range 3 {
			det.Add(det, newWide().Mul(x[0][j], cofactor[0][j]))
		}
		var next [3][3]*big.Float
		for i := range 3 {
			for j := range 3 {
				inverseT := newWide().Quo(cofactor[i][j], det)
				sum := newWide().Add(x[i][j], inverseT)
				next[i][j] = sum.Quo(sum, newWide().SetInt64(2))
			}
		}
		x = next
	}
	return x
}

func newWide() *big.Float { return new(big.Float).SetPrec(512) }
