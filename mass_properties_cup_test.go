package decad_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The cup mass path (docs/multibody-dynamics-design.md §8.8) is checked
// against the outer solid's exact moments minus the cavity's, each integrated
// in this file in closed form about the world origin, the difference formed
// before the centroidal tensor. Plate sides, height, thickness and density
// are dyadic. requireMomentsReadings caps every bound at 1e-9 of the tensor
// scale, which only the analytic path meets: a mesh fallback reading would
// fail it.
//
// Legs shown to fail (each deleted in mass_properties_cup.go or
// mass_properties_revolve.go, the fixture watched go red, then restored):
//   - The cavity's re-anchoring onto the outer mid level: with it dropped,
//     both fixtures miss a diagonal component.
//   - The placement's orthonormality-defect widening: with it zeroed, the
//     placed cup of TestMassPropertiesCupOuterMinusCavity misses a reference
//     component.
// The level and thickness displacement legs are recorded in
// mass_properties_cup_internal_test.go, whose displacements are large enough
// to separate them.

const cupDensity = 1.0 / 1024

// cupBox extrudes the 16×8 plate at the origin by 10 and shells away its top
// cap with the given thickness and options.
func cupBox(t *testing.T, thickness units.Value, opts ...decad.ShellOption) *decad.Body {
	t.Helper()
	box := boxBody(t, decad.New(), 0, 0, 16, 8, 10)
	cup, err := box.Shell(t.Context(), topCap(box), thickness, opts...)
	require.NoError(t, err)
	return cup
}

func TestMassPropertiesCupOuterMinusCavity(t *testing.T) {
	cup := cupBox(t, units.Millimeters(1))
	density := units.KilogramsPerCubicMillimeter(cupDensity)
	rho := new(big.Rat).SetFloat64(cupDensity)
	want := exactBoxMoments(ratVec(0, 0, 0), ratVec(16, 8, 10)).
		plus(exactBoxMoments(ratVec(1, 1, 1), ratVec(15, 7, 10)), -1)

	got, err := cup.MassProperties(t.Context(), density)
	require.NoError(t, err)
	requireMomentsReadings(t, got, want, rho, r3.Identity(), nil)
	// The cavity is open at the top, so the center sits below mid height.
	require.Less(t, got.Center.Value.Z, 5.0)

	pose := composedRotation(t, r3.NewVec(2, -3, 5), r3.NewVec(1, 1, 1), 40, 8)
	placed, err := cup.PlacedCopy(t.Context(), pose)
	require.NoError(t, err)
	turned, err := placed.MassProperties(t.Context(), density)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	q := polarFactor(heldRotation(pose, frame))
	requireMomentsReadings(t, turned, want, rho, pose, &q)
}

// TestMassPropertiesCupOutward shells outward: the outer section is the plate
// dilated by the thickness, its four convex corners rounded by quarter
// circles, so the outer prism's section moments come from the circular-arc
// integrals and carry their own enclosure.
func TestMassPropertiesCupOutward(t *testing.T) {
	cup := cupBox(t, units.Millimeters(2), decad.WithShellSense(decad.Outward))
	got, err := cup.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(cupDensity))
	require.NoError(t, err)

	// The rounded section on z ∈ [-2, 10]: the plate widened by 2 along x,
	// the plate widened by 2 along y, less the plate they share, plus a
	// quarter disk of radius 2 at each corner.
	pi := revolvePi(t)
	lo, hi := big.NewRat(-2, 1), big.NewRat(10, 1)
	outer := exactBoxMoments(ratVec(-2, 0, -2), ratVec(18, 8, 10)).
		plus(exactBoxMoments(ratVec(0, -2, -2), ratVec(16, 10, 10)), 1).
		plus(exactBoxMoments(ratVec(0, 0, -2), ratVec(16, 8, 10)), -1)
	for _, corner := range [][2]int64{{0, 0}, {16, 0}, {16, 8}, {0, 8}} {
		outer = outer.plus(quarterDiskPrism(pi, corner, lo, hi), 1)
	}
	want := outer.plus(exactBoxMoments(ratVec(0, 0, 0), ratVec(16, 8, 10)), -1)
	rho := new(big.Rat).SetFloat64(cupDensity)
	requireMomentsReadings(t, got, want, rho, r3.Identity(), nil)
}

// quarterDiskPrism is the prism z ∈ [lo, hi] over the quarter disk of radius
// 2 centered on the plate corner (cx, cy), in the quadrant facing away from
// the plate.
func quarterDiskPrism(pi *big.Rat, corner [2]int64, lo, hi *big.Rat) massMoments {
	cx, cy := corner[0], corner[1]
	sx, sy := int64(-1), int64(-1)
	if cx > 0 {
		sx = 1
	}
	if cy > 0 {
		sy = 1
	}
	// About the disk center, over the quadrant x·sx ≥ 0, y·sy ≥ 0, r = 2:
	// ∫dA = πr²/4, ∫x dA = sx·r³/3, ∫x² dA = πr⁴/16, ∫xy dA = sx·sy·r⁴/8.
	r := big.NewRat(2, 1)
	r2, r3c, r4 := ratProduct(r, r), ratProduct(r, r, r), ratProduct(r, r, r, r)
	area := new(big.Rat).Quo(ratProduct(pi, r2), big.NewRat(4, 1))
	mx := ratProduct(big.NewRat(sx, 1), new(big.Rat).Quo(r3c, big.NewRat(3, 1)))
	my := ratProduct(big.NewRat(sy, 1), new(big.Rat).Quo(r3c, big.NewRat(3, 1)))
	mxx := new(big.Rat).Quo(ratProduct(pi, r4), big.NewRat(16, 1))
	mxy := ratProduct(big.NewRat(sx*sy, 1), new(big.Rat).Quo(r4, big.NewRat(8, 1)))
	h := new(big.Rat).Sub(hi, lo)
	z1 := new(big.Rat).Quo(new(big.Rat).Sub(ratProduct(hi, hi), ratProduct(lo, lo)), big.NewRat(2, 1))
	z2 := new(big.Rat).Quo(new(big.Rat).Sub(ratProduct(hi, hi, hi), ratProduct(lo, lo, lo)), big.NewRat(3, 1))
	m := massMoments{volume: ratProduct(area, h)}
	m.first = [3]*big.Rat{ratProduct(mx, h), ratProduct(my, h), ratProduct(area, z1)}
	m.second = [3][3]*big.Rat{
		{ratProduct(mxx, h), ratProduct(mxy, h), ratProduct(mx, z1)},
		{ratProduct(mxy, h), ratProduct(mxx, h), ratProduct(my, z1)},
		{ratProduct(mx, z1), ratProduct(my, z1), ratProduct(area, z2)},
	}
	return m.shifted(ratVec(cx, cy, 0))
}

// TestCupInchWallEnclosesDenotedCup shells two plates with a 0.1 in wall.
// The denoted thickness is that magnitude rescaled to millimetres in exact
// rationals, and the denoted cavity is the plate eroded by it. The offset
// section the shell records is computed in floats, and its corners land a few
// units in the last place away from the denoted ones, so every reading the
// cup publishes — Volume, Centroid and MassProperties — must still enclose the
// exact closed form of the denoted cup.
func TestCupInchWallEnclosesDenotedCup(t *testing.T) {
	thickness := units.Inches(0.1)
	wall := new(big.Rat).Quo(
		new(big.Rat).Mul(new(big.Rat).SetFloat64(thickness.Mag()), new(big.Rat).SetFloat64(thickness.Unit().Factor())),
		new(big.Rat).SetFloat64(units.Millimeter.Factor()))
	density := units.KilogramsPerCubicMillimeter(cupDensity)
	rho := new(big.Rat).SetFloat64(cupDensity)
	for _, plate := range [][2]int64{{16, 8}, {100, 60}} {
		box := boxBody(t, decad.New(), 0, 0, float64(plate[0]), float64(plate[1]), 10)
		cup, err := box.Shell(t.Context(), topCap(box), thickness)
		require.NoError(t, err)

		hi := ratVec(plate[0], plate[1], 10)
		lo := [3]*big.Rat{wall, wall, wall}
		cavityHi := [3]*big.Rat{new(big.Rat).Sub(hi[0], wall), new(big.Rat).Sub(hi[1], wall), hi[2]}
		want := exactBoxMoments(ratVec(0, 0, 0), hi).plus(exactBoxMoments(lo, cavityHi), -1)

		volume, err := cup.Volume()
		require.NoError(t, err)
		requireReadingCovers(t, volume, want.volume)
		centroid, err := cup.Centroid()
		require.NoError(t, err)
		requireCenterCovers(t, centroid, want)

		got, err := cup.MassProperties(t.Context(), density)
		require.NoError(t, err)
		requireReadingCovers(t, got.Mass, ratProduct(rho, want.volume))
		requireCenterCovers(t, got.Center, want)
		exact := want.inertia(rho)
		for _, entry := range []struct {
			reading decad.Measurement
			i, j    int
		}{
			{got.Inertia.XX, 0, 0}, {got.Inertia.YY, 1, 1}, {got.Inertia.ZZ, 2, 2},
			{got.Inertia.XY, 0, 1}, {got.Inertia.XZ, 0, 2}, {got.Inertia.YZ, 1, 2},
		} {
			requireReadingCovers(t, entry.reading, exact[entry.i][entry.j])
		}
	}
}

// requireCenterCovers checks that every component of a centroid reading lies
// within its bound of the exact center of m, compared in exact rationals.
func requireCenterCovers(t *testing.T, reading decad.VecMeasurement, m massMoments) {
	t.Helper()
	held := [3]float64{reading.Value.X, reading.Value.Y, reading.Value.Z}
	bound := new(big.Rat).SetFloat64(reading.Bound.Base())
	for i := range held {
		exact := new(big.Rat).Quo(m.first[i], m.volume)
		deviation := new(big.Rat).Abs(new(big.Rat).Sub(exact, new(big.Rat).SetFloat64(held[i])))
		require.LessOrEqual(t, deviation.Cmp(bound), 0, "center component %d misses its bound", i)
	}
}
