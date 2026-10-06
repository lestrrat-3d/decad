package apitest_test

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The general revolve mass path (docs/multibody-dynamics-design.md §8.6) is
// checked against expectations built in this file from section integrals
// integrated independently of decad's Green's-theorem engine: each fixture's
// section is a region z ∈ [0, L], ρ between two affine functions of z, so
// ∫ρ^b·z^c dA is a one-dimensional polynomial integral done here in exact
// rationals. Every fixture revolves about the sketch u axis on the XY plane,
// so the local basis (axis, radial at φ = 0, sweep direction at φ = 0) is
// world (X, Y, Z) unless a fixture places it.
//
// Bound legs. The certificate's legs are the lower-order section moments'
// published bounds (widened into intervals for a circular section), the
// third-order circular enclosure, the π enclosure inside the sweep width,
// the orthonormality-defect widening of the world rotation, and the outward
// conversion of each interval to a float reading. Each was removed in turn:
//   - Orthonormality defect: zeroing it turns the rotated fixture red
//     (TestMassPropertiesRevolveRotated); a signed-permutation basis has a
//     zero defect, so no other fixture exercises it.
//   - Section-moment bounds: reading a circular section's lower-order moments
//     as their held floats with no bound turns the arc wedge red.
//   - Outward conversion: publishing each interval's midpoint with a zero
//     bound turns every fixture red.
//   - Third-order circular enclosure and π enclosure: collapsing either to
//     its lower endpoint leaves every fixture green. Both enclosures are
//     about 1e-60 or narrower relative to the value, far below the ~1e-16
//     relative rounding the float conversion charges, so no float reading
//     can observe them. The internal enclosure tests
//     (TestCircularThirdMomentWholeCircle, TestThirdOrderMomentsOfASector)
//     hold their width and containment instead.
// Each formula term was also removed or swapped: dropping the P·Pᵀ/V
// centroid shift, swapping ∫z²ρ with ∫zρ² in either place, swapping ∫cos
// with ∫sin or ∫cos² with ∫sin², or replacing ∫sinφcosφ turns at least one
// fixture red; the eighth turn is the fixture that separates the cos/sin
// swaps, since a quarter turn's two radial directions are symmetric.

// revolvePi is a rational stand-in for π. Its error is below 1e-79, far below
// the float rounding every reading's bound carries.
func revolvePi(t *testing.T) *big.Rat {
	t.Helper()
	pi, ok := new(big.Rat).SetString("3.14159265358979323846264338327950288419716939937510582097494459230781640628620899")
	require.True(t, ok)
	return pi
}

// revolveSection is the region z ∈ [0, length], lo(z) ≤ ρ ≤ hi(z), with
// lo(z) = lo0 + loSlope·z and hi(z) = hi0 + hiSlope·z.
type revolveSection struct {
	length, lo0, loSlope, hi0, hiSlope *big.Rat
}

// moment is ∫ρ^b·z^c dA = ∫₀ᴸ z^c·(hi(z)^(b+1) − lo(z)^(b+1))/(b+1) dz,
// expanded binomially and integrated term by term.
func (s revolveSection) moment(b, c int) *big.Rat {
	power := func(base, slope *big.Rat) *big.Rat {
		sum := new(big.Rat)
		for m := 0; m <= b+1; m++ {
			term := new(big.Rat).SetInt(new(big.Int).Binomial(int64(b+1), int64(m)))
			for range b + 1 - m {
				term.Mul(term, base)
			}
			for range m {
				term.Mul(term, slope)
			}
			for range c + m + 1 {
				term.Mul(term, s.length)
			}
			term.Quo(term, big.NewRat(int64(c+m+1), 1))
			sum.Add(sum, term)
		}
		return sum
	}
	out := new(big.Rat).Sub(power(s.hi0, s.hiSlope), power(s.lo0, s.loSlope))
	return out.Quo(out, big.NewRat(int64(b+1), 1))
}

// revolveSweep is the sweep's angular factors: ∫dφ, ∫cos φ, ∫sin φ,
// ∫cos² φ, ∫sin φ cos φ and ∫sin² φ.
type revolveSweep struct {
	width, cos, sin, cos2, sinCos, sin2 *big.Rat
}

func quarterSweep(pi *big.Rat) revolveSweep {
	return revolveSweep{
		width: new(big.Rat).Quo(pi, big.NewRat(2, 1)),
		cos:   big.NewRat(1, 1), sin: big.NewRat(1, 1),
		cos2:   new(big.Rat).Quo(pi, big.NewRat(4, 1)),
		sinCos: big.NewRat(1, 2),
		sin2:   new(big.Rat).Quo(pi, big.NewRat(4, 1)),
	}
}

// revolveExpectation is the mass and world tensor (XX, YY, ZZ, XY, XZ, YZ)
// a uniform-density revolve of the section integrals has, built from
// V = Δφ∫ρ, P = (Δφ∫zρ, ∫cos·∫ρ², ∫sin·∫ρ²) and the six Q terms.
func revolveExpectation(rho *big.Rat, sweep revolveSweep,
	r1, zr, r2, z2r, zr2, r3 *big.Rat) (*big.Rat, [6]*big.Rat) {
	mul := func(values ...*big.Rat) *big.Rat {
		out := big.NewRat(1, 1)
		for _, value := range values {
			out.Mul(out, value)
		}
		return out
	}
	volume := mul(sweep.width, r1)
	first := [3]*big.Rat{mul(sweep.width, zr), mul(sweep.cos, r2), mul(sweep.sin, r2)}
	second := [3][3]*big.Rat{
		{mul(sweep.width, z2r), mul(sweep.cos, zr2), mul(sweep.sin, zr2)},
		{mul(sweep.cos, zr2), mul(sweep.cos2, r3), mul(sweep.sinCos, r3)},
		{mul(sweep.sin, zr2), mul(sweep.sinCos, r3), mul(sweep.sin2, r3)},
	}
	var s [3][3]*big.Rat
	for i := range 3 {
		for j := range 3 {
			shift := new(big.Rat).Quo(mul(first[i], first[j]), volume)
			s[i][j] = new(big.Rat).Sub(second[i][j], shift)
		}
	}
	trace := new(big.Rat).Add(new(big.Rat).Add(s[0][0], s[1][1]), s[2][2])
	diagonal := func(i int) *big.Rat { return mul(rho, new(big.Rat).Sub(trace, s[i][i])) }
	mixed := func(i, j int) *big.Rat { return mul(new(big.Rat).Neg(rho), s[i][j]) }
	return mul(rho, volume), [6]*big.Rat{
		diagonal(0), diagonal(1), diagonal(2), mixed(0, 1), mixed(0, 2), mixed(1, 2),
	}
}

func inertiaEntries(reading decad.InertiaReading) [6]decad.Measurement {
	return [6]decad.Measurement{reading.XX, reading.YY, reading.ZZ, reading.XY, reading.XZ, reading.YZ}
}

func requireRevolveReadings(t *testing.T, got decad.MassProperties, mass *big.Rat, inertia [6]*big.Rat) {
	t.Helper()
	requireReadingCovers(t, got.Mass, mass)
	require.Equal(t, units.Mass, got.Mass.Value.Kind())
	for i, entry := range inertiaEntries(got.Inertia) {
		require.Equal(t, units.MomentOfInertia, entry.Value.Kind())
		requireReadingCovers(t, entry, inertia[i])
		// A reading that covers its exact value only because its bound is
		// wide would hide a missing term; every bound here is float rounding.
		expected, _ := inertia[i].Float64()
		require.LessOrEqual(t, entry.Bound.Base(), 1e-12*(1+absFloat(expected)))
	}
}

func absFloat(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}

// TestMassPropertiesRevolveQuarterRectangle revolves the off-axis rectangle
// z ∈ [0, 10], ρ ∈ [5, 15] a quarter turn. Its mixed components carry the
// independently integrated r³ (YZ), r²z (XY, XZ) and rz² (through the
// centroidal subtraction of YY and ZZ) terms. A rectangle's z and ρ are
// independent, so its XY and XZ are the exact zero the r²z term must cancel
// to; the triangle fixture below makes them nonzero.
func TestMassPropertiesRevolveQuarterRectangle(t *testing.T) {
	doc := decad.New()
	sk, profile := annularSketch(t)
	body, err := doc.Revolve(sk, profile, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(1.0 / 1024)
	got, err := body.MassProperties(t.Context(), density)
	require.NoError(t, err)

	section := revolveSection{
		length: big.NewRat(10, 1),
		lo0:    big.NewRat(5, 1), loSlope: new(big.Rat),
		hi0: big.NewRat(15, 1), hiSlope: new(big.Rat),
	}
	rCubed := section.moment(3, 0)
	r2z := section.moment(2, 1)
	rz2 := section.moment(1, 2)
	require.Equal(t, big.NewRat(125000, 1), rCubed)
	require.Equal(t, big.NewRat(162500, 3), r2z)
	require.Equal(t, big.NewRat(100000, 3), rz2)

	rho := new(big.Rat).SetFloat64(density.Base())
	mass, inertia := revolveExpectation(rho, quarterSweep(revolvePi(t)),
		section.moment(1, 0), section.moment(1, 1), section.moment(2, 0), rz2, r2z, rCubed)
	requireRevolveReadings(t, got, mass, inertia)
	require.Zero(t, inertia[3].Sign())
	require.Zero(t, inertia[4].Sign())
	require.Positive(t, inertia[5].Sign())
	require.Positive(t, got.Inertia.YZ.Value.Base())

	// The world center is the evaluator's bounded centroid: on the bisector
	// of the quarter turn at the section's mean axial position.
	pi := revolvePi(t)
	radial := new(big.Rat).Quo(section.moment(2, 0), new(big.Rat).Mul(quarterSweep(pi).width, section.moment(1, 0)))
	radialFloat, _ := radial.Float64()
	require.InDelta(t, 5, got.Center.Value.X, got.Center.Bound.Base())
	require.InDelta(t, radialFloat, got.Center.Value.Y, got.Center.Bound.Base()+1e-12)
	require.InDelta(t, radialFloat, got.Center.Value.Z, got.Center.Bound.Base()+1e-12)

	// An exact cardinal quarter turn about Z maps X→Y and Y→−X, so the
	// tensor is reordered with the products' signs following the basis.
	turn, err := r3.FromBasis(r3.Basis{
		EX: r3.NewVec(0, 1, 0), EY: r3.NewVec(-1, 0, 0), EZ: r3.NewVec(0, 0, 1),
	}, r3.NewVec(2, 3, 4))
	require.NoError(t, err)
	placed, err := body.PlacedCopy(t.Context(), turn)
	require.NoError(t, err)
	turned, err := placed.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Equal(t, got.Mass, turned.Mass)
	require.Equal(t, got.Inertia.XX, turned.Inertia.YY)
	require.Equal(t, got.Inertia.YY, turned.Inertia.XX)
	require.Equal(t, got.Inertia.ZZ, turned.Inertia.ZZ)
	require.Equal(t, -got.Inertia.YZ.Value.Base(), turned.Inertia.XZ.Value.Base())
	require.Equal(t, got.Inertia.YZ.Bound, turned.Inertia.XZ.Bound)

	again, err := body.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Equal(t, got, again)
}

// eighthSweep is [0, π/4], whose ∫cos² and ∫sin² differ, so a fixture swept
// over it tells the two radial directions apart. √2 is a 300-bit rational
// stand-in, as π is.
func eighthSweep(pi *big.Rat) revolveSweep {
	root, _ := new(big.Float).SetPrec(300).Sqrt(new(big.Float).SetPrec(300).SetInt64(2)).Rat(nil)
	halfRoot := new(big.Rat).Quo(root, big.NewRat(2, 1))
	eighthPi := new(big.Rat).Quo(pi, big.NewRat(8, 1))
	return revolveSweep{
		width:  new(big.Rat).Quo(pi, big.NewRat(4, 1)),
		cos:    halfRoot,
		sin:    new(big.Rat).Sub(big.NewRat(1, 1), halfRoot),
		cos2:   new(big.Rat).Add(eighthPi, big.NewRat(1, 4)),
		sinCos: big.NewRat(1, 4),
		sin2:   new(big.Rat).Sub(eighthPi, big.NewRat(1, 4)),
	}
}

// TestMassPropertiesRevolvePartialTriangle revolves the off-axis triangle
// (z, ρ) = (0, 4), (8, 4), (8, 8) a quarter and an eighth turn. Its ρ grows
// with z, so the r²z term no longer cancels and XY and XZ are nonzero; the
// eighth turn's unequal ∫cos² and ∫sin² separate YY from ZZ.
func TestMassPropertiesRevolvePartialTriangle(t *testing.T) {
	section := revolveSection{
		length: big.NewRat(8, 1),
		lo0:    big.NewRat(4, 1), loSlope: new(big.Rat),
		hi0: big.NewRat(4, 1), hiSlope: big.NewRat(1, 2),
	}
	pi := revolvePi(t)
	for _, tc := range []struct {
		name  string
		angle units.Value
		sweep revolveSweep
	}{
		{"quarter", units.Degrees(90), quarterSweep(pi)},
		{"eighth", units.Degrees(45), eighthSweep(pi)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			sk, profile := polygonSketch(t, [][2]float64{{0, 4}, {8, 4}, {8, 8}})
			body, err := doc.Revolve(sk, profile, uAxis, decad.AngleExtent{A: tc.angle, Dir: decad.Along})
			require.NoError(t, err)
			density := units.KilogramsPerCubicMillimeter(1.0 / 1024)
			got, err := body.MassProperties(t.Context(), density)
			require.NoError(t, err)
			rho := new(big.Rat).SetFloat64(density.Base())
			mass, inertia := revolveExpectation(rho, tc.sweep,
				section.moment(1, 0), section.moment(1, 1), section.moment(2, 0),
				section.moment(1, 2), section.moment(2, 1), section.moment(3, 0))
			requireRevolveReadings(t, got, mass, inertia)
			for _, mixed := range []decad.Measurement{got.Inertia.XY, got.Inertia.XZ, got.Inertia.YZ} {
				require.Greater(t, absFloat(mixed.Value.Base()), 1e3*mixed.Bound.Base())
			}
		})
	}
}

// TestMassPropertiesRevolveTorus revolves a circle of radius r = 3 centred
// R = 10 off the axis a full turn: M = ρ·2π²Rr², axial inertia
// M(R² + 3r²/4), transverse M(R²/2 + 5r²/8), no products.
func TestMassPropertiesRevolveTorus(t *testing.T) {
	density := units.KilogramsPerCubicMillimeter(1.0 / 1024)
	got, err := torusBody(t, decad.New(), 10, 3).MassProperties(t.Context(), density)
	require.NoError(t, err)
	pi := revolvePi(t)
	rho := new(big.Rat).SetFloat64(density.Base())
	mass := new(big.Rat).Mul(rho, big.NewRat(180, 1)) // 2·R·r² = 180
	mass.Mul(mass, pi)
	mass.Mul(mass, pi)
	axial := new(big.Rat).Mul(mass, big.NewRat(427, 4))      // 100 + 27/4
	transverse := new(big.Rat).Mul(mass, big.NewRat(445, 8)) // 50 + 45/8
	zero := new(big.Rat)
	requireRevolveReadings(t, got, mass, [6]*big.Rat{axial, transverse, transverse, zero, zero, zero})
	require.InDelta(t, 0, got.Center.Value.X, got.Center.Bound.Base())
	require.InDelta(t, 0, got.Center.Value.Y, got.Center.Bound.Base())
	require.InDelta(t, 0, got.Center.Value.Z, got.Center.Bound.Base())
}

// TestMassPropertiesRevolveArcWedge revolves a half disk of radius R = 5,
// its diameter on the axis and its boundary an ArcSeg, a quarter turn: a
// quarter ball. Its axial inertia about its own centroid is
// (2/5)MR² − M·d² with d² = 2·(3R/8)², i.e. 19MR²/160, and its YZ product is
// −ρ(Q_01 − P_0P_1/V) with ∫ρ³ = 4R⁵/15 and ∫ρ² = πR⁴/8 over the half disk.
func TestMassPropertiesRevolveArcWedge(t *testing.T) {
	doc := decad.New()
	sk, profile := semicircleSketch(t)
	body, err := doc.Revolve(sk, profile, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(1.0 / 1024)
	got, err := body.MassProperties(t.Context(), density)
	require.NoError(t, err)
	pi := revolvePi(t)
	rho := new(big.Rat).SetFloat64(density.Base())
	mass := new(big.Rat).Mul(rho, pi)
	mass.Mul(mass, big.NewRat(125, 3)) // πR³/3
	requireReadingCovers(t, got.Mass, mass)
	axial := new(big.Rat).Mul(mass, big.NewRat(19*25, 160))
	requireReadingCovers(t, got.Inertia.XX, axial)
	q01 := big.NewRat(2*3125, 15)                         // ½·4R⁵/15
	shift := new(big.Rat).Mul(pi, big.NewRat(3*3125, 64)) // (πR⁴/8)²/(πR³/3)
	yz := new(big.Rat).Mul(new(big.Rat).Neg(rho), new(big.Rat).Sub(q01, shift))
	requireReadingCovers(t, got.Inertia.YZ, yz)
	require.Positive(t, got.Inertia.YZ.Value.Base())
	for _, mixed := range []decad.Measurement{got.Inertia.XY, got.Inertia.XZ} {
		requireReadingCovers(t, mixed, new(big.Rat))
	}
}

// TestMassPropertiesRevolveRotated places the quarter-turn triangle under a
// rotation composed from eight steps about (1, 1, 1), so the held basis
// carries accumulated rounding. Every world component must enclose Q·I·Qᵀ,
// I the independent local tensor and Q the polar factor of the held basis
// (mass_properties_rotated_test.go's reference rotation); the sketch frame is
// XY, so the local basis (axis, radial, sweep) is the frame's (U, V, N).
//
// Shown-to-fail: zeroing the orthonormality-defect widening makes at least
// one component miss Q·I·Qᵀ.
func TestMassPropertiesRevolveRotated(t *testing.T) {
	doc := decad.New()
	sk, profile := polygonSketch(t, [][2]float64{{0, 4}, {8, 4}, {8, 8}})
	body, err := doc.Revolve(sk, profile, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	pose := composedRotation(t, r3.NewVec(2, -3, 5), r3.NewVec(1, 1, 1), 30, 8)
	placed, err := body.PlacedCopy(t.Context(), pose)
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(1.0 / 1024)
	got, err := placed.MassProperties(t.Context(), density)
	require.NoError(t, err)

	section := revolveSection{
		length: big.NewRat(8, 1),
		lo0:    big.NewRat(4, 1), loSlope: new(big.Rat),
		hi0: big.NewRat(4, 1), hiSlope: big.NewRat(1, 2),
	}
	rho := new(big.Rat).SetFloat64(density.Base())
	mass, inertia := revolveExpectation(rho, quarterSweep(revolvePi(t)),
		section.moment(1, 0), section.moment(1, 1), section.moment(2, 0),
		section.moment(1, 2), section.moment(2, 1), section.moment(3, 0))
	requireReadingCovers(t, got.Mass, mass)
	local := [3][3]*big.Rat{
		{inertia[0], inertia[3], inertia[4]},
		{inertia[3], inertia[1], inertia[5]},
		{inertia[4], inertia[5], inertia[2]},
	}
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	q := polarFactor(heldRotation(pose, frame))
	for _, entry := range []struct {
		reading decad.Measurement
		i, j    int
	}{
		{got.Inertia.XX, 0, 0}, {got.Inertia.YY, 1, 1}, {got.Inertia.ZZ, 2, 2},
		{got.Inertia.XY, 0, 1}, {got.Inertia.XZ, 0, 2}, {got.Inertia.YZ, 1, 2},
	} {
		want := newWide()
		for k := range 3 {
			for l := range 3 {
				term := newWide().Mul(q[entry.i][k], q[entry.j][l])
				want.Add(want, term.Mul(term, newWide().SetRat(local[k][l])))
			}
		}
		deviation := newWide().Sub(want, newWide().SetFloat64(entry.reading.Value.Base()))
		deviation.Abs(deviation)
		require.LessOrEqual(t, deviation.Cmp(newWide().SetFloat64(entry.reading.Bound.Base())), 0,
			"component (%d,%d) = %s ± %s misses %s", entry.i, entry.j,
			entry.reading.Value, entry.reading.Bound, want.Text('g', 20))
	}
	again, err := placed.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Equal(t, got, again)
}

func TestMassPropertiesRevolveRefusals(t *testing.T) {
	doc := decad.New()
	sk, profile := annularSketch(t)
	body, err := doc.Revolve(sk, profile, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(1.0 / 1024)

	sheet, err := doc.Revolve(sk, profile, uAxis,
		decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	reading, err := sheet.MassProperties(t.Context(), density)
	require.ErrorIs(t, err, decad.ErrNotSolid)
	require.Equal(t, decad.MassProperties{}, reading)

	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	reading, err = body.MassProperties(canceled, density)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, decad.MassProperties{}, reading)
}
