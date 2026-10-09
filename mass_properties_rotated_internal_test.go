package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The occupied-volume fixture gives a recorded 16×8×16 box displacements of
// 1/1024 mm on its section and both levels, so the E, R·E and R²·E legs of
// docs/multibody-dynamics-design.md §8.2 dominate rounding. The section is
// centered on the frame origin, the anchor of the volume moments, so its
// first moments vanish and no volume uncertainty leaks into the centroidal
// tensor through P Pᵀ/V: each leg has to cover its own moment order. Every box the
// record can denote within those displacements must lie inside the readings.
//
// Legs shown to fail (each deleted in mass_properties_rotated.go, the fixture
// watched go red, then restored):
//   - E zeroed entirely: the mass of every grown or shrunk box escapes.
//   - The V leg alone (E on volume): the mass escapes.
//   - The Q leg alone (R²·E on second moments): the inertia components escape.
//   - The P leg alone (R·E on first moments) did not turn this fixture red.
//     A denoted box here is centered like the record, so its first moments
//     stay zero; P reaches the tensor only through P Pᵀ/V, whose change a
//     region off center would show. The leg stays because dynamic-mass §2.2
//     charges every moment order and a denoted region need not be centered.

func rectangleRecord(u0, v0, u1, v1 float64) profileRecord {
	corners := []Point2{{U: u0, V: v0}, {U: u1, V: v0}, {U: u1, V: v1}, {U: u0, V: v1}}
	segments := make([]curveSegment, len(corners))
	for i, corner := range corners {
		segments[i] = lineSeg{Start: corner, End: corners[(i+1)%len(corners)], TEnd: 1}
	}
	return profileRecord{Outer: loopRecord{Segments: segments}}
}

func TestRotatedPrismMassChargesDisplacement(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	const delta = 1.0 / 1024
	pp := prismPayload{
		profile: rectangleRecord(-8, -4, 8, 4),
		frame:   frame,
		z0:      0, z1: 16,
		z0Delta: delta, z1Delta: delta, sectionDelta: delta,
		xform: r3.Identity(),
	}
	rho := 1.0 / 1024
	got, err := rotatedPrismMassProperties(t.Context(), pp, VecMeasurement{}, units.KilogramsPerCubicMillimeter(rho))
	require.NoError(t, err)

	// A denoted section moves each boundary point by at most delta: here by
	// half of it per side, grown or shrunk, so no corner moves farther than
	// delta. Each level moves by at most its own delta.
	half := new(big.Rat).SetFloat64(delta / 2)
	whole := new(big.Rat).SetFloat64(delta)
	zero := new(big.Rat)
	sections := [][2]*big.Rat{ // per-side growth along u and v
		{half, half}, {new(big.Rat).Neg(half), new(big.Rat).Neg(half)}, {zero, zero},
	}
	levels := [][2]*big.Rat{ // outward movement of the bottom and top levels
		{whole, whole}, {new(big.Rat).Neg(whole), new(big.Rat).Neg(whole)},
		{new(big.Rat).Neg(whole), whole}, {zero, zero},
	}
	density := new(big.Rat).SetFloat64(rho)
	for _, grow := range sections {
		for _, level := range levels {
			a := new(big.Rat).Add(big.NewRat(16, 1), new(big.Rat).Mul(big.NewRat(2, 1), grow[0]))
			b := new(big.Rat).Add(big.NewRat(8, 1), new(big.Rat).Mul(big.NewRat(2, 1), grow[1]))
			c := new(big.Rat).Add(big.NewRat(16, 1), new(big.Rat).Add(level[0], level[1]))
			mass := new(big.Rat).Mul(density, new(big.Rat).Mul(a, new(big.Rat).Mul(b, c)))
			requireRatCovered(t, got.Mass, mass)
			sizes := [3]*big.Rat{a, b, c}
			diagonal := [3]Measurement{got.Inertia.XX, got.Inertia.YY, got.Inertia.ZZ}
			for i, reading := range diagonal {
				p, q := sizes[(i+1)%3], sizes[(i+2)%3]
				sum := new(big.Rat).Add(new(big.Rat).Mul(p, p), new(big.Rat).Mul(q, q))
				requireRatCovered(t, reading, new(big.Rat).Quo(new(big.Rat).Mul(mass, sum), big.NewRat(12, 1)))
			}
			for _, reading := range []Measurement{got.Inertia.XY, got.Inertia.XZ, got.Inertia.YZ} {
				requireRatCovered(t, reading, zero)
			}
		}
	}

	// The widening stays below a tenth of each diagonal value, so the
	// readings still say something about the body.
	for _, reading := range []Measurement{got.Inertia.XX, got.Inertia.YY, got.Inertia.ZZ} {
		require.Less(t, reading.Bound.Base(), reading.Value.Base()/10)
	}
}

// TestRotatedPrismMassMatchesCardinalPath checks that the general path and
// the signed-permutation path publish identical readings for an undisplaced
// cardinal prism.
func TestRotatedPrismMassMatchesCardinalPath(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	triangle := []Point2{{U: 0, V: 0}, {U: 6, V: 0}, {U: 0, V: 8}}
	segments := make([]curveSegment, len(triangle))
	for i, corner := range triangle {
		segments[i] = lineSeg{Start: corner, End: triangle[(i+1)%len(triangle)], TEnd: 1}
	}
	turn, err := r3.FromBasis(r3.Basis{
		EX: r3.NewVec(0, 1, 0), EY: r3.NewVec(-1, 0, 0), EZ: r3.NewVec(0, 0, 1),
	}, r3.NewVec(30, 0, 0))
	require.NoError(t, err)
	pp := prismPayload{
		profile: profileRecord{Outer: loopRecord{Segments: segments}},
		frame:   frame, z0: 0, z1: 10, xform: turn,
	}
	density := units.KilogramsPerCubicMillimeter(1.0 / 1024)
	general, err := rotatedPrismMassProperties(t.Context(), pp, VecMeasurement{}, density)
	require.NoError(t, err)
	cardinal, err := prismMassProperties(t.Context(), &Body{}, pp, density)
	require.NoError(t, err)
	require.Equal(t, cardinal, general)
}

func requireRatCovered(t *testing.T, reading Measurement, exact *big.Rat) {
	t.Helper()
	held := new(big.Rat).SetFloat64(reading.Value.Base())
	bound := new(big.Rat).SetFloat64(reading.Bound.Base())
	deviation := new(big.Rat).Abs(new(big.Rat).Sub(exact, held))
	require.LessOrEqual(t, deviation.Cmp(bound), 0, "%s ± %s misses %s",
		reading.Value, reading.Bound, exact.FloatString(20))
}
