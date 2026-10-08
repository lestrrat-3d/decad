package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The displacement fixture gives a recorded cup — a 16×8 outer section on
// z ∈ [0, 10] around a 12×4 cavity on z ∈ [2, 10], both centered on the
// frame origin — a displacement of 1/1024 mm on each of its three levels and
// on its offset section, so the occupied-volume legs dominate rounding. Every
// cup the record can denote within those displacements must lie inside the
// readings: the open level moves both prisms' tops together, and each wall of
// the inward offset section (the cavity) moves by half the offset
// displacement, so no corner moves farther than it.
//
// Legs shown to fail (each deleted, the fixture watched go red, then
// restored):
//   - prismVolumeMoments' occupied-volume error E zeroed: the mass of a grown
//     or shrunk cup escapes.
//   - The offset displacement charged as the offset region's sectionDelta
//     (cupView.cavityPrism in shell_cup.go): with it dropped, a cup whose
//     cavity walls moved escapes.

func TestCupMassChargesDisplacement(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	const delta = 1.0 / 1024
	cp := cupView{
		outer:  rectangleRecord(-8, -4, 8, 4),
		cavity: rectangleRecord(-6, -2, 6, 2),
		frame:  frame,
		zOpen:  10, zOuter: 0, zCav: 2,
		zOpenDelta: delta, zOuterDelta: delta, zCavDelta: delta,
		thickness: 2, thicknessDelta: delta, offsetDelta: delta,
		sense: Inward,
		xform: r3.Identity(),
	}
	rho := 1.0 / 1024
	got, err := cupMassProperties(t.Context(), &Body{}, cp, units.KilogramsPerCubicMillimeter(rho))
	require.NoError(t, err)

	whole := new(big.Rat).SetFloat64(delta)
	half := new(big.Rat).SetFloat64(delta / 2)

	moves := func(step *big.Rat) []*big.Rat {
		return []*big.Rat{new(big.Rat).Neg(step), new(big.Rat), step}
	}
	density := new(big.Rat).SetFloat64(rho)
	r := func(v int64) *big.Rat { return big.NewRat(v, 1) }
	for _, open := range moves(whole) {
		for _, floor := range moves(whole) {
			for _, cavityFloor := range moves(whole) {
				for _, grow := range moves(half) {
					top := new(big.Rat).Add(r(10), open)
					outer := centeredBoxMoments(r(8), r(4), floor, top)
					cavity := centeredBoxMoments(new(big.Rat).Add(r(6), grow), new(big.Rat).Add(r(2), grow),
						new(big.Rat).Add(r(2), cavityFloor), top)
					requireCupCovered(t, got, outer, cavity, density)
				}
			}
		}
	}
	// The widening stays below a tenth of each diagonal value, so the
	// readings still say something about the body.
	for _, reading := range []Measurement{got.Inertia.XX, got.Inertia.YY, got.Inertia.ZZ} {
		require.Less(t, reading.Bound.Base(), reading.Value.Base()/10)
	}
}

// centeredBoxMoments is V, P and Q about the origin of the box
// [-hu, hu] × [-hv, hv] × [z0, z1].
func centeredBoxMoments(hu, hv, z0, z1 *big.Rat) [10]*big.Rat {
	mul := func(values ...*big.Rat) *big.Rat {
		out := big.NewRat(1, 1)
		for _, value := range values {
			out.Mul(out, value)
		}
		return out
	}
	a := mul(big.NewRat(4, 1), hu, hv)
	h := new(big.Rat).Sub(z1, z0)
	z1Sum := new(big.Rat).Quo(new(big.Rat).Sub(mul(z1, z1), mul(z0, z0)), big.NewRat(2, 1))
	z2Sum := new(big.Rat).Quo(new(big.Rat).Sub(mul(z1, z1, z1), mul(z0, z0, z0)), big.NewRat(3, 1))
	third := big.NewRat(1, 3)
	zero := new(big.Rat)
	// V, Pu, Pv, Pz, Quu, Qvv, Qzz, Quv, Quz, Qvz.
	return [10]*big.Rat{
		mul(a, h), zero, zero, mul(a, z1Sum),
		mul(a, hu, hu, third, h), mul(a, hv, hv, third, h), mul(a, z2Sum),
		zero, zero, zero,
	}
}

func requireCupCovered(t *testing.T, got MassProperties, outer, cavity [10]*big.Rat, density *big.Rat) {
	t.Helper()
	var m [10]*big.Rat
	for i := range m {
		m[i] = new(big.Rat).Sub(outer[i], cavity[i])
	}
	requireRatCovered(t, got.Mass, new(big.Rat).Mul(density, m[0]))
	shift := func(a, b int) *big.Rat { return new(big.Rat).Quo(new(big.Rat).Mul(m[a], m[b]), m[0]) }
	suu := new(big.Rat).Sub(m[4], shift(1, 1))
	svv := new(big.Rat).Sub(m[5], shift(2, 2))
	szz := new(big.Rat).Sub(m[6], shift(3, 3))
	diagonal := func(p, q *big.Rat) *big.Rat { return new(big.Rat).Mul(density, new(big.Rat).Add(p, q)) }
	requireRatCovered(t, got.Inertia.XX, diagonal(svv, szz))
	requireRatCovered(t, got.Inertia.YY, diagonal(suu, szz))
	requireRatCovered(t, got.Inertia.ZZ, diagonal(suu, svv))
	for _, reading := range []Measurement{got.Inertia.XY, got.Inertia.XZ, got.Inertia.YZ} {
		requireRatCovered(t, reading, new(big.Rat))
	}
}
