package decad

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestPrismLiftFactorBoundsTheHeldMap checks the stretch prismPointBound
// carries a plane-local error ball through: exactly 1 for an axis-aligned
// frame without placement, whose held map is exactly orthonormal, and for
// randomly rotated frames and placements a factor whose square covers every
// row's absolute sum of LᵀL, computed exactly from the held map, which
// bounds the largest eigenvalue of LᵀL (Gershgorin) and so the map's own
// stretch. The factor must also stay within 1e-12 of 1, and the bound must
// carry the error ball at that factor.
//
// Shown to fail on 2026-10-09: with prismLiftFactor answering 1 whatever the
// defect, rotated maps whose rows exceed 1 fail the cover check; with the
// fixed 4 back, the axis-aligned row and the tightness check fail.
func TestPrismLiftFactorBoundsTheHeldMap(t *testing.T) {
	t.Parallel()
	xy := canonicalPrismFrame(t)
	require.Equal(t, 1.0, prismLiftFactor(prismPayload{frame: xy, xform: r3.Identity()}),
		`an axis-aligned unplaced map is exactly orthonormal`)

	rng := rand.New(rand.NewPCG(53, 59))
	exceeded := 0
	for range 200 {
		axis := func() r3.Vec { return r3.Vec{X: rng.Float64() - 0.5, Y: rng.Float64() - 0.5, Z: rng.Float64() - 0.5} }
		frame, err := r3.NewFrame(r3.Vec{X: rng.Float64() * 100}, axis(), axis())
		require.NoError(t, err)
		xform, err := r3.Rotation(axis(), units.Degrees(rng.Float64()*360))
		require.NoError(t, err)
		pp := prismPayload{frame: frame, xform: xform}
		factor := prismLiftFactor(pp)
		require.LessOrEqual(t, factor, 1+1e-12)

		l, err := massmoment.PrismRotation(frame, xform)
		require.NoError(t, err)
		f := new(big.Rat).SetFloat64(factor)
		f2 := new(big.Rat).Mul(f, f)
		for i := range 3 {
			row := new(big.Rat)
			for j := range 3 {
				g := new(big.Rat)
				for k := range 3 {
					g.Add(g, new(big.Rat).Mul(l[k][i], l[k][j]))
				}
				row.Add(row, g.Abs(g))
			}
			if row.Cmp(big.NewRat(1, 1)) > 0 {
				exceeded++
			}
			require.GreaterOrEqual(t, f2.Cmp(row), 0,
				`the factor %v must cover row %d of LᵀL, %s`, factor, i, row.FloatString(20))
		}
	}
	require.Positive(t, exceeded, `some held maps stretch past 1, which the factor must cover`)

	// The bound carries the source ball at that factor, not four times it.
	pp := prismPayload{frame: xy, xform: r3.Identity()}
	b := 1e-3
	got := prismPointBound(pp, proofbound.MeasuredScalar(1, b), proofbound.MeasuredScalar(2, b), proofbound.MeasuredScalar(3, b))
	require.LessOrEqual(t, got, proofbound.Radius3D(b)*(1+1e-12),
		`an exact lift charges the ball once, %v, not %v`, proofbound.Radius3D(b), got)
	require.GreaterOrEqual(t, got, math.Sqrt(3)*b)
}
