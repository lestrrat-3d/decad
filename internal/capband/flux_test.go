package capband_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// TestRawFluxChargesTheProvenCornerSkew checks the chord-versus-locus term
// reads the patch's proven corner skews, not the difference of its two held
// windows. Here the held windows coincide to the last bit, which is what two
// float Atan2 readings can return for corner ends whose exact angle differs,
// and the proven skews are positive. The flux bound must carry at least the
// swept-volume part of the term at the larger proven skew, three times
// SweptVolumeAllow(maxRadius·Φ, area).
//
// Shown to fail: with chordLocusResidualAllow reading max(capTh0 − th0,
// th1 − capTh1) again, both bounds are equal and the skewed patch's carries
// nothing for its skew.
func TestRawFluxChargesTheProvenCornerSkew(t *testing.T) {
	t.Parallel()
	base := capband.Patch{
		Circular: true, SweepCCW: true,
		SideRadius: 10, CapRadius: 9,
		Th0: 0.2, Th1: 1.4, CapTh0: 0.2, CapTh1: 1.4,
		SideZ: 19, CapZ: 20,
	}
	skewed := base
	skewed.SkewStart, skewed.SkewEnd = 3e-7, 5e-7

	plain := capband.RawFlux(base)
	charged := capband.RawFlux(skewed)
	require.Equal(t, plain.Value, charged.Value, `the skew moves the bound, never the held flux`)

	area, areaBound := capband.AreaOf(skewed)
	swept := proofbound.ProductUpper(3, proofbound.SweptVolumeAllow(
		proofbound.ProductUpper(skewed.SideRadius, skewed.SkewEnd),
		proofbound.AbsSumUpper(area, areaBound)))
	require.Positive(t, swept)
	require.GreaterOrEqual(t, charged.Bound, plain.Bound+swept*(1-1e-12),
		`the skewed patch's flux bound %v carries the swept term %v on top of %v`, charged.Bound, swept, plain.Bound)
}
