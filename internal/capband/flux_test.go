package capband_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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

// TestRawFluxChargesTheCornerFlux checks a Cone patch's chord-locus term
// carries its corner slivers' flux (Patch.CornerFlux) on top of the rest of
// its bound, with the corner skews zero and with them positive, and that the
// first-moment reading ChordLocusVolume returns at least a third of it. The
// held flux does not move.
//
// Shown to fail on 2026-10-09: with chordLocusResidualAllow passing zero for
// the corner flux, every charged bound equals the plain one and the
// zero-skew patch's ChordLocusVolume is zero.
func TestRawFluxChargesTheCornerFlux(t *testing.T) {
	t.Parallel()
	const corner = 0.25
	for _, tc := range []struct {
		name       string
		start, end float64
	}{
		{name: `zero skews`},
		{name: `positive skews`, start: 3e-3, end: 5e-3},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			plain := capband.Patch{
				Circular: true, SweepCCW: true,
				SideRadius: 10, CapRadius: 9,
				Th0: 0.2, Th1: 1.4, CapTh0: 0.2 + tc.start, CapTh1: 1.4 - tc.end,
				SkewStart: tc.start, SkewEnd: tc.end,
				SideZ: 19, CapZ: 20,
			}
			charged := plain
			charged.CornerFlux = corner

			p, c := capband.RawFlux(plain), capband.RawFlux(charged)
			require.Equal(t, p.Value, c.Value, `the corner flux moves the bound, never the held flux`)
			require.GreaterOrEqual(t, c.Bound, p.Bound+corner*(1-1e-12),
				`the charged bound %v must carry the corner flux %v on top of %v`, c.Bound, corner, p.Bound)

			vol, _ := capband.ChordLocusVolume(charged)
			require.GreaterOrEqual(t, vol, corner/3, `the first-moment reading's volume must carry the corner flux`)
		})
	}
}

// TestAxisAnchoredLevelsKeepsTheExactHeight checks the chord-versus-locus
// references' axial shift. Their cone must keep the band's exact height, so
// the shifted pair's exact difference must equal the original pair's, and the
// side level must land within half an ulp of the height from zero. The second
// case's float difference rounds (0.1 + 0.2 is not the float 0.3...04), so
// it reaches the TwoSum residual.
//
// Shown to fail on 2026-10-09: returning (0, capZ − sideZ) without the
// residual breaks the exact difference on the rounding case.
func TestAxisAnchoredLevelsKeepsTheExactHeight(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name        string
		sideZ, capZ float64
	}{
		{name: `exact difference`, sideZ: 16, capZ: 20},
		{name: `rounded difference`, sideZ: -0.2, capZ: 0.1},
		{name: `far from the sketch plane`, sideZ: 1e6 - 4, capZ: 1e6},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			side, capZ := capband.AxisAnchoredLevels(tc.sideZ, tc.capZ)
			want := new(big.Rat).Sub(proofarith.FloatRat(tc.capZ), proofarith.FloatRat(tc.sideZ))
			got := new(big.Rat).Sub(proofarith.FloatRat(capZ), proofarith.FloatRat(side))
			require.Zero(t, want.Cmp(got), `the shifted levels' exact difference %v must equal the band's %v`,
				got.FloatString(30), want.FloatString(30))
			require.LessOrEqual(t, math.Abs(side), math.Nextafter(capZ, math.Inf(1))-capZ,
				`the side level %v must sit within an ulp of the height %v from zero`, side, capZ)
		})
	}
}
