package decad

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// This file checks the cap-chamfer Cone patch's exact-rational closed forms
// against their float transcriptions: the first-moment Fourier coefficients
// term by term (coneMomentTermsX/Y/Z), and the volume flux enclosure
// (conePatchFluxInterval) against the float closed form patchRawFlux's
// fallback evaluates. The float forms below are the reference; they are not
// called by production code.

// TestCapBlendConeMomentCoefficientsMatchReference checks every rational
// Fourier coefficient against the float reference: the same (k, m) list in
// the same order, and each Ac, As within 1e-12 relative of the reference.
// The tuples cover a tangent quarter-turn patch, a far-placed skewed patch
// with negative H, and an apex patch (R0 = 0); Z runs at a near and a far
// side level.
//
// Shown to fail on 2026-10-04: changing the 9 in X's (0, 1) coefficient to 8
// failed the X axis, and changing the 6 in Z's (0, 0) z0·z0·R1² term to 5
// failed the Z axis.
func TestCapBlendConeMomentCoefficientsMatchReference(t *testing.T) {
	t.Parallel()
	type tuple struct{ R0, R1, H, c, dS, dC float64 }
	tuples := []tuple{
		{12, 11.5, 0.5, 36, 1.5707963267948966, 1.5707963267948966},
		{10, 12, -0.5, -988, 2.7, 2.3},
		{0, 2, 0.25, 7, 0.9, 0.9},
	}
	check := func(t *testing.T, label string, got []phaseTerm, want []referencePhaseTerm) {
		t.Helper()
		require.Len(t, got, len(want), `%s: term count`, label)
		for i := range want {
			require.Equal(t, want[i].k, got[i].K(), `%s: term %d k`, label, i)
			require.Equal(t, want[i].m, got[i].M(), `%s: term %d m`, label, i)
			ac, _ := got[i].Ac().Float64()
			as, _ := got[i].As().Float64()
			require.LessOrEqual(t, math.Abs(ac-want[i].ac), 1e-12*math.Max(1, math.Abs(want[i].ac)),
				`%s: term %d (%d, %d) ac %v, reference %v`, label, i, want[i].k, want[i].m, ac, want[i].ac)
			require.LessOrEqual(t, math.Abs(as-want[i].as), 1e-12*math.Max(1, math.Abs(want[i].as)),
				`%s: term %d (%d, %d) as %v, reference %v`, label, i, want[i].k, want[i].m, as, want[i].as)
		}
	}
	for _, tc := range tuples {
		R0, R1, H := proofarith.FloatRat(tc.R0), proofarith.FloatRat(tc.R1), proofarith.FloatRat(tc.H)
		c, dS, dC := proofarith.FloatRat(tc.c), proofarith.FloatRat(tc.dS), proofarith.FloatRat(tc.dC)
		label := fmt.Sprintf(`%+v`, tc)
		check(t, label+` X`, coneMomentTermsX(R0, R1, H, c, dS, dC),
			referenceMomentTermsX(tc.R0, tc.R1, tc.H, tc.c, tc.dS, tc.dC))
		check(t, label+` Y`, coneMomentTermsY(R0, R1, H, c, dS, dC),
			referenceMomentTermsY(tc.R0, tc.R1, tc.H, tc.c, tc.dS, tc.dC))
		// The far level is -1000, not -1e6: at -1e6 the float reference loses
		// about 3e-10 relative to cancellation of its z0² terms, while the
		// rational stays exact.
		for _, z0 := range []float64{15.5, -1000} {
			check(t, fmt.Sprintf(`%s Z z0=%g`, label, z0), coneMomentTermsZ(R0, R1, H, proofarith.FloatRat(z0), dS, dC),
				referenceMomentTermsZ(tc.R0, tc.R1, tc.H, z0, tc.dS, tc.dC))
		}
	}
}

// TestCapBlendConeFluxIntervalEnclosesFloatClosedForm checks
// conePatchFluxInterval two ways. Its width must stay at the radian
// enclosure's own grid level (each sine or cosine is about 8e-29 wide)
// scaled by the patch's magnitude, so a regression to a magnitude envelope
// shows. Its midpoint must agree with the float closed form patchRawFlux's
// fallback evaluates, which is the transcription check on every term.
//
// Shown to fail on 2026-10-04: replacing originR1 by a zero point interval
// moved the midpoint off the float closed form on every row, and replacing
// intCos by the point 1 did so on the skewed, major-arc and hole-style rows,
// and publishing a zero bound from patchRawFlux's enclosure arm failed the
// positive-bound check on the tangent and far-placed rows.
func TestCapBlendConeFluxIntervalEnclosesFloatClosedForm(t *testing.T) {
	t.Parallel()
	plate := capPatchGeom{
		circular: true, sweepCCW: true,
		cU: 36, cV: 22, sideRadius: 12, capRadius: 11.5,
		th0: 0, th1: math.Pi / 2, capTh0: 0, capTh1: math.Pi / 2,
		sideZ: 15.5, capZ: 16,
	}
	far := plate
	far.cU, far.cV = 988, 1000
	skewed := plate
	skewed.th0, skewed.th1, skewed.capTh0, skewed.capTh1 = 0.3, 2.1, 0.35, 2.05
	major := plate
	major.th0, major.th1, major.capTh0, major.capTh1 = -2.9, 2.9, -2.85, 2.85
	hole := plate
	hole.cU, hole.cV = 50, 50
	hole.sideRadius, hole.capRadius = 10, 10.5
	hole.th0, hole.th1, hole.capTh0, hole.capTh1 = 0.2, 1.4, 0.25, 1.35

	for _, tc := range []struct {
		name string
		g    capPatchGeom
	}{
		{`tangent plate patch`, plate},
		{`far-placed plate patch`, far},
		{`skewed window`, skewed},
		{`major-arc window`, major},
		{`hole-style patch`, hole},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			g := tc.g
			iv, ok := conePatchFluxInterval(g)
			require.True(t, ok)

			H := g.capZ - g.sideZ
			envelope := math.Abs(H) * (math.Abs(g.sideRadius) + math.Abs(g.capRadius)) *
				(math.Abs(g.cU) + math.Abs(g.cV) + math.Abs(g.sideRadius) + math.Abs(g.capRadius) + math.Abs(g.sideZ))
			width, _ := new(big.Rat).Sub(iv.Hi, iv.Lo).Float64()
			require.LessOrEqual(t, width, 1e-24*(1+envelope),
				`%s: the enclosure (%v wide) must stay at the radian grid's own level`, tc.name, width)

			want := referenceConeFlux(g)
			mid, _ := intervalMid(iv).Float64()
			require.LessOrEqual(t, math.Abs(mid-want), 1e-9*(1+envelope),
				`%s: the enclosure's midpoint (%v) must match the float closed form (%v)`, tc.name, mid, want)

			// patchRawFlux holds the enclosure's midpoint and publishes at
			// least the enclosure's reach from it, which is never zero.
			flux := patchRawFlux(g)
			require.Equal(t, mid, flux.Value, `%s: patchRawFlux must hold the enclosure's midpoint`, tc.name)
			require.Greater(t, flux.Bound, 0.0, `%s: patchRawFlux's bound must be positive`, tc.name)
			require.GreaterOrEqual(t, flux.Bound, proofbound.IntervalFloatError(iv, mid),
				`%s: patchRawFlux's bound (%v) must cover the enclosure's reach`, tc.name, flux.Bound)
		})
	}
}

// referenceConeFlux is the float closed form patchRawFlux's non-whole-turn
// fallback evaluates (poly + origin + cross·ruledAngleCos), copied term for
// term.
func referenceConeFlux(g capPatchGeom) float64 {
	R0, R1 := g.sideRadius, g.capRadius
	z0, z1 := g.sideZ, g.capZ
	thS0, thS1 := g.th0, g.th1
	thC0, thC1 := g.capTh0, g.capTh1
	H := z1 - z0
	dS := thS1 - thS0
	dC := thC1 - thC0
	sinS0, cosS0 := math.Sincos(thS0)
	sinS1, cosS1 := math.Sincos(thS1)
	sinC0, cosC0 := math.Sincos(thC0)
	sinC1, cosC1 := math.Sincos(thC1)
	originR0 := H / 2 * R0 * (g.cU*(sinS1-sinS0) + g.cV*(cosS0-cosS1))
	originR1 := H / 2 * R1 * (g.cU*(sinC1-sinC0) + g.cV*(cosC0-cosC1))
	origin := originR0 + originR1
	dR := R1 - R0
	dSC := dS - dC
	polyZ0 := R1*R1*dSC - dS*dR*(R0+R1)
	poly := z0*polyZ0/2 + R0*R0*H*dS/2
	crossZ0 := -R0 * R1 * dSC
	cross := z0*crossZ0/2 + R0*R1*H*dC/2
	intCos := ruledAngleCos(thS0, thS1, thC0, thC1)
	return poly + origin + cross*intCos
}

// referencePhaseTerm is phaseTerm with float coefficients, the type the float
// reference below builds.
type referencePhaseTerm struct {
	k, m   int
	ac, as float64
}

// referenceMomentTermsX/Y/Z are the float transcriptions of
// coneMomentTermsX/Y/Z, kept verbatim as the reference the rational
// coefficients are checked against.
func referenceMomentTermsX(R0, R1, H, cU, dS, dC float64) []referencePhaseTerm {
	return []referencePhaseTerm{
		{0, 0, H * cU * (R0*R0*dS + R1*R1*dC) / 6, 0},
		{0, 1, H * R1 * (2*R0*R0*dC + 4*R0*R0*dS + 9*R1*R1*dC + 24*cU*cU*dC) / 96, 0},
		{1, 0, H * R0 * (9*R0*R0*dS + 4*R1*R1*dC + 2*R1*R1*dS + 24*cU*cU*dS) / 96, 0},
		{0, 2, H * R1 * R1 * cU * dC / 6, 0},
		{1, -1, H * R0 * R1 * cU * (dC + dS) / 12, 0},
		{1, 1, H * R0 * R1 * cU * (dC + dS) / 12, 0},
		{2, 0, H * R0 * R0 * cU * dS / 6, 0},
		{0, 3, H * R1 * R1 * R1 * dC / 32, 0},
		{1, -2, H * R0 * R1 * R1 * (2*dC + dS) / 96, 0},
		{1, 2, H * R0 * R1 * R1 * (2*dC + dS) / 96, 0},
		{2, -1, H * R0 * R0 * R1 * (dC + 2*dS) / 96, 0},
		{2, 1, H * R0 * R0 * R1 * (dC + 2*dS) / 96, 0},
		{3, 0, H * R0 * R0 * R0 * dS / 32, 0},
	}
}

func referenceMomentTermsY(R0, R1, H, cV, dS, dC float64) []referencePhaseTerm {
	return []referencePhaseTerm{
		{0, 0, H * cV * (R0*R0*dS + R1*R1*dC) / 6, 0},
		{0, 1, 0, H * R1 * (2*R0*R0*dC + 4*R0*R0*dS + 9*R1*R1*dC + 24*cV*cV*dC) / 96},
		{1, 0, 0, H * R0 * (9*R0*R0*dS + 4*R1*R1*dC + 2*R1*R1*dS + 24*cV*cV*dS) / 96},
		{0, 2, -H * R1 * R1 * cV * dC / 6, 0},
		{1, -1, H * R0 * R1 * cV * (dC + dS) / 12, 0},
		{1, 1, -H * R0 * R1 * cV * (dC + dS) / 12, 0},
		{2, 0, -H * R0 * R0 * cV * dS / 6, 0},
		{0, 3, 0, -H * R1 * R1 * R1 * dC / 32},
		{1, -2, 0, -H * R0 * R1 * R1 * (2*dC + dS) / 96},
		{1, 2, 0, -H * R0 * R1 * R1 * (2*dC + dS) / 96},
		{2, -1, 0, H * R0 * R0 * R1 * (dC + 2*dS) / 96},
		{2, 1, 0, -H * R0 * R0 * R1 * (dC + 2*dS) / 96},
		{3, 0, 0, -H * R0 * R0 * R0 * dS / 32},
	}
}

func referenceMomentTermsZ(R0, R1, H, z0, dS, dC float64) []referencePhaseTerm {
	return []referencePhaseTerm{
		{0, 0, (H*H*R0*R0*dS - 3*H*H*R1*R1*dC + 4*H*R0*R0*dS*z0 - 8*H*R1*R1*dC*z0 + 6*R0*R0*dS*z0*z0 - 6*R1*R1*dC*z0*z0) / 24, 0},
		{1, -1, R0 * R1 * (3*H*H*dC - H*H*dS + 8*H*dC*z0 - 4*H*dS*z0 + 6*dC*z0*z0 - 6*dS*z0*z0) / 24, 0},
	}
}
