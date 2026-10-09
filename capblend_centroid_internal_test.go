package decad

import (
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"

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
	check := func(t *testing.T, label string, got []capband.PhaseTerm, want []referencePhaseTerm) {
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
		check(t, label+` X`, capband.ConeMomentTermsX(R0, R1, H, c, dS, dC),
			referenceMomentTermsX(tc.R0, tc.R1, tc.H, tc.c, tc.dS, tc.dC))
		check(t, label+` Y`, capband.ConeMomentTermsY(R0, R1, H, c, dS, dC),
			referenceMomentTermsY(tc.R0, tc.R1, tc.H, tc.c, tc.dS, tc.dC))
		// The far level is -1000, not -1e6: at -1e6 the float reference loses
		// about 3e-10 relative to cancellation of its z0² terms, while the
		// rational stays exact.
		for _, z0 := range []float64{15.5, -1000} {
			check(t, fmt.Sprintf(`%s Z z0=%g`, label, z0), capband.ConeMomentTermsZ(R0, R1, H, proofarith.FloatRat(z0), dS, dC),
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
		Circular: true, SweepCCW: true,
		CU: 36, CV: 22, SideRadius: 12, CapRadius: 11.5,
		Th0: 0, Th1: math.Pi / 2, CapTh0: 0, CapTh1: math.Pi / 2,
		SideZ: 15.5, CapZ: 16,
	}
	far := plate
	far.CU, far.CV = 988, 1000
	skewed := plate
	skewed.Th0, skewed.Th1, skewed.CapTh0, skewed.CapTh1 = 0.3, 2.1, 0.35, 2.05
	major := plate
	major.Th0, major.Th1, major.CapTh0, major.CapTh1 = -2.9, 2.9, -2.85, 2.85
	hole := plate
	hole.CU, hole.CV = 50, 50
	hole.SideRadius, hole.CapRadius = 10, 10.5
	hole.Th0, hole.Th1, hole.CapTh0, hole.CapTh1 = 0.2, 1.4, 0.25, 1.35

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
			iv, ok := capband.ConeFluxInterval(g)
			require.True(t, ok)

			H := g.CapZ - g.SideZ
			envelope := math.Abs(H) * (math.Abs(g.SideRadius) + math.Abs(g.CapRadius)) *
				(math.Abs(g.CU) + math.Abs(g.CV) + math.Abs(g.SideRadius) + math.Abs(g.CapRadius) + math.Abs(g.SideZ))
			width, _ := new(big.Rat).Sub(iv.Hi, iv.Lo).Float64()
			require.LessOrEqual(t, width, 1e-24*(1+envelope),
				`%s: the enclosure (%v wide) must stay at the radian grid's own level`, tc.name, width)

			want := referenceConeFlux(g)
			mid, _ := intervalMid(iv).Float64()
			require.LessOrEqual(t, math.Abs(mid-want), 1e-9*(1+envelope),
				`%s: the enclosure's midpoint (%v) must match the float closed form (%v)`, tc.name, mid, want)

			// patchRawFlux holds the enclosure's midpoint and publishes at
			// least the enclosure's reach from it, which is never zero.
			flux := capband.RawFlux(g)
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
	R0, R1 := g.SideRadius, g.CapRadius
	z0, z1 := g.SideZ, g.CapZ
	thS0, thS1 := g.Th0, g.Th1
	thC0, thC1 := g.CapTh0, g.CapTh1
	H := z1 - z0
	dS := thS1 - thS0
	dC := thC1 - thC0
	sinS0, cosS0 := math.Sincos(thS0)
	sinS1, cosS1 := math.Sincos(thS1)
	sinC0, cosC0 := math.Sincos(thC0)
	sinC1, cosC1 := math.Sincos(thC1)
	originR0 := H / 2 * R0 * (g.CU*(sinS1-sinS0) + g.CV*(cosS0-cosS1))
	originR1 := H / 2 * R1 * (g.CU*(sinC1-sinC0) + g.CV*(cosC0-cosC1))
	origin := originR0 + originR1
	dR := R1 - R0
	dSC := dS - dC
	polyZ0 := R1*R1*dSC - dS*dR*(R0+R1)
	poly := z0*polyZ0/2 + R0*R0*H*dS/2
	crossZ0 := -R0 * R1 * dSC
	cross := z0*crossZ0/2 + R0*R1*H*dC/2
	intCos := capband.RuledAngleCos(thS0, thS1, thC0, thC1)
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

// regionPatch is a Cone patch read in its own centre's frame: radius r0 at
// v = 0 over the side window [s0, s1], radius r1 at v = 1 over the cap window
// [c0, c1] (put on the side window's branch), and the straight homotopy
// between the wide sector and the built ruled patch.
type regionPatch struct {
	r0, r1, h      float64
	s0, s1, c0, c1 float64
}

func regionPatchOf(g capPatchGeom) regionPatch {
	c0, c1 := capband.WindowOnBranch(g.CapTh0, g.CapTh1, g.Th0)
	return regionPatch{r0: g.SideRadius, r1: g.CapRadius, h: g.CapZ - g.SideZ, s0: g.Th0, s1: g.Th1, c0: c0, c1: c1}
}

// at returns H_λ(u, v)'s plane-local position about the centre.
func (p regionPatch) at(lambda, u, v float64) (float64, float64) {
	ts, tc := p.s0+u*(p.s1-p.s0), p.c0+u*(p.c1-p.c0)
	x := (1-v)*p.r0*math.Cos(ts) + v*p.r1*((1-lambda)*math.Cos(ts)+lambda*math.Cos(tc))
	y := (1-v)*p.r0*math.Sin(ts) + v*p.r1*((1-lambda)*math.Sin(ts)+lambda*math.Sin(tc))
	return x, y
}

// azimuth is the angle of (x, y) measured from s0, read without a branch cut
// inside the patch's window.
func (p regionPatch) azimuth(x, y float64) float64 {
	c, s := math.Cos(p.s0), math.Sin(p.s0)
	return p.s0 + math.Atan2(c*y-s*x, c*x+s*y)
}

// radiusAt returns the radius at which H_λ's level-v curve crosses azimuth
// theta, found by bisection on u: the curve's azimuth increases with u.
func (p regionPatch) radiusAt(lambda, v, theta float64) float64 {
	lo, hi := 0.0, 1.0
	for range 200 {
		mid := (lo + hi) / 2
		if p.azimuth(p.at(lambda, mid, v)) < theta {
			lo = mid
		} else {
			hi = mid
		}
	}
	return math.Hypot(p.at(lambda, (lo+hi)/2, v))
}

// TestChordLocusRegionPointsLieInTheShell samples the crossings
// proofbound.ChordLocusRegionAllow's proof bounds and checks each lies in the
// shell under the cone that the region term charges, on the narrow window of
// capband's chord-locus tests (R = 10, window 0.001 rad, trimmed 2e-4 rad at
// each corner) and on the quarter disk R = 60 chamfered 4 mm.
//
// At a middle azimuth, between the two corner wedges, the built patch must
// cross the ray within capband.ChordLocusBuiltDeficit inside the cone radius
// and not outside it; the samples cover the window at ten heights. At an
// azimuth in a corner wedge, every point of the built patch and of the sliver
// surface joining the corner-foot locus to the built ruling at equal heights
// must lie within capband.ChordLocusCornerDeficit inside the cone radius and
// not outside it. On the quarter disk the locus is the line foot
// (√((R−t)² − t²), t) and its mirror; on the narrow window, whose locus is
// not modelled, the sliver is sampled from both ends of the corner wedge.
//
// Shown to fail on 2026-10-09: with ChordLocusBuiltDeficitUpper's 1/4 cut to
// 1/10, the quarter disk's middle samples leave the shell (its worst sits
// 0.44 of the bound inside the cone), and with ChordLocusCornerDeficitUpper's
// 3/8 cut to 1/10, the narrow window's sliver samples and the quarter disk's
// built-patch samples do.
func TestChordLocusRegionPointsLieInTheShell(t *testing.T) {
	t.Parallel()
	narrow := capPatchGeom{
		Circular: true, SweepCCW: true,
		SideRadius: 10, CapRadius: 9,
		Th0: 0.3, Th1: 0.301, CapTh0: 0.3 + 2e-4, CapTh1: 0.301 - 2e-4,
		SideZ: 19, CapZ: 20,
	}
	at := func(r, th float64) Point2 { return Point2{U: r * math.Cos(th), V: r * math.Sin(th)} }
	var ok bool
	narrow.SkewStart, ok = capband.CornerSkewUpper(0, 0, at(10, narrow.Th0), at(9, narrow.CapTh0), narrow.CapTh0-narrow.Th0)
	require.True(t, ok)
	narrow.SkewEnd, ok = capband.CornerSkewUpper(0, 0, at(10, narrow.Th1), at(9, narrow.CapTh1), narrow.CapTh1-narrow.Th1)
	require.True(t, ok)

	const qR, qD = 60.0, 4.0
	quarter := chamferedCircularBand(t, quarterDiskSection(qR), 20, qD, nil).geom
	require.Positive(t, capPatchWindowSkew(quarter), `the quarter disk's wall is mitered at both ends`)
	// quarterLocus is the corner-foot locus at height fraction v: the foot
	// of the line through the corner offset by t = v·d, on the circle
	// offset by t.
	quarterLocus := func(p regionPatch, v float64, start bool) []float64 {
		t := v * qD
		foot := math.Sqrt((qR-t)*(qR-t) - t*t)
		a, b := p.azimuth(foot, t), p.azimuth(t, foot)
		if math.Abs(a-p.s0) > math.Abs(b-p.s0) {
			a, b = b, a
		}
		if start {
			return []float64{a}
		}
		return []float64{b}
	}

	for _, tc := range []struct {
		name  string
		g     capPatchGeom
		locus func(p regionPatch, v float64, start bool) []float64
	}{
		{name: `narrow window`, g: narrow, locus: func(p regionPatch, _ float64, start bool) []float64 {
			if start {
				return []float64{p.s0, p.c0}
			}
			return []float64{p.c1, p.s1}
		}},
		{name: `quarter disk`, g: quarter, locus: quarterLocus},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			p := regionPatchOf(tc.g)
			require.Less(t, p.s0, p.c0)
			require.Less(t, p.c1, p.s1)
			built := capband.ChordLocusBuiltDeficit(tc.g)
			deficit := capband.ChordLocusCornerDeficit(tc.g)
			worst := 0.0
			rAt := func(v float64) float64 { return p.r0 + (p.r1-p.r0)*v }

			for i := range 10 {
				v := 0.05 + 0.1*float64(i)
				for j := range 21 {
					theta := p.c0 + float64(j)/20*(p.c1-p.c0)
					outer, inner := rAt(v), p.radiusAt(1, v, theta)
					require.LessOrEqual(t, inner, outer*(1+1e-12), `the built patch lies inside the cone`)
					require.GreaterOrEqual(t, inner, outer-built,
						`v=%v θ=%v: the built patch at radius %v leaves the shell [%v, %v]`, v, theta, inner, outer-built, outer)
					worst = math.Max(worst, (outer-inner)/built)
				}
			}
			require.Positive(t, worst, `the built patch dips inside the cone at a middle azimuth`)

			for _, v := range []float64{0.1, 0.5, 0.9} {
				r := rAt(v)
				for _, start := range []bool{true, false} {
					u := 0.0
					if !start {
						u = 1
					}
					qx, qy := p.at(1, u, v)
					// The built patch at corner azimuths: its level curve from
					// the ruling to the cap window's edge.
					edge := p.c0
					if !start {
						edge = p.c1
					}
					for _, w := range []float64{0, 0.5, 1} {
						theta := p.azimuth(qx, qy) + w*(edge-p.azimuth(qx, qy))
						rho := p.radiusAt(1, v, theta)
						require.LessOrEqual(t, rho, r*(1+1e-12), `the built patch lies inside the cone`)
						require.GreaterOrEqual(t, rho, r-deficit, `v=%v θ=%v: the built patch at radius %v leaves the shell [%v, %v]`, v, theta, rho, r-deficit, r)
					}
					for _, a := range tc.locus(p, v, start) {
						px, py := r*math.Cos(a), r*math.Sin(a)
						for _, s := range []float64{0, 0.25, 0.5, 0.75, 1} {
							rho := math.Hypot(px+s*(qx-px), py+s*(qy-py))
							require.LessOrEqual(t, rho, r*(1+1e-12), `the sliver lies inside the cone`)
							require.GreaterOrEqual(t, rho, r-deficit,
								`v=%v locus azimuth %v s=%v: the sliver at radius %v leaves the shell [%v, %v]`, v, a, s, rho, r-deficit, r)
						}
					}
				}
			}
		})
	}
}

// TestCapBandCoordUpperCoversTheCornerLoci checks the claim the band's
// first-moment coordinate reach rests on, on a real build: a 100 x 100 plate
// with a quarter-sector hole (centre (50, 50), radius 20, from the +u axis to
// the +v axis), its end cap chamfered 4 mm. The hole's material lies outside
// it, so its offset family grows outward: at offset t the lines through the
// centre move to u = 50 − t and v = 50 − t and the arc to radius 20 + t, and
// the foot where a line meets the arc runs from (70, 50) out to
// (50 + √(24² − 4²), 46) ≈ (73.7, 46), past every coordinate of the hole
// itself. Each foot must lie between the hole and the cap contour the build
// records (cbp.contourOf): on a circle no smaller than the hole's arc and no
// larger than the contour's, and on a line between the hole's and the
// contour's. Each foot must also lie within capband.CoordUpper, the bound the
// first-moment terms read.
//
// Shown to fail on 2026-10-09: with the cap contour taken at dc/2, the feet
// past t = 2 lie outside it. The coordinate bound alone does not discriminate
// here: a recorded arc's envelope |cu| + |cv| + 2R already reaches past the
// feet.
func TestCapBandCoordUpperCoversTheCornerLoci(t *testing.T) {
	t.Parallel()
	const cx, cy, r, d = 50.0, 50.0, 20.0, 4.0
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 100)
	s.Fix(rect.A)
	o := s.CreatePoint(cx, cy)
	px := s.CreatePoint(cx+r, cy)
	py := s.CreatePoint(cx, cy+r)
	s.Fix(o)
	s.Fix(px)
	s.Fix(py)
	s.CreateLine(o, px)
	s.CreateLine(py, o)
	s.CreateArc(o, px, py)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	prof := s.Profiles()[0]
	for _, p := range s.Profiles() {
		if len(p.Holes) > len(prof.Holes) {
			prof = p
		}
	}
	require.Len(t, prof.Holes, 1)
	body, err := New().Extrude(s, prof, Distance{D: units.Millimeters(20), Dir: Along})
	require.NoError(t, err)
	chamfered, err := body.Chamfer(t.Context(), Edges(CreatedBy(CapEnd(body))), units.Millimeters(d))
	require.NoError(t, err)
	cbp, ok := chamfered.payload.(capBlendPayload)
	require.True(t, ok)
	require.Len(t, cbp.profile.Holes, 1)

	loop := cbp.profile.Holes[0]
	const matSign = -1.0
	setback := cbp.setbackAt(matSign)
	require.Equal(t, d, setback.dc)
	capBoundary, err := cbp.contourOf(t.Context(), 0, loop, setback.dc)
	require.NoError(t, err)
	delta, ok := cbp.bandDelta[capBandKey{loop: 1, start: false}]
	require.True(t, ok, `the hole's end-cap band records its contour displacement`)
	capZB := cbp.capBandLevel(cbp.z1, matSign)
	sideZB := proofbound.BoundedAdd(capZB, proofbound.MeasuredScalar(matSign*setback.ds, setback.dsDelta))
	coordUpper, err := capband.CoordUpper(loop, capBoundary, delta, sideZB, capZB, freeform.NewFreeformWork())
	require.NoError(t, err)

	// The contour's arc radius and its two straight walls' offset positions.
	capRadius, capLineU, capLineV := 0.0, math.Inf(1), math.Inf(1)
	for _, seg := range capBoundary.Segments {
		switch sg := seg.(type) {
		case arcSeg:
			// The reflex corner at the centre adds a connector arc of radius
			// d; the offset wall is the larger arc.
			capRadius = math.Max(capRadius, math.Hypot(sg.Start.U-sg.Center.U, sg.Start.V-sg.Center.V))
		case lineSeg:
			if sg.Start.U == sg.End.U {
				capLineU = math.Min(capLineU, sg.Start.U)
			}
			if sg.Start.V == sg.End.V {
				capLineV = math.Min(capLineV, sg.Start.V)
			}
		}
	}
	require.Positive(t, capRadius, `the cap contour records the offset arc`)
	const tol = 1e-9

	reach := 0.0
	for i := range 17 {
		off := d * float64(i) / 16
		foot := math.Sqrt((r+off)*(r+off) - off*off)
		for k, p := range [][2]float64{
			{cx + foot, cy - off}, // the +u line meets the arc
			{cx - off, cy + foot}, // the arc meets the +v line
			{cx - off, cy - off},  // the two lines meet
		} {
			if k < 2 {
				// A line-arc foot sits on the circle of radius 20 + off.
				dist := math.Hypot(p[0]-cx, p[1]-cy)
				require.GreaterOrEqual(t, dist, r-tol, `offset %v: the foot lies outside the hole's arc`, off)
				require.LessOrEqual(t, dist, capRadius+tol,
					`offset %v: the foot at radius %v must lie inside the contour's arc %v`, off, dist, capRadius)
			}
			require.GreaterOrEqual(t, math.Min(p[0], p[1]), math.Min(capLineU, capLineV)-tol,
				`offset %v: the foot (%v, %v) must lie inside the contour's straight walls`, off, p[0], p[1])
			m := math.Max(math.Abs(p[0]), math.Abs(p[1]))
			reach = math.Max(reach, m)
			require.LessOrEqual(t, m, coordUpper,
				`offset %v: the corner foot (%v, %v) must lie within the coordinate bound %v`, off, p[0], p[1], coordUpper)
		}
	}
	require.Greater(t, reach, cx+r, `the loci reach past the hole's own coordinates`)
}

// TestChordLocusSpanFluxEnclosesTheDenotedFlux checks the denoted surface's
// flux enclosure the chord-versus-locus volume term reads
// (capband.ChordLocusSpanFlux) on the quarter disk R = 60 chamfered 4 mm, a
// real build whose two corners carry locus spans. The reference integrates
// the cone's flux density about the arc's axis, r(v)·R0·h per unit angle and
// height fraction, over the window between the two corner-foot loci, which
// for a corner where a line through the centre meets the arc is the foot
// (√((R−t)² − t²), t) at offset t = v·d. The enclosure must hold the
// reference and be at most an eighth as wide as the sandwich between the
// wide and narrow sectors.
//
// Shown to fail on 2026-10-09: with chordLocusSpanFlux leaving out the end
// corner's share, the enclosure misses the reference.
func TestChordLocusSpanFluxEnclosesTheDenotedFlux(t *testing.T) {
	t.Parallel()
	const qR, qD = 60.0, 4.0
	g := chamferedCircularBand(t, quarterDiskSection(qR), 20, qD, nil).geom
	require.NotEmpty(t, g.Locus0)
	require.NotEmpty(t, g.Locus1)

	sideZ, capZ := capband.AxisAnchoredLevels(g.SideZ, g.CapZ)
	h := capZ - sideZ
	// Composite Simpson's rule over 2000 intervals; the integrand is smooth
	// on [0, 1], so its error sits far below the tolerance.
	const n = 2000
	ref := 0.0
	for i := range n + 1 {
		v := float64(i) / n
		w := 2.0
		switch {
		case i == 0 || i == n:
			w = 1
		case i%2 == 1:
			w = 4
		}
		off := v * qD
		foot := math.Sqrt((qR-off)*(qR-off) - off*off)
		phi := math.Atan2(off, foot)
		r := g.SideRadius + (g.CapRadius-g.SideRadius)*v
		width := (g.Th1 - g.Th0) - 2*phi
		ref += w / (3 * n) * r * (g.SideRadius*h - sideZ*(g.CapRadius-g.SideRadius)) * width
	}
	if !g.SweepCCW {
		ref = -ref
	}

	span, ok := capband.ChordLocusSpanFlux(g)
	require.True(t, ok)
	lo, _ := span.Lo.Float64()
	hi, _ := span.Hi.Float64()
	tol := 1e-12 * math.Abs(ref)
	require.True(t, lo-tol <= ref && ref <= hi+tol, `the span enclosure [%v, %v] must hold the denoted flux %v`, lo, hi, ref)

	wide, narrow, _ := capband.ChordLocusFluxes(g)
	require.LessOrEqual(t, hi-lo, math.Abs(wide.Value-narrow.Value)/8,
		`the span enclosure (%v wide) must be far tighter than the sandwich (%v)`, hi-lo, math.Abs(wide.Value-narrow.Value))
}

// TestSegmentCoordinateUpperCoversTheArc checks the local coordinate envelope
// capband.CoordUpper reads for an arc (capband.SegmentCoordinateUpper) against the arc
// itself, over a randomized sweep of centres, radii and start and end
// directions, minor and major arcs and arcs crossing every axis: every point
// sampled along the counter-clockwise sweep must lie within the envelope, and
// the envelope must sit within a few ulps of the arc's own largest coordinate
// magnitude, read from its two ends and every axis direction the sweep
// passes. A line's envelope is its larger end.
//
// Shown to fail on 2026-10-09: with the axis directions left out, sampled
// points past the ends' reach leave the envelope; with the envelope read as
// the walk's |cu| + |cv| + 2R, the tightness check fails.
func TestSegmentCoordinateUpperCoversTheArc(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(43, 47))
	for range 500 {
		cu, cv := (rng.Float64()-0.5)*200, (rng.Float64()-0.5)*200
		r := 0.5 + rng.Float64()*50
		a0 := rng.Float64() * 2 * math.Pi
		sweep := 0.01 + rng.Float64()*(2*math.Pi-0.02)
		a1 := a0 + sweep
		// End is pinned off the circle on purpose: the arc ends at radius r
		// in its direction.
		endScale := 0.5 + rng.Float64()
		seg := arcSeg{
			Center: Point2{U: cu, V: cv},
			Start:  Point2{U: cu + r*math.Cos(a0), V: cv + r*math.Sin(a0)},
			End:    Point2{U: cu + endScale*r*math.Cos(a1), V: cv + endScale*r*math.Sin(a1)},
			TStart: 0, TEnd: 1,
		}
		got, ok := capband.SegmentCoordinateUpper(seg)
		require.True(t, ok)
		radius := math.Hypot(seg.Start.U-cu, seg.Start.V-cv)
		th0 := math.Atan2(seg.Start.V-cv, seg.Start.U-cu)
		th1 := math.Atan2(seg.End.V-cv, seg.End.U-cu)
		span := math.Mod(th1-th0, 2*math.Pi)
		if span <= 0 {
			span += 2 * math.Pi
		}
		at := func(th float64) float64 {
			return math.Max(math.Abs(cu+radius*math.Cos(th)), math.Abs(cv+radius*math.Sin(th)))
		}
		want := math.Max(at(th0), at(th0+span))
		for k := range 4 {
			axis := th0 + math.Mod(float64(k)*math.Pi/2-th0+4*math.Pi, 2*math.Pi)
			if axis <= th0+span {
				want = math.Max(want, at(axis))
			}
		}
		for i := range 400 {
			p := at(th0 + span*float64(i)/399)
			require.LessOrEqual(t, p, got*(1+1e-12),
				`centre (%v, %v) r=%v sweep from %v by %v: the point at %v leaves the envelope %v`, cu, cv, r, th0, span, p, got)
		}
		require.LessOrEqual(t, got, want*(1+1e-9)+1e-9,
			`centre (%v, %v) r=%v sweep from %v by %v: the envelope %v must sit at the arc's largest coordinate %v`, cu, cv, r, th0, span, got, want)
	}

	line := lineSeg{Start: Point2{U: -3, V: 1}, End: Point2{U: 2, V: -5}, TStart: 0, TEnd: 1}
	got, ok := capband.SegmentCoordinateUpper(line)
	require.True(t, ok)
	require.Equal(t, 5.0, got)
	_, ok = capband.SegmentCoordinateUpper(lineSeg{Start: Point2{}, End: Point2{U: 1}, TStart: -1, TEnd: 1})
	require.False(t, ok, `a range past the entity's own reads the walk's envelope instead`)
}

// TestChordLocusSliverCubeCoversTheSlivers checks the sliver term the region
// bound reads (capband.ChordLocusSliverCube) on the quarter disk R = 60
// chamfered 4 mm. At height fraction v each corner's sliver spans the angle
// between the corner-foot locus, the foot (√((R−t)² − t²), t) at t = 4v at
// angle φ(v) from the corner's ray, and the built ruling's point
// (1−v)·R0 + v·R1·e^{i·φ(1)}. The term must cover Σ ∫ δ³ dv over both
// corners, integrated by Simpson's rule, and sit well under the skews'
// cubes it replaces. The true slivers are thin (about 1.4e-14 here); the
// term is set by the spans' own widths, about 2e-7, against the cubes'
// 7.3e-4.
//
// Shown to fail on 2026-10-09: with the sliver term answering zero, it falls
// below the reference.
func TestChordLocusSliverCubeCoversTheSlivers(t *testing.T) {
	t.Parallel()
	const qR, qD = 60.0, 4.0
	g := chamferedCircularBand(t, quarterDiskSection(qR), 20, qD, nil).geom
	require.NotEmpty(t, g.Locus0)
	phiAt := func(v float64) float64 {
		off := v * qD
		return math.Atan2(off, math.Sqrt((qR-off)*(qR-off)-off*off))
	}
	s := phiAt(1)
	r0, r1 := g.SideRadius, g.CapRadius
	const n = 2000
	ref := 0.0
	for i := range n + 1 {
		v := float64(i) / n
		w := 2.0
		switch {
		case i == 0 || i == n:
			w = 1
		case i%2 == 1:
			w = 4
		}
		psi := math.Atan2(v*r1*math.Sin(s), (1-v)*r0+v*r1*math.Cos(s))
		delta := math.Abs(phiAt(v) - psi)
		ref += w / (3 * n) * delta * delta * delta
	}
	ref *= 2 // the two corners mirror each other
	require.Positive(t, ref)

	got := capband.ChordLocusSliverCube(g)
	require.GreaterOrEqual(t, got, ref*(1-1e-9), `the sliver term %v must cover Σ ∫ δ³ dv = %v`, got, ref)
	fallback := proofbound.ChordLocusSliverCubeUpper(g.SkewStart, g.SkewEnd)
	require.LessOrEqual(t, got, fallback/8, `the spans must cut the skews' cubes %v by far, not to %v`, fallback, got)
}

// TestChordLocusRegionVolumeCoversTheDip checks the region volume the first
// moment reads (capband.ChordLocusVolume) against the region's measure on the
// quarter disk R = 60 chamfered 4 mm, computed from the geometry: at each
// height the built patch's level curve dips inside the cone, and over the
// azimuths where both the denoted surface (the cone over the window between
// the two corner-foot loci) and the built patch cross, the region's
// cross-section is ∫ (r² − |B|²)/2 dθ. The slivers between the loci and the
// rulings add a measure of order ∫ δ³, about 1e-14 here, which the reference
// leaves out. The term must cover the reference and sit within 8 times it
// (here 9.63 against 2.86, which matches the published volume's own residual
// against the erosion family).
//
// Shown to fail on 2026-10-09: with ChordLocusShellUpper's dip factor 1/12
// cut to 1/48, the term falls below the reference.
func TestChordLocusRegionVolumeCoversTheDip(t *testing.T) {
	t.Parallel()
	const qR, qD = 60.0, 4.0
	g := chamferedCircularBand(t, quarterDiskSection(qR), 20, qD, nil).geom
	phiAt := func(v float64) float64 {
		off := v * qD
		return math.Atan2(off, math.Sqrt((qR-off)*(qR-off)-off*off))
	}
	s := phiAt(1)
	r0, r1 := g.SideRadius, g.CapRadius
	h := math.Abs(g.CapZ - g.SideZ)
	th0, th1 := g.Th0, g.Th1
	c0, c1 := th0+s, th1-s
	section := func(v float64) float64 {
		r := r0 + (r1-r0)*v
		lo, hi := th0+phiAt(v), th1-phiAt(v)
		const m = 4000
		area := 0.0
		prevTh, prevF := 0.0, 0.0
		for k := range m + 1 {
			u := float64(k) / m
			ts, tc := th0+u*(th1-th0), c0+u*(c1-c0)
			x := (1-v)*r0*math.Cos(ts) + v*r1*math.Cos(tc)
			y := (1-v)*r0*math.Sin(ts) + v*r1*math.Sin(tc)
			th := math.Atan2(y, x)
			f := 0.0
			if th >= lo && th <= hi {
				f = (r*r - (x*x + y*y)) / 2
			}
			if k > 0 {
				area += (th - prevTh) * (f + prevF) / 2
			}
			prevTh, prevF = th, f
		}
		return area
	}
	const n = 200
	ref := 0.0
	for i := range n + 1 {
		v := float64(i) / n
		w := 2.0
		switch {
		case i == 0 || i == n:
			w = 1
		case i%2 == 1:
			w = 4
		}
		ref += w / (3 * n) * section(v)
	}
	ref *= h
	require.Positive(t, ref)

	got, _ := capband.ChordLocusVolume(g)
	require.GreaterOrEqual(t, got, ref*(1-1e-6), `the region volume %v must cover the dip's measure %v`, got, ref)
	require.LessOrEqual(t, got, 8*ref, `the region volume %v must sit within 8 times the dip's measure %v`, got, ref)
}
