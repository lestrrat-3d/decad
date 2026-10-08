package decad

import (
	"fmt"
	"math"
	"math/big"
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

// area integrates H_λ's area by tensor Gauss-Legendre quadrature.
func (p regionPatch) area(lambda float64) float64 {
	const n = 48
	xs, ws := gaussLegendreNodes(n)
	total := 0.0
	const eps = 1e-6
	for i, u := range xs {
		for j, v := range xs {
			ux0, uy0 := p.at(lambda, u-eps, v)
			ux1, uy1 := p.at(lambda, u+eps, v)
			vx0, vy0 := p.at(lambda, u, v-eps)
			vx1, vy1 := p.at(lambda, u, v+eps)
			du := [3]float64{(ux1 - ux0) / (2 * eps), (uy1 - uy0) / (2 * eps), 0}
			dv := [3]float64{(vx1 - vx0) / (2 * eps), (vy1 - vy0) / (2 * eps), p.h}
			nx := du[1]*dv[2] - du[2]*dv[1]
			ny := du[2]*dv[0] - du[0]*dv[2]
			nz := du[0]*dv[1] - du[1]*dv[0]
			total += ws[i] * ws[j] * math.Sqrt(nx*nx+ny*ny+nz*nz)
		}
	}
	return total
}

// gaussLegendreNodes returns the n-point Gauss-Legendre nodes and weights on
// [0, 1].
func gaussLegendreNodes(n int) ([]float64, []float64) {
	x := make([]float64, n)
	w := make([]float64, n)
	for i := range n {
		z := math.Cos(math.Pi * (float64(i) + 0.75) / (float64(n) + 0.5))
		var dp float64
		for range 100 {
			p1, p2 := 1.0, 0.0
			for j := 1; j <= n; j++ {
				p1, p2 = ((2*float64(j)-1)*z*p1-(float64(j)-1)*p2)/float64(j), p1
			}
			dp = float64(n) * (z*p1 - p2) / (z*z - 1)
			next := z - p1/dp
			if math.Abs(next-z) < 1e-16 {
				z = next
				break
			}
			z = next
		}
		x[i] = (1 - z) / 2
		w[i] = 1 / ((1 - z*z) * dp * dp)
	}
	return x, w
}

// TestChordLocusRegionPointsLieOnCoveredSurfaces samples the region between
// a Cone patch's wide sector and its built ruled patch and checks the two
// containments proofbound.ChordLocusRegionAllow's proof rests on, on the
// narrow window of capband's chord-locus tests (R = 10, window 0.001 rad,
// trimmed 2e-4 rad at each corner) and on the quarter disk R = 60 chamfered
// 4 mm.
//
// At a middle azimuth, between the two corner wedges, every radius between
// the built patch's crossing and the cone's lies on some homotopy surface
// H_λ: the test finds λ by bisection, checks the surface passes through the
// point, and checks the patch's homotopy area bound
// (capband.ChordLocusHomotopyArea) covers that surface's area. At an azimuth
// in a corner wedge, every point of the built patch and of the sliver
// surface joining the corner-foot locus to the built ruling at equal heights
// must lie within capband.ChordLocusCornerDeficit inside the cone radius and
// not outside it. On the quarter disk the locus is the line foot
// (√((R−t)² − t²), t) and its mirror; on the narrow window, whose locus is
// not modelled, the sliver is sampled from both ends of the corner wedge.
//
// Shown to fail on 2026-10-09: with ChordLocusHomotopyArea answering the
// built patch's capband.AreaOf value plus bound, the narrow window's surfaces
// up to λ ≈ 0.75 exceed it, and with ChordLocusCornerDeficitUpper's 3/8 cut
// to 1/10, the narrow window's sliver samples and the quarter disk's
// built-patch samples fall outside the shell (the narrow window's worst
// sliver sample sits 0.123·R·Φ² inside the cone).
func TestChordLocusRegionPointsLieOnCoveredSurfaces(t *testing.T) {
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
			areaBound := capband.ChordLocusHomotopyArea(tc.g)
			deficit := capband.ChordLocusCornerDeficit(tc.g)
			rAt := func(v float64) float64 { return p.r0 + (p.r1-p.r0)*v }

			areas := map[float64]float64{}
			for _, v := range []float64{0.1, 0.5, 0.9} {
				// A symmetric trim leaves the middle ruling exact, so the samples
				// avoid the window's centre.
				for _, f := range []float64{0.1, 0.3, 0.8} {
					theta := p.c0 + f*(p.c1-p.c0)
					outer, inner := rAt(v), p.radiusAt(1, v, theta)
					require.Less(t, inner, outer, `the built patch dips inside the cone at a middle azimuth`)
					for _, s := range []float64{0.25, 0.5, 0.75} {
						rho := inner + s*(outer-inner)
						lo, hi := 0.0, 1.0
						for range 200 {
							mid := (lo + hi) / 2
							if p.radiusAt(mid, v, theta) > rho {
								lo = mid
							} else {
								hi = mid
							}
						}
						lambda := (lo + hi) / 2
						require.InDelta(t, rho, p.radiusAt(lambda, v, theta), 1e-9*outer,
							`v=%v θ=%v ρ=%v: H_λ at λ=%v must pass through the point`, v, theta, rho, lambda)
						a, seen := areas[lambda]
						if !seen {
							a = p.area(lambda)
							areas[lambda] = a
						}
						require.LessOrEqual(t, a, areaBound,
							`v=%v θ=%v ρ=%v: H_λ at λ=%v has area %v past the bound %v`, v, theta, rho, lambda, a, areaBound)
					}
				}
			}

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
// contour's. Each foot must also lie within capBandCoordUpper, the bound the
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
	capBoundary, err := cbp.contourOf(t.Context(), loop, setback.dc)
	require.NoError(t, err)
	delta, ok := cbp.bandDelta[capBandKey{loop: 1, start: false}]
	require.True(t, ok, `the hole's end-cap band records its contour displacement`)
	capZB := cbp.capBandLevel(cbp.z1, matSign)
	sideZB := proofbound.BoundedAdd(capZB, proofbound.MeasuredScalar(matSign*setback.ds, setback.dsDelta))
	coordUpper, err := capBandCoordUpper(loop, capBoundary, delta, sideZB, capZB, freeform.NewFreeformWork())
	require.NoError(t, err)

	// The contour's arc radius and its two straight walls' offset positions.
	capRadius, capLineU, capLineV := 0.0, math.Inf(1), math.Inf(1)
	for _, seg := range capBoundary.Segments {
		switch sg := seg.(type) {
		case ArcSeg:
			// The reflex corner at the centre adds a connector arc of radius
			// d; the offset wall is the larger arc.
			capRadius = math.Max(capRadius, math.Hypot(sg.Start.U-sg.Center.U, sg.Start.V-sg.Center.V))
		case LineSeg:
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
