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

// TestChordLocusVolumeChargesTheProvenCornerSkew checks the region term the
// first moment reads (capband.ChordLocusVolume) reads the patch's proven
// corner skews, not the difference of its two held windows. Here the held
// windows coincide to the last bit, which is what two float Atan2 readings can
// return for corner ends whose exact angle differs, and the proven skews are
// positive. The volume must carry the region term's shell at the proven
// skews, H·R·(w·R·Φ²/4 + (s0 + s1)·(3/8)·R·Φ²), with R the larger radius, w
// the window width and H the band height.
//
// Shown to fail on 2026-10-09: with chordLocusRegionAllow reading
// max(capTh0 − th0, th1 − capTh1) for the skews, the volume is zero, and with
// proofbound.ChordLocusRegionAllow composing the shell without scaling it by
// 3 to flux, the volume is a third of the shell.
func TestChordLocusVolumeChargesTheProvenCornerSkew(t *testing.T) {
	t.Parallel()
	base := capband.Patch{
		Circular: true, SweepCCW: true,
		SideRadius: 10, CapRadius: 9,
		Th0: 0.2, Th1: 1.4, CapTh0: 0.2, CapTh1: 1.4,
		SideZ: 19, CapZ: 20,
	}
	skewed := base
	skewed.SkewStart, skewed.SkewEnd = 3e-7, 5e-7

	require.Equal(t, capband.RawFlux(base).Value, capband.RawFlux(skewed).Value, `the skew moves bounds, never the held flux`)
	plain, _ := capband.ChordLocusVolume(base)
	require.Zero(t, plain, `coinciding windows with zero skews charge nothing`)

	charged, _ := capband.ChordLocusVolume(skewed)
	shell := proofbound.ChordLocusShellUpper(skewed.SideRadius, skewed.Th1-skewed.Th0,
		skewed.SkewStart, skewed.SkewEnd, skewed.CapZ-skewed.SideZ)
	require.Positive(t, shell)
	require.GreaterOrEqual(t, charged, shell*(1-1e-12),
		`the skewed patch's region volume %v carries the shell %v`, charged, shell)
}

// TestRawFluxChargesTheCornerFlux checks a Cone patch's chord-locus term
// carries its corner slivers' flux (Patch.CornerFlux) on top of the rest of
// its bound, with the corner skews zero and with them positive, and that the
// first-moment reading ChordLocusVolume does not read it: the corner slivers
// lie in the region term's corner shell. The held flux does not move.
//
// Shown to fail on 2026-10-09: with chordLocusResidualAllow passing zero for
// the corner flux, every charged bound equals the plain one, and with
// chordLocusRegionAllow still adding the corner flux, the two patches'
// ChordLocusVolume readings differ.
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

			plainVol, _ := capband.ChordLocusVolume(plain)
			chargedVol, _ := capband.ChordLocusVolume(charged)
			require.Equal(t, plainVol, chargedVol, `the region volume holds the slivers in its shell, not the corner flux`)
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

// ruledFluxAboutAxis integrates the flux P·(Pu × Pv) of the ruled surface
// from the side arc (SideRadius over side0..side1 at level 0) to the cap arc
// (CapRadius over cap0..cap1 at level CapZ − SideZ), both angles linear in u,
// about the arc's axis at the side level. The integrand is a cubic in v and
// smooth in u, so tensor Gauss-Legendre at 32 nodes sits at float64 roundoff.
func ruledFluxAboutAxis(g capband.Patch, side0, side1, cap0, cap1 float64) float64 {
	xs, ws := gaussLegendre(32)
	as, ac := side1-side0, cap1-cap0
	r0, r1, h := g.SideRadius, g.CapRadius, g.CapZ-g.SideZ
	total := 0.0
	for i, u := range xs {
		ts, tc := side0+u*as, cap0+u*ac
		sx, sy := r0*math.Cos(ts), r0*math.Sin(ts)
		cx, cy := r1*math.Cos(tc), r1*math.Sin(tc)
		dsx, dsy := -as*r0*math.Sin(ts), as*r0*math.Cos(ts)
		dcx, dcy := -ac*r1*math.Sin(tc), ac*r1*math.Cos(tc)
		for j, v := range xs {
			px, py, pz := (1-v)*sx+v*cx, (1-v)*sy+v*cy, v*h
			ux, uy := (1-v)*dsx+v*dcx, (1-v)*dsy+v*dcy
			vx, vy, vz := cx-sx, cy-sy, h
			nx, ny, nz := uy*vz, -ux*vz, ux*vy-uy*vx
			total += ws[i] * ws[j] * (px*nx + py*ny + pz*nz)
		}
	}
	return total
}

// TestChordLocusFluxTermCoversTheBuiltPatch checks the chord-versus-locus
// flux term on a narrow window (R = 10, window 0.001 rad, dc = ds = 1, the
// cap window trimmed 2e-4 rad at each corner) against quadratures of the
// three fluxes it reads. Each enclosure must hold its quadrature, the built
// flux must lie within the proven excursion ε of the interval between the
// narrow and wide fluxes, and the term must cover the built flux's distance
// from either end of that interval, since the denoted flux can sit at either
// end. The wide and built patches end on different rulings, so W − B holds
// both corner triangles, about ½·R0·R1·H·Φ each; a displacement-times-area
// bound on |W − B| reaches under a hundredth of it here.
//
// Shown to fail on 2026-10-09: with ChordLocusVolumeAllow dropping the
// |W − N| leg, the term falls below W − B; with ε left out of the
// enclosure check, nothing changes here (ε is zero on this patch), which
// TestChordLocusVolumeAllowRoundsOutward's excursion rows cover instead.
func TestChordLocusFluxTermCoversTheBuiltPatch(t *testing.T) {
	t.Parallel()
	g := skewedPatch(t, 0.3, 0.301, 0, 2e-4)
	g.SweepCCW = true
	refW := ruledFluxAboutAxis(g, g.Th0, g.Th1, g.Th0, g.Th1)
	refN := ruledFluxAboutAxis(g, g.CapTh0, g.CapTh1, g.CapTh0, g.CapTh1)
	refB := ruledFluxAboutAxis(g, g.Th0, g.Th1, g.CapTh0, g.CapTh1)
	// quad absorbs the quadrature's float64 roundoff, far below every gap
	// the test reads.
	quad := 1e-12 * refW

	wide, narrow, built := capband.ChordLocusFluxes(g)
	for _, c := range []struct {
		name string
		got  proofbound.BoundedScalar
		want float64
	}{{`wide`, wide, refW}, {`narrow`, narrow, refN}, {`built`, built, refB}} {
		require.LessOrEqual(t, math.Abs(c.got.Value-c.want), c.got.Bound+quad,
			`the %s flux %v ± %v must enclose the quadrature %v`, c.name, c.got.Value, c.got.Bound, c.want)
	}
	require.Positive(t, refW-refB, `the built patch's corner rulings sit inside the wide sector's`)

	eps := proofbound.ChordLocusBuiltExcursion(wide.Value, wide.Bound, narrow.Value, narrow.Bound, built.Value, built.Bound)
	require.GreaterOrEqual(t, refB, math.Min(refN, refW)-eps-quad, `the built flux must lie within ε of the interval`)
	require.LessOrEqual(t, refB, math.Max(refN, refW)+eps+quad, `the built flux must lie within ε of the interval`)

	term := capband.ChordLocusFluxAllow(g)
	worst := math.Max(refW-refB, refB-refN)
	require.GreaterOrEqual(t, term, worst+quad,
		`the term %v must cover the built flux's distance %v from either end of [N, W]`, term, worst)
	t.Logf(`W − B = %v, B − N = %v, term = %v, ε = %v`, refW-refB, refB-refN, term, eps)
}

// TestChordLocusFluxTermCoversATurnedWindow checks the chord-versus-locus
// flux term when one corner narrows the window and the other widens it: the
// narrow window of TestChordLocusFluxTermCoversTheBuiltPatch with its cap
// window turned 2e-4 rad, so neither window holds the other. The denoted
// surface is modelled with corner-foot azimuths a(v) = th0 + τ·v² and
// b(v) = th1 + τ·√v, each monotone between its corner's side and cap ends as
// every real locus is, and its flux is integrated as the cone sector over
// [a(v), b(v)]. The term must cover the gap between that flux and the built
// patch's, and the references must hold the denoted flux between them.
//
// Shown to fail on 2026-10-09: with chordLocusFluxes reading the side and
// cap windows as the references, both sectors have the same width and flux,
// the side sector's flux falls below the denoted one, and the term, 9e-10,
// no longer covers the 0.0064 gap.
func TestChordLocusFluxTermCoversATurnedWindow(t *testing.T) {
	t.Parallel()
	const tau = 2e-4
	g := skewedPatch(t, 0.3, 0.301, tau, 0)
	g.SweepCCW = true
	xs, ws := gaussLegendre(64)
	h := g.CapZ - g.SideZ
	denoted := 0.0
	for i, v := range xs {
		r := g.SideRadius + (g.CapRadius-g.SideRadius)*v
		a := g.Th0 + tau*v*v
		b := g.Th1 + tau*math.Sqrt(v)
		denoted += ws[i] * h * g.SideRadius * r * (b - a)
	}
	built := ruledFluxAboutAxis(g, g.Th0, g.Th1, g.CapTh0, g.CapTh1)
	gap := math.Abs(denoted - built)
	require.Positive(t, gap)

	wide, narrow, _ := capband.ChordLocusFluxes(g)
	require.LessOrEqual(t, narrow.Value-narrow.Bound, denoted, `the intersection sector's flux must not exceed the denoted flux`)
	require.GreaterOrEqual(t, wide.Value+wide.Bound, denoted, `the union sector's flux must reach the denoted flux`)
	term := capband.ChordLocusFluxAllow(g)
	require.GreaterOrEqual(t, term, gap*(1+1e-9),
		`the term %v must cover the gap %v between the denoted and built fluxes`, term, gap)
}

// TestChordLocusVolumeRefusesAnUnboundedCorner checks the region term the
// first moment reads answers +Inf for a patch whose corner flux the build
// could not bound: its proof reads each corner-foot locus's azimuth as
// monotone, which a fold, where the two carriers touch, breaks.
//
// Shown to fail on 2026-10-09: with chordLocusRegionAllow not reading the
// corner flux, the volume is finite.
func TestChordLocusVolumeRefusesAnUnboundedCorner(t *testing.T) {
	t.Parallel()
	g := skewedPatch(t, 0.3, 1.2, 0, 0.05)
	finite, _ := capband.ChordLocusVolume(g)
	require.Positive(t, finite)
	require.False(t, math.IsInf(finite, 1))
	g.CornerFlux = math.Inf(1)
	vol, _ := capband.ChordLocusVolume(g)
	require.True(t, math.IsInf(vol, 1), `an unbounded corner leaves the region unbounded, not %v`, vol)
}
