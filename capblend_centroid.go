package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
)

// This file is docs/modify-reach-design.md §8.4's closed-form first moments
// for the cap-loop chamfer payload — the fix for A8: evalCapBlendContext used
// to publish a centroid that was an area-weighted average of each face's own
// first-loop-start VERTEX (capBlendCentroidEstimate, gone), bounded only by
// the body's own bounding box. That was not a centroid; it was a stand-in
// with no formula behind it at all, wrong by 10 mm on a cylinder 21.5 mm
// across. This file computes the real thing: M = ∫ p dV in the payload's own
// plane-local (u, v, z) coordinates — the same frame capBandVolume already
// works in, valid because a rigid map preserves volume and transforms the
// moment linearly — decomposed exactly the way the volume already is,
// M = Σ_loops sign_li · [M_slab(li) + M_band(li, start) + M_band(li, end)],
// with sign_li the SAME per-loop sign evalCapBlendContext already applies to
// slabVolume/bandVolume. capBlendCentroidGeometryBound (this file, the second
// half of the old estimate) remains the geometric safety net every analytic
// centroid falls back to, now a CEILING on the formula answer via math.Min,
// never the whole bound.
//
// The slab term is shell_cup.go's loopEnclosedMomentsContext (a signed first
// moment sibling of loopEnclosedAreaContext) times the straight height, with
// the z component the elementary A·h·(zLo+zHi)/2. The band term is the
// divergence theorem with F = (u²/2, 0, 0), (0, v²/2, 0), (0, 0, z²/2) over
// the SAME closed sub-solid capBandVolume already integrates (the two flat
// disks plus the patches): patchFirstMomentFlux is patchRawFlux's per-axis
// sibling, exact rationally for a Plane patch (exactPlanePatchMoment) and a
// closed-form Fourier sum for a Cone/apex/whole-turn one
// (coneMomentTermsX/Y/Z), derived by computer algebra and verified against a
// fine numerical double integral of the raw flux integrand and against this
// package's own shipped volume formula (both reproduced independently, never
// merely asserted) — see capBandMoment's own doc for the disk/patch sign
// composition, identical to capBandVolume's.

// patchFirstMomentFlux is one patch's own raw first-moment contribution —
// Mx, My, Mz, the divergence-theorem flux of F = (u²/2, 0, 0), (0, v²/2, 0),
// (0, 0, z²/2) respectively, taken relative to the plane-local origin — with
// its own proven bound. The per-axis sibling of patchRawFlux
// (capblend_moments.go), dispatching on g.circular exactly where that
// function does.
func patchFirstMomentFlux(g capPatchGeom) (mu, mv, mz boundedScalar) {
	if !g.circular {
		return planePatchMoment(g)
	}
	return conePatchMoment(g)
}

// planePatchMoment is the flat Plane patch's own first moment, split into the
// SAME two triangles patchRawFlux/patchAreaOf use (v0v1v2, v0v2v3): exact
// rationally wherever every coordinate lifts (exactPlanePatchMoment), the
// float fallback only for a patch whose coordinates do not (mirroring
// patchRawFlux's own exactPlanePatchFlux/float split, capblend_moments.go).
func planePatchMoment(g capPatchGeom) (mu, mv, mz boundedScalar) {
	v0 := r3.NewVec(g.sideA.U, g.sideA.V, g.sideZ)
	v1 := r3.NewVec(g.sideB.U, g.sideB.V, g.sideZ)
	v2 := r3.NewVec(g.capB.U, g.capB.V, g.capZ)
	v3 := r3.NewVec(g.capA.U, g.capA.V, g.capZ)
	if ex, ey, ez, ok := exactPlanePatchMoment(v0, v1, v2, v3); ok {
		hx, _ := ex.Float64()
		hy, _ := ey.Float64()
		hz, _ := ez.Float64()
		return measuredScalar(hx, rationalFloatError(ex, hx)),
			measuredScalar(hy, rationalFloatError(ey, hy)),
			measuredScalar(hz, rationalFloatError(ez, hz))
	}
	return floatPlanePatchMoment(v0, v1, v2, v3)
}

// exactPlanePatchMoment is the flat quad patch's first moment — the per-axis
// sibling of exactPlanePatchFlux (capblend_moments.go) — as exact rationals:
// for a triangle a, b, c with unnormalized normal N = (b−a)×(c−a),
//
//	∫_T (x²/2)·n_x dA = N_x·(x_a²+x_b²+x_c²+x_a·x_b+x_b·x_c+x_c·x_a) / 24
//
// (and the y, z analogues, replacing x with y or z throughout — the same
// identity by the coordinates' own symmetry), derived by parametrizing the
// triangle P = a + s(b−a) + t(c−a) over the unit simplex, where n dA = N ds dt
// with N constant and x affine in (s, t): ∫∫_simplex x² ds dt =
// (Σx_i² + Σ_{i<j} x_i·x_j)/12, and multiplying by N_x/2 gives the /24 —
// verified both symbolically (independent computer-algebra re-derivation) and
// numerically against a fine double integral. Every coordinate is a payload
// float64, hence an exact rational, and every operation is +, −, ×, ÷24 —
// closed over the rationals — so this is the true moment of the quad the
// topology actually built, with no rounding anywhere on the way; its one
// caller rounds each component to a float64 once and reports that single
// rounding as the bound, exactly as exactPlanePatchFlux's caller does for the
// flux.
//
// It reports ok=false rather than panicking where a coordinate does not
// lift, mirroring exactPlanePatchFlux: a public measurement must refuse or
// bound, never abort.
func exactPlanePatchMoment(v0, v1, v2, v3 r3.Vec) (mx, my, mz *big.Rat, ok bool) {
	lift := func(v r3.Vec) (ratV3, bool) {
		x, y, z := floatRat(v.X), floatRat(v.Y), floatRat(v.Z)
		if x == nil || y == nil || z == nil {
			return ratV3{}, false
		}
		return ratV3{x, y, z}, true
	}
	r0, ok0 := lift(v0)
	r1, ok1 := lift(v1)
	r2, ok2 := lift(v2)
	r3v, ok3 := lift(v3)
	if !ok0 || !ok1 || !ok2 || !ok3 {
		return nil, nil, nil, false
	}
	tri := func(a, b, c ratV3) [3]*big.Rat {
		n := rvCross(rvSub(b, a), rvSub(c, a))
		var out [3]*big.Rat
		for i := range out {
			sq := ratAdd(
				ratMul(a[i], a[i]), ratMul(b[i], b[i]), ratMul(c[i], c[i]),
				ratMul(a[i], b[i]), ratMul(b[i], c[i]), ratMul(c[i], a[i]),
			)
			out[i] = new(big.Rat).Quo(new(big.Rat).Mul(n[i], sq), big.NewRat(24, 1))
		}
		return out
	}
	t1 := tri(r0, r1, r2)
	t2 := tri(r0, r2, r3v)
	return ratAdd(t1[0], t2[0]), ratAdd(t1[1], t2[1]), ratAdd(t1[2], t2[2]), true
}

// floatPlanePatchMoment is exactPlanePatchMoment's float fallback for a patch
// whose coordinates do not lift to exact rationals (non-finite geometry) —
// the identical rare case exactPlanePatchFlux's own float arm covers, no
// better served by rational arithmetic here either. The bound is a generous,
// never-tight structural envelope in the SAME spirit as tripleProductUpper:
// each axis's own six-term square sum is at most 6·coordMax², and the
// unnormalized normal's own magnitude is bounded by crossProductUpper,
// independent of what the computed value happens to be.
func floatPlanePatchMoment(v0, v1, v2, v3 r3.Vec) (mx, my, mz boundedScalar) {
	compute := func(a, b, c r3.Vec) r3.Vec {
		n := b.Sub(a).Cross(c.Sub(a))
		sq := func(ai, bi, ci float64) float64 { return ai*ai + bi*bi + ci*ci + ai*bi + bi*ci + ci*ai }
		return r3.NewVec(n.X*sq(a.X, b.X, c.X)/24, n.Y*sq(a.Y, b.Y, c.Y)/24, n.Z*sq(a.Z, b.Z, c.Z)/24)
	}
	envelope := func(a, b, c r3.Vec) float64 {
		coordMax := math.Max(vecMaxAbs(a), math.Max(vecMaxAbs(b), vecMaxAbs(c)))
		crossUpper := crossProductUpper(b.Sub(a), c.Sub(a))
		return productUpper(crossUpper, productUpper(6, productUpper(coordMax, coordMax))) / 24
	}
	m1 := compute(v0, v1, v2)
	m2 := compute(v0, v2, v3)
	sum := m1.Add(m2)
	bound := analyticRoundBound(absSumUpper(envelope(v0, v1, v2), envelope(v0, v2, v3)))
	return measuredScalar(sum.X, bound), measuredScalar(sum.Y, bound), measuredScalar(sum.Z, bound)
}

// phaseTerm is one closed-form Fourier term of a ruled Cone patch's own
// first-moment integral: Ac·cos(k·θS+m·θC) + As·sin(k·θS+m·θC), θS(a) =
// thS0+a·dS, θC(a) = thC0+a·dC (coneMomentTermsX/Y/Z's own doc derives Ac,
// As).
type phaseTerm struct {
	k, m   int
	ac, as float64
}

// coneMomentTermsX/Y/Z are the ruled Cone (or apex/whole-circle) patch's own
// Mx, My, Mz closed forms (docs/modify-reach-design.md §8.4's "Cone / ruled
// patch"): with S(a) = (R0·cosθS, R0·sinθS, 0), C(a) = (R1·cosθC, R1·sinθC,
// H), P(a, b) = (1−b)S(a) + b·C(a) + (cU, cV, z0) the same ruled
// parametrization patchRawFlux integrates, ∫∫(x²/2)·Nx, ∫∫(y²/2)·Ny,
// ∫∫(z²/2)·Nz over the unit square reduce — after the exact polynomial
// b-integral (x, y, z and Nx, Ny, Nz are each affine in b, so x²·Nx etc. are
// cubic Bernstein forms in b with an elementary ∫₀¹) and a product-to-sum
// reduction of the resulting a-trig products — to a FINITE sum of
// Ac·cos(k·θS+m·θC) + As·sin(k·θS+m·θC) terms, every phase with
// |k|+|m| <= 3, Ac and As RATIONAL in R0, R1, H, the eccentric center
// coordinate (cU for Mx, cV for My) and dS = θS1−θS0, dC = θC1−θC0 — the same
// shape ruledAngleCos already reduces the volume's own single (k, m) = (1,
// −1) cross term to, generalized here to the full set the SQUARED coordinate
// in the moment field introduces.
//
// The derivation was done by computer algebra (symbolic double integration,
// complex-exponential collection into these canonical (k, m) terms) and
// verified two ways: against a fine numerical double integral of the raw
// P_a×P_b flux integrand across randomized configurations, and — the
// stronger check — against this package's own shipped, independently-tested
// analytic volume formula on the ask's own cylinder r10 h8 chamfer 0.5mm
// fixture, where the assembled centroid (this file's capBandMoment plus the
// slab term) reproduces the closed-form frustum-plus-slab centroid
// 3.9881863539 to float64 precision.
func coneMomentTermsX(R0, R1, H, cU, dS, dC float64) []phaseTerm {
	return []phaseTerm{
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

func coneMomentTermsY(R0, R1, H, cV, dS, dC float64) []phaseTerm {
	return []phaseTerm{
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

func coneMomentTermsZ(R0, R1, H, z0, dS, dC float64) []phaseTerm {
	return []phaseTerm{
		{0, 0, (H*H*R0*R0*dS - 3*H*H*R1*R1*dC + 4*H*R0*R0*dS*z0 - 8*H*R1*R1*dC*z0 + 6*R0*R0*dS*z0*z0 - 6*R1*R1*dC*z0*z0) / 24, 0},
		{1, -1, R0 * R1 * (3*H*H*dC - H*H*dS + 8*H*dC*z0 - 4*H*dS*z0 + 6*dC*z0*z0 - 6*dS*z0*z0) / 24, 0},
	}
}

// sumPhaseTerms evaluates Σ (ac·cos(k·θS+m·θC) + as·sin(k·θS+m·θC)) via the
// SAME sincHalf reduction ruledAngleCos already uses for the volume's own
// single cross term, generalized to every phase this closed form needs:
// ∫₀¹cos(α0+a(α1−α0))da = cos((α0+α1)/2)·sincHalf(α1−α0) (and the sin
// analogue) — continuous at a zero window, so a tangent join or a degenerate
// patch reads through the SAME formula rather than a separate branch.
// envelope is the STRUCTURAL magnitude bound Σ|ac|+|as|, never read off the
// computed value: |cos|, |sin| and |sincHalf| never exceed 1 for any real
// argument, so no term's TRUE magnitude can exceed its own coefficients'.
func sumPhaseTerms(terms []phaseTerm, thS0, thS1, thC0, thC1 float64) (value, envelope float64) {
	for _, t := range terms {
		phase0 := float64(t.k)*thS0 + float64(t.m)*thC0
		phase1 := float64(t.k)*thS1 + float64(t.m)*thC1
		mid := (phase0 + phase1) / 2
		width := phase1 - phase0
		s := sincHalf(width)
		value += t.ac*math.Cos(mid)*s + t.as*math.Sin(mid)*s
		envelope = absSumUpper(envelope, t.ac, t.as)
	}
	return value, envelope
}

// wholeTurnPhaseSum is the structural reduction for a whole-turn patch
// (capPatchGeom.wholeTurn): θS(a) and θC(a) are IDENTICAL functions of a —
// the same floats on both directrices, capblend_geom.go's cornerless closed
// circle — so every term with k+m != 0 integrates to EXACTLY zero (a full
// period's cos and sin both integrate to zero), read directly from that
// STRUCTURAL fact rather than from Sincos near a multiple of pi (fl(2π) is
// not 2π). Every surviving term (k+m == 0) then has phase identically 0 —
// cos(0) = 1, sincHalf(0) = 1, both exact — so the sum is the surviving
// terms' own ac, with no trig call at all: the moment's own analogue of
// patchRawFlux's wholeTurn branch, where the eccentric origin term's true
// value is exactly zero for the same reason.
func wholeTurnPhaseSum(terms []phaseTerm) (value, envelope float64) {
	for _, t := range terms {
		if t.k+t.m != 0 {
			continue
		}
		value += t.ac
		envelope = absSumUpper(envelope, t.ac)
	}
	return value, envelope
}

// conePatchMoment is the ruled Cone (or apex/whole-circle) patch's own first
// moment, with its own proven bound. wholeTurn skips Sincos entirely (its
// value is exact arithmetic, analyticRoundBound's ordinary rounding budget);
// otherwise the bound is conservativeValueError against the sum's own
// structural envelope — a DOCUMENTED loose term (docs/modify-reach-design.md
// §8.4's PR B note): every |cos|, |sin|, |sincHalf| factor is capped at 1, so
// the envelope from the coefficients alone bounds the true value regardless
// of what Sincos returned, without ever reading a budget off the computed
// value — the same discipline patchRawFlux's own originEnv/crossEnv follow,
// and the same fallback moments.go's conservativeValueError doc reserves for
// "wherever no certified rational bracket admits it". sweepCCW's negation
// mirrors patchRawFlux's own: the closed form is taken over the NORMALIZED
// (th0 < th1) window, which is the patch's actual orientation only while its
// own walk runs counter-clockwise.
func conePatchMoment(g capPatchGeom) (mu, mv, mz boundedScalar) {
	R0, R1 := g.sideRadius, g.capRadius
	H := g.capZ - g.sideZ
	thS0, thS1 := g.th0, g.th1
	thC0, thC1 := g.capTh0, g.capTh1
	dS, dC := thS1-thS0, thC1-thC0

	sum := func(terms []phaseTerm) boundedScalar {
		if g.wholeTurn {
			value, envelope := wholeTurnPhaseSum(terms)
			return measuredScalar(value, analyticRoundBound(envelope))
		}
		value, envelope := sumPhaseTerms(terms, thS0, thS1, thC0, thC1)
		return measuredScalar(value, conservativeValueError(value, envelope))
	}

	mxR := sum(coneMomentTermsX(R0, R1, H, g.cU, dS, dC))
	myR := sum(coneMomentTermsY(R0, R1, H, g.cV, dS, dC))
	mzR := sum(coneMomentTermsZ(R0, R1, H, g.sideZ, dS, dC))

	if !g.sweepCCW {
		mxR.value, myR.value, mzR.value = -mxR.value, -myR.value, -mzR.value
	}
	return mxR, myR, mzR
}

// loopCoordinateUpper is one loop's own coordinate envelope
// (profileCoordinateUpper, extrude.go, wrapped as a single-outer-loop
// ProfileRecord) — sweptMomentAllow's coordUpper input, one dimension's worth
// of the SAME envelope prismCentroidGeometryBound already forms for a whole
// profile.
func loopCoordinateUpper(loop LoopRecord, work *freeformWork) (float64, error) {
	return profileCoordinateUpper(ProfileRecord{Outer: loop}, work)
}

// capBandMoment is one loop's chamfer-band first-moment contribution — Mx,
// My, Mz, the divergence-theorem flux of F = (u²/2, 0, 0), (0, v²/2, 0),
// (0, 0, z²/2) — over the SAME closed sub-solid capBandVolume integrates: the
// two flat disks (the loop's own enclosed area at capZ and at sideZ) plus the
// patches (buildCapBand's geom).
//
// The two disks are FLAT (constant z), so their outward normal is ±ẑ:
// n_u = n_v = 0 everywhere on them, and Green's theorem gives them NO Mu/Mv
// contribution at all — only Mz, at (level²/2)·(±area), the divergence
// theorem's own reduction of capBandVolume's (level)·(±area) volume term one
// power higher (docs/modify-reach-design.md §8.4). Every patch, in contrast,
// is built from the loop's OWN walk, so the SAME -matSign·orient sign
// correction capBandVolume applies to its own patchRawFlux sum applies here
// (§8.4's "Signs"): -matSign covers the AXIAL half, orient — the loop's own
// signed-area sign — the IN-PLANE half.
//
// The cap contour's own displacement (delta) is composed ONCE, after the
// sum, via bounds.go's sweptMomentAllow — never inside a disk term or inside
// patchFirstMomentFlux itself, both of which already read the SAME displaced
// cap-level coordinates and would double the charge (capBandVolume's
// identical rule for sweptVolumeAllow). areaUpper is the same surface the
// contour's displacement acted on (this band's patches plus the cap disk
// they close on); coordUpper is the band's own coordinate envelope — the
// ORIGINAL loop's (loopCoordinateUpper) AND the built cap boundary's
// (capLoopBoundary, widened by delta since that boundary is itself only
// known to within delta of the one it denotes), plus the two axial levels.
// The band's material lies between the two loops, so a bound taken from the
// original loop alone can fall short wherever the offset moves a coordinate
// OUTWARD — capArea's own boundary is exactly that displaced coordinate set,
// and sweptMomentAllow's own contract (bounds.go) requires coordUpper to
// bound every point the difference volume can hold.
func capBandMoment(ctx context.Context, loop LoopRecord, cbp capBlendPayload, geom []capPatchGeom, capZ, matSign, delta float64, work *freeformWork) (mu, mv, mz boundedScalar, err error) {
	capZB := cbp.capBandLevel(capZ, matSign)
	sideZB := boundedAdd(capZB, exactScalar(matSign*cbp.d))
	sideZ := sideZB.value

	signedArea, err := loopSignedAreaBudget(newWorkBudget(ctx), loop)
	if err != nil {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, err
	}
	orient := 1.0
	if signedArea < 0 {
		orient = -1
	}

	sideArea, err := loopEnclosedAreaContext(ctx, loop)
	if err != nil {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, err
	}
	capBoundary, err := capLoopBoundary(ctx, loop, cbp.d)
	if err != nil {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, err
	}
	capArea, err := loopEnclosedAreaContext(ctx, capBoundary)
	if err != nil {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, err
	}

	half := exactScalar(0.5)
	capZTerm := boundedMul(boundedMul(boundedMul(capZB, capZB), half), boundedMul(exactScalar(-matSign), capArea))
	sideZTerm := boundedMul(boundedMul(boundedMul(sideZB, sideZB), half), boundedMul(exactScalar(matSign), sideArea))
	muTotal := boundedScalar{}
	mvTotal := boundedScalar{}
	mzTotal := boundedAdd(capZTerm, sideZTerm)

	patchAreaTotal := boundedScalar{}
	for _, g := range geom {
		pmu, pmv, pmz := patchFirstMomentFlux(g)
		sign := -matSign * orient
		muTotal = boundedAdd(muTotal, measuredScalar(sign*pmu.value, pmu.bound))
		mvTotal = boundedAdd(mvTotal, measuredScalar(sign*pmv.value, pmv.bound))
		mzTotal = boundedAdd(mzTotal, measuredScalar(sign*pmz.value, pmz.bound))
		pa, pb := patchAreaOf(g)
		patchAreaTotal = boundedAdd(patchAreaTotal, measuredScalar(pa, pb))
	}

	if delta > 0 {
		areaUpper := absSumUpper(patchAreaTotal.value, patchAreaTotal.bound, capArea.value, capArea.bound)
		coordUpper, cerr := loopCoordinateUpper(loop, work)
		if cerr != nil {
			return boundedScalar{}, boundedScalar{}, boundedScalar{}, cerr
		}
		capCoordUpper, cerr := loopCoordinateUpper(capBoundary, work)
		if cerr != nil {
			return boundedScalar{}, boundedScalar{}, boundedScalar{}, cerr
		}
		coordUpper = math.Max(coordUpper, absSumUpper(capCoordUpper, delta))
		coordUpper = math.Max(coordUpper, absSumUpper(math.Abs(sideZ), sideZB.bound))
		coordUpper = math.Max(coordUpper, absSumUpper(math.Abs(capZ), capZB.bound))
		allow := sweptMomentAllow(delta, areaUpper, coordUpper)
		muTotal.bound = absSumUpper(muTotal.bound, allow)
		mvTotal.bound = absSumUpper(mvTotal.bound, allow)
		mzTotal.bound = absSumUpper(mzTotal.bound, allow)
	}
	return muTotal, mvTotal, mzTotal, nil
}

// capBlendCentroidGeometryBound is the geometric safety net every analytic
// centroid falls back to: the true centroid of a bounded solid lies within
// its own bounding box, so |estimate-true| is bounded by the box's own reach
// from the estimate — sound whatever the estimate's own accuracy. It is now a
// CEILING on the closed-form first-moment answer (evalCapBlendContext's
// math.Min), never published on its own: the old capBlendCentroidEstimate's
// face-average estimate is gone (docs/modify-reach-design.md §8.4's
// closed-form first moments replace it), and faceRepresentativePoint had no
// other caller and is gone with it.
//
// The reach is maximized over all EIGHT corners of the box, and that is the
// whole of the proof rather than a thoroughness flourish. p -> |p - estimate|
// is convex, so its maximum over the box — a convex hull of its eight
// corners — is attained AT a corner; taking the max over all eight therefore
// bounds the distance to every point the box holds, the true centroid among
// them, wherever the estimate itself sits. Reading only Min and Max leaves
// six corners unexamined, and a box whose extent along one axis is far larger
// than along another puts its farthest corner among exactly those six: the
// reported bound would then be smaller than the estimate's own error and
// enclose nothing.
//
// The box's own Bound is added on top for the same reason: the safety net is
// "the true centroid lies within the box", and where a face of the box is
// itself known only to a displacement, the box that provably contains the
// body is the reported one widened by it.
func capBlendCentroidGeometryBound(estimate r3.Vec, bounds Box) float64 {
	xs := [2]float64{bounds.Min.X, bounds.Max.X}
	ys := [2]float64{bounds.Min.Y, bounds.Max.Y}
	zs := [2]float64{bounds.Min.Z, bounds.Max.Z}
	reach := 0.0
	for _, x := range xs {
		for _, y := range ys {
			for _, z := range zs {
				dd := r3.NewVec(x, y, z).Sub(estimate).Len()
				if dd > reach {
					reach = dd
				}
			}
		}
	}
	return absSumUpper(reach, bounds.Bound.Mag())
}
