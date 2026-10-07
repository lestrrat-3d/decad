package capband

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// patchFirstMomentFlux is one patch's own raw first-moment contribution —
// Mx, My, Mz, the divergence-theorem flux of F = (u²/2, 0, 0), (0, v²/2, 0),
// (0, 0, z²/2) respectively, taken relative to the plane-local origin — with
// its own proven bound. The per-axis sibling of patchRawFlux
// (capblend_moments.go), dispatching on g.Circular exactly where that
// function does.
func patchFirstMomentFlux(g Patch) (mu, mv, mz proofbound.BoundedScalar) {
	if !g.Circular {
		return planePatchMoment(g)
	}
	return conePatchMoment(g)
}

// planePatchMoment is the flat Plane patch's own first moment, split into the
// SAME two triangles patchRawFlux/patchAreaOf use (v0v1v2, v0v2v3): exact
// rationally wherever every coordinate lifts (exactPlanePatchMoment), the
// float fallback only for a patch whose coordinates do not (mirroring
// patchRawFlux's own exactPlanePatchFlux/float split, capblend_moments.go).
func planePatchMoment(g Patch) (mu, mv, mz proofbound.BoundedScalar) {
	v0 := r3.NewVec(g.SideA.U, g.SideA.V, g.SideZ)
	v1 := r3.NewVec(g.SideB.U, g.SideB.V, g.SideZ)
	v2 := r3.NewVec(g.CapB.U, g.CapB.V, g.CapZ)
	v3 := r3.NewVec(g.CapA.U, g.CapA.V, g.CapZ)
	if ex, ey, ez, ok := exactPlanePatchMoment(v0, v1, v2, v3); ok {
		hx, _ := ex.Float64()
		hy, _ := ey.Float64()
		hz, _ := ez.Float64()
		return proofbound.MeasuredScalar(hx, proofarith.RationalFloatError(ex, hx)),
			proofbound.MeasuredScalar(hy, proofarith.RationalFloatError(ey, hy)),
			proofbound.MeasuredScalar(hz, proofarith.RationalFloatError(ez, hz))
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
	lift := func(v r3.Vec) (proofarith.DyV3, bool) {
		x, okX := proofarith.DyOf(v.X)
		y, okY := proofarith.DyOf(v.Y)
		z, okZ := proofarith.DyOf(v.Z)
		if !okX || !okY || !okZ {
			return proofarith.DyV3{}, false
		}
		return proofarith.DyV3{x, y, z}, true
	}
	r0, ok0 := lift(v0)
	r1, ok1 := lift(v1)
	r2, ok2 := lift(v2)
	r3v, ok3 := lift(v3)
	if !ok0 || !ok1 || !ok2 || !ok3 {
		return nil, nil, nil, false
	}
	tri := func(a, b, c proofarith.DyV3) [3]*big.Rat {
		n := proofarith.DvCross(proofarith.DvSub(b, a), proofarith.DvSub(c, a))
		var out [3]*big.Rat
		for i := range out {
			sq := proofarith.DyAdd(
				proofarith.DyAdd(proofarith.DyAdd(proofarith.DyMul(a[i], a[i]), proofarith.DyMul(b[i], b[i])), proofarith.DyMul(c[i], c[i])),
				proofarith.DyAdd(proofarith.DyAdd(proofarith.DyMul(a[i], b[i]), proofarith.DyMul(b[i], c[i])), proofarith.DyMul(c[i], a[i])),
			)
			// The twenty-fourth is the one step that leaves the dyadic set, so
			// the product converts here and nowhere earlier (dyadic.go).
			out[i] = new(big.Rat).Quo(proofarith.DyMul(n[i], sq).Rat(), big.NewRat(24, 1))
		}
		return out
	}
	t1 := tri(r0, r1, r2)
	t2 := tri(r0, r2, r3v)
	return proofbound.RatAdd(t1[0], t2[0]), proofbound.RatAdd(t1[1], t2[1]), proofbound.RatAdd(t1[2], t2[2]), true
}

// floatPlanePatchMoment is exactPlanePatchMoment's float fallback for a patch
// whose coordinates do not lift to exact rationals (non-finite geometry) —
// the identical rare case exactPlanePatchFlux's own float arm covers, no
// better served by rational arithmetic here either. The bound is a generous,
// never-tight structural envelope in the SAME spirit as tripleProductUpper:
// each axis's own six-term square sum is at most 6·coordMax², and the
// unnormalized normal's own magnitude is bounded by crossProductUpper,
// independent of what the computed value happens to be.
func floatPlanePatchMoment(v0, v1, v2, v3 r3.Vec) (mx, my, mz proofbound.BoundedScalar) {
	compute := func(a, b, c r3.Vec) r3.Vec {
		n := b.Sub(a).Cross(c.Sub(a))
		sq := func(ai, bi, ci float64) float64 { return ai*ai + bi*bi + ci*ci + ai*bi + bi*ci + ci*ai }
		return r3.NewVec(n.X*sq(a.X, b.X, c.X)/24, n.Y*sq(a.Y, b.Y, c.Y)/24, n.Z*sq(a.Z, b.Z, c.Z)/24)
	}
	envelope := func(a, b, c r3.Vec) float64 {
		coordMax := math.Max(proofbound.VecMaxAbs(a), math.Max(proofbound.VecMaxAbs(b), proofbound.VecMaxAbs(c)))
		crossUpper := crossProductUpper(b.Sub(a), c.Sub(a))
		return proofbound.ProductUpper(crossUpper, proofbound.ProductUpper(6, proofbound.ProductUpper(coordMax, coordMax))) / 24
	}
	m1 := compute(v0, v1, v2)
	m2 := compute(v0, v2, v3)
	sum := m1.Add(m2)
	bound := proofbound.AnalyticRoundBound(proofbound.AbsSumUpper(envelope(v0, v1, v2), envelope(v0, v2, v3)))
	return proofbound.MeasuredScalar(sum.X, bound), proofbound.MeasuredScalar(sum.Y, bound), proofbound.MeasuredScalar(sum.Z, bound)
}

// PhaseTerm is one closed-form Fourier term of a ruled Cone patch's own
// first-moment integral: Ac·cos(k·θS+m·θC) + As·sin(k·θS+m·θC), θS(a) =
// thS0+a·dS, θC(a) = thC0+a·dC (coneMomentTermsX/Y/Z's own doc derives Ac,
// As). Ac and As are exact rationals in the patch's own held floats.
type PhaseTerm struct {
	k, m   int
	ac, as *big.Rat
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
// in the moment field introduces. Every input is an exact rational and every
// coefficient is built by +, −, × and an integer division, so the
// coefficients carry no rounding at all.
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
func coneMomentTermsX(R0, R1, H, cU, dS, dC *big.Rat) []PhaseTerm {
	two, four, nine, twentyFour := big.NewRat(2, 1), big.NewRat(4, 1), big.NewRat(9, 1), big.NewRat(24, 1)
	return []PhaseTerm{
		{0, 0, ratScale(proofbound.RatMul(H, cU, proofbound.RatAdd(proofbound.RatMul(R0, R0, dS), proofbound.RatMul(R1, R1, dC))), 1, 6), new(big.Rat)},
		{0, 1, ratScale(proofbound.RatMul(H, R1, proofbound.RatAdd(proofbound.RatMul(two, R0, R0, dC), proofbound.RatMul(four, R0, R0, dS), proofbound.RatMul(nine, R1, R1, dC), proofbound.RatMul(twentyFour, cU, cU, dC))), 1, 96), new(big.Rat)},
		{1, 0, ratScale(proofbound.RatMul(H, R0, proofbound.RatAdd(proofbound.RatMul(nine, R0, R0, dS), proofbound.RatMul(four, R1, R1, dC), proofbound.RatMul(two, R1, R1, dS), proofbound.RatMul(twentyFour, cU, cU, dS))), 1, 96), new(big.Rat)},
		{0, 2, ratScale(proofbound.RatMul(H, R1, R1, cU, dC), 1, 6), new(big.Rat)},
		{1, -1, ratScale(proofbound.RatMul(H, R0, R1, cU, proofbound.RatAdd(dC, dS)), 1, 12), new(big.Rat)},
		{1, 1, ratScale(proofbound.RatMul(H, R0, R1, cU, proofbound.RatAdd(dC, dS)), 1, 12), new(big.Rat)},
		{2, 0, ratScale(proofbound.RatMul(H, R0, R0, cU, dS), 1, 6), new(big.Rat)},
		{0, 3, ratScale(proofbound.RatMul(H, R1, R1, R1, dC), 1, 32), new(big.Rat)},
		{1, -2, ratScale(proofbound.RatMul(H, R0, R1, R1, proofbound.RatAdd(proofbound.RatMul(two, dC), dS)), 1, 96), new(big.Rat)},
		{1, 2, ratScale(proofbound.RatMul(H, R0, R1, R1, proofbound.RatAdd(proofbound.RatMul(two, dC), dS)), 1, 96), new(big.Rat)},
		{2, -1, ratScale(proofbound.RatMul(H, R0, R0, R1, proofbound.RatAdd(dC, proofbound.RatMul(two, dS))), 1, 96), new(big.Rat)},
		{2, 1, ratScale(proofbound.RatMul(H, R0, R0, R1, proofbound.RatAdd(dC, proofbound.RatMul(two, dS))), 1, 96), new(big.Rat)},
		{3, 0, ratScale(proofbound.RatMul(H, R0, R0, R0, dS), 1, 32), new(big.Rat)},
	}
}

func coneMomentTermsY(R0, R1, H, cV, dS, dC *big.Rat) []PhaseTerm {
	two, four, nine, twentyFour := big.NewRat(2, 1), big.NewRat(4, 1), big.NewRat(9, 1), big.NewRat(24, 1)
	return []PhaseTerm{
		{0, 0, ratScale(proofbound.RatMul(H, cV, proofbound.RatAdd(proofbound.RatMul(R0, R0, dS), proofbound.RatMul(R1, R1, dC))), 1, 6), new(big.Rat)},
		{0, 1, new(big.Rat), ratScale(proofbound.RatMul(H, R1, proofbound.RatAdd(proofbound.RatMul(two, R0, R0, dC), proofbound.RatMul(four, R0, R0, dS), proofbound.RatMul(nine, R1, R1, dC), proofbound.RatMul(twentyFour, cV, cV, dC))), 1, 96)},
		{1, 0, new(big.Rat), ratScale(proofbound.RatMul(H, R0, proofbound.RatAdd(proofbound.RatMul(nine, R0, R0, dS), proofbound.RatMul(four, R1, R1, dC), proofbound.RatMul(two, R1, R1, dS), proofbound.RatMul(twentyFour, cV, cV, dS))), 1, 96)},
		{0, 2, ratScale(proofbound.RatMul(H, R1, R1, cV, dC), -1, 6), new(big.Rat)},
		{1, -1, ratScale(proofbound.RatMul(H, R0, R1, cV, proofbound.RatAdd(dC, dS)), 1, 12), new(big.Rat)},
		{1, 1, ratScale(proofbound.RatMul(H, R0, R1, cV, proofbound.RatAdd(dC, dS)), -1, 12), new(big.Rat)},
		{2, 0, ratScale(proofbound.RatMul(H, R0, R0, cV, dS), -1, 6), new(big.Rat)},
		{0, 3, new(big.Rat), ratScale(proofbound.RatMul(H, R1, R1, R1, dC), -1, 32)},
		{1, -2, new(big.Rat), ratScale(proofbound.RatMul(H, R0, R1, R1, proofbound.RatAdd(proofbound.RatMul(two, dC), dS)), -1, 96)},
		{1, 2, new(big.Rat), ratScale(proofbound.RatMul(H, R0, R1, R1, proofbound.RatAdd(proofbound.RatMul(two, dC), dS)), -1, 96)},
		{2, -1, new(big.Rat), ratScale(proofbound.RatMul(H, R0, R0, R1, proofbound.RatAdd(dC, proofbound.RatMul(two, dS))), 1, 96)},
		{2, 1, new(big.Rat), ratScale(proofbound.RatMul(H, R0, R0, R1, proofbound.RatAdd(dC, proofbound.RatMul(two, dS))), -1, 96)},
		{3, 0, new(big.Rat), ratScale(proofbound.RatMul(H, R0, R0, R0, dS), -1, 32)},
	}
}

func coneMomentTermsZ(R0, R1, H, z0, dS, dC *big.Rat) []PhaseTerm {
	three, four, six, eight := big.NewRat(3, 1), big.NewRat(4, 1), big.NewRat(6, 1), big.NewRat(8, 1)
	neg := func(v *big.Rat) *big.Rat { return new(big.Rat).Neg(v) }
	return []PhaseTerm{
		{0, 0, ratScale(proofbound.RatAdd(
			proofbound.RatMul(H, H, R0, R0, dS),
			neg(proofbound.RatMul(three, H, H, R1, R1, dC)),
			proofbound.RatMul(four, H, R0, R0, dS, z0),
			neg(proofbound.RatMul(eight, H, R1, R1, dC, z0)),
			proofbound.RatMul(six, R0, R0, dS, z0, z0),
			neg(proofbound.RatMul(six, R1, R1, dC, z0, z0)),
		), 1, 24), new(big.Rat)},
		{1, -1, ratScale(proofbound.RatMul(R0, R1, proofbound.RatAdd(
			proofbound.RatMul(three, H, H, dC),
			neg(proofbound.RatMul(H, H, dS)),
			proofbound.RatMul(eight, H, dC, z0),
			neg(proofbound.RatMul(four, H, dS, z0)),
			proofbound.RatMul(six, dC, z0, z0),
			neg(proofbound.RatMul(six, dS, z0, z0)),
		)), 1, 24), new(big.Rat)},
	}
}

// phaseSumInterval encloses Σ (ac·∫cos(k·θS+m·θC) + as·∫sin(k·θS+m·θC)) over the
// unit parameter, each phase integral through phaseIntegralInterval:
// ∫₀¹cos(α0+a(α1−α0))da = cos((α0+α1)/2)·sincHalf(α1−α0) (and the sin
// analogue), with α0 = k·thS0+m·thC0 and α1 = k·thS1+m·thC1 formed exactly.
// The form is continuous at a zero window, so a tangent join or a degenerate
// patch reads through the SAME enclosure rather than a separate branch. The
// coefficients are exact rationals and every phase integral is a certified
// enclosure, so the sum contains the true moment of the patch at its held
// parameters whatever the platform's math package returns.
//
// phases caches each (k, m) phase's enclosure across the three axes of one
// patch, which share their phases; a nil map computes every phase afresh.
// The cache is a speed measure only: a hit returns exactly what a miss
// computes.
func phaseSumInterval(terms []PhaseTerm, thS0, thS1, thC0, thC1 *big.Rat, phases map[[2]int][2]proofbound.RatInterval) (proofbound.RatInterval, bool) {
	total := proofbound.PointInterval(new(big.Rat))
	for _, t := range terms {
		key := [2]int{t.k, t.m}
		enclosure, hit := phases[key]
		if !hit {
			k, m := big.NewRat(int64(t.k), 1), big.NewRat(int64(t.m), 1)
			a0 := proofbound.RatAdd(proofbound.RatMul(k, thS0), proofbound.RatMul(m, thC0))
			a1 := proofbound.RatAdd(proofbound.RatMul(k, thS1), proofbound.RatMul(m, thC1))
			cosIv, sinIv, ok := phaseIntegralInterval(a0, a1)
			if !ok {
				return proofbound.RatInterval{}, false
			}
			enclosure = [2]proofbound.RatInterval{cosIv, sinIv}
			if phases != nil {
				phases[key] = enclosure
			}
		}
		total = proofbound.IntervalAdd(total, proofbound.IntervalAdd(proofbound.IntervalScale(enclosure[0], t.ac), proofbound.IntervalScale(enclosure[1], t.as)))
	}
	return total, true
}

// wholeTurnPhaseSum is the structural reduction for a whole-turn patch
// (Patch.WholeTurn): θS(a) and θC(a) are IDENTICAL functions of a —
// the same floats on both directrices, capblend_geom.go's cornerless closed
// circle — so every term with k+m != 0 integrates to EXACTLY zero (a full
// period's cos and sin both integrate to zero), read directly from that
// STRUCTURAL fact rather than from Sincos near a multiple of pi (fl(2π) is
// not 2π). Every surviving term (k+m == 0) then has phase identically 0 —
// cos(0) = 1, sincHalf(0) = 1, both exact — so the sum is the surviving
// terms' own ac, an exact rational with no trigonometric enclosure at all:
// the moment's own analogue of patchRawFlux's wholeTurn branch, where the
// eccentric origin term's true value is exactly zero for the same reason.
func wholeTurnPhaseSum(terms []PhaseTerm) *big.Rat {
	sum := new(big.Rat)
	for _, t := range terms {
		if t.k+t.m != 0 {
			continue
		}
		sum.Add(sum, t.ac)
	}
	return sum
}

// conePatchMoment is the ruled Cone (or apex/whole-circle) patch's own first
// moment, with its own proven bound. Every held parameter lifts to an exact
// rational and H, dS, dC are formed exactly, so each Fourier coefficient is
// exact (coneMomentTermsX/Y/Z). A whole-turn patch's sum is then an exact
// rational (wholeTurnPhaseSum) and its bound is that rational's one rounding
// into a float64 (rationalFloatError). Every other patch's sum is an interval
// whose phase integrals are certified enclosures (phaseSumInterval); the
// held value is the nearest float to its midpoint and the bound is the
// interval's reach from it (proofbound.IntervalFloatError), so it grows with neither the
// arc centre's distance from the plane-local origin nor the coefficients'
// magnitude. A parameter that does not lift (non-finite geometry, which
// buildCapBand already refuses) answers 0 with an infinite bound rather than
// a reading nothing proves. sweepCCW's negation mirrors patchRawFlux's own:
// the closed form is taken over the NORMALIZED (th0 < th1) window, which is
// the patch's actual orientation only while its own walk runs
// counter-clockwise.
func conePatchMoment(g Patch) (mu, mv, mz proofbound.BoundedScalar) {
	R0, R1 := proofarith.FloatRat(g.SideRadius), proofarith.FloatRat(g.CapRadius)
	z0, z1 := proofarith.FloatRat(g.SideZ), proofarith.FloatRat(g.CapZ)
	thS0, thS1 := proofarith.FloatRat(g.Th0), proofarith.FloatRat(g.Th1)
	thC0, thC1 := proofarith.FloatRat(g.CapTh0), proofarith.FloatRat(g.CapTh1)
	cU, cV := proofarith.FloatRat(g.CU), proofarith.FloatRat(g.CV)
	for _, r := range []*big.Rat{R0, R1, z0, z1, thS0, thS1, thC0, thC1, cU, cV} {
		if r == nil {
			unproven := proofbound.MeasuredScalar(0, math.Inf(1))
			return unproven, unproven, unproven
		}
	}
	H := new(big.Rat).Sub(z1, z0)
	dS := new(big.Rat).Sub(thS1, thS0)
	dC := new(big.Rat).Sub(thC1, thC0)

	phases := make(map[[2]int][2]proofbound.RatInterval)
	sum := func(terms []PhaseTerm) proofbound.BoundedScalar {
		if g.WholeTurn {
			exact := wholeTurnPhaseSum(terms)
			held, _ := exact.Float64()
			return proofbound.MeasuredScalar(held, proofarith.RationalFloatError(exact, held))
		}
		iv, ok := phaseSumInterval(terms, thS0, thS1, thC0, thC1, phases)
		if !ok {
			return proofbound.MeasuredScalar(0, math.Inf(1))
		}
		held, _ := intervalMid(iv).Float64()
		return proofbound.MeasuredScalar(held, proofbound.IntervalFloatError(iv, held))
	}

	mxR := sum(coneMomentTermsX(R0, R1, H, cU, dS, dC))
	myR := sum(coneMomentTermsY(R0, R1, H, cV, dS, dC))
	mzR := sum(coneMomentTermsZ(R0, R1, H, z0, dS, dC))

	if !g.SweepCCW {
		mxR.Value, myR.Value, mzR.Value = -mxR.Value, -myR.Value, -mzR.Value
	}
	return mxR, myR, mzR
}

// FirstMomentFlux returns one cap band's patch first moments and bounds.
func FirstMomentFlux(g Patch) (proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar) {
	return patchFirstMomentFlux(g)
}

// ConeMomentTermsX returns the exact Fourier terms for the first coordinate.
func ConeMomentTermsX(R0, R1, H, cU, dS, dC *big.Rat) []PhaseTerm {
	return coneMomentTermsX(R0, R1, H, cU, dS, dC)
}

// ConeMomentTermsY returns the exact Fourier terms for the second coordinate.
func ConeMomentTermsY(R0, R1, H, cV, dS, dC *big.Rat) []PhaseTerm {
	return coneMomentTermsY(R0, R1, H, cV, dS, dC)
}

// ConeMomentTermsZ returns the exact Fourier terms for the axial coordinate.
func ConeMomentTermsZ(R0, R1, H, z0, dS, dC *big.Rat) []PhaseTerm {
	return coneMomentTermsZ(R0, R1, H, z0, dS, dC)
}

// K and M are the harmonic's two phase multipliers.
func (t PhaseTerm) K() int { return t.k }
func (t PhaseTerm) M() int { return t.m }

// Ac and As are the harmonic's exact cosine and sine coefficients.
func (t PhaseTerm) Ac() *big.Rat { return t.ac }
func (t PhaseTerm) As() *big.Rat { return t.as }

func ratScale(value *big.Rat, num, den int64) *big.Rat {
	return new(big.Rat).Mul(value, big.NewRat(num, den))
}
