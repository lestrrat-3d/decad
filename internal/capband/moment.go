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
// unnormalized normal's own magnitude is bounded by proofbound.CrossProductUpper,
// independent of what the computed value happens to be.
func floatPlanePatchMoment(v0, v1, v2, v3 r3.Vec) (mx, my, mz proofbound.BoundedScalar) {
	compute := func(a, b, c r3.Vec) r3.Vec {
		n := b.Sub(a).Cross(c.Sub(a))
		sq := func(ai, bi, ci float64) float64 { return ai*ai + bi*bi + ci*ci + ai*bi + bi*ci + ci*ai }
		return r3.NewVec(n.X*sq(a.X, b.X, c.X)/24, n.Y*sq(a.Y, b.Y, c.Y)/24, n.Z*sq(a.Z, b.Z, c.Z)/24)
	}
	envelope := func(a, b, c r3.Vec) float64 {
		coordMax := math.Max(proofbound.VecMaxAbs(a), math.Max(proofbound.VecMaxAbs(b), proofbound.VecMaxAbs(c)))
		crossUpper := proofbound.CrossProductUpper(b.Sub(a), c.Sub(a))
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
	return ratTerms(momentTermsX(ratRing{}, R0, R1, H, cU, dS, dC))
}

func coneMomentTermsY(R0, R1, H, cV, dS, dC *big.Rat) []PhaseTerm {
	return ratTerms(momentTermsY(ratRing{}, R0, R1, H, cV, dS, dC))
}

func coneMomentTermsZ(R0, R1, H, z0, dS, dC *big.Rat) []PhaseTerm {
	return ratTerms(momentTermsZ(ratRing{}, R0, R1, H, z0, dS, dC))
}

// momentTerm is one PhaseTerm over the number type T: exact rationals for the
// held parameters, rational intervals for parameters boxed by their HeldAllow.
type momentTerm[T any] struct {
	k, m   int
	ac, as T
}

// ring is the arithmetic the moment coefficients are built from. ratRing
// computes them exactly; ivRing encloses them over intervals, and is
// inclusion-monotonic, so a coefficient built from boxed parameters contains
// the coefficient at every parameter value the boxes hold.
type ring[T any] interface {
	mul(xs ...T) T
	add(xs ...T) T
	scale(x T, num, den int64) T
	int(n int64) T
	zero() T
}

// Both rings satisfy ring for the number type they build over.
var (
	_ ring[*big.Rat]               = ratRing{}
	_ ring[proofbound.RatInterval] = ivRing{}
)

type ratRing struct{}

func (ratRing) mul(xs ...*big.Rat) *big.Rat               { return proofbound.RatMul(xs...) }
func (ratRing) add(xs ...*big.Rat) *big.Rat               { return proofbound.RatAdd(xs...) }
func (ratRing) scale(x *big.Rat, num, den int64) *big.Rat { return ratScale(x, num, den) }
func (ratRing) int(n int64) *big.Rat                      { return big.NewRat(n, 1) }
func (ratRing) zero() *big.Rat                            { return new(big.Rat) }

type ivRing struct{}

func (ivRing) mul(xs ...proofbound.RatInterval) proofbound.RatInterval {
	out := xs[0]
	for _, x := range xs[1:] {
		out = proofbound.IntervalMul(out, x)
	}
	return out
}

func (ivRing) add(xs ...proofbound.RatInterval) proofbound.RatInterval {
	out := xs[0]
	for _, x := range xs[1:] {
		out = proofbound.IntervalAdd(out, x)
	}
	return out
}

func (ivRing) scale(x proofbound.RatInterval, num, den int64) proofbound.RatInterval {
	return proofbound.IntervalScale(x, big.NewRat(num, den))
}

func (ivRing) int(n int64) proofbound.RatInterval {
	return proofbound.PointInterval(big.NewRat(n, 1))
}

func (ivRing) zero() proofbound.RatInterval { return proofbound.PointInterval(new(big.Rat)) }

func ratTerms(terms []momentTerm[*big.Rat]) []PhaseTerm {
	out := make([]PhaseTerm, len(terms))
	for i, t := range terms {
		out[i] = PhaseTerm(t)
	}
	return out
}

func momentTermsX[T any](r ring[T], R0, R1, H, cU, dS, dC T) []momentTerm[T] {
	two, four, nine, twentyFour := r.int(2), r.int(4), r.int(9), r.int(24)
	return []momentTerm[T]{
		{0, 0, r.scale(r.mul(H, cU, r.add(r.mul(R0, R0, dS), r.mul(R1, R1, dC))), 1, 6), r.zero()},
		{0, 1, r.scale(r.mul(H, R1, r.add(r.mul(two, R0, R0, dC), r.mul(four, R0, R0, dS), r.mul(nine, R1, R1, dC), r.mul(twentyFour, cU, cU, dC))), 1, 96), r.zero()},
		{1, 0, r.scale(r.mul(H, R0, r.add(r.mul(nine, R0, R0, dS), r.mul(four, R1, R1, dC), r.mul(two, R1, R1, dS), r.mul(twentyFour, cU, cU, dS))), 1, 96), r.zero()},
		{0, 2, r.scale(r.mul(H, R1, R1, cU, dC), 1, 6), r.zero()},
		{1, -1, r.scale(r.mul(H, R0, R1, cU, r.add(dC, dS)), 1, 12), r.zero()},
		{1, 1, r.scale(r.mul(H, R0, R1, cU, r.add(dC, dS)), 1, 12), r.zero()},
		{2, 0, r.scale(r.mul(H, R0, R0, cU, dS), 1, 6), r.zero()},
		{0, 3, r.scale(r.mul(H, R1, R1, R1, dC), 1, 32), r.zero()},
		{1, -2, r.scale(r.mul(H, R0, R1, R1, r.add(r.mul(two, dC), dS)), 1, 96), r.zero()},
		{1, 2, r.scale(r.mul(H, R0, R1, R1, r.add(r.mul(two, dC), dS)), 1, 96), r.zero()},
		{2, -1, r.scale(r.mul(H, R0, R0, R1, r.add(dC, r.mul(two, dS))), 1, 96), r.zero()},
		{2, 1, r.scale(r.mul(H, R0, R0, R1, r.add(dC, r.mul(two, dS))), 1, 96), r.zero()},
		{3, 0, r.scale(r.mul(H, R0, R0, R0, dS), 1, 32), r.zero()},
	}
}

func momentTermsY[T any](r ring[T], R0, R1, H, cV, dS, dC T) []momentTerm[T] {
	two, four, nine, twentyFour := r.int(2), r.int(4), r.int(9), r.int(24)
	return []momentTerm[T]{
		{0, 0, r.scale(r.mul(H, cV, r.add(r.mul(R0, R0, dS), r.mul(R1, R1, dC))), 1, 6), r.zero()},
		{0, 1, r.zero(), r.scale(r.mul(H, R1, r.add(r.mul(two, R0, R0, dC), r.mul(four, R0, R0, dS), r.mul(nine, R1, R1, dC), r.mul(twentyFour, cV, cV, dC))), 1, 96)},
		{1, 0, r.zero(), r.scale(r.mul(H, R0, r.add(r.mul(nine, R0, R0, dS), r.mul(four, R1, R1, dC), r.mul(two, R1, R1, dS), r.mul(twentyFour, cV, cV, dS))), 1, 96)},
		{0, 2, r.scale(r.mul(H, R1, R1, cV, dC), -1, 6), r.zero()},
		{1, -1, r.scale(r.mul(H, R0, R1, cV, r.add(dC, dS)), 1, 12), r.zero()},
		{1, 1, r.scale(r.mul(H, R0, R1, cV, r.add(dC, dS)), -1, 12), r.zero()},
		{2, 0, r.scale(r.mul(H, R0, R0, cV, dS), -1, 6), r.zero()},
		{0, 3, r.zero(), r.scale(r.mul(H, R1, R1, R1, dC), -1, 32)},
		{1, -2, r.zero(), r.scale(r.mul(H, R0, R1, R1, r.add(r.mul(two, dC), dS)), -1, 96)},
		{1, 2, r.zero(), r.scale(r.mul(H, R0, R1, R1, r.add(r.mul(two, dC), dS)), -1, 96)},
		{2, -1, r.zero(), r.scale(r.mul(H, R0, R0, R1, r.add(dC, r.mul(two, dS))), 1, 96)},
		{2, 1, r.zero(), r.scale(r.mul(H, R0, R0, R1, r.add(dC, r.mul(two, dS))), -1, 96)},
		{3, 0, r.zero(), r.scale(r.mul(H, R0, R0, R0, dS), -1, 32)},
	}
}

func momentTermsZ[T any](r ring[T], R0, R1, H, z0, dS, dC T) []momentTerm[T] {
	three, four, six, eight := r.int(3), r.int(4), r.int(6), r.int(8)
	neg := func(v T) T { return r.scale(v, -1, 1) }
	return []momentTerm[T]{
		{0, 0, r.scale(r.add(
			r.mul(H, H, R0, R0, dS),
			neg(r.mul(three, H, H, R1, R1, dC)),
			r.mul(four, H, R0, R0, dS, z0),
			neg(r.mul(eight, H, R1, R1, dC, z0)),
			r.mul(six, R0, R0, dS, z0, z0),
			neg(r.mul(six, R1, R1, dC, z0, z0)),
		), 1, 24), r.zero()},
		{1, -1, r.scale(r.mul(R0, R1, r.add(
			r.mul(three, H, H, dC),
			neg(r.mul(H, H, dS)),
			r.mul(eight, H, dC, z0),
			neg(r.mul(four, H, dS, z0)),
			r.mul(six, dC, z0, z0),
			neg(r.mul(six, dS, z0, z0)),
		)), 1, 24), r.zero()},
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
// rational (wholeTurnPhaseSum); every other patch's sum is an interval whose
// phase integrals are certified enclosures (phaseSumInterval), and the held
// value is the nearest float to its midpoint.
//
// The bound is not read off that held enclosure, because the held angles are
// float Atan2 results and an ArcSeg's held side radius a math.Hypot: the patch
// the closed band needs is the one at the reference values g.Held names. The
// same closed form is enclosed again with each angle and radius boxed by its
// allowance — the coefficients over interval arithmetic (ivRing), each phase
// integral widened by its phase's own allowance (HeldAllow.phaseAllow) — and the
// bound is that enclosure's reach from the held value. A whole-turn patch's
// windows are a full period by construction, so its boxed sum reads both
// swept angles as exactly 2π from the rational bracket of π rather than from
// the held floats. Neither grows with the arc centre's distance from the
// plane-local origin beyond the enclosure's own width.
//
// A parameter that does not lift (non-finite geometry, which buildCapBand
// already refuses) or an allowance that is not finite answers 0 with an
// infinite bound rather than a reading nothing proves. sweepCCW's negation
// mirrors patchRawFlux's own: the closed form is taken over the NORMALIZED
// (th0 < th1) window, which is the patch's actual orientation only while its
// own walk runs counter-clockwise.
func conePatchMoment(g Patch) (mu, mv, mz proofbound.BoundedScalar) {
	unproven := proofbound.MeasuredScalar(0, math.Inf(1))
	R0, R1 := proofarith.FloatRat(g.SideRadius), proofarith.FloatRat(g.CapRadius)
	z0, z1 := proofarith.FloatRat(g.SideZ), proofarith.FloatRat(g.CapZ)
	thS0, thS1 := proofarith.FloatRat(g.Th0), proofarith.FloatRat(g.Th1)
	thC0, thC1 := proofarith.FloatRat(g.CapTh0), proofarith.FloatRat(g.CapTh1)
	cU, cV := proofarith.FloatRat(g.CU), proofarith.FloatRat(g.CV)
	for _, r := range []*big.Rat{R0, R1, z0, z1, thS0, thS1, thC0, thC1, cU, cV} {
		if r == nil {
			return unproven, unproven, unproven
		}
	}
	boxes, ok := momentBoxes(g)
	if !ok {
		return unproven, unproven, unproven
	}
	H := new(big.Rat).Sub(z1, z0)
	dS := new(big.Rat).Sub(thS1, thS0)
	dC := new(big.Rat).Sub(thC1, thC0)

	phases := make(map[[2]int][2]proofbound.RatInterval)
	sum := func(terms []PhaseTerm, wide []momentTerm[proofbound.RatInterval]) proofbound.BoundedScalar {
		if g.WholeTurn {
			exact := wholeTurnPhaseSum(terms)
			held, _ := exact.Float64()
			return proofbound.MeasuredScalar(held, proofbound.IntervalFloatError(wholeTurnPhaseSumInterval(wide), held))
		}
		iv, ok := phaseSumInterval(terms, thS0, thS1, thC0, thC1, phases)
		if !ok {
			return unproven
		}
		held, _ := intervalMid(iv).Float64()
		if g.Held.zero() {
			return proofbound.MeasuredScalar(held, proofbound.IntervalFloatError(iv, held))
		}
		total := proofbound.PointInterval(new(big.Rat))
		for _, t := range wide {
			enclosure := phases[[2]int{t.k, t.m}]
			allow := g.Held.phaseAllow(t.k, t.m)
			total = proofbound.IntervalAdd(total, proofbound.IntervalAdd(
				proofbound.IntervalMul(widenBy(enclosure[0], allow), t.ac),
				proofbound.IntervalMul(widenBy(enclosure[1], allow), t.as)))
		}
		return proofbound.MeasuredScalar(held, proofbound.IntervalFloatError(total, held))
	}

	mxR := sum(coneMomentTermsX(R0, R1, H, cU, dS, dC), momentTermsX(ivRing{}, boxes.R0, boxes.R1, boxes.H, boxes.cU, boxes.dS, boxes.dC))
	myR := sum(coneMomentTermsY(R0, R1, H, cV, dS, dC), momentTermsY(ivRing{}, boxes.R0, boxes.R1, boxes.H, boxes.cV, boxes.dS, boxes.dC))
	mzR := sum(coneMomentTermsZ(R0, R1, H, z0, dS, dC), momentTermsZ(ivRing{}, boxes.R0, boxes.R1, boxes.H, boxes.z0, boxes.dS, boxes.dC))

	if !g.SweepCCW {
		mxR.Value, myR.Value, mzR.Value = -mxR.Value, -myR.Value, -mzR.Value
	}
	return mxR, myR, mzR
}

// momentParams are a Cone patch's moment parameters as intervals, each held
// number boxed by its g.Held allowance and every recorded or exactly formed one
// a point.
type momentParams struct {
	R0, R1, H, cU, cV, z0, dS, dC proofbound.RatInterval
}

// momentBoxes boxes g's moment parameters. A whole-turn patch's two swept
// angles are its true 2π, never the held window's difference. ok is false where
// a parameter does not lift or an allowance is not finite.
func momentBoxes(g Patch) (momentParams, bool) {
	if !g.Held.finite() {
		return momentParams{}, false
	}
	R0, ok0 := heldBox(g.SideRadius, g.Held.SideRadius)
	R1, ok1 := heldBox(g.CapRadius, g.Held.CapRadius)
	thS0, okS0 := heldBox(g.Th0, g.Held.Th0)
	thS1, okS1 := heldBox(g.Th1, g.Held.Th1)
	thC0, okC0 := heldBox(g.CapTh0, g.Held.CapTh0)
	thC1, okC1 := heldBox(g.CapTh1, g.Held.CapTh1)
	z0, z1 := proofarith.FloatRat(g.SideZ), proofarith.FloatRat(g.CapZ)
	cU, cV := proofarith.FloatRat(g.CU), proofarith.FloatRat(g.CV)
	if !ok0 || !ok1 || !okS0 || !okS1 || !okC0 || !okC1 || z0 == nil || z1 == nil || cU == nil || cV == nil {
		return momentParams{}, false
	}
	out := momentParams{
		R0: R0, R1: R1,
		H:  proofbound.PointInterval(new(big.Rat).Sub(z1, z0)),
		cU: proofbound.PointInterval(cU), cV: proofbound.PointInterval(cV), z0: proofbound.PointInterval(z0),
		dS: proofbound.IntervalSub(thS1, thS0),
		dC: proofbound.IntervalSub(thC1, thC0),
	}
	if g.WholeTurn {
		out.dS, out.dC = proofbound.TwoPiInterval(), proofbound.TwoPiInterval()
	}
	return out, true
}

// wholeTurnPhaseSumInterval is wholeTurnPhaseSum over interval coefficients:
// every k+m = 0 term's phase is identically zero, so it contributes its cosine
// coefficient exactly, and every other term integrates to zero over the full
// period.
func wholeTurnPhaseSumInterval(terms []momentTerm[proofbound.RatInterval]) proofbound.RatInterval {
	sum := proofbound.PointInterval(new(big.Rat))
	for _, t := range terms {
		if t.k+t.m != 0 {
			continue
		}
		sum = proofbound.IntervalAdd(sum, t.ac)
	}
	return sum
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
