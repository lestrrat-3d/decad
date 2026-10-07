package capband

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// patchRawFlux is one patch's contribution to the band's total raw flux
// (3x its own volume-by-divergence-theorem share), taken relative to the
// plane-local origin (0, 0, 0), WITH its own proven bound. A flat Plane
// patch's contribution is the tetrahedron identity
// (Flux_triangle = 3*tetraVolume = (1/2)*v0.(v1 x v2)), evaluated over
// big.Rat; a Cone patch's is the closed-form polynomial-plus-trig integral
// over its linear-in-s ruled parametrization, evaluated in floats.
//
// The two arms differ in KIND of bound and that is §8.4's own rule ("report
// each exactly representable result as Exact") deciding it, not a preference.
// A Plane patch's flux is a polynomial in the payload's own float coordinates,
// so its exact value is a rational the arithmetic can carry and the ONLY
// rounding is the final one into a float64: exactPlanePatchFlux takes it, and
// rationalFloatError reports the committed |rational - held| — zero wherever
// the true flux is a float64, which is what lets an all-Plane cap-loop band
// publish the Exact volume §8.4 owes it. Evaluating the same identity in
// floats forfeits that Exact and cannot get it back: a triple product cancels,
// so the only honest budget for a float evaluation is an envelope of the
// ABSOLUTE terms it is built from (tripleProductUpper), and an envelope over
// an exact value is still positive, which publishes Approximate. The float
// path therefore survives only as the fallback for a patch whose coordinates
// do not lift (a non-finite one), where the float result is no better.
//
// A Cone patch's flux holds sines and cosines no rational carries, so
// conePatchFluxInterval evaluates the same closed form over exact rationals
// with each trig factor read through a certified enclosure, and the bound is
// that interval's reach from its held midpoint. The enclosure always has
// width, so a mixed section holding one circular wall stays Approximate
// however exactly its Plane patches integrate. The whole-turn arm (no trig
// term survives) and a patch whose parameters do not lift keep the float
// closed form and its envelope bound.
//
// A regular wall's Cone patch (Patch.Circular with a nonzero
// sideRadius, i.e. neither a reflex corner's apex patch nor a cornerless
// whole circle) is bounded by two STRAIGHT rulings between two directrices
// that, at a non-tangential miter corner, sweep DIFFERENT angular windows
// (th0/th1 the side, capTh0/capTh1 the trimmed cap — capblend_geom.go). What
// this function integrates is the exact flux of THAT ruled patch — the
// straight-Line3-bounded surface the topology actually builds — and its bound
// above is sound for exactly that solid, but it is NOT a bound against the
// DENOTED chamfer: the true miter locus at axial fraction s is the section
// offset by s*d (docs/modify-reach-design.md §8.3's own reduction), a curve
// the straight ruling only touches at s=0 and s=1 and chords in between. That
// gap is real, grows as the setback approaches the section's own inradius (the
// cap window closes hard against the side one), and can exceed the arithmetic
// bound above by several times over — internal/proofbound/bounds.go's proofbound.ChordLocusVolumeAllow is the
// dedicated proven term for it (chordLocusResidualAllow below gathers this
// patch's own inputs to it), added into the bound whenever the two windows
// genuinely differ and left at its own zero, unchanged, wherever they
// coincide. TestCapBlendErosionFamilyVolumeBoundEncloses judges the composed
// bound against an independent erosion-family reference, including the
// setback-limit family this term exists for.
func patchRawFlux(g Patch) proofbound.BoundedScalar {
	if !g.Circular {
		v0 := r3.NewVec(g.SideA.U, g.SideA.V, g.SideZ)
		v1 := r3.NewVec(g.SideB.U, g.SideB.V, g.SideZ)
		v2 := r3.NewVec(g.CapB.U, g.CapB.V, g.CapZ)
		v3 := r3.NewVec(g.CapA.U, g.CapA.V, g.CapZ)
		if exact, ok := ExactPlaneFlux(v0, v1, v2, v3); ok {
			held, _ := exact.Float64()
			return proofbound.MeasuredScalar(held, proofarith.RationalFloatError(exact, held))
		}
		tri := func(a, b, c r3.Vec) float64 { return a.Dot(b.Cross(c)) }
		value := 0.5*tri(v0, v1, v2) + 0.5*tri(v0, v2, v3)
		// tripleProductUpper is an upper bound on every intermediate of one
		// triple product AND on the product itself, so the two together are
		// the envelope of the whole expression; the dropped halvings only
		// widen it.
		env := proofbound.AbsSumUpper(tripleProductUpper(v0, v1, v2), tripleProductUpper(v0, v2, v3))
		return proofbound.MeasuredScalar(value, proofbound.AnalyticRoundBound(env))
	}
	R0, R1 := g.SideRadius, g.CapRadius
	z0, z1 := g.SideZ, g.CapZ
	thS0, thS1 := g.Th0, g.Th1
	thC0, thC1 := g.CapTh0, g.CapTh1
	H := z1 - z0
	dS := thS1 - thS0
	dC := thC1 - thC0

	// The STRAIGHT-SLANT RULED PATCH (docs/modify-reach-design.md §8.3): the
	// surface between the SIDE arc (radius R0, level z0, sweeping thS0..thS1,
	// the wall's own full recorded window) and the CAP arc (radius R1, level
	// z1, sweeping thC0..thC1, the offset TRIMMED at the mitered corner
	// feet), joined by STRAIGHT rulings — exactly the Line3 slant edges the
	// topology actually builds — never a single rotationally-symmetric cone
	// sector spanning one shared window (the old, wrong shape whenever a
	// circular wall meets a neighbour at a non-tangential miter corner).
	//
	// P(u, v) = (1-v)*(R0*cos(thS(u)), R0*sin(thS(u)), z0)
	//         +    v *(R1*cos(thC(u)), R1*sin(thC(u)), z1),
	// thS(u) = thS0 + u*dS, thC(u) = thC0 + u*dC, u, v in [0, 1] — matching
	// all four boundary edges exactly (the two arcs at v=0/v=1, the two
	// slants at u=0/u=1). Its raw flux P.(Pu x Pv), integrated over the unit
	// square, splits — by linearity in the plane-local origin shift (cU, cV)
	// — into an ORIGIN term (one sub-term per directrix, generalizing the old
	// shared-window eccentric term1) and a LOCAL term: a pure polynomial
	// piece (poly, generalizing term2+term3) plus one CROSS term whose own
	// trig factor, ruledAngleCos, is closed form because thS(u) and thC(u)
	// are both linear in u (a product-to-sum reduction, not a numerical
	// integral). thS0=thC0 and dS=dC (a tangent join, or the degenerate
	// single-window cases below) collapses this exactly onto the OLD
	// term1+term2+term3 formula — verified algebraically and numerically,
	// never merely asserted.
	sinS0, cosS0 := math.Sincos(thS0)
	sinS1, cosS1 := math.Sincos(thS1)
	sinC0, cosC0 := math.Sincos(thC0)
	sinC1, cosC1 := math.Sincos(thC1)
	originR0 := H / 2 * R0 * (g.CU*(sinS1-sinS0) + g.CV*(cosS0-cosS1))
	originR1 := H / 2 * R1 * (g.CU*(sinC1-sinC0) + g.CV*(cosC0-cosC1))
	origin := originR0 + originR1

	// poly and cross are regrouped about the band's OWN z origin — z0, the
	// side level — rather than left as functions of the two absolute levels
	// z0 AND z1 independently: z1 = z0 + H splits each into a piece
	// proportional to z0 (the band's own position along the sweep, which can
	// sit arbitrarily far from the plane-local origin) and a piece
	// proportional to H alone (the band's own small axial extent) — the same
	// split the cU/cV origin term above already makes for the in-plane
	// eccentricity, and a constant z shift across every patch of a band and
	// the cap disk it closes on cancels in the closed-surface sum the same
	// way. Both identities are exact (reassociated, not approximated):
	// R0²z1dS/2 - R1²z0dC/2 = z0·(R1²dSC - dS·dR·(R0+R1))/2 + R0²·H·dS/2, and
	// R0R1/2·(z1dC - z0dS) = z0·R0R1·(dC-dS)/2 + R0R1·H·dC/2. dSC = dS - dC is
	// the WINDOW SKEW — exactly zero at a tangent join or either degenerate
	// patch (capTh0/capTh1 literally th0/th1) — so at every one of those
	// already-shipped junctions the z0-proportional part of poly collapses to
	// -z0·dS·dR·(R0+R1)/2, the SAME dR-scaled product the pre-restructure
	// term3 already carried, and cross's z0-proportional part vanishes
	// outright, rather than the unconstrained R0²/R1² an envelope read
	// straight off z0, z1 independently would carry.
	dR := R1 - R0
	dSC := dS - dC
	polyZ0 := R1*R1*dSC - dS*dR*(R0+R1)
	poly := z0*polyZ0/2 + R0*R0*H*dS/2
	crossZ0 := -R0 * R1 * dSC
	cross := z0*crossZ0/2 + R0*R1*H*dC/2

	absH, absR0, absR1 := math.Abs(H), math.Abs(R0), math.Abs(R1)
	absZ0, absDS, absDC, absDR, absDSC := math.Abs(z0), math.Abs(dS), math.Abs(dC), math.Abs(dR), math.Abs(dSC)
	// polyEnv envelopes poly and every intermediate of the regrouped form
	// above: the z0-proportional coefficient's own envelope (a window-skew
	// term plus a dR-scaled one, both zero-or-small at the already-shipped
	// junctions) multiplied by |z0|, plus the H-only piece — with the /2
	// divisors dropped because dropping a divisor above one only widens an
	// envelope. It carries no libm result, so its rounding is ordinary basic
	// arithmetic and proofbound.AnalyticRoundBound is the budget for it.
	polyZ0Env := proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.ProductUpper(absR1, absR1), absDSC),
		proofbound.ProductUpper(proofbound.ProductUpper(absDS, absDR), proofbound.AbsSumUpper(absR0, absR1)),
	)
	polyEnv := proofbound.AbsSumUpper(
		proofbound.ProductUpper(absZ0, polyZ0Env),
		proofbound.ProductUpper(proofbound.ProductUpper(absR0, absR0), proofbound.ProductUpper(absH, absDS)),
	)
	// polyEnv, originEnv and crossEnv serve the whole-turn arm and the
	// non-finite fallback below; every other patch takes
	// conePatchFluxInterval's enclosure instead and reads none of them.
	// origin and cross both carry a trig factor — origin through Sincos
	// directly, cross through ruledAngleCos's cos/sinc — and internal/proofbound/bounded.go's
	// proofbound.AnalyticRoundBound states the rule both break: Go gives Sin, Cos and
	// Atan2 no ulp contract, so a result computed through them never rests on
	// that roundoff budget alone. originEnv/crossEnv are the STRUCTURAL
	// magnitude envelopes that stand in its place instead — never read off
	// the computed value, so they bound the TRUE sub-term regardless of what
	// the platform's libm returned: |sin| and |cos| never exceed 1, so
	// origin's magnitude never exceeds |H|*(|R0|+|R1|)*(|cU|+|cV|), and
	// |ruledAngleCos| never exceeds 1 (|cos| <= 1, |sinc| <= 1 for every
	// real argument), so cross's magnitude never exceeds the same
	// z0-proportional-plus-H-only envelope poly's own regrouping gives it.
	originEnv := proofbound.ProductUpper(proofbound.ProductUpper(absH, proofbound.AbsSumUpper(absR0, absR1)), proofbound.AbsSumUpper(g.CU, g.CV))
	crossZ0Env := proofbound.ProductUpper(proofbound.ProductUpper(absR0, absR1), absDSC)
	crossEnv := proofbound.AbsSumUpper(
		proofbound.ProductUpper(absZ0, crossZ0Env),
		proofbound.ProductUpper(proofbound.ProductUpper(absR0, absR1), proofbound.ProductUpper(absH, absDC)),
	)

	var flux, bound float64
	switch {
	case g.WholeTurn:
		// Structural: this patch's window is a genuinely FULL period built
		// from the SAME floats on both directrices (capblend_geom.go's
		// cornerless closed circle branch, and every apex patch's degenerate
		// R0 = 0 side), so thetaS(u) == thetaC(u) identically for every u —
		// ruledAngleCos's TRUE value is exactly 1, never read off Sincos or
		// sinc here at all — and origin's own TRUE value is exactly zero (a
		// full period integrates both cos and sin to zero, capblend_geom.go's
		// structural flag, never a comparison of the windows). The held
		// origin is therefore its own whole error, and cross (now ordinary
		// arithmetic, no trig) is charged the same rounding budget poly is.
		flux = poly + cross + origin
		trigBound := proofbound.UpRound(math.Abs(origin))
		bound = proofbound.AbsSumUpper(proofbound.AnalyticRoundBound(proofbound.AbsSumUpper(polyEnv, originEnv, crossEnv)), trigBound)
	default:
		iv, ok := conePatchFluxInterval(g)
		if !ok {
			// A parameter that does not lift (non-finite geometry) leaves the
			// float closed form and its structural magnitude envelope.
			intCos := ruledAngleCos(thS0, thS1, thC0, thC1)
			trig := origin + cross*intCos
			flux = poly + trig
			trigBound := proofbound.ConservativeValueError(trig, proofbound.AbsSumUpper(originEnv, crossEnv))
			bound = proofbound.AbsSumUpper(proofbound.AnalyticRoundBound(proofbound.AbsSumUpper(polyEnv, originEnv, crossEnv)), trigBound)
			break
		}
		held, _ := intervalMid(iv).Float64()
		flux = held
		bound = proofbound.IntervalFloatError(iv, held)
	}
	if !g.WholeTurn {
		// The arithmetic bound above is only for the STRAIGHT-RULED patch
		// this evaluator actually builds; at a non-tangential corner (where
		// the cap-level window genuinely differs from the side-level one) it
		// says nothing about that patch's own gap from the TRUE curved miter
		// locus the construction denotes (internal/proofbound/bounds.go's proofbound.ChordLocusVolumeAllow).
		bound = proofbound.AbsSumUpper(bound, chordLocusResidualAllow(g))
	}
	if !g.SweepCCW {
		// The integral is taken over the NORMALIZED window th0 < th1, which
		// orients the patch radially OUTWARD from its own centre. That is the
		// orientation the patch's own walk gives only while the walk runs
		// counter-clockwise; a clockwise-walked one (a hole's whole circle, a
		// concave arc, and every reflex corner's apex connector) bounds the
		// mirror surface, so its flux is negated back to the sense its walk
		// actually has. The caller then rotates the whole band into the
		// virtual band's own orientation.
		return proofbound.MeasuredScalar(-flux, bound)
	}
	return proofbound.MeasuredScalar(flux, bound)
}

// chordLocusResidualAllow gathers this ONE patch's own inputs to
// internal/proofbound/bounds.go's proofbound.ChordLocusVolumeAllow: the two reference cone-sector fluxes
// (the wide, side-window-only reading and the narrow, cap-window-only one —
// both this same patchRawFlux formula, degenerate to the ordinary
// rotationally-symmetric cone sector once a patch's two directrices share one
// window) and this patch's own held area, the surface internal/proofbound/bounds.go's
// proofbound.SweptVolumeAllow needs. Zero wherever the two windows already coincide (a
// tangent join, or either degenerate patch), matching every already-shipped
// reading those configurations publish.
//
// g.CapTh0/g.CapTh1 and g.Th0/g.Th1 are not guaranteed to share a branch:
// capWallSweep (capblend_geom.go) anchors capTh0 at a raw Atan2, always in
// (-pi, pi], while th0 comes from the recorded arc range and can sit a full
// turn away for a major-arc wall — the same corner, described a multiple of
// 2*pi apart. capWindowOnBranch puts the cap window back on th0's own branch,
// shifting capTh0 and capTh1 together so the window's WIDTH (capTh1-capTh0)
// is untouched, before either is differenced against th0/th1 below; every
// other reader of these two fields (patchAreaOf, patchRawFlux) only ever
// takes a WITHIN-pair difference (a width), which a shared branch shift
// cannot change, so this is the one site the mismatch reaches.
func chordLocusResidualAllow(g Patch) float64 {
	capTh0, capTh1 := capWindowOnBranch(g.CapTh0, g.CapTh1, g.Th0)
	windowSkewMax := math.Max(capTh0-g.Th0, g.Th1-capTh1)
	// windowSkewMax is never negative: this is a checkable fact, not a
	// defensive assumption. The cap window is the SIDE window trimmed by
	// erosion — the cap contour is the wall's own offset, and offsetting a
	// point radially preserves its polar angle about the wall's centre — so
	// the cap window is always a SUBSET of the side window and can only be
	// narrower or equal, never wider. For a circle/line miter this reduces to
	// a closed form: with the corner's own half-angle cosine a = cos(alpha)
	// and setback d, the per-corner skew is a' - a = d(1-a)/(R+d) >= 0 for a
	// hole wall (offset radius R+d) and d(1+a)/(R-d) >= 0 for an outer wall
	// (offset radius R-d) — both non-negative for every admitted 0 < d < R
	// and -1 <= a <= 1, zero only at a tangent join (a = 1) or a
	// degenerate/whole-turn patch, which is the only way this guard fires.
	if windowSkewMax <= 0 {
		return 0
	}
	wideGeom, narrowGeom := g, g
	wideGeom.CapTh0, wideGeom.CapTh1 = g.Th0, g.Th1
	narrowGeom.Th0, narrowGeom.Th1 = capTh0, capTh1
	narrowGeom.CapTh0, narrowGeom.CapTh1 = capTh0, capTh1
	wide := patchRawFlux(wideGeom)
	narrow := patchRawFlux(narrowGeom)
	pa, pb := patchAreaOf(g)
	return proofbound.ChordLocusVolumeAllow(wide.Value, wide.Bound, narrow.Value, narrow.Bound,
		g.SideRadius, g.CapRadius, windowSkewMax, proofbound.AbsSumUpper(pa, pb))
}

// capWindowOnBranch shifts the cap-level window (capTh0, capTh1) by the
// integer multiple of 2*pi that lands capTh0 nearest th0, preserving the
// window's own width (capTh1-capTh0) exactly — a pure rotation of the branch
// index, never of the angle the window actually spans. The two windows
// describe the SAME corner (capTh0 is the offset foot near the wall's own
// th0 corner), so the nearest branch is the true one wherever a genuine
// (non-tangent) miter's own skew stays under half a turn, which a chamfer
// setback small relative to the wall's own radius always keeps it.
func capWindowOnBranch(capTh0, capTh1, th0 float64) (float64, float64) {
	shift := 2 * math.Pi * math.Round((th0-capTh0)/(2*math.Pi))
	return capTh0 + shift, capTh1 + shift
}

// ruledAngleCos is the closed form of the ruled patch's one remaining
// integral, ∫[0,1] cos(thetaS(u) - thetaC(u))du with thetaS(u) = thS0 + u*dS
// and thetaC(u) = thC0 + u*dC both linear in u: writing phi0 = thS0-thC0,
// phi1 = thS1-thC1 and delta = dS-dC, the antiderivative is
// (sin(phi0+delta)-sin(phi0))/delta = (sin(phi1)-sin(phi0))/delta, which the
// product-to-sum identity restates as cos((phi0+phi1)/2)*sinc(delta/2) — the
// numerically stable form (no difference-of-sines cancellation as delta gets
// small, and no division at all at delta = 0, the equal-window case a tangent
// join or either degenerate patch reaches). Both factors are magnitude-capped
// at 1 for every real argument, which is what lets patchRawFlux's own
// envelope bound this whole term without trusting Sin/Cos's accuracy.
func ruledAngleCos(thS0, thS1, thC0, thC1 float64) float64 {
	phi0 := thS0 - thC0
	phi1 := thS1 - thC1
	delta := (thS1 - thS0) - (thC1 - thC0)
	return math.Cos((phi0+phi1)/2) * sincHalf(delta)
}

// sincHalf returns sin(x/2)/(x/2), continuous (and exactly 1, no library call
// at all) at x = 0.
func sincHalf(x float64) float64 {
	h := x / 2
	if h == 0 {
		return 1
	}
	return math.Sin(h) / h
}

// sincHalfInterval encloses sin(x/2)/(x/2) for an exact rational x: exactly 1
// at x == 0 (no enclosure needed), otherwise the certified sine of h = x/2
// divided by the exact nonzero point h. survey2d.IntervalQuo never refuses here: the
// divisor is a nonzero point interval.
func sincHalfInterval(x *big.Rat) (proofbound.RatInterval, bool) {
	if x.Sign() == 0 {
		return proofbound.PointInterval(big.NewRat(1, 1)), true
	}
	h := new(big.Rat).Mul(x, big.NewRat(1, 2))
	sin, _, ok := survey2d.RadSinCosInterval(h)
	if !ok {
		return proofbound.RatInterval{}, false
	}
	return survey2d.IntervalQuo(sin, proofbound.PointInterval(h))
}

// phaseIntegralInterval encloses ∫₀¹ cos(a0 + u·(a1−a0)) du and the sine
// analogue for exact rational a0, a1, via the product-to-sum form
// cos(mid)·sinc(width/2), sin(mid)·sinc(width/2) that ruledAngleCos and
// phaseSumInterval already state (mid = (a0+a1)/2, width = a1−a0). Both
// factors are certified enclosures (survey2d.RadSinCosInterval, sincHalfInterval) and
// proofbound.IntervalMul is inclusion-monotonic, so each output contains the true
// integral whatever the platform's math package returns.
func phaseIntegralInterval(a0, a1 *big.Rat) (cosIv, sinIv proofbound.RatInterval, ok bool) {
	mid := new(big.Rat).Mul(new(big.Rat).Add(a0, a1), big.NewRat(1, 2))
	width := new(big.Rat).Sub(a1, a0)
	s, c, okMid := survey2d.RadSinCosInterval(mid)
	if !okMid {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	sc, okSinc := sincHalfInterval(width)
	if !okSinc {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	return proofbound.IntervalMul(c, sc), proofbound.IntervalMul(s, sc), true
}

// conePatchFluxInterval encloses the EXACT raw flux of the ruled Cone patch
// at its held parameters — the same closed form patchRawFlux's float arm
// evaluates (poly + origin + cross·intCos) — over rationals, with the four
// endpoint sines/cosines and the ruled-angle integral read through the
// certified radian enclosure. ok is false only where a parameter does not
// lift to a rational (non-finite geometry).
//
// Why the enclosure is sound, and why its reach is the whole bound:
//   - every non-trigonometric input is a float64 field, hence an exact
//     rational, and H, dS, dC, dR, dSC are formed exactly, so no subtraction
//     rounds;
//   - the four endpoint sines/cosines (survey2d.RadSinCosInterval) and the ruled-angle
//     integral (phaseIntegralInterval) are enclosed, and interval arithmetic
//     is inclusion-monotonic, so the result contains the exact closed form at
//     the held parameters whatever the platform's math package does;
//   - its caller holds the nearest float to the midpoint and publishes
//     proofbound.IntervalFloatError, the larger distance to either end rounded up, so
//     the true flux in the interval lies within that bound of the held value.
//
// Nothing here grows with the arc centre's distance from the plane-local
// origin beyond the enclosure's own width, which is set by the radian-to-turn
// grid (survey2d.TurnGridShift), not by a magnitude envelope. The width is never zero
// for a non-whole-turn patch: survey2d.RadSinCosInterval answers a non-point interval
// for every nonzero rational, so a Cone patch stays Approximate.
func conePatchFluxInterval(g Patch) (proofbound.RatInterval, bool) {
	R0, R1 := proofarith.FloatRat(g.SideRadius), proofarith.FloatRat(g.CapRadius)
	z0, z1 := proofarith.FloatRat(g.SideZ), proofarith.FloatRat(g.CapZ)
	thS0, thS1 := proofarith.FloatRat(g.Th0), proofarith.FloatRat(g.Th1)
	thC0, thC1 := proofarith.FloatRat(g.CapTh0), proofarith.FloatRat(g.CapTh1)
	cU, cV := proofarith.FloatRat(g.CU), proofarith.FloatRat(g.CV)
	for _, r := range []*big.Rat{R0, R1, z0, z1, thS0, thS1, thC0, thC1, cU, cV} {
		if r == nil {
			return proofbound.RatInterval{}, false
		}
	}
	half := big.NewRat(1, 2)
	H := new(big.Rat).Sub(z1, z0)
	dS := new(big.Rat).Sub(thS1, thS0)
	dC := new(big.Rat).Sub(thC1, thC0)
	dR := new(big.Rat).Sub(R1, R0)
	dSC := new(big.Rat).Sub(dS, dC)

	sS0, cS0, okS0 := survey2d.RadSinCosInterval(thS0)
	sS1, cS1, okS1 := survey2d.RadSinCosInterval(thS1)
	sC0, cC0, okC0 := survey2d.RadSinCosInterval(thC0)
	sC1, cC1, okC1 := survey2d.RadSinCosInterval(thC1)
	if !okS0 || !okS1 || !okC0 || !okC1 {
		return proofbound.RatInterval{}, false
	}
	// origin = H/2·R0·(cU·(sinS1−sinS0) + cV·(cosS0−cosS1))
	//        + H/2·R1·(cU·(sinC1−sinC0) + cV·(cosC0−cosC1)).
	originR0 := proofbound.IntervalScale(
		proofbound.IntervalAdd(proofbound.IntervalScale(proofbound.IntervalSub(sS1, sS0), cU), proofbound.IntervalScale(proofbound.IntervalSub(cS0, cS1), cV)),
		proofbound.RatMul(H, half, R0))
	originR1 := proofbound.IntervalScale(
		proofbound.IntervalAdd(proofbound.IntervalScale(proofbound.IntervalSub(sC1, sC0), cU), proofbound.IntervalScale(proofbound.IntervalSub(cC0, cC1), cV)),
		proofbound.RatMul(H, half, R1))
	origin := proofbound.IntervalAdd(originR0, originR1)

	// poly = z0·(R1²·dSC − dS·dR·(R0+R1))/2 + R0²·H·dS/2 and
	// cross = z0·(−R0·R1·dSC)/2 + R0·R1·H·dC/2, patchRawFlux's own regrouped
	// forms — exact identities, so the grouping costs nothing over rationals.
	polyZ0 := new(big.Rat).Sub(proofbound.RatMul(R1, R1, dSC), proofbound.RatMul(dS, dR, proofbound.RatAdd(R0, R1)))
	poly := proofbound.RatAdd(proofbound.RatMul(z0, polyZ0, half), proofbound.RatMul(R0, R0, H, dS, half))
	crossZ0 := new(big.Rat).Neg(proofbound.RatMul(R0, R1, dSC))
	cross := proofbound.RatAdd(proofbound.RatMul(z0, crossZ0, half), proofbound.RatMul(R0, R1, H, dC, half))

	// intCos = ∫₀¹ cos(thS(u) − thC(u)) du, the phase running from
	// phi0 = thS0−thC0 to phi1 = thS1−thC1: ruledAngleCos's
	// cos((phi0+phi1)/2)·sincHalf(phi1−phi0), enclosed.
	phi0 := new(big.Rat).Sub(thS0, thC0)
	phi1 := new(big.Rat).Sub(thS1, thC1)
	intCos, _, ok := phaseIntegralInterval(phi0, phi1)
	if !ok {
		return proofbound.RatInterval{}, false
	}
	return proofbound.IntervalAdd(proofbound.IntervalAdd(proofbound.PointInterval(poly), origin), proofbound.IntervalScale(intCos, cross)), true
}

// tripleProductUpper bounds |a·(b×c)| and every intermediate the float
// evaluation of it passes through: each cross-product component is a
// difference of two of the six products below, and each final product is one
// of a_i times such a difference, so their absolute sum dominates all of
// them. It is the envelope proofbound.AnalyticRoundBound is owed for a determinant,
// whose own value cancels to an arbitrarily small fraction of it.
func tripleProductUpper(a, b, c r3.Vec) float64 {
	return proofbound.AbsSumUpper(
		proofbound.ProductUpper(math.Abs(a.X), proofbound.ProductUpper(math.Abs(b.Y), math.Abs(c.Z))),
		proofbound.ProductUpper(math.Abs(a.X), proofbound.ProductUpper(math.Abs(b.Z), math.Abs(c.Y))),
		proofbound.ProductUpper(math.Abs(a.Y), proofbound.ProductUpper(math.Abs(b.Z), math.Abs(c.X))),
		proofbound.ProductUpper(math.Abs(a.Y), proofbound.ProductUpper(math.Abs(b.X), math.Abs(c.Z))),
		proofbound.ProductUpper(math.Abs(a.Z), proofbound.ProductUpper(math.Abs(b.X), math.Abs(c.Y))),
		proofbound.ProductUpper(math.Abs(a.Z), proofbound.ProductUpper(math.Abs(b.Y), math.Abs(c.X))),
	)
}

// crossProductUpper bounds every component of a×b and every product the float
// evaluation forms on the way: each component is a difference of two of the
// six products below, so their absolute sum dominates all of them AND the
// resulting vector's own norm. A band patch is a thin ruled quad, so that
// difference is exactly where its cross product cancels — the wall direction
// crossed with the offset displacement is smaller than either term by the
// ratio of the wall's length to the setback — and a budget read off the
// surviving area under-counts by that ratio.
func crossProductUpper(a, b r3.Vec) float64 {
	return proofbound.AbsSumUpper(
		proofbound.ProductUpper(math.Abs(a.Y), math.Abs(b.Z)),
		proofbound.ProductUpper(math.Abs(a.Z), math.Abs(b.Y)),
		proofbound.ProductUpper(math.Abs(a.Z), math.Abs(b.X)),
		proofbound.ProductUpper(math.Abs(a.X), math.Abs(b.Z)),
		proofbound.ProductUpper(math.Abs(a.X), math.Abs(b.Y)),
		proofbound.ProductUpper(math.Abs(a.Y), math.Abs(b.X)),
	)
}

// RawFlux returns one cap band's patch flux and its proven bound.
func RawFlux(g Patch) proofbound.BoundedScalar { return patchRawFlux(g) }

// WindowOnBranch aligns the cap window with the side window's branch.
func WindowOnBranch(capTh0, capTh1, th0 float64) (float64, float64) {
	return capWindowOnBranch(capTh0, capTh1, th0)
}

// RuledAngleCos evaluates the ruled patch's closed-form angular integral.
func RuledAngleCos(thS0, thS1, thC0, thC1 float64) float64 {
	return ruledAngleCos(thS0, thS1, thC0, thC1)
}

// ConeFluxInterval encloses a cone patch's raw flux.
func ConeFluxInterval(g Patch) (proofbound.RatInterval, bool) { return conePatchFluxInterval(g) }

func intervalMid(a proofbound.RatInterval) *big.Rat {
	return new(big.Rat).Mul(new(big.Rat).Add(a.Lo, a.Hi), big.NewRat(1, 2))
}

// ExactPlaneFlux is the flat quad patch's raw flux
// (1/2)*v0.(v1 x v2) + (1/2)*v0.(v2 x v3) as an exact rational. Every
// coordinate is a held float64, and the result rounds only at the caller.
// A non-finite coordinate returns false.
func ExactPlaneFlux(v0, v1, v2, v3 r3.Vec) (*big.Rat, bool) {
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
		return nil, false
	}
	// Halving is a shift, so the flux stays dyadic until this boundary.
	sum := proofarith.DyAdd(proofarith.DvDot(r0, proofarith.DvCross(r1, r2)), proofarith.DvDot(r0, proofarith.DvCross(r2, r3v)))
	return proofarith.DyShift(sum, -1).Rat(), true
}
