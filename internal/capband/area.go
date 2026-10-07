package capband

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// patchAreaOf is one patch's own surface area and the proven bound on it:
// a two-triangle cross-product sum for a Plane, a closed form for a Cone. The
// Plane reading is NEVER exact. Its two terms are each a float subtraction, a
// float cross product and a float norm — r3's own Len is a nested Hypot, which
// carries no ulp contract at all — and their sum rounds once more, so the held
// value is a float evaluation and proofbound.SumSlop is the proven, never-zero bound
// internal/proofbound/bounds.go keeps for that shape (the same one boolean_body.go's mesh facet
// areas take). Returning a zero bound there would publish an Exact the
// arithmetic has not earned, on the face AND in the body's own area sum.
//
// proofbound.SumSlop alone is not the whole of it, because it charges each term a few
// ulps of the term's OWN value and a band patch's cross product cancels
// before that value is reached. crossProductUpper carries the envelope those
// products actually reach, so the charge tracks the terms rather than what
// they cancelled to — the same correction capBandVolume makes for the flux.
//
// Neither arm's arithmetic bound speaks for the cap-level directrix's own
// contour displacement (capblend_contour.go): both read g's coordinates as
// exact inputs, but a chamfered patch's cap-level corner sits where a float
// offset solve put it, not where the offset denotes. g.ContourAllow is that
// allowance, computed once at build time (internal/proofbound/bounds.go's proofbound.BandPatchAreaAllow)
// from this patch's own held chord and slant, and it is zero wherever the
// band's contour displacement is zero — an axis-aligned section's exact
// miters — which is what keeps TestCapBlendPlanePatchVolumeIsExact's area
// arithmetic unchanged there.
//
// The SIDE level owes the same kind of allowance and both arms charge it:
// g.SideZ is capZ + matSign*d rounded to a float (g.LevelDelta,
// capblend_geom.go), so the whole side directrix sits that far from the level
// it denotes, and an arm reading g.SideZ as an exact input bounds only the
// patch it BUILT. It is the dominant residual wherever the sweep is large
// enough to round that sum: on a right-triangle band (legs 9e4 x 3e6 mm, 1e15
// mm sweep, 0.2 mm chamfer) levelDelta is 0.04999999999999999 mm, and
// substituting the HELD side level for the denoted one in a 512-bit reference
// drops the three patches' residuals from 3358.207374649123 /
// 111990.60743768104 / 111940.24564437279 mm^2 to 5.236122866634288e-06 /
// 8.892307483919435e-06 / 2.7272066797437425e-06 mm^2. internal/proofbound/bounds.go's
// proofbound.BandLevelAreaAllow is that charge, and it is zero wherever the sum is exact
// (an ordinary sweep and setback), which is what leaves every tight reading
// tight.
func patchAreaOf(g Patch) (float64, float64) {
	if !g.Circular {
		v0 := r3.NewVec(g.SideA.U, g.SideA.V, g.SideZ)
		v1 := r3.NewVec(g.SideB.U, g.SideB.V, g.SideZ)
		v2 := r3.NewVec(g.CapB.U, g.CapB.V, g.CapZ)
		v3 := r3.NewVec(g.CapA.U, g.CapA.V, g.CapZ)
		a1 := v1.Sub(v0).Cross(v2.Sub(v0)).Len() / 2
		a2 := v2.Sub(v0).Cross(v3.Sub(v0)).Len() / 2
		area := a1 + a2
		crossEnv := proofbound.AbsSumUpper(
			crossProductUpper(v1.Sub(v0), v2.Sub(v0)),
			crossProductUpper(v2.Sub(v0), v3.Sub(v0)),
		)
		bound := proofbound.AbsSumUpper(proofbound.SumSlop(2, proofbound.AbsSumUpper(a1, a2)), proofbound.AnalyticRoundBound(crossEnv), patchDisplacementAreaAllow(g))
		return area, bound
	}
	R0, R1 := g.SideRadius, g.CapRadius
	H := g.CapZ - g.SideZ
	dR := R1 - R0
	slant := math.Hypot(dR, H)
	// dth is the CAP-level (trimmed) sweep — the actual held chord's own
	// angle, capTh1-capTh0 — never the wall's own recorded thS1-thS0: a
	// regular wall's two directrices generally sweep different angles at a
	// non-tangent miter corner (docs/modify-reach-design.md §8.3), the same
	// fact patchRawFlux's ruled-patch integral reads off both windows.
	dth := math.Abs(g.CapTh1 - g.CapTh0)
	area := dth / 2 * (R0 + R1) * slant
	// This formula is the constant-slant frustum-sector shape unchanged; it
	// is never claimed exact even when the two windows coincide (a Plane
	// patch's own two-triangle bound above is the same kind of arithmetic-
	// only envelope, never a tight one). Where the windows genuinely differ,
	// the true ruled surface's own rulings are not all the same length —
	// this formula's own "slant" assumption — so windowSkew widens the
	// envelope by the reading the OTHER window would have given, a sound
	// (generous, not tight) allowance for the gap between the two: the true
	// area lies within the family this envelope already covers, and
	// windowSkew is zero wherever the windows coincide (a tangent join, or
	// either degenerate patch), leaving the rest of the bound unchanged.
	// A reviewer reported this term failing to enclose on a REACHABLE body — a
	// pi/2 sector sketch extruded 1e15 mm, cap chamfered 0.3 mm — quoting a
	// true ruled-patch area of 2.9431408325276074 mm^2 against a published
	// bound of 1.9647343287969778 mm^2. That reference is wrong, not this
	// bound: it applied TRAPEZOID weights (1,2,2,...,2,1 per axis) under
	// Simpson's 1/(9n^2) normalizer, so it returns exactly 4/9 of the
	// integral. Its ratio to a converged reference is 0.4444444457 at n=200,
	// 0.4444444447 at n=400 and 0.4444444445 at n=1200 — approaching 4/9
	// rather than shrinking, which a genuine discretization error would.
	// Tensor-product Gauss-Legendre on [0,1]^2 of |P_u x P_t| gives
	// 6.4373261229841905 (n=8) through 6.4373261229841869 (n=64), successive
	// deltas at float64 roundoff, and a correctly weighted composite Simpson
	// agrees at 6.4373261229841914 (n=800). The true residual there is
	// 0.6331514578014259, which this bound encloses with 3.10x margin. A sweep
	// of 451 reachable Cone patches (height 10..1e16, phi pi/6..6, d 0.05..4,
	// R 3/10/250) against GL-64/GL-96 found zero violations at a minimum
	// enclosure ratio of 2.875871x.
	//
	// What that sweep does NOT cover, and what this term is therefore still
	// only measured on: a side window of [0,2pi] against a cap window of
	// [0.25,0.251] does exceed the bound, but reaching it means assigning
	// Patch's fields directly. A regular wall's cap window is the
	// offset trimmed at its own miter feet, so the skew is corner-local, and
	// no reachable body in the sweep produced one.
	sideDth := math.Abs(g.Th1 - g.Th0)
	windowSkew := proofbound.ProductUpper(math.Abs(sideDth-dth), proofbound.ProductUpper(proofbound.AbsSumUpper(R0, R1), slant))
	// The core term (everything but windowSkew, contourAllow and the level
	// allowance below, which stand unchanged either way) takes the SMALLER of
	// two independently sound
	// bounds: the unconditional envelope proofbound.ConservativeValueError always gives,
	// and coneFrustumAreaBracket's certified interval — sound wherever it
	// manages to build one, +Inf (hence never the min) where it cannot. This
	// can only shrink the published bound, never grow it.
	core := math.Min(
		proofbound.ConservativeValueError(area, dth*(R0+R1)*(math.Abs(dR)+math.Abs(H))),
		coneFrustumAreaBracket(R0, R1, H, dth, g.CapThAllow, area),
	)
	bound := proofbound.AbsSumUpper(core, windowSkew, patchDisplacementAreaAllow(g))
	return area, bound
}

// patchDisplacementAreaAllow is the part of a band patch's own area bound that
// comes from the two DISPLACEMENTS its directrices carry rather than from the
// arithmetic that evaluated its area: the cap contour's own in-plane
// displacement (contourAllow, proofbound.BandPatchAreaAllow at build time) beside the side
// level's axial one (proofbound.BandLevelAreaAllow over levelDelta). patchAreaOf composes
// it into the bound it publishes, and the tessellator charges the same two
// terms per patch into the mesh's own area slack
// (docs/tessellation-reach-design.md §7), so neither reader can be told a
// different story about how far this patch's area can move.
//
// The level allowance is the one term neither reading of a Cone patch's core
// speaks for: coneFrustumAreaBracket lifts H as an EXACT rational (so it
// brackets the area of the patch this build HOLDS), and the fallback envelope
// covers it only by accident of its own width. The axial displacement charged
// is the side level's own rounding plus whatever the capZ − sideZ subtraction
// commits, and the frustum sector's two directrix arcs are Δθ·R0 and Δθ·R1,
// read at an upper bound on the true window rather than at the held one. A
// Plane patch's two directrices are its own two chords, each plane-local at a
// single level, so each 3D length IS its 2D one.
func patchDisplacementAreaAllow(g Patch) float64 {
	if !g.Circular {
		return proofbound.AbsSumUpper(g.ContourAllow, proofbound.BandLevelAreaAllow(g.LevelDelta,
			proofbound.AbsSumUpper(chordUpper2(g.SideA, g.SideB), chordUpper2(g.CapA, g.CapB))))
	}
	h := g.CapZ - g.SideZ
	dth := math.Abs(g.CapTh1 - g.CapTh0)
	return proofbound.AbsSumUpper(g.ContourAllow, proofbound.BandLevelAreaAllow(
		proofbound.AbsSumUpper(g.LevelDelta, proofarith.AddRoundError(g.CapZ, -g.SideZ, h)),
		proofbound.ProductUpper(proofbound.AbsSumUpper(dth, g.CapThAllow), proofbound.AbsSumUpper(g.SideRadius, g.CapRadius)),
	))
}

// chordUpper2 is a PROVEN upper bound on the distance between two plane-local
// points, taken without trusting any libm contract: the 1-norm dominates the
// 2-norm, each coordinate difference is one float subtraction, and
// proofbound.AnalyticRoundBound covers those two roundings at the sum's own magnitude.
func chordUpper2(a, b Point) float64 {
	raw := proofbound.AbsSumUpper(b.U-a.U, b.V-a.V)
	return proofbound.AbsSumUpper(raw, proofbound.AnalyticRoundBound(raw))
}

// coneFrustumAreaBracket is the certified interval bound on patchAreaOf's
// Cone-arm closed form A = (Δθ/2)·(R0+R1)·√(ΔR²+H²) — the constant-slant
// frustum-sector area that formula denotes. R0, R1 (the patch's own
// side/cap radii) and H (= capZ − sideZ) are payload float64s, hence exact
// rationals with NO rounding on the way, and ΔR is formed HERE as the exact
// rational R1−R0 rather than lifted from the caller's own float subtraction:
// fl(R1−R0) is a DIFFERENT number wherever the two radii are far enough apart
// to round, and an interval built on it encloses the area of a frustum whose
// slant is √(fl(R1−R0)²+H²) — a patch nobody holds and nobody denotes. Only
// the outward arm of the cap offset reaches that (a hole or concave arc, whose
// cap radius is R+d; the inward arm's R−d is Sterbenz-exact), and it starts
// rounding at an ordinary setback-to-radius ratio, so forming the difference
// inside is what keeps the interval a statement about the held patch.
//
// The remaining inexact factors are the sweep Δθ — bracketed by dthAllow,
// capThAllow's own proven enclosure of the true window (an proofbound.Atan2Interval
// bracket on the patch's own offset feet, or proofbound.PiLower/proofbound.PiUpper for the
// structurally whole-turn circle; capblend_contour.go/capblend_geom.go) — and
// the square root, rounded outward by survey2d.IntervalSqrt. Interval arithmetic is
// inclusion-monotonic, so the composed product [dth−dthAllow, dth+dthAllow] ×
// [R0+R1] × [slant] encloses the true A whatever the platform's Hypot, Atan2
// or Sincos returned: nowhere here is an ulp contract on any of them assumed.
// Returns +Inf where a factor fails to lift (non-finite geometry), so the
// caller's math.Min falls back to the unconditional envelope.
//
// "The true A" here is the true area of the frustum sector the HELD R0, R1
// and H describe, and lifting those three exactly is precisely what makes it
// so. It is not a claim about the patch the chamfer DENOTES: H is capZ minus
// a ROUNDED side level, and the distance between the two patches is the
// caller's own separate charge (proofbound.BandLevelAreaAllow), which no tightening here
// can ever substitute for. That charge already covers the H subtraction's own
// rounding (patchAreaOf folds addRoundError(capZ, −sideZ, H) in beside the
// level's displacement), which is why H stays a parameter where ΔR does not.
func coneFrustumAreaBracket(R0, R1, H, dth, dthAllow, held float64) float64 {
	if proofbound.IsNonFinite(dthAllow) {
		return math.Inf(1)
	}
	rR0, rR1 := proofarith.FloatRat(R0), proofarith.FloatRat(R1)
	rH := proofarith.FloatRat(H)
	rdth, rAllow := proofarith.FloatRat(dth), proofarith.FloatRat(dthAllow)
	if rR0 == nil || rR1 == nil || rH == nil || rdth == nil || rAllow == nil {
		return math.Inf(1)
	}
	rdR := new(big.Rat).Sub(rR1, rR0)
	slantIv, ok := survey2d.IntervalSqrt(proofbound.IntervalAdd(survey2d.IntervalSquare(proofbound.PointInterval(rdR)), survey2d.IntervalSquare(proofbound.PointInterval(rH))))
	if !ok {
		return math.Inf(1)
	}
	radiiIv := proofbound.PointInterval(new(big.Rat).Add(rR0, rR1))
	// [dth-dthAllow, dth+dthAllow], taken EXACTLY over rationals (never as a
	// float subtraction/addition, which could round the interval's own
	// endpoint the wrong way and silently exclude the true value it is
	// supposed to enclose).
	dthIv := proofbound.Interval(new(big.Rat).Sub(rdth, rAllow), new(big.Rat).Add(rdth, rAllow))
	areaIv := proofbound.IntervalScale(proofbound.IntervalMul(dthIv, proofbound.IntervalMul(radiiIv, slantIv)), big.NewRat(1, 2))
	return proofbound.IntervalFloatError(areaIv, held)
}

// AreaOf returns a patch's area and its proven bound.
func AreaOf(g Patch) (float64, float64) { return patchAreaOf(g) }

// DisplacementAreaAllow bounds the patch's directrix displacement.
func DisplacementAreaAllow(g Patch) float64 { return patchDisplacementAreaAllow(g) }

// FrustumAreaBracket bounds the held cone-sector area against its interval.
func FrustumAreaBracket(R0, R1, H, dth, dthAllow, held float64) float64 {
	return coneFrustumAreaBracket(R0, R1, H, dth, dthAllow, held)
}
