package capband

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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
// before that value is reached. proofbound.CrossProductUpper carries the envelope those
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
// g.SideZ is capZ + matSign*ds rounded to a float (g.LevelDelta,
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
			proofbound.CrossProductUpper(v1.Sub(v0), v2.Sub(v0)),
			proofbound.CrossProductUpper(v2.Sub(v0), v3.Sub(v0)),
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
	// This formula is the constant-slant frustum-sector shape, read against the
	// cap window alone. The patch the build holds is ruled between two windows
	// that differ at a mitered corner, and coneSkewAreaAllow is the proven
	// bound on how far that ruled patch's area sits from this formula's
	// frustum sector at the same cap window. It is zero wherever the two
	// directrices' ends lie on one ray from the centre at both corners.
	//
	// The held side radius is an ArcSeg's math.Hypot of its recorded Start, up
	// to g.Held.SideRadius from the radius the record states, and both terms
	// charge that span: coneSkewAreaAllow bounds the skew term over the whole
	// span, and sideRadiusAreaAllow bounds the frustum sector's own move.
	skewAllow := proofbound.AbsSumUpper(coneSkewAreaAllow(g), sideRadiusAreaAllow(g))
	// The core term (everything but skewAllow, contourAllow and the level
	// allowance, which stand unchanged either way) takes the SMALLER of
	// two independently sound
	// bounds: the unconditional envelope proofbound.ConservativeValueError always gives,
	// and coneFrustumAreaBracket's certified interval — sound wherever it
	// manages to build one, +Inf (hence never the min) where it cannot. This
	// can only shrink the published bound, never grow it.
	core := math.Min(
		proofbound.ConservativeValueError(area, dth*(R0+R1)*(math.Abs(dR)+math.Abs(H))),
		coneFrustumAreaBracket(R0, R1, H, dth, g.CapThAllow, area),
	)
	bound := proofbound.AbsSumUpper(core, skewAllow, patchDisplacementAreaAllow(g))
	return area, bound
}

// sideRadiusAreaAllow bounds how far the frustum sector
// A = (αc/2)·(R0+R1)·L, L = √(ΔR²+H²), moves when its side radius R0 moves
// by e = g.Held.SideRadius. L is 1-Lipschitz in R0 and never exceeds
// R0 + R1 + |H|, so |ΔA| ≤ (αc/2)·e·(L + R0 + R1 + e) ≤ αc·e·(2·(R0+R1) + |H| + e),
// with αc read at |held| + CapThAllow. Every step rounds up, and a non-finite
// input answers +Inf.
func sideRadiusAreaAllow(g Patch) float64 {
	e := g.Held.SideRadius
	if e == 0 {
		return 0
	}
	if !(e > 0) || proofbound.IsNonFinite(e) {
		return math.Inf(1)
	}
	alpha := proofbound.AbsSumUpper(math.Abs(g.CapTh1-g.CapTh0), g.CapThAllow)
	reach := proofbound.AbsSumUpper(g.SideRadius, g.CapRadius, g.SideRadius, g.CapRadius, proofbound.UpRound(math.Abs(g.CapZ-g.SideZ)), e)
	return proofbound.ProductUpper(proofbound.ProductUpper(alpha, e), reach)
}

// coneSkewAreaAllow is the proven bound on |A − A₀| for a Cone patch, where A
// is the area of the ruled patch the build holds and A₀ the frustum sector
// (Δθ/2)·(R0+R1)·√(ΔR²+H²) at the patch's exact cap sweep αc
// (docs/modify-reach-design.md §8.4 derives it).
//
// The ruled patch is P(u,t) = (1−t)·S(u) + t·C(u) over [0,1]², with the side
// end S(u) at radius R0 and angle θs(u) = θs0 + u·αs, the cap end C(u) at
// radius R1 and angle θc(u) = θc0 + u·αc, both linear in u, and the two levels
// H apart. The corner skew φ(u) = θc(u) − θs(u) is linear in u, so |φ(u)| never
// exceeds Φ = max(Φs, Φe), the larger corner skew (SkewStart, SkewEnd), and the
// side sweep differs from the cap one by αs − αc = φ(0) − φ(1), at most
// Φs + Φe. In the frame turned to θs(u), the area integrand is |V| with
//
//	V = (b·H, −a·H, a·R1·sin φ − b·(R1·cos φ − R0)),
//	a = −t·αc·R1·sin φ,  b = (1−t)·αs·R0 + t·αc·R1·cos φ,
//
// and A₀'s integrand is |G| with G = (b₀·H, 0, −b₀·ΔR), b₀ = αc·((1−t)·R0 + t·R1).
// Their difference regroups exactly as
//
//	V − G = (b−b₀)·(H, 0, −ΔR) + (0, −a·H, 0) + (0, 0, ((1−t)·αs·R0·R1 − t·αc·R1²)·(1 − cos φ)),
//	b − b₀ = (1−t)·R0·(αs − αc) − t·αc·R1·(1 − cos φ),
//
// so with |sin φ| ≤ Φ and 1 − cos φ ≤ Φ²/2, and ||V| − |G|| ≤ |V − G|
// integrated over t (each of t and 1−t integrates to 1/2):
//
//	|A − A₀| ≤ L·(R0·(Φs+Φe)/2 + αc·R1·Φ²/4) + R1·Φ²·(αs·R0 + αc·R1)/4 + H·αc·R1·Φ/2,
//
// with L = √(ΔR²+H²), αc read at its upper bound |held| + CapThAllow and αs at
// αc + Φs + Φe.
//
// R0 here is the radius the side record states, which the held SideRadius
// matches only to within e = Held.SideRadius. The term is not monotone in R0:
// L shrinks as R0 grows toward R1, so on an outward patch (R1 > R0) with a
// short slant the term is largest at the BOTTOM of the span. Every factor is
// non-negative, so the term is bounded over the whole span |r − R0| ≤ e by
// reading R0 at R0 + e and L at L(R0) + e, since L is 1-Lipschitz in R0.
//
// Every factor is lifted from a float64 exactly, the one square
// root rounds up (RatSqrtUp), and the sum is formed over rationals and rounded
// up once, so the bound assumes no ulp contract anywhere. It is exactly zero
// when both corner skews are, whatever αc is.
//
// A non-finite input answers +Inf, which publishes an unbounded area rather
// than an understated one.
func coneSkewAreaAllow(g Patch) float64 {
	if g.SkewStart == 0 && g.SkewEnd == 0 {
		return 0
	}
	rHeld, rE := proofarith.FloatRat(g.SideRadius), proofarith.FloatRat(g.Held.SideRadius)
	rR1 := proofarith.FloatRat(g.CapRadius)
	rCap, rSide := proofarith.FloatRat(g.CapZ), proofarith.FloatRat(g.SideZ)
	rDth, rAllow := proofarith.FloatRat(math.Abs(g.CapTh1-g.CapTh0)), proofarith.FloatRat(g.CapThAllow)
	rPs, rPe := proofarith.FloatRat(g.SkewStart), proofarith.FloatRat(g.SkewEnd)
	if rHeld == nil || rE == nil || rR1 == nil || rCap == nil || rSide == nil || rDth == nil || rAllow == nil || rPs == nil || rPe == nil ||
		rHeld.Sign() < 0 || rE.Sign() < 0 || rR1.Sign() < 0 || rAllow.Sign() < 0 || rPs.Sign() < 0 || rPe.Sign() < 0 {
		return math.Inf(1)
	}
	rat := func() *big.Rat { return new(big.Rat) }
	h := rat().Abs(rat().Sub(rCap, rSide))
	// R0 and L are read at their upper bounds over the held side radius span
	// |r − R0| ≤ e: R0 + e, and the held slant plus e (L is 1-Lipschitz in r).
	rR0 := rat().Add(rHeld, rE)
	dR := rat().Sub(rR1, rHeld)
	slantHeld := proofarith.FloatRat(proofbound.RatSqrtUp(rat().Add(rat().Mul(dR, dR), rat().Mul(h, h))))
	if slantHeld == nil {
		return math.Inf(1)
	}
	slant := rat().Add(slantHeld, rE)
	phi := rPs
	if rPe.Cmp(phi) > 0 {
		phi = rPe
	}
	phiSq := rat().Mul(phi, phi)
	skewSum := rat().Add(rPs, rPe)
	alphaC := rat().Add(rDth, rAllow)
	alphaS := rat().Add(alphaC, skewSum)
	half, quarter := big.NewRat(1, 2), big.NewRat(1, 4)

	// L·(R0·(Φs+Φe)/2 + αc·R1·Φ²/4)
	first := rat().Mul(slant, rat().Add(
		rat().Mul(half, rat().Mul(rR0, skewSum)),
		rat().Mul(quarter, rat().Mul(alphaC, rat().Mul(rR1, phiSq))),
	))
	// R1·Φ²·(αs·R0 + αc·R1)/4
	second := rat().Mul(quarter, rat().Mul(rR1, rat().Mul(phiSq,
		rat().Add(rat().Mul(alphaS, rR0), rat().Mul(alphaC, rR1)))))
	// H·αc·R1·Φ/2
	third := rat().Mul(half, rat().Mul(h, rat().Mul(alphaC, rat().Mul(rR1, phi))))
	return proofbound.RatFloatUp(rat().Add(first, rat().Add(second, third)))
}

// CornerSkewUpper is a proven upper bound on the exact angle about (cU, cV)
// between one corner's side directrix end and its cap directrix end, the
// corner skew coneSkewAreaAllow reads. Both ends and the centre are float64s,
// so the two directions from the centre are exact rationals, and the angle
// between them is the proofbound.Atan2Interval enclosure of their cross and dot
// products: no libm accuracy is assumed.
//
// That enclosure is the principal angle, and the skew the ruled patch takes is
// the one on the branch the held windows name, held (the cap window's own
// corner angle, put on the side window's branch, minus the side window's). The
// two agree when both lie inside (−π/2, π/2): any other branch sits more than π
// from held. ok is false otherwise, and false where either direction is zero.
func CornerSkewUpper(cU, cV float64, side, capEnd Point, held float64) (float64, bool) {
	if !(math.Abs(held) < math.Pi/2) {
		return 0, false
	}
	rc := func(a, b float64) *big.Rat {
		ra, rb := proofarith.FloatRat(a), proofarith.FloatRat(b)
		if ra == nil || rb == nil {
			return nil
		}
		return new(big.Rat).Sub(ra, rb)
	}
	au, av := rc(side.U, cU), rc(side.V, cV)
	bu, bv := rc(capEnd.U, cU), rc(capEnd.V, cV)
	if au == nil || av == nil || bu == nil || bv == nil {
		return 0, false
	}
	if (au.Sign() == 0 && av.Sign() == 0) || (bu.Sign() == 0 && bv.Sign() == 0) {
		return 0, false
	}
	cross := new(big.Rat).Sub(new(big.Rat).Mul(au, bv), new(big.Rat).Mul(av, bu))
	dot := new(big.Rat).Add(new(big.Rat).Mul(au, bu), new(big.Rat).Mul(av, bv))
	if cross.Sign() == 0 && dot.Sign() > 0 {
		return 0, true
	}
	angle := proofbound.Atan2Interval(cross, dot, false)
	upper := new(big.Rat).Abs(angle.Hi)
	if lo := new(big.Rat).Abs(angle.Lo); lo.Cmp(upper) > 0 {
		upper = lo
	}
	if upper.Cmp(proofbound.HalfPiInterval().Lo) >= 0 {
		return 0, false
	}
	return proofbound.RatFloatUp(upper), true
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
// the square root, rounded outward by proofbound.IntervalSqrt. Interval arithmetic is
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
	slantIv, ok := proofbound.IntervalSqrt(proofbound.IntervalAdd(proofbound.IntervalSquare(proofbound.PointInterval(rdR)), proofbound.IntervalSquare(proofbound.PointInterval(rH))))
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

// SkewAreaAllow bounds a Cone patch's ruled area against its frustum sector.
func SkewAreaAllow(g Patch) float64 { return coneSkewAreaAllow(g) }
