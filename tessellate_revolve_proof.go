package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the proof half of docs/tessellation-design.md §13's increment T2
// (docs/tessellation-reach-design.md §6, R3): everything the revolve
// tessellator must PROVE about the mesh it assembles, kept apart from the
// assembly itself in tessellate_revolve.go.
//
// It owns four things:
//
//   - §8's two coordinate stages. deltaC is the displacement from every IDEAL
//     unplaced sample — the exact evaluation of X(z, ρ, φ) on the payload's own
//     floats — to the binary64 vertex this build stored for it, enclosed with
//     rational intervals throughout and never with a library trig call. deltaR
//     is the displacement from the exact rigid image of that stored unplaced
//     vertex to the final placed one.
//   - §8's tolerance split, which reserves both of them before a single chord
//     is chosen and refuses when nothing is left.
//   - §9's endpoint and homotopy audits, discharged in ONE pass over the final
//     stored triangles (revolveContactAudit's own doc comment carries the
//     derivation).
//   - §10.2's Ecell, the cut-stable area allowance of one wall cell, in closed
//     form by complete sign decomposition (revolveCellAreaSlack).
//
// Nothing here samples anything to decide anything. Every enclosure is a
// ratInterval built from the payload's held float64s, which are exact
// rationals, so the answers do not move with the platform's libm or with FMA
// contraction.

// revolveTrigGapPrior is the a-priori ceiling on how far one stored cosine or
// sine sits from the exact value it stands for. The tessellator does not call
// math.Sincos: it encloses the ideal angle's cosine and sine as rational
// intervals (turnSinCosInterval / radSinCosInterval, neither of which ever
// compares against π) and stores the float64 NEAREST the enclosure's midpoint.
// So the stored value is within half an ulp of a point inside an enclosure
// whose own width is below 2⁻¹⁸⁰ — comfortably inside 2⁻⁵⁰ — and the bound is
// a fact about the construction rather than an assumption about a library.
//
// It exists because §8 orders the tolerance split BEFORE the chord counts, and
// the count decides how many angles there are: the budget therefore needs a
// bound that does not depend on the count. The tessellator MEASURES the real
// gap at every angle it emits and refuses if any of them exceeds this ceiling,
// so the a-priori figure is held to account rather than trusted.
const revolveTrigGapPrior = 0x1p-50

// revolveEvalRoundUlps is the a-priori ulp allowance, at the mesh's own
// coordinate magnitude, for the float64 arithmetic that turns the payload's
// numbers into one stored unplaced vertex: the axis basis (a3, w, e0, e1 — two
// products and two sums each, plus e1's cross product), then X's own three
// scalings and three vector sums. Twenty-odd roundings, each at most half an
// ulp at a magnitude the coordinate envelope covers, so 256 is generous by an
// order of magnitude and still lands far below any tolerance a caller can
// state. As with the trig ceiling, the measured deltaC is checked against the
// budget this figure bought.
const revolveEvalRoundUlps = 256

// revolveStationRoundUlps is the a-priori ulp allowance, at the meridian's own
// coordinate magnitude, for one CHORDED meridian station's stored (z, ρ) pair
// (docs/tessellation-reach-design.md §6, R4). A station is stored as the float
// NEAREST the certified enclosure of the point its record denotes
// (revolveArcStation), so its gap is half an ulp plus that enclosure's own
// width; eight ulps covers both with room to spare while staying far below any
// tolerance a caller can state.
//
// It exists for revolveTrigGapPrior's reason one level down: §8 splits the
// tolerance BEFORE the meridian counts are chosen, and how many stations there
// are is exactly what the count decides, so the split needs a per-sample
// ceiling that does not depend on it. Every station's real gap is MEASURED as
// it is emitted and refused if it exceeds this, so the a-priori figure is held
// to account rather than trusted.
const revolveStationRoundUlps = 8

// revolveAngular is the global angular sequence docs/tessellation-design.md §8
// makes load-bearing: ONE chord count for the whole mesh, so adjacent generator
// faces share their complete latitude edge, a full turn closes without a
// tolerance seam, and one cell proof applies at every radius.
//
// cos/sin are the stored values every ring is built from; cosIv/sinIv are the
// certified enclosures of the IDEAL angle they stand for. gap is the largest
// measured distance between the two over the whole sequence.
type revolveAngular struct {
	n       int // angular chord count (the number of angular INTERVALS)
	samples int // vertices per off-axis ring: n for a full turn, n+1 for a partial sweep
	cos     []float64
	sin     []float64
	cosIv   []ratInterval
	sinIv   []ratInterval
	gap     float64
	// step is the exact enclosure of ONE angular interval's true width, the
	// dφ every Ecell integral reads.
	step ratInterval
}

// revolveAngularSequence builds the angular sequence for n chords.
//
// A partial sweep's angle φ0 + l·(φ1 − φ0)/n is an exact rational in the
// payload's own two floats, so radSinCosInterval encloses it directly. A full
// turn starting at zero is l/n of a TURN, which turnSinCosInterval encloses
// without π entering at all; a full turn starting elsewhere falls back to the
// same radian enclosure over φ0 + 2π·l/n, widened by the 2π enclosure's own
// (sub-2⁻²⁴⁰) width.
func revolveAngularSequence(phi0, phi1 float64, full bool, n int) (revolveAngular, error) {
	if n <= 0 {
		return revolveAngular{}, fmt.Errorf(`%w: a revolve mesh needs at least one angular chord`, ErrDegenerate)
	}
	r0, r1 := floatRat(phi0), floatRat(phi1)
	if r0 == nil || r1 == nil {
		return revolveAngular{}, fmt.Errorf(`%w: the sweep interval is not finite, so no angular sample can be enclosed`, ErrUnsupported)
	}
	out := revolveAngular{n: n, samples: n + 1}
	if full {
		out.samples = n
	}
	nRat := new(big.Rat).SetInt64(int64(n))
	if full {
		out.step = intervalScale(twoPiInterval(), new(big.Rat).Inv(nRat))
	} else {
		out.step = pointInterval(new(big.Rat).Quo(new(big.Rat).Sub(r1, r0), nRat))
	}
	for l := range out.samples {
		frac := new(big.Rat).SetFrac64(int64(l), int64(n))
		var cosIv, sinIv ratInterval
		switch {
		case full && r0.Sign() == 0:
			sinIv, cosIv = turnSinCosInterval(frac)
		case full:
			angle := intervalAdd(pointInterval(r0), intervalScale(twoPiInterval(), frac))
			var ok bool
			sinIv, cosIv, ok = radSinCosSpan(angle)
			if !ok {
				return revolveAngular{}, errRevolveAngleEnclosure
			}
			// A full turn's last interval closes onto its first sample, so the
			// sequence never states φ1 and no seam ring is emitted.
		default:
			angle := new(big.Rat).Add(r0, new(big.Rat).Mul(frac, new(big.Rat).Sub(r1, r0)))
			var ok bool
			sinIv, cosIv, ok = radSinCosInterval(angle)
			if !ok {
				return revolveAngular{}, errRevolveAngleEnclosure
			}
		}
		cosHeld, _ := intervalMid(cosIv).Float64()
		sinHeld, _ := intervalMid(sinIv).Float64()
		if isNonFinite(cosHeld) || isNonFinite(sinHeld) {
			return revolveAngular{}, errRevolveAngleEnclosure
		}
		gap := math.Max(intervalFloatError(cosIv, cosHeld), intervalFloatError(sinIv, sinHeld))
		if isNonFinite(gap) || gap > revolveTrigGapPrior {
			return revolveAngular{}, fmt.Errorf(`%w: an angular sample's stored cosine and sine sit farther from the angle they denote than this mesh reserved for them`, ErrUnsupported)
		}
		out.cos = append(out.cos, cosHeld)
		out.sin = append(out.sin, sinHeld)
		out.cosIv = append(out.cosIv, cosIv)
		out.sinIv = append(out.sinIv, sinIv)
		out.gap = math.Max(out.gap, gap)
	}
	return out, nil
}

var errRevolveAngleEnclosure = fmt.Errorf(`%w: an angular sample's cosine and sine cannot be enclosed, so this mesh can state no construction bound`, ErrUnsupported)

// revolveIdealBasis is docs/tessellation-design.md §8's axis basis as the EXACT
// expression the payload's own floats denote, rather than the float64 triple
// the build stores for it: a3 = O + aU·U + aV·V, w = dU·U + dV·V,
// e0 = −dV·U + dU·V and e1 = w × e0. The gap between this and the stored basis
// is one of the terms deltaC measures.
func revolveIdealBasis(rp revolvePayload) (revolveBasis3Iv, bool) {
	origin, ok0 := ivVec3Of(rp.frame.Origin())
	fu, ok1 := ivVec3Of(rp.frame.U())
	fv, ok2 := ivVec3Of(rp.frame.V())
	aU, aV := floatRat(rp.ax.aU), floatRat(rp.ax.aV)
	dU, dV := floatRat(rp.ax.dU), floatRat(rp.ax.dV)
	if !ok0 || !ok1 || !ok2 || aU == nil || aV == nil || dU == nil || dV == nil {
		return revolveBasis3Iv{}, false
	}
	scale := func(v ivVec3, s *big.Rat) ivVec3 { return ivVec3Mul(v, pointInterval(s)) }
	a3 := ivVec3Add(origin, ivVec3Add(scale(fu, aU), scale(fv, aV)))
	w := ivVec3Add(scale(fu, dU), scale(fv, dV))
	e0 := ivVec3Add(scale(fu, new(big.Rat).Neg(dV)), scale(fv, dU))
	return revolveBasis3Iv{a3: a3, w: w, e0: e0, e1: ivVec3Cross(w, e0)}, true
}

// revolveBasis3Iv is revolveBasis enclosed exactly.
type revolveBasis3Iv struct {
	a3, w, e0, e1 ivVec3
}

// revolveMeridianEnclosure encloses one meridian sample's IDEAL axis
// coordinates (z, ρ) from the recorded plane-local point the axis
// re-expression consumed.
//
// Two displacements enter here that no other stage can state. The recorded
// point itself is only known to within the walk's own endpoint bound (a trimmed
// line's lerp), and axisFrame.walk SNAPS a radial coordinate inside snapTol
// onto exactly zero, which is decad's own act rather than a recorded number. A
// pole vertex therefore carries the whole discarded magnitude as construction
// displacement, and only a sample the arithmetic already put exactly on the
// axis carries none.
func revolveMeridianEnclosure(ax axisFrame, u, v float64, bound walkEndBound) (ratInterval, ratInterval, bool) {
	ru, rv := floatRat(u), floatRat(v)
	bu, bv := floatRat(math.Abs(bound.u)), floatRat(math.Abs(bound.v))
	if ru == nil || rv == nil || bu == nil || bv == nil {
		return ratInterval{}, ratInterval{}, false
	}
	return axisCoordInterval(ax,
		intervalWiden(pointInterval(ru), bu),
		intervalWiden(pointInterval(rv), bv),
	)
}

// axisCoordInterval carries an enclosed PLANE-local point into the axis
// coordinates (z, ρ) through the payload's own axis frame, with no rounding
// anywhere: aU/aV/dU/dV are float64 and therefore exact rationals, and the
// whole map is two products and one sum per coordinate.
//
// The frame's own four numbers are read as exact leaves, which is what
// axisFrame.toAxis itself denotes — the IDEAL sample docs/tessellation-design.md
// §8 measures a stored vertex against is the exact evaluation on the payload's
// held floats, not on the axis a longer derivation would call true. The
// difference between those two axes is the frame's own recorded uncertainty
// (axisFrame.toAxisRhoBound), which revolve's moments engine folds in
// separately and no mesh coordinate re-charges here.
func axisCoordInterval(ax axisFrame, u, v ratInterval) (ratInterval, ratInterval, bool) {
	aU, aV := floatRat(ax.aU), floatRat(ax.aV)
	dU, dV := floatRat(ax.dU), floatRat(ax.dV)
	if aU == nil || aV == nil || dU == nil || dV == nil {
		return ratInterval{}, ratInterval{}, false
	}
	du := intervalSub(u, pointInterval(aU))
	dv := intervalSub(v, pointInterval(aV))
	z := intervalAdd(intervalScale(du, dU), intervalScale(dv, dV))
	rho := intervalSub(intervalScale(dv, dU), intervalScale(du, dV))
	return z, rho, true
}

// revolveIdealPoint encloses X(z, ρ, φ) exactly: the ideal unplaced sample
// docs/tessellation-design.md §8 measures every stored vertex against.
func revolveIdealPoint(b revolveBasis3Iv, z, rho, cos, sin ratInterval) ivVec3 {
	radial := ivVec3Add(ivVec3Mul(b.e0, cos), ivVec3Mul(b.e1, sin))
	return ivVec3Add(b.a3, ivVec3Add(ivVec3Mul(b.w, z), ivVec3Mul(radial, rho)))
}

// revolveCoordMax is §8's upward-rounded envelope of every ideal unplaced
// analytic-boundary coordinate. It is the magnitude the construction rounds AT,
// which is what both a-priori allowances below are stated against.
func revolveCoordMax(b revolveBasis, zAbsMax, rhoMax float64) float64 {
	worst := 0.0
	for _, axis := range [3]int{0, 1, 2} {
		component := func(v r3.Vec) float64 {
			switch axis {
			case 0:
				return v.X
			case 1:
				return v.Y
			default:
				return v.Z
			}
		}
		worst = math.Max(worst, upRound(absSumUpper(
			component(b.a3),
			productUpper(zAbsMax, math.Abs(component(b.w))),
			productUpper(rhoMax, absSumUpper(component(b.e0), component(b.e1))),
		)))
	}
	return worst
}

// revolveConstructionPrior is the count-independent ceiling on deltaC the
// tolerance split spends before any angular count exists (§8 step 1).
//
// Every term of the ideal-to-stored gap is bounded here without knowing how
// many angles the mesh will carry: meridianGap is the largest gap a meridian
// sample's own (z, ρ) already showed, the trig term is the stored cosine and
// sine's ceiling times the radius they scale, and the last term is the float
// arithmetic's own ulps at the coordinate envelope. The tessellator measures
// the real deltaC afterwards and refuses if it exceeds what this bought.
func revolveConstructionPrior(b revolveBasis, meridianGap, rhoMax, coordMax float64) float64 {
	radialUnit := absSumUpper(vecMaxAbs(b.e0), vecMaxAbs(b.e1))
	perCoord := absSumUpper(
		productUpper(meridianGap, absSumUpper(vecMaxAbs(b.w), radialUnit)),
		productUpper(productUpper(rhoMax, revolveTrigGapPrior), radialUnit),
		productUpper(revolveEvalRoundUlps, ulpOf(math.Max(coordMax, 1))),
	)
	return radius3D(perCoord)
}

// revolveBudget is docs/tessellation-design.md §8's tolerance split: both
// coordinate stages are reserved from the requested tolerance BEFORE any chord
// count is chosen, and a tolerance they exhaust refuses rather than returning a
// mesh whose bound it cannot honour. Both subtractions round downward, which is
// the direction that leaves the reservation whole.
func revolveBudget(tol, deltaC, deltaR float64) (float64, error) {
	available := downRound(downRound(tol - deltaC - deltaR))
	if available <= 0 || isNonFinite(available) {
		requested := units.Millimeters(tol)
		reserved := units.Millimeters(absSumUpper(deltaC, deltaR))
		return 0, fmt.Errorf(`%w: requested tolerance %s leaves no chord budget above this revolve's own coordinate construction and placement displacement %s; retry with a tolerance greater than %s`, ErrUnsupported, requested, reserved, reserved)
	}
	return available, nil
}

// exactRigidPointRound measures the displacement docs/tessellation-design.md §8
// calls deltaR for ONE vertex: the gap between the EXACT rigid image of the
// stored unplaced point and the binary64 vertex the placement wrote for it. It
// is exactPrismPointRound's second half, applied to a point this build already
// holds rather than to a plane-local triple, and it answers exactly zero for an
// identity placement, whose products are by one and zero and whose sums commit
// no rounding at all.
func exactRigidPointRound(xform r3.Transform, unplaced, held r3.Vec) float64 {
	if !finiteVec(unplaced) || !finiteVec(held) {
		return math.Inf(1)
	}
	basis := xform.Basis()
	translation := xform.Translation()
	if !finiteVec(basis.EX) || !finiteVec(basis.EY) || !finiteVec(basis.EZ) || !finiteVec(translation) {
		return math.Inf(1)
	}
	p := ratVec(unplaced)
	ex, ey, ez, t := ratVec(basis.EX), ratVec(basis.EY), ratVec(basis.EZ), ratVec(translation)
	perCoord := 0.0
	for i := range 3 {
		exact := new(big.Rat).Add(
			new(big.Rat).Add(
				new(big.Rat).Mul(ex[i], p[0]),
				new(big.Rat).Mul(ey[i], p[1]),
			),
			new(big.Rat).Add(
				new(big.Rat).Mul(ez[i], p[2]),
				t[i],
			),
		)
		perCoord = math.Max(perCoord, rationalFloatError(exact, vecComponent(held, i)))
	}
	return radius3D(perCoord)
}

func vecComponent(v r3.Vec, i int) float64 {
	switch i {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

// revolveCellAreaSlack is docs/tessellation-design.md §10.2's Ecell for one
// wall cell of a LINE generator, in closed form by COMPLETE SIGN
// DECOMPOSITION — tess §15's first admissible path, chosen here because the
// decomposition turns out to need no root isolation at all.
//
// The reason is that both densities collapse. Over the common domain
// (t, u) ∈ [0,1]², with t along the meridian chord and u across one angular
// interval, the true patch is
//
//	Ftrue(t, u) = a3 + z(t)·w + ρ(t)·e(φ0 + u·dφ)
//
// with z and ρ AFFINE in t, so ∂Ftrue/∂t × ∂Ftrue/∂u = ρ(t)·dφ·(ρ'·w − z'·e),
// and w ⊥ e are both unit, giving
//
//	Jtrue(t, u) = L·dφ·ρ(t),   L = √(z'² + ρ'²)
//
// which does not depend on u at all. The held facet is FLAT, so its own
// parameterisation is affine and Jheld is the constant 2·area over each half of
// the domain — the half the fixed diagonal cuts. Their difference is therefore
// LINEAR in t on each half, its single zero is an exact rational quotient, and
// each sign-fixed piece integrates in closed form. No polynomial root
// isolation, no interval subdivision, and clearance_poly.go's Sturm engine is
// not reached: the certified enclosures of cos dφ and sin dφ enter only through
// the ideal triangle's own area, never inside a root isolation, so the
// widening tess §9's open question worried about cannot lose a sign here.
//
// The two halves carry the weights their domains give them: the diagonal splits
// the unit square into {0 ≤ u ≤ t ≤ 1}, whose u-measure at t is t, and
// {0 ≤ t ≤ u ≤ 1}, whose u-measure is 1 − t.
//
// Every input is an enclosure, and the answer is an upper bound on
// ∫|Jtrue − Jheld| for EVERY member of it: with f_lo ≤ f ≤ f_hi pointwise,
// |f| ≤ |f_lo| + (f_hi − f_lo), and both of those integrate exactly.
//
// The answer is a BOUND on the local density difference, never an estimate of
// the two areas' own gap, and §10.2 forbids it from cancelling. A planar
// annulus cell shows the difference plainly: its true density is linear in the
// meridian parameter while its flat facet's is constant, so the non-cancelling
// integral reads well above the near-agreement of the two total areas. That is
// the reading a later boolean needs, since a boolean can retain one sign lobe
// of an error whose whole-cell sum vanishes.
//
// rho0/rho1 are the cell's two meridian radii, meridian encloses L, step
// encloses dφ, and twoArea encloses twice the ideal triangle's area — one entry
// per half, in the order (diagonal-low half, diagonal-high half).
func revolveCellAreaSlack(rho0, rho1 float64, meridian, step ratInterval, twoArea [2]ratInterval) float64 {
	r0, r1 := floatRat(rho0), floatRat(rho1)
	if r0 == nil || r1 == nil {
		return math.Inf(1)
	}
	dRho := new(big.Rat).Sub(r1, r0)
	scale := intervalMul(meridian, step)
	total := new(big.Rat)
	for half, weight := range [2]int{revolveWeightT, revolveWeightOneMinusT} {
		total.Add(total, revolveHalfCellSlack(r0, dRho, scale, twoArea[half], weight))
	}
	return ratFloatUp(total)
}

// revolveFanAreaSlack is revolveCellAreaSlack for a cell with ONE ring on the
// axis: the held facets are a fan of single triangles rather than quads, and
// the domain is the whole unit square with the pole edge collapsed.
//
// With the pole at t = 0 the held map is P + t·((1−u)·A + u·B), whose Jacobian
// is exactly t·|A × B|, while Jtrue is L·dφ·ρ1·t — both LINEAR in t through the
// origin, so the difference is again linear and the same closed form answers.
// The pole at t = 1 is the mirror image, in 1 − t.
func revolveFanAreaSlack(rhoOff float64, poleFirst bool, meridian, step, twoArea ratInterval) float64 {
	r := floatRat(rhoOff)
	if r == nil {
		return math.Inf(1)
	}
	// f(t) = (L·dφ·ρ_off − 2A)·t for a pole at t = 0, and the same constant
	// times (1 − t) for a pole at t = 1.
	c := intervalSub(intervalScale(intervalMul(meridian, step), r), twoArea)
	lo, hi := c.lo, c.hi
	alpha, beta := lo, new(big.Rat)
	if !poleFirst {
		alpha, beta = new(big.Rat).Neg(lo), new(big.Rat).Set(lo)
	}
	width := new(big.Rat).Sub(hi, lo)
	widthAlpha, widthBeta := width, new(big.Rat)
	if !poleFirst {
		widthAlpha, widthBeta = new(big.Rat).Neg(width), new(big.Rat).Set(width)
	}
	total := new(big.Rat).Add(
		absLinearIntegral(alpha, beta, revolveWeightOne),
		absLinearIntegral(widthAlpha, widthBeta, revolveWeightOne),
	)
	return ratFloatUp(total)
}

// revolveHalfCellSlack is one half-domain's contribution to Ecell: the exact
// ∫|f_lo|·w plus the exact ∫(f_hi − f_lo)·w, with
// f(t) = L·dφ·(ρ0 + t·Δρ) − 2A.
func revolveHalfCellSlack(rho0, dRho *big.Rat, scale, twoArea ratInterval, weight int) *big.Rat {
	alphaLo := new(big.Rat).Mul(scale.lo, dRho)
	betaLo := new(big.Rat).Sub(new(big.Rat).Mul(scale.lo, rho0), twoArea.hi)
	spread := new(big.Rat).Sub(scale.hi, scale.lo)
	alphaGap := new(big.Rat).Mul(spread, dRho)
	betaGap := new(big.Rat).Add(
		new(big.Rat).Mul(spread, rho0),
		new(big.Rat).Sub(twoArea.hi, twoArea.lo),
	)
	return new(big.Rat).Add(
		absLinearIntegral(alphaLo, betaLo, weight),
		absLinearIntegral(alphaGap, betaGap, weight),
	)
}

// The three weights absLinearIntegral integrates against: the whole unit
// interval, and the two halves the fixed cell diagonal cuts the unit square
// into.
const (
	revolveWeightOne = iota
	revolveWeightT
	revolveWeightOneMinusT
)

// absLinearIntegral is the exact ∫₀¹ |α·t + β|·w(t) dt over the rationals. The
// integrand's single zero −β/α is isolated exactly, the unit interval is split
// there when the zero falls strictly inside it, and each sign-fixed piece is
// integrated through its own polynomial primitive. This is the whole of
// docs/tessellation-design.md §10.2's "isolate every zero ... then integrate
// each sign-fixed region in closed form" for a straight generator.
func absLinearIntegral(alpha, beta *big.Rat, weight int) *big.Rat {
	one := big.NewRat(1, 1)
	bounds := []*big.Rat{new(big.Rat), one}
	if alpha.Sign() != 0 {
		root := new(big.Rat).Quo(new(big.Rat).Neg(beta), alpha)
		if root.Sign() > 0 && root.Cmp(one) < 0 {
			bounds = []*big.Rat{new(big.Rat), root, one}
		}
	}
	total := new(big.Rat)
	for i := 0; i+1 < len(bounds); i++ {
		piece := new(big.Rat).Sub(
			linearWeightPrimitive(alpha, beta, bounds[i+1], weight),
			linearWeightPrimitive(alpha, beta, bounds[i], weight),
		)
		total.Add(total, piece.Abs(piece))
	}
	return total
}

// linearWeightPrimitive evaluates the antiderivative of (α·t + β)·w(t) at t.
func linearWeightPrimitive(alpha, beta, t *big.Rat, weight int) *big.Rat {
	t2 := new(big.Rat).Mul(t, t)
	t3 := new(big.Rat).Mul(t2, t)
	switch weight {
	case revolveWeightT:
		// (αt + β)·t = αt² + βt.
		return new(big.Rat).Add(
			new(big.Rat).Mul(alpha, new(big.Rat).Mul(t3, big.NewRat(1, 3))),
			new(big.Rat).Mul(beta, new(big.Rat).Mul(t2, big.NewRat(1, 2))),
		)
	case revolveWeightOneMinusT:
		// (αt + β)(1 − t) = −αt² + (α − β)t + β.
		return new(big.Rat).Add(
			new(big.Rat).Add(
				new(big.Rat).Mul(new(big.Rat).Neg(alpha), new(big.Rat).Mul(t3, big.NewRat(1, 3))),
				new(big.Rat).Mul(new(big.Rat).Sub(alpha, beta), new(big.Rat).Mul(t2, big.NewRat(1, 2))),
			),
			new(big.Rat).Mul(beta, t),
		)
	default:
		// (αt + β)·1.
		return new(big.Rat).Add(
			new(big.Rat).Mul(alpha, new(big.Rat).Mul(t2, big.NewRat(1, 2))),
			new(big.Rat).Mul(beta, t),
		)
	}
}

// ivTwoTriangleArea encloses twice the area of a triangle whose corners are
// themselves enclosed — the |A × B| the held facet's own Jacobian is.
func ivTwoTriangleArea(p0, p1, p2 ivVec3) (ratInterval, bool) {
	n := ivVec3Cross(ivVec3Sub(p1, p0), ivVec3Sub(p2, p0))
	return intervalSqrt(ivVec3NormSq(n))
}

// revolveAuditTri is one triangle's exact lift, held for the whole audit: its
// three corners, the three edge vectors (u = p1−p0, v = p2−p0, w = p2−p1),
// their cross product, and proven upper bounds on the three edge lengths.
// Every predicate below reads these rather than rebuilding them per pair,
// exactly as loft_audit.go's own audit data does.
type revolveAuditTri struct {
	p          [3]ratV3
	u, v, w, n ratV3
	lu, lv, lw float64
	box        [2]r3.Vec
	// off[k] holds the two corner offsets measured from corner k, in
	// increasing corner order, each with its proven length bound.
	off [3][2]revolveOffset
}

func newRevolveAuditTri(verts []r3.Vec, tri [3]int) (revolveAuditTri, bool) {
	var out revolveAuditTri
	for k, vi := range tri {
		if !finiteVec(verts[vi]) {
			return out, false
		}
		out.p[k] = ratVec(verts[vi])
	}
	out.u = rvSub(out.p[1], out.p[0])
	out.v = rvSub(out.p[2], out.p[0])
	out.w = rvSub(out.p[2], out.p[1])
	out.n = rvCross(out.u, out.v)
	out.lu = rvLenUpper(out.u)
	out.lv = rvLenUpper(out.v)
	out.lw = rvLenUpper(out.w)
	out.box = triBox(verts, tri)
	out.off[0] = [2]revolveOffset{{v: out.u, length: out.lu}, {v: out.v, length: out.lv}}
	out.off[1] = [2]revolveOffset{revolveOffsetOf(rvSub(out.p[0], out.p[1])), {v: out.w, length: out.lw}}
	out.off[2] = [2]revolveOffset{revolveOffsetOf(rvSub(out.p[0], out.p[2])), revolveOffsetOf(rvSub(out.p[1], out.p[2]))}
	return out, true
}

// revolveSeparated proves two triangles stay farther apart than 2·delta, so
// no member of the displaced family they stand for can touch. It is the
// separating-axis theorem over the exact rationals: a single axis on which the
// two projections leave a gap wider than the two triangles' own displacement
// budget proves the disjointness for the whole family, since a point sliding by
// at most delta moves its projection onto a unit axis by at most delta.
//
// The seventeen candidates are the two face normals, the nine edge-pair cross
// products, and each face normal crossed with its own three edges — the set
// that decides every disjoint pair of triangles, coplanar ones included. A
// candidate that vanishes carries no information and is skipped, and failing on
// all of them is a refusal, never an admission. The axes are built in the same
// order as before, one at a time as the loop reaches them, and each one's
// length bound is tried in its cheap form (|x × y| ≤ |x||y|) before its exact
// form, so a pair that a low-index axis decides never pays for the rest.
func revolveSeparated(a, b revolveAuditTri, delta float64) bool {
	margin := productUpper(2, delta)
	if isNonFinite(margin) {
		return false
	}
	ea := [3]ratV3{a.u, a.v, a.w}
	eb := [3]ratV3{b.u, b.v, b.w}
	la := [3]float64{a.lu, a.lv, a.lw}
	lb := [3]float64{b.lu, b.lv, b.lw}
	lnA := productUpper(a.lu, a.lv)
	lnB := productUpper(b.lu, b.lv)
	// axis is one candidate, built only when the candidates before it have
	// failed, beside a cheap proven upper bound on its length: |x × y| ≤ |x||y|.
	type axis struct {
		g     ratV3
		bound float64
	}
	next := func(gi int) axis {
		switch {
		case gi == 0:
			return axis{a.n, lnA}
		case gi == 1:
			return axis{b.n, lnB}
		case gi < 11:
			x, y := (gi-2)/3, (gi-2)%3
			return axis{rvCross(ea[x], eb[y]), productUpper(la[x], lb[y])}
		case gi < 14:
			x := gi - 11
			return axis{rvCross(a.n, ea[x]), productUpper(lnA, la[x])}
		default:
			y := gi - 14
			return axis{rvCross(b.n, eb[y]), productUpper(lnB, lb[y])}
		}
	}
	for gi := range 17 {
		ax := next(gi)
		g := ax.g
		if g[0].Sign() == 0 && g[1].Sign() == 0 && g[2].Sign() == 0 {
			continue
		}
		aLo, aHi := rvProject(a.p, g)
		bLo, bHi := rvProject(b.p, g)
		gap := new(big.Rat).Sub(bLo, aHi)
		if other := new(big.Rat).Sub(aLo, bHi); other.Cmp(gap) > 0 {
			gap = other
		}
		if gap.Sign() <= 0 {
			continue
		}
		// The cheap bound is at least the exact one, so a gap that clears it
		// clears the exact one too; a gap that does not is retried exactly.
		if cheap := floatRat(productUpper(margin, ax.bound)); cheap != nil && gap.Cmp(cheap) > 0 {
			return true
		}
		need := floatRat(productUpper(margin, rvLenUpper(g)))
		if need != nil && gap.Cmp(need) > 0 {
			return true
		}
	}
	return false
}

// rvProject is the exact projection range of a triangle's three corners onto
// one axis, before normalisation.
func rvProject(p [3]ratV3, g ratV3) (*big.Rat, *big.Rat) {
	lo := rvDot(p[0], g)
	hi := new(big.Rat).Set(lo)
	for _, q := range p[1:] {
		d := rvDot(q, g)
		if d.Cmp(lo) < 0 {
			lo = d
		}
		if d.Cmp(hi) > 0 {
			hi = d
		}
	}
	return lo, hi
}

// boxGapExceeds is the float pre-filter in front of revolveSeparated: two
// axis-aligned boxes separated on one coordinate by more than margin prove the
// same thing the exact test would, for the cost of six comparisons. The
// difference is rounded DOWNWARD, so a gap this reports is one the exact
// arithmetic also has.
func boxGapExceeds(a, b [2]r3.Vec, margin float64) bool {
	gap := func(x, y float64) bool { return downRound(y-x) > margin }
	return gap(a[1].X, b[0].X) || gap(b[1].X, a[0].X) ||
		gap(a[1].Y, b[0].Y) || gap(b[1].Y, a[0].Y) ||
		gap(a[1].Z, b[0].Z) || gap(b[1].Z, a[0].Z)
}

// revolveContactAudit is docs/tessellation-design.md §9's complete boundary
// audit: every facet has positive area, adjacent facets meet ONLY along the
// vertex or edge their indices share, and no non-adjacent pair touches at all.
//
// §9 asks for that verdict four times over — at the ideal-coordinate endpoint,
// at the stored unplaced endpoint, and across the two affine homotopies that
// join them. This runs it ONCE, at the final stored coordinates, against the
// COMBINED displacement delta = deltaC + deltaR, and that single pass carries
// all four. The argument is the one §9's own homotopies are built on:
//
//   - Every mesh on the path is a vertex-wise displacement of the final stored
//     mesh by at most delta. The construction stage joins the ideal unplaced
//     mesh to the stored unplaced one within deltaC; the exact rigid placement
//     is an ISOMETRY, so it carries that whole family into placed space
//     unchanged in shape and within deltaR of the final vertices; the placement
//     stage joins the rigid image to the final mesh within deltaR. Composing
//     the two, every vertex of every intermediate boundary lies within
//     deltaC + deltaR of the vertex this mesh stored for it.
//   - Every reading the verdict rests on is a POLYNOMIAL in those vertices, so
//     a bound on the vertices' motion bounds the reading's
//     (perturbBilinearAllow). A reading whose stored value exceeds its own
//     allowance cannot change sign anywhere on the family, and a reading that
//     is structurally zero — a corner lying in a plane that was BUILT through
//     it — stays zero at every point of the family.
//   - Those signs decide the contact outright. A pair sharing a vertex or an
//     edge is isolated by a half-space whose boundary plane contains the shared
//     feature identically, with one triangle inside it and the other's
//     non-shared corners strictly outside (auditRevolvePair's own doc comment
//     carries the candidates). A pair sharing nothing is proven apart by a
//     fixed separating axis with the same margin.
//
// So every intermediate boundary is embedded and its contact relations are the
// ones the stored mesh has, which is exactly what §9 charges to deltaC and
// deltaR. A pair the audit cannot decide is ErrUnsupported (§12), never an
// admission.
func revolveContactAudit(budget *workBudget, verts []r3.Vec, tris [][3]int, delta float64) error {
	if err := budget.err(); err != nil {
		return err
	}
	if isNonFinite(delta) || delta < 0 {
		return fmt.Errorf(`%w: this revolve mesh states no finite coordinate displacement, so its facets cannot be audited`, ErrUnsupported)
	}
	data := make([]revolveAuditTri, len(tris))
	for i, tri := range tris {
		if err := budget.step(); err != nil {
			return err
		}
		t, ok := newRevolveAuditTri(verts, tri)
		if !ok {
			return fmt.Errorf(`%w: revolve facet %d holds a coordinate that is not finite`, ErrUnsupported, i)
		}
		if err := requireRevolveFacetArea(t, i, delta); err != nil {
			return err
		}
		data[i] = t
	}
	if err := budget.err(); err != nil {
		return err
	}

	f := len(tris)
	pairs, ok := wallChoose2(uint64(f))
	if !ok || pairs > maxFacetPairTestsPerCall {
		return fmt.Errorf(`%w: this revolve mesh's facet-pair audit needs %d exact tests, past the fixed ceiling of %d; retry with a coarser chord tolerance`, ErrUnsupported, pairs, maxFacetPairTestsPerCall)
	}
	margin := productUpper(2, delta)
	for i := range f {
		for j := i + 1; j < f; j++ {
			if err := budget.step(); err != nil {
				return err
			}
			shared, count := sharedVertexIndices(tris[i], tris[j])
			if count == 0 && boxGapExceeds(data[i].box, data[j].box, margin) {
				continue
			}
			if err := auditRevolvePair(data, tris, shared, count, i, j, delta); err != nil {
				return err
			}
		}
	}
	return budget.err()
}

// requireRevolveFacetArea proves one facet keeps a positive area everywhere on
// the displaced family: its exact held area, bracketed from BELOW, exceeds the
// most a displacement of delta at each corner can take from it
// (docs/tessellation-design.md §5's own per-triangle area allowance, read here
// as a gate rather than as a slack term).
func requireRevolveFacetArea(t revolveAuditTri, i int, delta float64) error {
	held := ratSqrtDown(rvDot(t.n, t.n))
	allow := productUpper(2, absSumUpper(
		productUpper(delta, absSumUpper(t.lu, t.lv)),
		productUpper(productUpper(2, delta), delta),
	))
	if isNonFinite(held) || isNonFinite(allow) || held <= allow {
		return fmt.Errorf(`%w: revolve facet %d does not keep a positive area under the coordinate displacement this mesh carries`, ErrUnsupported, i)
	}
	return nil
}

// auditRevolvePair decides one facet pair against the contact its shared vertex
// indices require, under revolveContactAudit's own displacement margin.
//
// A pair sharing nothing is proven apart by a fixed separating axis. A pair
// sharing a vertex or an edge is REQUIRED to touch there, so what has to be
// proven instead is that it touches NOWHERE ELSE, and every proof of that below
// has the same shape: a half-space H whose boundary plane contains the shared
// feature, with one triangle inside H and the other's non-shared corners
// strictly outside. Then the intersection is contained in the boundary plane,
// and the strictly-outside triangle meets that plane in exactly the shared
// feature, so the two triangles meet in exactly it too.
//
// The boundary plane cannot be a FIXED one: the shared feature moves along the
// homotopy, and the plane has to keep containing it. So every candidate is
// built as a POLYNOMIAL in the pair's own corners — a triangle normal, a normal
// crossed with one of its own edges, or the shared edge's rejection of an
// apex — whose defining incidences hold identically rather than numerically.
// Only the strict-side readings are then charged a perturbation allowance.
func auditRevolvePair(data []revolveAuditTri, tris [][3]int, shared [3]int, count, i, j int, delta float64) error {
	switch count {
	case 3:
		return fmt.Errorf(`%w: revolve facets %d and %d are the same triangle`, ErrUnsupported, i, j)
	case 0:
		if revolveSeparated(data[i], data[j], delta) {
			return nil
		}
		return fmt.Errorf(`%w: revolve facets %d and %d share no vertex, and this mesh cannot prove they stay apart under its own coordinate displacement`, ErrUnsupported, i, j)
	case 1:
		if revolveVertexIsolated(data[i], tris[i], data[j], tris[j], shared[0], delta) ||
			revolveVertexIsolated(data[j], tris[j], data[i], tris[i], shared[0], delta) {
			return nil
		}
		return fmt.Errorf(`%w: revolve facets %d and %d do not provably meet only at the vertex they share`, ErrUnsupported, i, j)
	default:
		if revolveEdgeIsolated(data[i], tris[i], data[j], tris[j], shared, delta) ||
			revolveEdgeIsolated(data[j], tris[j], data[i], tris[i], shared, delta) {
			return nil
		}
		return fmt.Errorf(`%w: revolve facets %d and %d do not provably meet only along the edge they share`, ErrUnsupported, i, j)
	}
}

// revolveSepAxis is one candidate boundary plane through the shared feature,
// carried as the plane's own (unnormalised) normal: the exact vector, a proven
// upper bound on its length, and a proven bound on how far that vector itself
// moves when every corner it was built from slides by up to the audit's margin.
type revolveSepAxis struct {
	g      ratV3
	length float64
	drift  float64
}

// sideOf reads which side of the candidate plane an offset lies on, and whether
// that reading survives the whole displaced family. The plane passes through the
// shared feature, so the offset is measured from a shared corner.
func (ax revolveSepAxis) sideOf(offset ratV3, offsetLen, offsetDrift float64) (int, bool) {
	h := rvDot(ax.g, offset)
	if h.Sign() == 0 {
		return 0, false
	}
	allow := perturbBilinearAllow(ax.length, offsetLen, ax.drift, offsetDrift)
	if isNonFinite(allow) {
		return 0, false
	}
	bound := floatRat(allow)
	if bound == nil || new(big.Rat).Abs(h).Cmp(bound) <= 0 {
		return 0, false
	}
	return h.Sign(), true
}

// perturbBilinearAllow bounds |a'∘b' − a∘b| for a dot or cross product when a
// and b slide by at most da and db: the two first-order terms plus the second.
func perturbBilinearAllow(la, lb, da, db float64) float64 {
	return absSumUpper(productUpper(la, db), productUpper(lb, da), productUpper(da, db))
}

// revolveNormalAxis is the candidate whose plane IS a triangle's own plane:
// every corner of that triangle reads exactly zero on it, identically, for the
// whole family.
func revolveNormalAxis(t revolveAuditTri, delta float64) revolveSepAxis {
	e := productUpper(2, delta)
	return revolveSepAxis{
		g:      t.n,
		length: productUpper(t.lu, t.lv),
		drift:  perturbBilinearAllow(t.lu, t.lv, e, e),
	}
}

// revolveEdgeFanAxis is the candidate whose plane contains a triangle's own
// plane normal and one of its edges through the shared corner. The edge itself
// reads exactly zero (a determinant with a repeated vector), so that triangle's
// only reading to charge is its remaining corner.
func revolveEdgeFanAxis(t revolveAuditTri, edge revolveOffset, delta float64) revolveSepAxis {
	e := productUpper(2, delta)
	normal := revolveNormalAxis(t, delta)
	return revolveSepAxis{
		g:      rvCross(normal.g, edge.v),
		length: productUpper(normal.length, edge.length),
		drift:  perturbBilinearAllow(normal.length, edge.length, normal.drift, e),
	}
}

// revolveRejectionAxis is the candidate for a SHARED EDGE: the rejection of one
// triangle's apex off that edge, (d×u)×d = |d|²u − (d·u)d. Both shared corners
// read exactly zero on it — the first by construction, the second because the
// scalar triple product repeats d — and the apex reads the Gram determinant
// |d|²|u|² − (d·u)², which Cauchy-Schwarz makes non-negative identically. So
// that whole triangle sits in the closed half-space for the entire family with
// nothing to charge, and only the other triangle's apex is read.
func revolveRejectionAxis(d, u ratV3, dLen, uLen, delta float64) revolveSepAxis {
	e := productUpper(2, delta)
	cross := rvCross(d, u)
	crossLen := productUpper(dLen, uLen)
	crossDrift := perturbBilinearAllow(dLen, uLen, e, e)
	return revolveSepAxis{
		g:      rvCross(cross, d),
		length: productUpper(crossLen, dLen),
		drift:  perturbBilinearAllow(crossLen, dLen, crossDrift, e),
	}
}

// revolveVertexIsolated proves the pair meets only at the vertex they share,
// with the boundary plane built from triangle a. Four candidates are tried, and
// every one of them holds a identically inside its own closed half-space:
//
//   - a's own plane, on which all three of its corners read an identical zero;
//   - a's plane rotated onto either of its two edges at the shared corner, on
//     which that edge reads zero identically (a determinant repeating a vector)
//     and only a's remaining corner has to be signed;
//   - a's plane rotated onto the CHORD between its two other corners, on which
//     those two corners read the SAME value identically — their difference is a
//     determinant repeating the chord — so a still sits on one side. This is the
//     candidate that answers a pair of exactly opposite sectors of one pole fan,
//     where each triangle's own edge rays run straight into the other's.
//
// A fifth family is not built from a's plane at all: the EDGE-PAIR planes,
// whose normal g = eA × eB takes one edge of EACH triangle at the shared
// corner. Both of those edges read an identical zero on g — a determinant
// repeating a vector, twice over — so the plane contains one whole edge of a
// and one whole edge of b for every member of the displaced family, and only
// the two remaining corners have to be signed. It is the family that answers a
// partial cap's fan triangle against the wall triangle of the NEXT meridian
// chord, a pair no plane through a's own normal decides: a's normal reads the
// wall's in-plane corner at a numerical zero it cannot sign, and every rotation
// of it reads the wall's two corners with opposite signs, because the off-plane
// corner's in-plane component is fixed in sign and only shrinks as dφ² under
// refinement. Without this family that pair is undecidable at every angular
// count, so a partial-sweep revolve carrying a chorded arc refuses however fine
// the chording.
//
// The mirror, with the roles swapped, is the caller's second call.
func revolveVertexIsolated(a revolveAuditTri, triA [3]int, b revolveAuditTri, triB [3]int, shared int, delta float64) bool {
	e := productUpper(2, delta)
	ai := triangleVertexSlot(triA, shared)
	bi := triangleVertexSlot(triB, shared)
	if ai < 0 || bi < 0 {
		return false
	}
	aOff := revolveCornerOffsets(a, ai)
	bOff := revolveCornerOffsets(b, bi)

	// signed reads the common sign of the listed offsets, or reports that this
	// candidate cannot sign them all.
	signed := func(ax revolveSepAxis, offs []revolveOffset) (int, bool) {
		side := 0
		for _, o := range offs {
			s, ok := ax.sideOf(o.v, o.length, e)
			if !ok {
				return 0, false
			}
			if side == 0 {
				side = s
			} else if side != s {
				return 0, false
			}
		}
		return side, true
	}
	try := func(ax revolveSepAxis, check []revolveOffset) bool {
		side, ok := signed(ax, check)
		if !ok {
			return false
		}
		want, ok := signed(ax, bOff[:])
		if !ok || want == 0 {
			return false
		}
		return side == 0 || side != want
	}
	if try(revolveNormalAxis(a, delta), nil) {
		return true
	}
	for k := range 2 {
		if try(revolveEdgeFanAxis(a, aOff[k], delta), []revolveOffset{aOff[1-k]}) {
			return true
		}
	}
	chord := revolveOffsetOf(rvSub(aOff[0].v, aOff[1].v))
	if try(revolveEdgeFanAxis(a, chord, delta), aOff[:]) {
		return true
	}
	// The edge-pair family. g = eA × eB zeroes both eA and eB identically, so
	// the plane holds one edge of each triangle for the WHOLE family rather
	// than at the stored coordinates alone. That leaves a with two corners on
	// the plane and one strictly off it, and b likewise, so a sits in one
	// closed half-space and b in the other and their intersection lies in the
	// plane. A triangle with two corners on a plane and its third strictly off
	// meets that plane in exactly the closed segment between the two, so the
	// intersection is contained in eA ∩ eB — two segments from the shared
	// corner that a non-zero g makes non-parallel, and which therefore meet
	// only at that corner.
	//
	// g stays non-zero over the whole family without a separate test: a member
	// whose g vanished would read zero on both remaining corners, and sideOf
	// has already proven each of them strictly outside its own perturbation
	// allowance, which bounds exactly how far that reading can move. length
	// bounds |eA × eB| by the product of the two factor lengths and drift is
	// perturbBilinearAllow over the two factors sliding by e, so the charge is
	// the one every other candidate here makes. A reading inside its allowance
	// stays UNDECIDED and the candidate is skipped, never admitted.
	for k := range 2 {
		for m := range 2 {
			g := rvCross(aOff[k].v, bOff[m].v)
			if rvIsZero(g) {
				continue
			}
			ax := revolveSepAxis{
				g:      g,
				length: productUpper(aOff[k].length, bOff[m].length),
				drift:  perturbBilinearAllow(aOff[k].length, bOff[m].length, e, e),
			}
			sa, okA := ax.sideOf(aOff[1-k].v, aOff[1-k].length, e)
			sb, okB := ax.sideOf(bOff[1-m].v, bOff[1-m].length, e)
			if okA && okB && sa != sb {
				return true
			}
		}
	}
	return false
}

// revolveOffset is one corner offset from the pair's shared corner, beside the
// proven upper bound on its length every perturbation allowance reads.
type revolveOffset struct {
	v      ratV3
	length float64
}

func revolveOffsetOf(v ratV3) revolveOffset {
	return revolveOffset{v: v, length: rvLenUpper(v)}
}

// revolveCornerOffsets is a triangle's two corners other than the one at slot
// at, measured from that one.
func revolveCornerOffsets(t revolveAuditTri, at int) [2]revolveOffset {
	return t.off[at]
}

// revolveEdgeIsolated proves the pair meets only along the edge they share,
// with the boundary plane built from triangle a: either a's own plane, or the
// rejection of a's apex off the shared edge — the candidate that answers the
// COPLANAR case every planar cell and every cap triangulation produces, where
// a's own plane says nothing at all.
func revolveEdgeIsolated(a revolveAuditTri, triA [3]int, b revolveAuditTri, triB [3]int, shared [3]int, delta float64) bool {
	e := productUpper(2, delta)
	p0 := triangleVertexSlot(triA, shared[0])
	p1 := triangleVertexSlot(triA, shared[1])
	apexA := triangleApexIndex(triA, shared[0], shared[1])
	apexB := triangleApexIndex(triB, shared[0], shared[1])
	if p0 < 0 || p1 < 0 || apexA < 0 || apexB < 0 {
		return false
	}
	bApex := triangleVertexSlot(triB, apexB)
	aApex := triangleVertexSlot(triA, apexA)
	if bApex < 0 || aApex < 0 {
		return false
	}
	offB := rvSub(b.p[bApex], a.p[p0])
	lenB := rvLenUpper(offB)
	if _, ok := revolveNormalAxis(a, delta).sideOf(offB, lenB, e); ok {
		return true
	}
	d := rvSub(a.p[p1], a.p[p0])
	u := rvSub(a.p[aApex], a.p[p0])
	ax := revolveRejectionAxis(d, u, rvLenUpper(d), rvLenUpper(u), delta)
	side, ok := ax.sideOf(offB, lenB, e)
	// a itself lies in the g ≥ 0 half-space identically (the Gram determinant),
	// so the pair is isolated exactly when b's apex reads strictly negative.
	return ok && side < 0
}

// triangleVertexSlot is the corner index a triangle carries a given vertex at,
// or −1 when it carries none.
func triangleVertexSlot(tri [3]int, v int) int {
	for k, x := range tri {
		if x == v {
			return k
		}
	}
	return -1
}

// requireVertexLinks is docs/tessellation-design.md §9's construction safety
// net: the combinatorial link of every stored vertex — the edge each incident
// triangle contributes between its other two corners — must be ONE connected
// cycle with every vertex of degree two. A pinched pole passes the
// directed-edge audit and fails here, which is the whole reason the link audit
// exists beside it.
func requireVertexLinks(ctx context.Context, m *Mesh) error {
	budget := newWorkBudget(ctx)
	links := make(map[int]map[int][]int, len(m.vertices))
	add := func(center, from, to int) {
		l, ok := links[center]
		if !ok {
			l = map[int][]int{}
			links[center] = l
		}
		l[from] = append(l[from], to)
		l[to] = append(l[to], from)
	}
	for _, tri := range m.triangles {
		if err := budget.step(); err != nil {
			return err
		}
		add(tri[0], tri[1], tri[2])
		add(tri[1], tri[2], tri[0])
		add(tri[2], tri[0], tri[1])
	}
	// Vertex index order, never map order: a refusal names the FIRST vertex
	// that fails, so two runs over the same mesh report the same one.
	for center := range m.vertices {
		link, ok := links[center]
		if !ok {
			continue
		}
		if err := budget.step(); err != nil {
			return err
		}
		start := -1
		for v, nbrs := range link {
			if len(nbrs) != 2 {
				return fmt.Errorf(`%w: the mesh vertex at index %d has a pinched link: its neighbour %d meets %d link edges rather than two`, ErrUnsupported, center, v, len(nbrs))
			}
			if start < 0 || v < start {
				start = v
			}
		}
		if start < 0 {
			continue
		}
		seen := map[int]struct{}{start: {}}
		prev, cur := -1, start
		for {
			nbrs := link[cur]
			next := nbrs[0]
			if next == prev {
				next = nbrs[1]
			}
			if next == start {
				break
			}
			if _, done := seen[next]; done {
				return fmt.Errorf(`%w: the mesh vertex at index %d has a pinched link`, ErrUnsupported, center)
			}
			seen[next] = struct{}{}
			prev, cur = cur, next
		}
		if len(seen) != len(link) {
			return fmt.Errorf(`%w: the mesh vertex at index %d has a link of %d cycles rather than one`, ErrUnsupported, center, 1+len(link)-len(seen))
		}
	}
	return budget.err()
}
