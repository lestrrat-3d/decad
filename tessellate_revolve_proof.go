package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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
// proofbound.RatInterval built from the payload's held float64s, which are exact
// rationals, so the answers do not move with the platform's libm or with FMA
// contraction.

// revolveTrigGapPrior is the a-priori ceiling on how far one stored cosine or
// sine sits from the exact value it stands for. The tessellator does not call
// math.Sincos: it encloses the ideal angle's cosine and sine as rational
// intervals (proofbound.TurnSinCosInterval / survey2d.RadSinCosInterval, neither of which ever
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
	cosIv   []proofbound.RatInterval
	sinIv   []proofbound.RatInterval
	gap     float64
	// step is the exact enclosure of ONE angular interval's true width, the
	// dφ every Ecell integral reads.
	step proofbound.RatInterval
}

// revolveAngularSequence builds the angular sequence for n chords, over the
// payload's own denotation (docs/evaluator-design.md §6): sample l's angle is
// enclosed as enc(phi0) + (l/n)·(enc(phi1) − enc(phi0)), so the stored
// cosine/sine is checked against the angle the RECORD denotes, not merely the
// held float the resolver rounded to. Wherever the payload's denotation
// cannot state an end exactly (den.phi0/den.phi1 invalid — a ToFaceAngular
// stop, a payload literal with none, or an angle unit this evaluator does not
// denote), this falls back to the prior reading over the held floats alone,
// which reproduces today's construction exactly: a partial sweep's angle
// φ0 + l·(φ1 − φ0)/n as an exact rational in the payload's own two floats
// (survey2d.RadSinCosInterval), a full turn starting at zero as l/n of a TURN
// (proofbound.TurnSinCosInterval, no π entering at all), and a full turn starting
// elsewhere as the same radian enclosure over φ0 + 2π·l/n, widened by the 2π
// enclosure's own (sub-2⁻²⁴⁰) width.
func revolveAngularSequence(rp revolvePayload, n int) (revolveAngular, error) {
	if n <= 0 {
		return revolveAngular{}, fmt.Errorf(`%w: a revolve mesh needs at least one angular chord`, ErrDegenerate)
	}
	phi0, phi1, full := rp.phi0, rp.phi1, rp.full
	r0, r1 := proofarith.FloatRat(phi0), proofarith.FloatRat(phi1)
	if r0 == nil || r1 == nil {
		return revolveAngular{}, fmt.Errorf(`%w: the sweep interval is not finite, so no angular sample can be enclosed`, ErrUnsupported)
	}
	enc0, ok0 := rp.den.phi0.enclosure()
	enc1, ok1 := rp.den.phi1.enclosure()
	haveDen := ok0 && ok1
	var diff proofbound.RatInterval
	if haveDen {
		diff = proofbound.IntervalSub(enc1, enc0)
	}
	out := revolveAngular{n: n, samples: n + 1}
	if full {
		out.samples = n
	}
	nRat := new(big.Rat).SetInt64(int64(n))
	if full {
		out.step = proofbound.IntervalScale(proofbound.TwoPiInterval(), new(big.Rat).Inv(nRat))
	} else {
		out.step = proofbound.PointInterval(new(big.Rat).Quo(new(big.Rat).Sub(r1, r0), nRat))
	}
	for l := range out.samples {
		frac := new(big.Rat).SetFrac64(int64(l), int64(n))
		var cosIv, sinIv proofbound.RatInterval
		switch {
		case haveDen:
			angle := proofbound.IntervalAdd(enc0, proofbound.IntervalScale(diff, frac))
			var ok bool
			sinIv, cosIv, ok = radSinCosSpan(angle)
			if !ok {
				return revolveAngular{}, errRevolveAngleEnclosure
			}
		case full && r0.Sign() == 0:
			sinIv, cosIv = proofbound.TurnSinCosInterval(frac)
		case full:
			angle := proofbound.IntervalAdd(proofbound.PointInterval(r0), proofbound.IntervalScale(proofbound.TwoPiInterval(), frac))
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
			sinIv, cosIv, ok = survey2d.RadSinCosInterval(angle)
			if !ok {
				return revolveAngular{}, errRevolveAngleEnclosure
			}
		}
		cosHeld, _ := intervalMid(cosIv).Float64()
		sinHeld, _ := intervalMid(sinIv).Float64()
		if proofbound.IsNonFinite(cosHeld) || proofbound.IsNonFinite(sinHeld) {
			return revolveAngular{}, errRevolveAngleEnclosure
		}
		gap := math.Max(proofbound.IntervalFloatError(cosIv, cosHeld), proofbound.IntervalFloatError(sinIv, sinHeld))
		if proofbound.IsNonFinite(gap) || gap > revolveTrigGapPrior {
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
	origin, ok0 := survey2d.IvVec3Of(rp.frame.Origin())
	fu, ok1 := survey2d.IvVec3Of(rp.frame.U())
	fv, ok2 := survey2d.IvVec3Of(rp.frame.V())
	aU, aV := proofarith.FloatRat(rp.ax.aU), proofarith.FloatRat(rp.ax.aV)
	dU, dV := proofarith.FloatRat(rp.ax.dU), proofarith.FloatRat(rp.ax.dV)
	if !ok0 || !ok1 || !ok2 || aU == nil || aV == nil || dU == nil || dV == nil {
		return revolveBasis3Iv{}, false
	}
	scale := func(v survey2d.IvVec3, s *big.Rat) survey2d.IvVec3 {
		return survey2d.IvVec3Mul(v, proofbound.PointInterval(s))
	}
	a3 := survey2d.IvVec3Add(origin, survey2d.IvVec3Add(scale(fu, aU), scale(fv, aV)))
	w := survey2d.IvVec3Add(scale(fu, dU), scale(fv, dV))
	e0 := survey2d.IvVec3Add(scale(fu, new(big.Rat).Neg(dV)), scale(fv, dU))
	return revolveBasis3Iv{a3: a3, w: w, e0: e0, e1: ivVec3Cross(w, e0)}, true
}

// revolveBasis3Iv is revolveBasis enclosed exactly.
type revolveBasis3Iv struct {
	a3, w, e0, e1 survey2d.IvVec3
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
func revolveMeridianEnclosure(ax axisFrame, u, v float64, bound proofbound.WalkEndBound) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	ru, rv := proofarith.FloatRat(u), proofarith.FloatRat(v)
	bu, bv := proofarith.FloatRat(math.Abs(bound.U)), proofarith.FloatRat(math.Abs(bound.V))
	if ru == nil || rv == nil || bu == nil || bv == nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	return axisCoordInterval(ax,
		survey2d.IntervalWiden(proofbound.PointInterval(ru), bu),
		survey2d.IntervalWiden(proofbound.PointInterval(rv), bv),
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
func axisCoordInterval(ax axisFrame, u, v proofbound.RatInterval) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	aU, aV := proofarith.FloatRat(ax.aU), proofarith.FloatRat(ax.aV)
	dU, dV := proofarith.FloatRat(ax.dU), proofarith.FloatRat(ax.dV)
	if aU == nil || aV == nil || dU == nil || dV == nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	du := proofbound.IntervalSub(u, proofbound.PointInterval(aU))
	dv := proofbound.IntervalSub(v, proofbound.PointInterval(aV))
	z := proofbound.IntervalAdd(proofbound.IntervalScale(du, dU), proofbound.IntervalScale(dv, dV))
	rho := proofbound.IntervalSub(proofbound.IntervalScale(dv, dU), proofbound.IntervalScale(du, dV))
	return z, rho, true
}

// revolveIdealPoint encloses X(z, ρ, φ) exactly: the ideal unplaced sample
// docs/tessellation-design.md §8 measures every stored vertex against.
func revolveIdealPoint(b revolveBasis3Iv, z, rho, cos, sin proofbound.RatInterval) survey2d.IvVec3 {
	radial := survey2d.IvVec3Add(survey2d.IvVec3Mul(b.e0, cos), survey2d.IvVec3Mul(b.e1, sin))
	return survey2d.IvVec3Add(b.a3, survey2d.IvVec3Add(survey2d.IvVec3Mul(b.w, z), survey2d.IvVec3Mul(radial, rho)))
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
		worst = math.Max(worst, proofbound.UpRound(proofbound.AbsSumUpper(
			component(b.a3),
			proofbound.ProductUpper(zAbsMax, math.Abs(component(b.w))),
			proofbound.ProductUpper(rhoMax, proofbound.AbsSumUpper(component(b.e0), component(b.e1))),
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
	radialUnit := proofbound.AbsSumUpper(proofbound.VecMaxAbs(b.e0), proofbound.VecMaxAbs(b.e1))
	perCoord := proofbound.AbsSumUpper(
		proofbound.ProductUpper(meridianGap, proofbound.AbsSumUpper(proofbound.VecMaxAbs(b.w), radialUnit)),
		proofbound.ProductUpper(proofbound.ProductUpper(rhoMax, revolveTrigGapPrior), radialUnit),
		proofbound.ProductUpper(revolveEvalRoundUlps, proofbound.UlpOf(math.Max(coordMax, 1))),
	)
	return proofbound.Radius3D(perCoord)
}

// revolveBudget is docs/tessellation-design.md §8's tolerance split: both
// coordinate stages are reserved from the requested tolerance BEFORE any chord
// count is chosen, and a tolerance they exhaust refuses rather than returning a
// mesh whose bound it cannot honour. Both subtractions round downward, which is
// the direction that leaves the reservation whole.
func revolveBudget(tol, deltaC, deltaR float64) (float64, error) {
	available := freeform.DownRound(freeform.DownRound(tol - deltaC - deltaR))
	if available <= 0 || proofbound.IsNonFinite(available) {
		requested := units.Millimeters(tol)
		reserved := units.Millimeters(proofbound.AbsSumUpper(deltaC, deltaR))
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
	if !proofbound.FiniteVec(unplaced) || !proofbound.FiniteVec(held) {
		return math.Inf(1)
	}
	basis := xform.Basis()
	translation := xform.Translation()
	if !proofbound.FiniteVec(basis.EX) || !proofbound.FiniteVec(basis.EY) || !proofbound.FiniteVec(basis.EZ) || !proofbound.FiniteVec(translation) {
		return math.Inf(1)
	}
	p := proofarith.DyVec(unplaced)
	ex, ey, ez, t := proofarith.DyVec(basis.EX), proofarith.DyVec(basis.EY), proofarith.DyVec(basis.EZ), proofarith.DyVec(translation)
	perCoord := 0.0
	for i := range 3 {
		exact := proofarith.DyAdd(
			proofarith.DyAdd(proofarith.DyMul(ex[i], p[0]), proofarith.DyMul(ey[i], p[1])),
			proofarith.DyAdd(proofarith.DyMul(ez[i], p[2]), t[i]),
		)
		perCoord = math.Max(perCoord, proofarith.DyadicFloatError(exact, vecComponent(held, i)))
	}
	return proofbound.Radius3D(perCoord)
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
// isolation, no interval subdivision, and internal/freeform/clearance_poly.go's Sturm engine is
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
func revolveCellAreaSlack(rho0, rho1 float64, meridian, step proofbound.RatInterval, twoArea [2]proofbound.RatInterval) float64 {
	r0, r1 := proofarith.FloatRat(rho0), proofarith.FloatRat(rho1)
	if r0 == nil || r1 == nil {
		return math.Inf(1)
	}
	dRho := new(big.Rat).Sub(r1, r0)
	scale := proofbound.IntervalMul(meridian, step)
	total := new(big.Rat)
	for half, weight := range [2]int{revolveWeightT, revolveWeightOneMinusT} {
		total.Add(total, revolveHalfCellSlack(r0, dRho, scale, twoArea[half], weight))
	}
	return proofbound.RatFloatUp(total)
}

// revolveFanAreaSlack is revolveCellAreaSlack for a cell with ONE ring on the
// axis: the held facets are a fan of single triangles rather than quads, and
// the domain is the whole unit square with the pole edge collapsed.
//
// With the pole at t = 0 the held map is P + t·((1−u)·A + u·B), whose Jacobian
// is exactly t·|A × B|, while Jtrue is L·dφ·ρ1·t — both LINEAR in t through the
// origin, so the difference is again linear and the same closed form answers.
// The pole at t = 1 is the mirror image, in 1 − t.
func revolveFanAreaSlack(rhoOff float64, poleFirst bool, meridian, step, twoArea proofbound.RatInterval) float64 {
	r := proofarith.FloatRat(rhoOff)
	if r == nil {
		return math.Inf(1)
	}
	// f(t) = (L·dφ·ρ_off − 2A)·t for a pole at t = 0, and the same constant
	// times (1 − t) for a pole at t = 1.
	c := proofbound.IntervalSub(proofbound.IntervalScale(proofbound.IntervalMul(meridian, step), r), twoArea)
	lo, hi := c.Lo, c.Hi
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
	return proofbound.RatFloatUp(total)
}

// revolveHalfCellSlack is one half-domain's contribution to Ecell: the exact
// ∫|f_lo|·w plus the exact ∫(f_hi − f_lo)·w, with
// f(t) = L·dφ·(ρ0 + t·Δρ) − 2A.
func revolveHalfCellSlack(rho0, dRho *big.Rat, scale, twoArea proofbound.RatInterval, weight int) *big.Rat {
	alphaLo := new(big.Rat).Mul(scale.Lo, dRho)
	betaLo := new(big.Rat).Sub(new(big.Rat).Mul(scale.Lo, rho0), twoArea.Hi)
	spread := new(big.Rat).Sub(scale.Hi, scale.Lo)
	alphaGap := new(big.Rat).Mul(spread, dRho)
	betaGap := new(big.Rat).Add(
		new(big.Rat).Mul(spread, rho0),
		new(big.Rat).Sub(twoArea.Hi, twoArea.Lo),
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
func ivTwoTriangleArea(p0, p1, p2 survey2d.IvVec3) (proofbound.RatInterval, bool) {
	n := ivVec3Cross(ivVec3Sub(p1, p0), ivVec3Sub(p2, p0))
	return survey2d.IntervalSqrt(survey2d.IvVec3NormSq(n))
}

// revolveAuditTri is one triangle's exact lift, held for the whole audit: its
// three corners, the three edge vectors (u = p1−p0, v = p2−p0, w = p2−p1),
// their cross product, and proven upper bounds on the three edge lengths.
// Every predicate below reads these rather than rebuilding them per pair,
// exactly as loft_audit.go's own audit data does. fp holds the stored float
// corners, and fu, fv, fw and fn enclose u, v, w and n in float intervals for
// the pre-test (tessellate_revolve_filter.go).
type revolveAuditTri struct {
	p              [3]proofarith.DyV3
	u, v, w, n     proofarith.DyV3
	lu, lv, lw     float64
	box            [2]r3.Vec
	fp             [3]r3.Vec
	fu, fv, fw, fn revIvVec
	// off[k] holds the two corner offsets measured from corner k, in
	// increasing corner order, each with its proven length bound.
	off [3][2]revolveOffset
}

func newRevolveAuditTri(verts []r3.Vec, tri [3]int) (revolveAuditTri, bool) {
	var out revolveAuditTri
	for k, vi := range tri {
		if !proofbound.FiniteVec(verts[vi]) {
			return out, false
		}
		out.p[k] = proofarith.DyVec(verts[vi])
		out.fp[k] = verts[vi]
	}
	out.u = proofarith.DvSub(out.p[1], out.p[0])
	out.v = proofarith.DvSub(out.p[2], out.p[0])
	out.w = proofarith.DvSub(out.p[2], out.p[1])
	out.n = proofarith.DvCross(out.u, out.v)
	out.fu = revIvPointDiff(out.fp[1], out.fp[0])
	out.fv = revIvPointDiff(out.fp[2], out.fp[0])
	out.fw = revIvPointDiff(out.fp[2], out.fp[1])
	out.fn = revIvCross(out.fu, out.fv)
	out.lu = proofbound.DvLenUpper(out.u)
	out.lv = proofbound.DvLenUpper(out.v)
	out.lw = proofbound.DvLenUpper(out.w)
	out.box = meshbool.TriBox(verts, tri)
	out.off[0] = [2]revolveOffset{{v: out.u, f: out.fu, length: out.lu}, {v: out.v, f: out.fv, length: out.lv}}
	out.off[1] = [2]revolveOffset{
		revolveOffsetOf(proofarith.DvSub(out.p[0], out.p[1]), revIvPointDiff(out.fp[0], out.fp[1])),
		{v: out.w, f: out.fw, length: out.lw},
	}
	out.off[2] = [2]revolveOffset{
		revolveOffsetOf(proofarith.DvSub(out.p[0], out.p[2]), revIvPointDiff(out.fp[0], out.fp[2])),
		revolveOffsetOf(proofarith.DvSub(out.p[1], out.p[2]), revIvPointDiff(out.fp[1], out.fp[2])),
	}
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
// form, so a pair that a low-index axis decides never pays for the rest. The
// float pre-test runs the same walk first (revolveSeparatedFloat); an axis it
// accepts is one the exact walk accepts too.
func revolveSeparated(a, b revolveAuditTri, delta float64) bool {
	return revolveSeparatedFloat(a, b, delta) || revolveSeparatedExact(a, b, delta)
}

// revolveSeparatedExact is revolveSeparated's walk over the exact Dyadic axes.
func revolveSeparatedExact(a, b revolveAuditTri, delta float64) bool {
	margin := proofbound.ProductUpper(2, delta)
	if proofbound.IsNonFinite(margin) {
		return false
	}
	ea := [3]proofarith.DyV3{a.u, a.v, a.w}
	eb := [3]proofarith.DyV3{b.u, b.v, b.w}
	la := [3]float64{a.lu, a.lv, a.lw}
	lb := [3]float64{b.lu, b.lv, b.lw}
	lnA := proofbound.ProductUpper(a.lu, a.lv)
	lnB := proofbound.ProductUpper(b.lu, b.lv)
	// axis is one candidate, built only when the candidates before it have
	// failed, beside a cheap proven upper bound on its length: |x × y| ≤ |x||y|.
	type axis struct {
		g     proofarith.DyV3
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
			return axis{proofarith.DvCross(ea[x], eb[y]), proofbound.ProductUpper(la[x], lb[y])}
		case gi < 14:
			x := gi - 11
			return axis{proofarith.DvCross(a.n, ea[x]), proofbound.ProductUpper(lnA, la[x])}
		default:
			y := gi - 14
			return axis{proofarith.DvCross(b.n, eb[y]), proofbound.ProductUpper(lnB, lb[y])}
		}
	}
	for gi := range 17 {
		ax := next(gi)
		g := ax.g
		if proofarith.DvIsZero(g) {
			continue
		}
		aLo, aHi := dvProject(a.p, g)
		bLo, bHi := dvProject(b.p, g)
		gap := proofarith.DySubScalar(bLo, aHi)
		if other := proofarith.DySubScalar(aLo, bHi); proofarith.DyCmp(other, gap) > 0 {
			gap = other
		}
		if gap.Sign() <= 0 {
			continue
		}
		// The cheap bound is at least the exact one, so a gap that clears it
		// clears the exact one too; a gap that does not is retried exactly.
		if cheap, ok := proofarith.DyOf(proofbound.ProductUpper(margin, ax.bound)); ok && proofarith.DyCmp(gap, cheap) > 0 {
			return true
		}
		if need, ok := proofarith.DyOf(proofbound.ProductUpper(margin, proofbound.DvLenUpper(g))); ok && proofarith.DyCmp(gap, need) > 0 {
			return true
		}
	}
	return false
}

// dvProject is the exact projection range of a triangle's three corners onto
// one axis, before normalisation.
func dvProject(p [3]proofarith.DyV3, g proofarith.DyV3) (proofarith.Dyadic, proofarith.Dyadic) {
	lo := proofarith.DvDot(p[0], g)
	hi := lo
	for _, q := range p[1:] {
		d := proofarith.DvDot(q, g)
		if proofarith.DyCmp(d, lo) < 0 {
			lo = d
		}
		if proofarith.DyCmp(d, hi) > 0 {
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
	gap := func(x, y float64) bool { return freeform.DownRound(y-x) > margin }
	return gap(a[1].X, b[0].X) || gap(b[1].X, a[0].X) ||
		gap(a[1].Y, b[0].Y) || gap(b[1].Y, a[0].Y) ||
		gap(a[1].Z, b[0].Z) || gap(b[1].Z, a[0].Z)
}

// requireRevolveFacetAreas proves docs/tessellation-design.md §1's positive-area
// row over the whole mesh and returns the audit triangles it built on the way.
//
// It stands OUTSIDE revolveContactAudit, and runs at every verification level,
// because the two answer different questions at different costs. Positive area
// is per FACET and linear; a zero-area facet has no normal, so a renderer, the
// exporter's own facet normals and the boolean all need it. Contact is per
// facet PAIR and quadratic, and it is what a caller who only wants to draw the
// mesh declines. Folding the first into the second would silently drop §1's
// positive-area row from every mesh built below VerifyBoundary.
//
// The returned slice is parallel to tris, so revolveContactAudit consumes it
// rather than walking the facets a second time. delta is the combined
// coordinate displacement deltaC + deltaR, as the audit's own doc comment
// derives it.
func requireRevolveFacetAreas(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int, delta float64) ([]revolveAuditTri, error) {
	if err := budget.Err(); err != nil {
		return nil, err
	}
	if proofbound.IsNonFinite(delta) || delta < 0 {
		return nil, fmt.Errorf(`%w: this revolve mesh states no finite coordinate displacement, so its facets cannot be audited`, ErrUnsupported)
	}
	data := make([]revolveAuditTri, len(tris))
	for i, tri := range tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		t, ok := newRevolveAuditTri(verts, tri)
		if !ok {
			return nil, fmt.Errorf(`%w: revolve facet %d holds a coordinate that is not finite`, ErrUnsupported, i)
		}
		if err := requireRevolveFacetArea(t, i, delta); err != nil {
			return nil, err
		}
		data[i] = t
	}
	return data, budget.Err()
}

// revolveContactAudit is docs/tessellation-design.md §9's facet-contact audit:
// adjacent facets meet ONLY along the vertex or edge their indices share, and
// no non-adjacent pair touches at all. It runs at VerifyBoundary and above, and
// data is requireRevolveFacetAreas' own output for the same triangle set, which
// has already proven every facet positive-area.
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
func revolveContactAudit(budget *proofbound.WorkBudget, data []revolveAuditTri, tris [][3]int, delta float64) error {
	if err := budget.Err(); err != nil {
		return err
	}
	if len(data) != len(tris) {
		return fmt.Errorf(`%w: the revolve facet audit holds %d triangles for a mesh of %d facets`, ErrUnsupported, len(data), len(tris))
	}

	f := len(tris)
	pairs, ok := proofbound.WallChoose2(uint64(f))
	if !ok || pairs > proofbound.MaxFacetPairTestsPerCall {
		return fmt.Errorf(`%w: this revolve mesh's facet-pair audit needs %d exact tests, past the fixed ceiling of %d; retry with a coarser chord tolerance`, ErrUnsupported, pairs, proofbound.MaxFacetPairTestsPerCall)
	}
	margin := proofbound.ProductUpper(2, delta)
	for i := range f {
		for j := i + 1; j < f; j++ {
			if err := budget.Step(); err != nil {
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
	return budget.Err()
}

// requireRevolveFacetArea proves one facet keeps a positive area everywhere on
// the displaced family: its exact held area, bracketed from BELOW, exceeds the
// most a displacement of delta at each corner can take from it
// (docs/tessellation-design.md §5's own per-triangle area allowance, read here
// as a gate rather than as a slack term).
func requireRevolveFacetArea(t revolveAuditTri, i int, delta float64) error {
	held := proofarith.DySqrtDown(proofarith.DvDot(t.n, t.n))
	allow := proofbound.ProductUpper(2, proofbound.AbsSumUpper(
		proofbound.ProductUpper(delta, proofbound.AbsSumUpper(t.lu, t.lv)),
		proofbound.ProductUpper(proofbound.ProductUpper(2, delta), delta),
	))
	if proofbound.IsNonFinite(held) || proofbound.IsNonFinite(allow) || held <= allow {
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
// f encloses g in float intervals for the pre-test.
type revolveSepAxis struct {
	g      proofarith.DyV3
	f      revIvVec
	length float64
	drift  float64
}

// sideOf reads which side of the candidate plane an offset lies on, and whether
// that reading survives the whole displaced family. The plane passes through the
// shared feature, so the offset is measured from a shared corner. The float
// pre-test reads the same dot product first (revIvSide) and answers for the
// exact reading wherever its interval settles it.
func (ax revolveSepAxis) sideOf(o revolveOffset, offsetDrift float64) (int, bool) {
	allow := perturbBilinearAllow(ax.length, o.length, ax.drift, offsetDrift)
	if proofbound.IsNonFinite(allow) {
		return 0, false
	}
	if side, ok := revIvSide(revIvDot(ax.f, o.f), allow); ok {
		return side, side != 0
	}
	return ax.sideOfExact(o.v, o.length, offsetDrift)
}

// sideOfExact is sideOf over the exact Dyadic dot product alone.
func (ax revolveSepAxis) sideOfExact(offset proofarith.DyV3, offsetLen, offsetDrift float64) (int, bool) {
	h := proofarith.DvDot(ax.g, offset)
	if h.IsZero() {
		return 0, false
	}
	allow := perturbBilinearAllow(ax.length, offsetLen, ax.drift, offsetDrift)
	if proofbound.IsNonFinite(allow) {
		return 0, false
	}
	bound, ok := proofarith.DyOf(allow)
	if !ok || proofarith.DyCmp(proofarith.DyAbs(h), bound) <= 0 {
		return 0, false
	}
	return h.Sign(), true
}

// perturbBilinearAllow bounds |a'∘b' − a∘b| for a dot or cross product when a
// and b slide by at most da and db: the two first-order terms plus the second.
func perturbBilinearAllow(la, lb, da, db float64) float64 {
	return proofbound.AbsSumUpper(proofbound.ProductUpper(la, db), proofbound.ProductUpper(lb, da), proofbound.ProductUpper(da, db))
}

// revolveNormalAxis is the candidate whose plane IS a triangle's own plane:
// every corner of that triangle reads exactly zero on it, identically, for the
// whole family.
func revolveNormalAxis(t revolveAuditTri, delta float64) revolveSepAxis {
	e := proofbound.ProductUpper(2, delta)
	return revolveSepAxis{
		g:      t.n,
		f:      t.fn,
		length: proofbound.ProductUpper(t.lu, t.lv),
		drift:  perturbBilinearAllow(t.lu, t.lv, e, e),
	}
}

// revolveEdgeFanAxis is the candidate whose plane contains a triangle's own
// plane normal and one of its edges through the shared corner. The edge itself
// reads exactly zero (a determinant with a repeated vector), so that triangle's
// only reading to charge is its remaining corner.
func revolveEdgeFanAxis(t revolveAuditTri, edge revolveOffset, delta float64) revolveSepAxis {
	e := proofbound.ProductUpper(2, delta)
	normal := revolveNormalAxis(t, delta)
	return revolveSepAxis{
		g:      proofarith.DvCross(normal.g, edge.v),
		f:      revIvCross(normal.f, edge.f),
		length: proofbound.ProductUpper(normal.length, edge.length),
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
func revolveRejectionAxis(d, u revolveOffset, delta float64) revolveSepAxis {
	e := proofbound.ProductUpper(2, delta)
	dLen, uLen := d.length, u.length
	cross := proofarith.DvCross(d.v, u.v)
	crossLen := proofbound.ProductUpper(dLen, uLen)
	crossDrift := perturbBilinearAllow(dLen, uLen, e, e)
	return revolveSepAxis{
		g:      proofarith.DvCross(cross, d.v),
		f:      revIvCross(revIvCross(d.f, u.f), d.f),
		length: proofbound.ProductUpper(crossLen, dLen),
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
	e := proofbound.ProductUpper(2, delta)
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
			s, ok := ax.sideOf(o, e)
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
	chord := revolveOffsetOf(proofarith.DvSub(aOff[0].v, aOff[1].v), revIvSubVec(aOff[0].f, aOff[1].f))
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
			g := proofarith.DvCross(aOff[k].v, bOff[m].v)
			if proofarith.DvIsZero(g) {
				continue
			}
			ax := revolveSepAxis{
				g:      g,
				f:      revIvCross(aOff[k].f, bOff[m].f),
				length: proofbound.ProductUpper(aOff[k].length, bOff[m].length),
				drift:  perturbBilinearAllow(aOff[k].length, bOff[m].length, e, e),
			}
			sa, okA := ax.sideOf(aOff[1-k], e)
			sb, okB := ax.sideOf(bOff[1-m], e)
			if okA && okB && sa != sb {
				return true
			}
		}
	}
	return false
}

// revolveOffset is one corner offset from the pair's shared corner, beside the
// proven upper bound on its length every perturbation allowance reads. f
// encloses v in float intervals for the pre-test.
type revolveOffset struct {
	v      proofarith.DyV3
	f      revIvVec
	length float64
}

func revolveOffsetOf(v proofarith.DyV3, f revIvVec) revolveOffset {
	return revolveOffset{v: v, f: f, length: proofbound.DvLenUpper(v)}
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
	e := proofbound.ProductUpper(2, delta)
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
	offB := revolveOffsetOf(proofarith.DvSub(b.p[bApex], a.p[p0]), revIvPointDiff(b.fp[bApex], a.fp[p0]))
	if _, ok := revolveNormalAxis(a, delta).sideOf(offB, e); ok {
		return true
	}
	d := revolveOffsetOf(proofarith.DvSub(a.p[p1], a.p[p0]), revIvPointDiff(a.fp[p1], a.fp[p0]))
	u := revolveOffsetOf(proofarith.DvSub(a.p[aApex], a.p[p0]), revIvPointDiff(a.fp[aApex], a.fp[p0]))
	ax := revolveRejectionAxis(d, u, delta)
	side, ok := ax.sideOf(offB, e)
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
	budget := proofbound.NewWorkBudget(ctx)
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
		if err := budget.Step(); err != nil {
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
		if err := budget.Step(); err != nil {
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
	return budget.Err()
}
