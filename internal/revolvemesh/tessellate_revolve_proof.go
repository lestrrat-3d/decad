package revolvemesh

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/tessellation"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// RevolveTrigGapPrior is the a-priori ceiling on how far one stored cosine or
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
const RevolveTrigGapPrior = 0x1p-50

// RevolveEvalRoundUlps is the a-priori ulp allowance, at the mesh's own
// coordinate magnitude, for the float64 arithmetic that turns the payload's
// numbers into one stored unplaced vertex: the axis basis (a3, w, e0, e1 — two
// products and two sums each, plus e1's cross product), then X's own three
// scalings and three vector sums. Twenty-odd roundings, each at most half an
// ulp at a magnitude the coordinate envelope covers, so 256 is generous by an
// order of magnitude and still lands far below any tolerance a caller can
// state. As with the trig ceiling, the measured deltaC is checked against the
// budget this figure bought.
const RevolveEvalRoundUlps = 256

// RevolveStationRoundUlps is the a-priori ulp allowance, at the meridian's own
// coordinate magnitude, for one CHORDED meridian station's stored (z, ρ) pair
// (docs/tessellation-reach-design.md §6, R4). A station is stored as the float
// NEAREST the certified enclosure of the point its record denotes
// (revolveArcStation), so its gap is half an ulp plus that enclosure's own
// width; eight ulps covers both with room to spare while staying far below any
// tolerance a caller can state.
//
// It exists for RevolveTrigGapPrior's reason one level down: §8 splits the
// tolerance BEFORE the meridian counts are chosen, and how many stations there
// are is exactly what the count decides, so the split needs a per-sample
// ceiling that does not depend on it. Every station's real gap is MEASURED as
// it is emitted and refused if it exceeds this, so the a-priori figure is held
// to account rather than trusted.
const RevolveStationRoundUlps = 8

// RevolveAngular is the global angular sequence docs/tessellation-design.md §8
// makes load-bearing: ONE chord count for the whole mesh, so adjacent generator
// faces share their complete latitude edge, a full turn closes without a
// tolerance seam, and one cell proof applies at every radius.
//
// cos/sin are the stored values every ring is built from; cosIv/sinIv are the
// certified enclosures of the IDEAL angle they stand for. gap is the largest
// measured distance between the two over the whole sequence.
type RevolveAngular struct {
	N       int // angular chord count (the number of angular INTERVALS)
	Samples int // vertices per off-axis ring: n for a full turn, n+1 for a partial sweep
	Cos     []float64
	Sin     []float64
	CosIv   []proofbound.RatInterval
	SinIv   []proofbound.RatInterval
	Gap     float64
	// step is the exact enclosure of ONE angular interval's true width, the
	// dφ every Ecell integral reads.
	Step proofbound.RatInterval
}

var ErrRevolveAngleEnclosure = fmt.Errorf(`%w: an angular sample's cosine and sine cannot be enclosed, so this mesh can state no construction bound`, decaderr.ErrUnsupported)

// RevolveBasis3Iv is RevolveBasis enclosed exactly.
type RevolveBasis3Iv struct {
	A3, W, E0, E1 survey2d.IvVec3
}

// RevolveMeridianEnclosure encloses one meridian sample's IDEAL axis
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
func RevolveMeridianEnclosure(axU, axV, dirU, dirV, u, v float64, bound proofbound.WalkEndBound) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	ru, rv := proofarith.FloatRat(u), proofarith.FloatRat(v)
	bu, bv := proofarith.FloatRat(math.Abs(bound.U)), proofarith.FloatRat(math.Abs(bound.V))
	if ru == nil || rv == nil || bu == nil || bv == nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	return AxisCoordInterval(axU, axV, dirU, dirV,
		survey2d.IntervalWiden(proofbound.PointInterval(ru), bu),
		survey2d.IntervalWiden(proofbound.PointInterval(rv), bv),
	)
}

// AxisCoordInterval carries an enclosed PLANE-local point into the axis
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
func AxisCoordInterval(axU, axV, dirU, dirV float64, u, v proofbound.RatInterval) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	aU, aV := proofarith.FloatRat(axU), proofarith.FloatRat(axV)
	dU, dV := proofarith.FloatRat(dirU), proofarith.FloatRat(dirV)
	if aU == nil || aV == nil || dU == nil || dV == nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	du := proofbound.IntervalSub(u, proofbound.PointInterval(aU))
	dv := proofbound.IntervalSub(v, proofbound.PointInterval(aV))
	z := proofbound.IntervalAdd(proofbound.IntervalScale(du, dU), proofbound.IntervalScale(dv, dV))
	rho := proofbound.IntervalSub(proofbound.IntervalScale(dv, dU), proofbound.IntervalScale(du, dV))
	return z, rho, true
}

// RevolveIdealPoint encloses X(z, ρ, φ) exactly: the ideal unplaced sample
// docs/tessellation-design.md §8 measures every stored vertex against.
func RevolveIdealPoint(b RevolveBasis3Iv, z, rho, cos, sin proofbound.RatInterval) survey2d.IvVec3 {
	radial := survey2d.IvVec3Add(survey2d.IvVec3Mul(b.E0, cos), survey2d.IvVec3Mul(b.E1, sin))
	return survey2d.IvVec3Add(b.A3, survey2d.IvVec3Add(survey2d.IvVec3Mul(b.W, z), survey2d.IvVec3Mul(radial, rho)))
}

// RevolveCoordMax is §8's upward-rounded envelope of every ideal unplaced
// analytic-boundary coordinate. It is the magnitude the construction rounds AT,
// which is what both a-priori allowances below are stated against.
func RevolveCoordMax(b RevolveBasis, zAbsMax, rhoMax float64) float64 {
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
			component(b.A3),
			proofbound.ProductUpper(zAbsMax, math.Abs(component(b.W))),
			proofbound.ProductUpper(rhoMax, proofbound.AbsSumUpper(component(b.E0), component(b.E1))),
		)))
	}
	return worst
}

// RevolveConstructionPrior is the count-independent ceiling on deltaC the
// tolerance split spends before any angular count exists (§8 step 1).
//
// Every term of the ideal-to-stored gap is bounded here without knowing how
// many angles the mesh will carry: meridianGap is the largest gap a meridian
// sample's own (z, ρ) already showed, the trig term is the stored cosine and
// sine's ceiling times the radius they scale, and the last term is the float
// arithmetic's own ulps at the coordinate envelope. The tessellator measures
// the real deltaC afterwards and refuses if it exceeds what this bought.
func RevolveConstructionPrior(b RevolveBasis, meridianGap, rhoMax, coordMax float64) float64 {
	radialUnit := proofbound.AbsSumUpper(proofbound.VecMaxAbs(b.E0), proofbound.VecMaxAbs(b.E1))
	perCoord := proofbound.AbsSumUpper(
		proofbound.ProductUpper(meridianGap, proofbound.AbsSumUpper(proofbound.VecMaxAbs(b.W), radialUnit)),
		proofbound.ProductUpper(proofbound.ProductUpper(rhoMax, RevolveTrigGapPrior), radialUnit),
		proofbound.ProductUpper(RevolveEvalRoundUlps, proofbound.UlpOf(math.Max(coordMax, 1))),
	)
	return proofbound.Radius3D(perCoord)
}

// RevolveBudget is docs/tessellation-design.md §8's tolerance split: both
// coordinate stages are reserved from the requested tolerance BEFORE any chord
// count is chosen, and a tolerance they exhaust refuses rather than returning a
// mesh whose bound it cannot honour. Both subtractions round downward, which is
// the direction that leaves the reservation whole.
func RevolveBudget(tol, deltaC, deltaR float64) (float64, error) {
	available := freeform.DownRound(freeform.DownRound(tol - deltaC - deltaR))
	if available <= 0 || proofbound.IsNonFinite(available) {
		requested := units.Millimeters(tol)
		reserved := units.Millimeters(proofbound.AbsSumUpper(deltaC, deltaR))
		return 0, fmt.Errorf(`%w: requested tolerance %s leaves no chord budget above this revolve's own coordinate construction and placement displacement %s; retry with a tolerance greater than %s`, decaderr.ErrUnsupported, requested, reserved, reserved)
	}
	return available, nil
}

// ExactRigidPointRound measures the displacement docs/tessellation-design.md §8
// calls deltaR for ONE vertex: the gap between the EXACT rigid image of the
// stored unplaced point and the binary64 vertex the placement wrote for it. It
// is exactPrismPointRound's second half, applied to a point this build already
// holds rather than to a plane-local triple, and it answers exactly zero for an
// identity placement, whose products are by one and zero and whose sums commit
// no rounding at all.
func ExactRigidPointRound(xform r3.Transform, unplaced, held r3.Vec) float64 {
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
		perCoord = math.Max(perCoord, proofarith.DyadicFloatError(exact, VecComponent(held, i)))
	}
	return proofbound.Radius3D(perCoord)
}

func VecComponent(v r3.Vec, i int) float64 {
	switch i {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

// RevolveCellAreaSlack is docs/tessellation-design.md §10.2's Ecell for one
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
func RevolveCellAreaSlack(rho0, rho1 float64, meridian, step proofbound.RatInterval, twoArea [2]proofbound.RatInterval) float64 {
	r0, r1 := proofarith.FloatRat(rho0), proofarith.FloatRat(rho1)
	if r0 == nil || r1 == nil {
		return math.Inf(1)
	}
	dRho := new(big.Rat).Sub(r1, r0)
	scale := proofbound.IntervalMul(meridian, step)
	total := new(big.Rat)
	for half, weight := range [2]int{RevolveWeightT, RevolveWeightOneMinusT} {
		total.Add(total, RevolveHalfCellSlack(r0, dRho, scale, twoArea[half], weight))
	}
	return proofbound.RatFloatUp(total)
}

// RevolveFanAreaSlack is RevolveCellAreaSlack for a cell with ONE ring on the
// axis: the held facets are a fan of single triangles rather than quads, and
// the domain is the whole unit square with the pole edge collapsed.
//
// With the pole at t = 0 the held map is P + t·((1−u)·A + u·B), whose Jacobian
// is exactly t·|A × B|, while Jtrue is L·dφ·ρ1·t — both LINEAR in t through the
// origin, so the difference is again linear and the same closed form answers.
// The pole at t = 1 is the mirror image, in 1 − t.
func RevolveFanAreaSlack(rhoOff float64, poleFirst bool, meridian, step, twoArea proofbound.RatInterval) float64 {
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
		AbsLinearIntegral(alpha, beta, RevolveWeightOne),
		AbsLinearIntegral(widthAlpha, widthBeta, RevolveWeightOne),
	)
	return proofbound.RatFloatUp(total)
}

// RevolveHalfCellSlack is one half-domain's contribution to Ecell: the exact
// ∫|f_lo|·w plus the exact ∫(f_hi − f_lo)·w, with
// f(t) = L·dφ·(ρ0 + t·Δρ) − 2A.
func RevolveHalfCellSlack(rho0, dRho *big.Rat, scale, twoArea proofbound.RatInterval, weight int) *big.Rat {
	alphaLo := new(big.Rat).Mul(scale.Lo, dRho)
	betaLo := new(big.Rat).Sub(new(big.Rat).Mul(scale.Lo, rho0), twoArea.Hi)
	spread := new(big.Rat).Sub(scale.Hi, scale.Lo)
	alphaGap := new(big.Rat).Mul(spread, dRho)
	betaGap := new(big.Rat).Add(
		new(big.Rat).Mul(spread, rho0),
		new(big.Rat).Sub(twoArea.Hi, twoArea.Lo),
	)
	return new(big.Rat).Add(
		AbsLinearIntegral(alphaLo, betaLo, weight),
		AbsLinearIntegral(alphaGap, betaGap, weight),
	)
}

// The three weights AbsLinearIntegral integrates against: the whole unit
// interval, and the two halves the fixed cell diagonal cuts the unit square
// into.
const (
	RevolveWeightOne = iota
	RevolveWeightT
	RevolveWeightOneMinusT
)

// AbsLinearIntegral is the exact ∫₀¹ |α·t + β|·w(t) dt over the rationals. The
// integrand's single zero −β/α is isolated exactly, the unit interval is split
// there when the zero falls strictly inside it, and each sign-fixed piece is
// integrated through its own polynomial primitive. This is the whole of
// docs/tessellation-design.md §10.2's "isolate every zero ... then integrate
// each sign-fixed region in closed form" for a straight generator.
func AbsLinearIntegral(alpha, beta *big.Rat, weight int) *big.Rat {
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
			LinearWeightPrimitive(alpha, beta, bounds[i+1], weight),
			LinearWeightPrimitive(alpha, beta, bounds[i], weight),
		)
		total.Add(total, piece.Abs(piece))
	}
	return total
}

// LinearWeightPrimitive evaluates the antiderivative of (α·t + β)·w(t) at t.
func LinearWeightPrimitive(alpha, beta, t *big.Rat, weight int) *big.Rat {
	t2 := new(big.Rat).Mul(t, t)
	t3 := new(big.Rat).Mul(t2, t)
	switch weight {
	case RevolveWeightT:
		// (αt + β)·t = αt² + βt.
		return new(big.Rat).Add(
			new(big.Rat).Mul(alpha, new(big.Rat).Mul(t3, big.NewRat(1, 3))),
			new(big.Rat).Mul(beta, new(big.Rat).Mul(t2, big.NewRat(1, 2))),
		)
	case RevolveWeightOneMinusT:
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

// IvTwoTriangleArea encloses twice the area of a triangle whose corners are
// themselves enclosed — the |A × B| the held facet's own Jacobian is.
func IvTwoTriangleArea(p0, p1, p2 survey2d.IvVec3) (proofbound.RatInterval, bool) {
	n := survey2d.IvVec3Cross(survey2d.IvVec3Sub(p1, p0), survey2d.IvVec3Sub(p2, p0))
	return survey2d.IntervalSqrt(survey2d.IvVec3NormSq(n))
}

// RevolveAuditTri is one triangle's exact lift, held for the whole audit: its
// three corners, the three edge vectors (u = p1−p0, v = p2−p0, w = p2−p1),
// their cross product, and proven upper bounds on the three edge lengths.
// Every predicate below reads these rather than rebuilding them per pair,
// exactly as internal/loftmesh/loft_audit.go's own audit data does. fp holds the stored float
// corners, and fu, fv, fw and fn enclose u, v, w and n in float intervals for
// the pre-test (tessellate_revolve_filter.go).
type RevolveAuditTri struct {
	P              [3]proofarith.DyV3
	U, V, W, N     proofarith.DyV3
	Lu, Lv, Lw     float64
	Box            [2]r3.Vec
	Fp             [3]r3.Vec
	Fu, Fv, Fw, Fn RevIvVec
	// off[k] holds the two corner offsets measured from corner k, in
	// increasing corner order, each with its proven length bound.
	Off [3][2]RevolveOffset
}

func NewRevolveAuditTri(verts []r3.Vec, tri [3]int) (RevolveAuditTri, bool) {
	var out RevolveAuditTri
	for k, vi := range tri {
		if !proofbound.FiniteVec(verts[vi]) {
			return out, false
		}
		out.P[k] = proofarith.DyVec(verts[vi])
		out.Fp[k] = verts[vi]
	}
	out.U = proofarith.DvSub(out.P[1], out.P[0])
	out.V = proofarith.DvSub(out.P[2], out.P[0])
	out.W = proofarith.DvSub(out.P[2], out.P[1])
	out.N = proofarith.DvCross(out.U, out.V)
	out.Fu = RevIvPointDiff(out.Fp[1], out.Fp[0])
	out.Fv = RevIvPointDiff(out.Fp[2], out.Fp[0])
	out.Fw = RevIvPointDiff(out.Fp[2], out.Fp[1])
	out.Fn = RevIvCross(out.Fu, out.Fv)
	out.Lu = proofbound.DvLenUpper(out.U)
	out.Lv = proofbound.DvLenUpper(out.V)
	out.Lw = proofbound.DvLenUpper(out.W)
	out.Box = meshbool.TriBox(verts, tri)
	out.Off[0] = [2]RevolveOffset{{V: out.U, F: out.Fu, Length: out.Lu}, {V: out.V, F: out.Fv, Length: out.Lv}}
	out.Off[1] = [2]RevolveOffset{
		RevolveOffsetOf(proofarith.DvSub(out.P[0], out.P[1]), RevIvPointDiff(out.Fp[0], out.Fp[1])),
		{V: out.W, F: out.Fw, Length: out.Lw},
	}
	out.Off[2] = [2]RevolveOffset{
		RevolveOffsetOf(proofarith.DvSub(out.P[0], out.P[2]), RevIvPointDiff(out.Fp[0], out.Fp[2])),
		RevolveOffsetOf(proofarith.DvSub(out.P[1], out.P[2]), RevIvPointDiff(out.Fp[1], out.Fp[2])),
	}
	return out, true
}

// RevolveSeparated proves two triangles stay farther apart than 2·delta, so
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
// float pre-test runs the same walk first (RevolveSeparatedFloat); an axis it
// accepts is one the exact walk accepts too.
func RevolveSeparated(a, b RevolveAuditTri, delta float64) bool {
	return RevolveSeparatedFloat(a, b, delta) || RevolveSeparatedExact(a, b, delta)
}

// RevolveSeparatedExact is RevolveSeparated's walk over the exact Dyadic axes.
func RevolveSeparatedExact(a, b RevolveAuditTri, delta float64) bool {
	margin := proofbound.ProductUpper(2, delta)
	if proofbound.IsNonFinite(margin) {
		return false
	}
	ea := [3]proofarith.DyV3{a.U, a.V, a.W}
	eb := [3]proofarith.DyV3{b.U, b.V, b.W}
	la := [3]float64{a.Lu, a.Lv, a.Lw}
	lb := [3]float64{b.Lu, b.Lv, b.Lw}
	lnA := proofbound.ProductUpper(a.Lu, a.Lv)
	lnB := proofbound.ProductUpper(b.Lu, b.Lv)
	// axis is one candidate, built only when the candidates before it have
	// failed, beside a cheap proven upper bound on its length: |x × y| ≤ |x||y|.
	type axis struct {
		g     proofarith.DyV3
		bound float64
	}
	next := func(gi int) axis {
		switch {
		case gi == 0:
			return axis{a.N, lnA}
		case gi == 1:
			return axis{b.N, lnB}
		case gi < 11:
			x, y := (gi-2)/3, (gi-2)%3
			return axis{proofarith.DvCross(ea[x], eb[y]), proofbound.ProductUpper(la[x], lb[y])}
		case gi < 14:
			x := gi - 11
			return axis{proofarith.DvCross(a.N, ea[x]), proofbound.ProductUpper(lnA, la[x])}
		default:
			y := gi - 14
			return axis{proofarith.DvCross(b.N, eb[y]), proofbound.ProductUpper(lnB, lb[y])}
		}
	}
	for gi := range 17 {
		ax := next(gi)
		g := ax.g
		if proofarith.DvIsZero(g) {
			continue
		}
		aLo, aHi := DvProject(a.P, g)
		bLo, bHi := DvProject(b.P, g)
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

// DvProject is the exact projection range of a triangle's three corners onto
// one axis, before normalisation.
func DvProject(p [3]proofarith.DyV3, g proofarith.DyV3) (proofarith.Dyadic, proofarith.Dyadic) {
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

// BoxGapExceeds is the float pre-filter in front of RevolveSeparated: two
// axis-aligned boxes separated on one coordinate by more than margin prove the
// same thing the exact test would, for the cost of six comparisons. The
// difference is rounded DOWNWARD, so a gap this reports is one the exact
// arithmetic also has.
func BoxGapExceeds(a, b [2]r3.Vec, margin float64) bool {
	gap := func(x, y float64) bool { return freeform.DownRound(y-x) > margin }
	return gap(a[1].X, b[0].X) || gap(b[1].X, a[0].X) ||
		gap(a[1].Y, b[0].Y) || gap(b[1].Y, a[0].Y) ||
		gap(a[1].Z, b[0].Z) || gap(b[1].Z, a[0].Z)
}

// RequireRevolveFacetAreas proves docs/tessellation-design.md §1's positive-area
// row over the whole mesh and returns the audit triangles it built on the way.
//
// It stands OUTSIDE RevolveContactAudit, and runs at every verification level,
// because the two answer different questions at different costs. Positive area
// is per FACET and linear; a zero-area facet has no normal, so a renderer, the
// exporter's own facet normals and the boolean all need it. Contact is per
// facet PAIR and quadratic, and it is what a caller who only wants to draw the
// mesh declines. Folding the first into the second would silently drop §1's
// positive-area row from every mesh built below VerifyBoundary.
//
// The returned slice is parallel to tris, so RevolveContactAudit consumes it
// rather than walking the facets a second time. delta is the combined
// coordinate displacement deltaC + deltaR, as the audit's own doc comment
// derives it.
func RequireRevolveFacetAreas(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int, delta float64) ([]RevolveAuditTri, error) {
	if err := budget.Err(); err != nil {
		return nil, err
	}
	if proofbound.IsNonFinite(delta) || delta < 0 {
		return nil, fmt.Errorf(`%w: this revolve mesh states no finite coordinate displacement, so its facets cannot be audited`, decaderr.ErrUnsupported)
	}
	data := make([]RevolveAuditTri, len(tris))
	for i, tri := range tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		t, ok := NewRevolveAuditTri(verts, tri)
		if !ok {
			return nil, fmt.Errorf(`%w: revolve facet %d holds a coordinate that is not finite`, decaderr.ErrUnsupported, i)
		}
		if err := RequireRevolveFacetArea(t, i, delta); err != nil {
			return nil, err
		}
		data[i] = t
	}
	return data, budget.Err()
}

// RevolveContactAudit is docs/tessellation-design.md §9's facet-contact audit:
// adjacent facets meet ONLY along the vertex or edge their indices share, and
// no non-adjacent pair touches at all. It runs at VerifyBoundary and above, and
// data is RequireRevolveFacetAreas' own output for the same triangle set, which
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
//     (PerturbBilinearAllow). A reading whose stored value exceeds its own
//     allowance cannot change sign anywhere on the family, and a reading that
//     is structurally zero — a corner lying in a plane that was BUILT through
//     it — stays zero at every point of the family.
//   - Those signs decide the contact outright. A pair sharing a vertex or an
//     edge is isolated by a half-space whose boundary plane contains the shared
//     feature identically, with one triangle inside it and the other's
//     non-shared corners strictly outside (AuditRevolvePair's own doc comment
//     carries the candidates). A pair sharing nothing is proven apart by a
//     fixed separating axis with the same margin.
//
// So every intermediate boundary is embedded and its contact relations are the
// ones the stored mesh has, which is exactly what §9 charges to deltaC and
// deltaR. A pair the audit cannot decide is ErrUnsupported (§12), never an
// admission.
func RevolveContactAudit(budget *proofbound.WorkBudget, data []RevolveAuditTri, tris [][3]int, delta float64) error {
	if err := budget.Err(); err != nil {
		return err
	}
	if len(data) != len(tris) {
		return fmt.Errorf(`%w: the revolve facet audit holds %d triangles for a mesh of %d facets`, decaderr.ErrUnsupported, len(data), len(tris))
	}

	f := len(tris)
	pairs, ok := proofbound.WallChoose2(uint64(f))
	if !ok || pairs > proofbound.MaxFacetPairTestsPerCall {
		return fmt.Errorf(`%w: this revolve mesh's facet-pair audit needs %d exact tests, past the fixed ceiling of %d; retry with a coarser chord tolerance`, decaderr.ErrUnsupported, pairs, proofbound.MaxFacetPairTestsPerCall)
	}
	margin := proofbound.ProductUpper(2, delta)
	for i := range f {
		for j := i + 1; j < f; j++ {
			if err := budget.Step(); err != nil {
				return err
			}
			shared, count := tessellation.SharedVertexIndices(tris[i], tris[j])
			if count == 0 && BoxGapExceeds(data[i].Box, data[j].Box, margin) {
				continue
			}
			if err := AuditRevolvePair(data, tris, shared, count, i, j, delta); err != nil {
				return err
			}
		}
	}
	return budget.Err()
}

// RequireRevolveFacetArea proves one facet keeps a positive area everywhere on
// the displaced family: its exact held area, bracketed from BELOW, exceeds the
// most a displacement of delta at each corner can take from it
// (docs/tessellation-design.md §5's own per-triangle area allowance, read here
// as a gate rather than as a slack term).
func RequireRevolveFacetArea(t RevolveAuditTri, i int, delta float64) error {
	held := proofarith.DySqrtDown(proofarith.DvDot(t.N, t.N))
	allow := proofbound.ProductUpper(2, proofbound.AbsSumUpper(
		proofbound.ProductUpper(delta, proofbound.AbsSumUpper(t.Lu, t.Lv)),
		proofbound.ProductUpper(proofbound.ProductUpper(2, delta), delta),
	))
	if proofbound.IsNonFinite(held) || proofbound.IsNonFinite(allow) || held <= allow {
		return fmt.Errorf(`%w: revolve facet %d does not keep a positive area under the coordinate displacement this mesh carries`, decaderr.ErrUnsupported, i)
	}
	return nil
}

// AuditRevolvePair decides one facet pair against the contact its shared vertex
// indices require, under RevolveContactAudit's own displacement margin.
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
func AuditRevolvePair(data []RevolveAuditTri, tris [][3]int, shared [3]int, count, i, j int, delta float64) error {
	switch count {
	case 3:
		return fmt.Errorf(`%w: revolve facets %d and %d are the same triangle`, decaderr.ErrUnsupported, i, j)
	case 0:
		if RevolveSeparated(data[i], data[j], delta) {
			return nil
		}
		return fmt.Errorf(`%w: revolve facets %d and %d share no vertex, and this mesh cannot prove they stay apart under its own coordinate displacement`, decaderr.ErrUnsupported, i, j)
	case 1:
		if RevolveVertexIsolated(data[i], tris[i], data[j], tris[j], shared[0], delta) ||
			RevolveVertexIsolated(data[j], tris[j], data[i], tris[i], shared[0], delta) {
			return nil
		}
		return fmt.Errorf(`%w: revolve facets %d and %d do not provably meet only at the vertex they share`, decaderr.ErrUnsupported, i, j)
	default:
		if RevolveEdgeIsolated(data[i], tris[i], data[j], tris[j], shared, delta) ||
			RevolveEdgeIsolated(data[j], tris[j], data[i], tris[i], shared, delta) {
			return nil
		}
		return fmt.Errorf(`%w: revolve facets %d and %d do not provably meet only along the edge they share`, decaderr.ErrUnsupported, i, j)
	}
}

// RevolveSepAxis is one candidate boundary plane through the shared feature,
// carried as the plane's own (unnormalised) normal: the exact vector, a proven
// upper bound on its length, and a proven bound on how far that vector itself
// moves when every corner it was built from slides by up to the audit's margin.
// f encloses g in float intervals for the pre-test.
type RevolveSepAxis struct {
	G      proofarith.DyV3
	F      RevIvVec
	Length float64
	Drift  float64
}

// sideOf reads which side of the candidate plane an offset lies on, and whether
// that reading survives the whole displaced family. The plane passes through the
// shared feature, so the offset is measured from a shared corner. The float
// pre-test reads the same dot product first (RevIvSide) and answers for the
// exact reading wherever its interval settles it.
func (ax RevolveSepAxis) SideOf(o RevolveOffset, offsetDrift float64) (int, bool) {
	allow := PerturbBilinearAllow(ax.Length, o.Length, ax.Drift, offsetDrift)
	if proofbound.IsNonFinite(allow) {
		return 0, false
	}
	if side, ok := RevIvSide(RevIvDot(ax.F, o.F), allow); ok {
		return side, side != 0
	}
	return ax.SideOfExact(o.V, o.Length, offsetDrift)
}

// sideOfExact is sideOf over the exact Dyadic dot product alone.
func (ax RevolveSepAxis) SideOfExact(offset proofarith.DyV3, offsetLen, offsetDrift float64) (int, bool) {
	h := proofarith.DvDot(ax.G, offset)
	if h.IsZero() {
		return 0, false
	}
	allow := PerturbBilinearAllow(ax.Length, offsetLen, ax.Drift, offsetDrift)
	if proofbound.IsNonFinite(allow) {
		return 0, false
	}
	bound, ok := proofarith.DyOf(allow)
	if !ok || proofarith.DyCmp(proofarith.DyAbs(h), bound) <= 0 {
		return 0, false
	}
	return h.Sign(), true
}

// PerturbBilinearAllow bounds |a'∘b' − a∘b| for a dot or cross product when a
// and b slide by at most da and db: the two first-order terms plus the second.
func PerturbBilinearAllow(la, lb, da, db float64) float64 {
	return proofbound.AbsSumUpper(proofbound.ProductUpper(la, db), proofbound.ProductUpper(lb, da), proofbound.ProductUpper(da, db))
}

// RevolveNormalAxis is the candidate whose plane IS a triangle's own plane:
// every corner of that triangle reads exactly zero on it, identically, for the
// whole family.
func RevolveNormalAxis(t RevolveAuditTri, delta float64) RevolveSepAxis {
	e := proofbound.ProductUpper(2, delta)
	return RevolveSepAxis{
		G:      t.N,
		F:      t.Fn,
		Length: proofbound.ProductUpper(t.Lu, t.Lv),
		Drift:  PerturbBilinearAllow(t.Lu, t.Lv, e, e),
	}
}

// RevolveEdgeFanAxis is the candidate whose plane contains a triangle's own
// plane normal and one of its edges through the shared corner. The edge itself
// reads exactly zero (a determinant with a repeated vector), so that triangle's
// only reading to charge is its remaining corner.
func RevolveEdgeFanAxis(t RevolveAuditTri, edge RevolveOffset, delta float64) RevolveSepAxis {
	e := proofbound.ProductUpper(2, delta)
	normal := RevolveNormalAxis(t, delta)
	return RevolveSepAxis{
		G:      proofarith.DvCross(normal.G, edge.V),
		F:      RevIvCross(normal.F, edge.F),
		Length: proofbound.ProductUpper(normal.Length, edge.Length),
		Drift:  PerturbBilinearAllow(normal.Length, edge.Length, normal.Drift, e),
	}
}

// RevolveRejectionAxis is the candidate for a SHARED EDGE: the rejection of one
// triangle's apex off that edge, (d×u)×d = |d|²u − (d·u)d. Both shared corners
// read exactly zero on it — the first by construction, the second because the
// scalar triple product repeats d — and the apex reads the Gram determinant
// |d|²|u|² − (d·u)², which Cauchy-Schwarz makes non-negative identically. So
// that whole triangle sits in the closed half-space for the entire family with
// nothing to charge, and only the other triangle's apex is read.
func RevolveRejectionAxis(d, u RevolveOffset, delta float64) RevolveSepAxis {
	e := proofbound.ProductUpper(2, delta)
	dLen, uLen := d.Length, u.Length
	cross := proofarith.DvCross(d.V, u.V)
	crossLen := proofbound.ProductUpper(dLen, uLen)
	crossDrift := PerturbBilinearAllow(dLen, uLen, e, e)
	return RevolveSepAxis{
		G:      proofarith.DvCross(cross, d.V),
		F:      RevIvCross(RevIvCross(d.F, u.F), d.F),
		Length: proofbound.ProductUpper(crossLen, dLen),
		Drift:  PerturbBilinearAllow(crossLen, dLen, crossDrift, e),
	}
}

// RevolveVertexIsolated proves the pair meets only at the vertex they share,
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
func RevolveVertexIsolated(a RevolveAuditTri, triA [3]int, b RevolveAuditTri, triB [3]int, shared int, delta float64) bool {
	e := proofbound.ProductUpper(2, delta)
	ai := TriangleVertexSlot(triA, shared)
	bi := TriangleVertexSlot(triB, shared)
	if ai < 0 || bi < 0 {
		return false
	}
	aOff := RevolveCornerOffsets(a, ai)
	bOff := RevolveCornerOffsets(b, bi)

	// signed reads the common sign of the listed offsets, or reports that this
	// candidate cannot sign them all.
	signed := func(ax RevolveSepAxis, offs []RevolveOffset) (int, bool) {
		side := 0
		for _, o := range offs {
			s, ok := ax.SideOf(o, e)
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
	try := func(ax RevolveSepAxis, check []RevolveOffset) bool {
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
	if try(RevolveNormalAxis(a, delta), nil) {
		return true
	}
	for k := range 2 {
		if try(RevolveEdgeFanAxis(a, aOff[k], delta), []RevolveOffset{aOff[1-k]}) {
			return true
		}
	}
	chord := RevolveOffsetOf(proofarith.DvSub(aOff[0].V, aOff[1].V), RevIvSubVec(aOff[0].F, aOff[1].F))
	if try(RevolveEdgeFanAxis(a, chord, delta), aOff[:]) {
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
	// PerturbBilinearAllow over the two factors sliding by e, so the charge is
	// the one every other candidate here makes. A reading inside its allowance
	// stays UNDECIDED and the candidate is skipped, never admitted.
	for k := range 2 {
		for m := range 2 {
			g := proofarith.DvCross(aOff[k].V, bOff[m].V)
			if proofarith.DvIsZero(g) {
				continue
			}
			ax := RevolveSepAxis{
				G:      g,
				F:      RevIvCross(aOff[k].F, bOff[m].F),
				Length: proofbound.ProductUpper(aOff[k].Length, bOff[m].Length),
				Drift:  PerturbBilinearAllow(aOff[k].Length, bOff[m].Length, e, e),
			}
			sa, okA := ax.SideOf(aOff[1-k], e)
			sb, okB := ax.SideOf(bOff[1-m], e)
			if okA && okB && sa != sb {
				return true
			}
		}
	}
	return false
}

// RevolveOffset is one corner offset from the pair's shared corner, beside the
// proven upper bound on its length every perturbation allowance reads. f
// encloses v in float intervals for the pre-test.
type RevolveOffset struct {
	V      proofarith.DyV3
	F      RevIvVec
	Length float64
}

func RevolveOffsetOf(v proofarith.DyV3, f RevIvVec) RevolveOffset {
	return RevolveOffset{V: v, F: f, Length: proofbound.DvLenUpper(v)}
}

// RevolveCornerOffsets is a triangle's two corners other than the one at slot
// at, measured from that one.
func RevolveCornerOffsets(t RevolveAuditTri, at int) [2]RevolveOffset {
	return t.Off[at]
}

// RevolveEdgeIsolated proves the pair meets only along the edge they share,
// with the boundary plane built from triangle a: either a's own plane, or the
// rejection of a's apex off the shared edge — the candidate that answers the
// COPLANAR case every planar cell and every cap triangulation produces, where
// a's own plane says nothing at all.
func RevolveEdgeIsolated(a RevolveAuditTri, triA [3]int, b RevolveAuditTri, triB [3]int, shared [3]int, delta float64) bool {
	e := proofbound.ProductUpper(2, delta)
	p0 := TriangleVertexSlot(triA, shared[0])
	p1 := TriangleVertexSlot(triA, shared[1])
	apexA := tessellation.TriangleApexIndex(triA, shared[0], shared[1])
	apexB := tessellation.TriangleApexIndex(triB, shared[0], shared[1])
	if p0 < 0 || p1 < 0 || apexA < 0 || apexB < 0 {
		return false
	}
	bApex := TriangleVertexSlot(triB, apexB)
	aApex := TriangleVertexSlot(triA, apexA)
	if bApex < 0 || aApex < 0 {
		return false
	}
	offB := RevolveOffsetOf(proofarith.DvSub(b.P[bApex], a.P[p0]), RevIvPointDiff(b.Fp[bApex], a.Fp[p0]))
	if _, ok := RevolveNormalAxis(a, delta).SideOf(offB, e); ok {
		return true
	}
	d := RevolveOffsetOf(proofarith.DvSub(a.P[p1], a.P[p0]), RevIvPointDiff(a.Fp[p1], a.Fp[p0]))
	u := RevolveOffsetOf(proofarith.DvSub(a.P[aApex], a.P[p0]), RevIvPointDiff(a.Fp[aApex], a.Fp[p0]))
	ax := RevolveRejectionAxis(d, u, delta)
	side, ok := ax.SideOf(offB, e)
	// a itself lies in the g ≥ 0 half-space identically (the Gram determinant),
	// so the pair is isolated exactly when b's apex reads strictly negative.
	return ok && side < 0
}

// TriangleVertexSlot is the corner index a triangle carries a given vertex at,
// or −1 when it carries none.
func TriangleVertexSlot(tri [3]int, v int) int {
	for k, x := range tri {
		if x == v {
			return k
		}
	}
	return -1
}
