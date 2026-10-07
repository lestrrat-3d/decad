package revolvemesh

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// RevolveTrigGapPrior is the a-priori ceiling on how far one stored cosine or
// sine sits from the exact value it stands for. The tessellator does not call
// math.Sincos: it encloses the ideal angle's cosine and sine as rational
// intervals (proofbound.TurnSinCosInterval / proofbound.RadSinCosInterval, neither of which ever
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
	A3, W, E0, E1 proofbound.IvVec3
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
		proofbound.IntervalWiden(proofbound.PointInterval(ru), bu),
		proofbound.IntervalWiden(proofbound.PointInterval(rv), bv),
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
func RevolveIdealPoint(b RevolveBasis3Iv, z, rho, cos, sin proofbound.RatInterval) proofbound.IvVec3 {
	radial := proofbound.IvVec3Add(proofbound.IvVec3Mul(b.E0, cos), proofbound.IvVec3Mul(b.E1, sin))
	return proofbound.IvVec3Add(b.A3, proofbound.IvVec3Add(proofbound.IvVec3Mul(b.W, z), proofbound.IvVec3Mul(radial, rho)))
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
