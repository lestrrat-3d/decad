package clearance

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// revolveGapMeter measures RevolveCarrierResult.AxisGap: every carrier
// BuildRevolveCarriers builds is compared, through
// revolvemesh.RevolveLift.MeridianGap, against the recorded meridian it was
// re-expressed from.
//
// A straight wall is compared at its two ends. Its carrier and its record are
// both affine along the wall for any one true axis and sweep angle, so their
// difference is too, and the worst of the two ends bounds every point
// between. A circular wall is compared once, over its whole circle: its
// carrier's meridian angle is the record's plane angle less the held
// rotation β the walk subtracted (revolveaxis.Frame.Walk), so the comparison
// rotates the carrier back by the exact cos β and sin β of that held float.
// The carrier's meridian window is that same subtraction, rounded, so the
// window's own two ends can sit off the recorded arc's ends by an angle Δ, and
// a point within Δ of an end sits within R·Δ of the recorded arc; that term is
// charged beside the comparison.
//
// A cone is compared through the walk's own two ends rather than through the
// apex and half angle its carrier rebuilds from them; the rounding of that
// rebuild is the carrier's own construction, not the axis re-expression this
// meter covers.
type revolveGapMeter struct {
	in         RevolveCarrierInput
	a3p, wp    r3.Vec
	walls      [2]r3.Vec
	sinB, cosB proofbound.RatInterval
	gap        float64
}

func newRevolveGapMeter(in RevolveCarrierInput, a3p, wp, e0p, e1p r3.Vec) *revolveGapMeter {
	m := &revolveGapMeter{in: in, a3p: a3p, wp: wp, walls: [2]r3.Vec{e0p, e1p}}
	beta := proofarith.FloatRat(math.Atan2(in.Lift.DV, in.Lift.DU))
	if beta == nil {
		m.gap = math.Inf(1)
		return m
	}
	sin, cos, ok := proofbound.RadSinCosInterval(beta)
	if !ok {
		m.gap = math.Inf(1)
		return m
	}
	m.sinB, m.cosB = sin, cos
	return m
}

func (m *revolveGapMeter) charge(gap float64) { m.gap = math.Max(m.gap, gap) }

// along is the exact axis point a3p + wp·z a carrier denotes.
func (m *revolveGapMeter) along(z float64) (proofbound.IvVec3, bool) {
	a, ok1 := proofbound.IvVec3Of(m.a3p)
	w, ok2 := proofbound.IvVec3Of(m.wp)
	zi, ok3 := revolvemesh.FloatInterval(z)
	if !ok1 || !ok2 || !ok3 {
		return proofbound.IvVec3{}, false
	}
	return proofbound.IvVec3Add(a, proofbound.IvVec3Mul(w, zi)), true
}

// point charges one straight-wall sample: origin swept at radius rho by the
// radial pair, against the recorded point rec.
func (m *revolveGapMeter) point(origin proofbound.IvVec3, ok bool, radial [2]r3.Vec, rho float64, rec revolvemesh.RecordedMeridian) {
	r, rok := revolvemesh.FloatInterval(rho)
	if !ok || !rok {
		m.charge(math.Inf(1))
		return
	}
	zero := revolvemesh.ZeroInterval()
	held := revolvemesh.HeldMeridian{
		Origin: origin, Axis: m.wp, Radial: radial,
		Z:   [2]proofbound.RatInterval{zero, zero},
		Rho: [3]proofbound.RatInterval{r, zero, zero},
	}
	m.charge(m.in.Lift.MeridianGap(m.in.Axis, m.in.Transform, held, rec))
}

// circle charges one circular-wall sample: origin is the carrier's centre on
// the axis, centreRho its centre's own radius (zero for a sphere, which reads
// its centre as on the axis), w the axis-coordinate walk and plane its
// recorded plane-local walk.
func (m *revolveGapMeter) circle(origin proofbound.IvVec3, ok bool, radial [2]r3.Vec, centreRho float64, w, plane survey2d.SegmentWalk) {
	r, rok := revolvemesh.FloatInterval(w.Radius)
	c, cok := revolvemesh.FloatInterval(centreRho)
	if !ok || !rok || !cok || m.sinB.Lo == nil {
		m.charge(math.Inf(1))
		return
	}
	cosB, sinB := m.cosB, m.sinB
	if w.Closed {
		// A whole circle's carrier is the same set at every phase, so any
		// real rotation pairs it with the record. The held direction itself
		// is one wherever it is exactly unit — every coordinate axis — and
		// then the comparison carries no trigonometric enclosure at all.
		if du, dv, unit := m.unitDirection(); unit {
			cosB, sinB = du, dv
		}
	}
	rc, rs := proofbound.IntervalMul(r, cosB), proofbound.IntervalMul(r, sinB)
	held := revolvemesh.HeldMeridian{
		Origin: origin, Axis: m.wp, Radial: radial,
		Z:   [2]proofbound.RatInterval{rc, rs},
		Rho: [3]proofbound.RatInterval{c, proofbound.IntervalNeg(rs), rc},
	}
	rec := revolvemesh.RecordedMeridian{U: plane.CU, V: plane.CV, R: plane.Radius, RBound: plane.RadiusBound}
	gap := m.in.Lift.MeridianGap(m.in.Axis, m.in.Transform, held, rec)
	if !w.Closed {
		if window := m.windowCharge(w, plane); window > 0 {
			gap = proofbound.AbsSumUpper(gap, window)
		}
	}
	m.charge(gap)
}

// unitDirection reports the held axis direction (DU, DV) as exact cosine and
// sine when DU² + DV² is exactly one.
func (m *revolveGapMeter) unitDirection() (proofbound.RatInterval, proofbound.RatInterval, bool) {
	du, ok1 := revolvemesh.FloatInterval(m.in.Lift.DU)
	dv, ok2 := revolvemesh.FloatInterval(m.in.Lift.DV)
	if !ok1 || !ok2 {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	norm := new(big.Rat).Mul(du.Lo, du.Lo)
	norm.Add(norm, new(big.Rat).Mul(dv.Lo, dv.Lo))
	return du, dv, norm.Cmp(big.NewRat(1, 1)) == 0
}

// windowCharge is R·Δ: the recorded radius's upper bound times the larger
// exact angle by which the carrier's held window ends, rotated back by β, miss
// the recorded arc's own.
func (m *revolveGapMeter) windowCharge(w, plane survey2d.SegmentWalk) float64 {
	beta := proofarith.FloatRat(math.Atan2(m.in.Lift.DV, m.in.Lift.DU))
	worst := new(big.Rat)
	for _, pair := range [...][2]float64{{w.Th0, plane.Th0}, {w.Th1, plane.Th1}} {
		held, rec := proofarith.FloatRat(pair[0]), proofarith.FloatRat(pair[1])
		if held == nil || rec == nil || beta == nil {
			return math.Inf(1)
		}
		d := new(big.Rat).Add(held, beta)
		d.Sub(d, rec)
		d.Abs(d)
		if d.Cmp(worst) > 0 {
			worst = d
		}
	}
	if worst.Sign() == 0 {
		return 0
	}
	delta := proofbound.IntervalFloatError(proofbound.PointInterval(worst), 0)
	return proofbound.ProductUpper(proofbound.AbsSumUpper(plane.Radius, plane.RadiusBound), delta)
}

// recordedEnds are the recorded points a straight walk's two ends denote.
func recordedEnds(wall RevolveWall) (revolvemesh.RecordedMeridian, revolvemesh.RecordedMeridian) {
	return revolvemesh.RecordedMeridian{U: wall.First.StartU, V: wall.First.StartV, UV: wall.First.StartBound},
		revolvemesh.RecordedMeridian{U: wall.Last.EndU, V: wall.Last.EndV, UV: wall.Last.EndBound}
}

// wall charges the face f one wall built, read the way the kernel reads f.
func (m *revolveGapMeter) wall(wall RevolveWall, f *CFace) {
	w := wall.Walk
	start, end := recordedEnds(wall)
	switch wall.Kind {
	case revolveaxis.WallCylinder:
		o0, ok0 := m.along(w.StartU)
		o1, ok1 := m.along(w.EndU)
		m.point(o0, ok0, m.walls, f.Radius, start)
		m.point(o1, ok1, m.walls, f.Radius, end)
	case revolveaxis.WallPlane:
		o, ok := proofbound.IvVec3Of(f.O)
		m.point(o, ok, m.walls, w.StartV, start)
		m.point(o, ok, m.walls, w.EndV, end)
	case revolveaxis.WallCone:
		o0, ok0 := m.along(w.StartU)
		o1, ok1 := m.along(w.EndU)
		m.point(o0, ok0, m.walls, w.StartV, start)
		m.point(o1, ok1, m.walls, w.EndV, end)
	case revolveaxis.WallSphere:
		o, ok := proofbound.IvVec3Of(f.Anchor)
		m.circle(o, ok, m.walls, 0, w.SegmentWalk, wall.First)
	case revolveaxis.WallTorus:
		o, ok := proofbound.IvVec3Of(f.Anchor)
		m.circle(o, ok, m.walls, f.Major, w.SegmentWalk, wall.First)
	}
}

// capRegion charges one wall's element of the two caps' region: the cap
// plane a3p + wp·z + V·ρ, its region read in the walk's own axis
// coordinates — a straight walk's two ends, never a cylinder's mean radius,
// and a circular walk's centre at its own radius, never one read as on the
// axis. The comparison takes the cap's in-plane direction as the exact
// rotation of the walls' radial pair by the held angle, so it holds at both
// caps at once; how far the float V the cap was built from sits off that
// rotation is the cap's own construction rounding, the family
// BodyFaceTiltDelta charges for its normal, not the re-expression this meter
// covers.
func (m *revolveGapMeter) capRegion(wall RevolveWall) {
	w := wall.Walk
	if w.IsCircular() {
		o, ok := m.along(w.CU)
		m.circle(o, ok, m.walls, w.CV, w.SegmentWalk, wall.First)
		return
	}
	start, end := recordedEnds(wall)
	o0, ok0 := m.along(w.StartU)
	o1, ok1 := m.along(w.EndU)
	m.point(o0, ok0, m.walls, w.StartV, start)
	m.point(o1, ok1, m.walls, w.EndV, end)
}
