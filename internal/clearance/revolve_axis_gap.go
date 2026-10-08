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
// A straight walk that merged several recorded segments (survey2d.SideWalk's
// Segs) is also compared at every interior junction, at the carrier point its
// own float axis coordinate names (RevolveWall.Joints). Coalescing merges
// only walks that are exactly collinear in those coordinates
// (internal/boundarywalk), but the coordinates are a rounded re-expression, so
// the recorded junction can still sit off the line; carrier and record are
// affine between consecutive samples, so the worst sample bounds the wall.
//
// A cone is compared as its carrier stands: the apex the carrier rebuilt in
// float from the walk's ends, at the two axial window ends it trims to, with
// the radius growing at the exact slope |dρ|/|dz| of the walk's own float
// differences, and its apex vertex where the walk reaches the axis. That
// covers the apex's own rounding, which sits at the scale of the axial
// coordinate and so can dwarf every world coordinate of a part whose axis
// anchor lies far along its axis. The carrier holds that slope itself
// (CFace.Rise and CFace.Run), and every cell reads its sine, cosine, tangent
// and meridian distance off it, so no float half angle stands between the
// cone compared here and the cone the cells read.
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
// A cone takes cone instead.
func (m *revolveGapMeter) wall(wall RevolveWall, f *CFace) {
	w := wall.Walk
	start, end := recordedEnds(wall)
	switch wall.Kind {
	case revolveaxis.WallCylinder:
		o0, ok0 := m.along(w.StartU)
		o1, ok1 := m.along(w.EndU)
		m.point(o0, ok0, m.walls, f.Radius, start)
		m.point(o1, ok1, m.walls, f.Radius, end)
		for _, j := range wall.Joints {
			o, ok := m.along(j.Z)
			m.point(o, ok, m.walls, f.Radius, j.Rec)
		}
	case revolveaxis.WallPlane:
		o, ok := proofbound.IvVec3Of(f.O)
		m.point(o, ok, m.walls, w.StartV, start)
		m.point(o, ok, m.walls, w.EndV, end)
		for _, j := range wall.Joints {
			m.point(o, ok, m.walls, j.Rho, j.Rec)
		}
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
// coordinates — a straight walk's two ends and its joints, never a
// cylinder's mean radius, and a circular walk's centre at its own radius,
// never one read as on the axis. The comparison takes the cap's in-plane
// direction as the exact rotation of the walls' radial pair by the held
// angle, so it holds at both caps at once.
//
// How far the float V the cap was built from sits off that rotation is not
// charged, and it was not seen to matter (docs/clearance-design.md §2): it
// tilts the cap plane by about 1e-16 radians, so it moves a point at radius ρ
// across the plane by about ρ·1e-16, the size of the rounding every cell
// commits evaluating the plane's equation from its origin on the axis, ρ
// away; along the plane it moves only the region's trim, which changes a
// distance to second order. Ball fixtures at ρ ≈ 2¹⁰ and 2²⁰, swept by 0.3
// to 2.1 rad, put the truth at most a quarter of the published bound from
// the row's value. The cap's corners, where the rounding moves a point to
// first order, are topology vertices, and bodyGeom.widenDelta widens the row
// by their own bounds.
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
	for _, j := range wall.Joints {
		o, ok := m.along(j.Z)
		m.point(o, ok, m.walls, j.Rho, j.Rec)
	}
}

// cone charges the cone carrier f one wall built about the float apex apexZ:
// (f.Anchor + f.Axis·h) + Radial(φ)·h·s at each axial window end h (and at each
// joint's own), with s = |dρ|/|dz| the exact slope of the walk's own float
// differences, which is the slope the carrier holds as f.Rise over f.Run.
// An end on the axis is also the apex vertex BuildRevolveCarriers
// synthesizes at f.Anchor, so that end is compared there too.
func (m *revolveGapMeter) cone(wall RevolveWall, f *CFace, apexZ, dz, dr float64) {
	w := wall.Walk
	start, end := recordedEnds(wall)
	anchor, aok := proofbound.IvVec3Of(f.Anchor)
	axis, xok := proofbound.IvVec3Of(f.Axis)
	num, den := proofarith.FloatRat(math.Abs(dr)), proofarith.FloatRat(math.Abs(dz))
	if !aok || !xok || num == nil || den == nil || den.Sign() == 0 {
		m.charge(math.Inf(1))
		return
	}
	slope := proofbound.PointInterval(new(big.Rat).Quo(num, den))
	at := func(z float64, rec revolvemesh.RecordedMeridian) {
		h, ok := revolvemesh.FloatInterval(math.Abs(z - apexZ))
		if !ok {
			m.charge(math.Inf(1))
			return
		}
		// The axial offset joins the origin as an exact sum, the form
		// MeridianGap compares a point sample in.
		zero := revolvemesh.ZeroInterval()
		held := revolvemesh.HeldMeridian{
			Origin: proofbound.IvVec3Add(anchor, proofbound.IvVec3Mul(axis, h)), Axis: f.Axis, Radial: m.walls,
			Z:   [2]proofbound.RatInterval{zero, zero},
			Rho: [3]proofbound.RatInterval{proofbound.IntervalMul(h, slope), zero, zero},
		}
		m.charge(m.in.Lift.MeridianGap(m.in.Axis, m.in.Transform, held, rec))
	}
	at(w.StartU, start)
	at(w.EndU, end)
	for _, j := range wall.Joints {
		at(j.Z, j.Rec)
	}
	if w.StartV <= 0 {
		m.point(anchor, true, m.walls, 0, start)
	}
	if w.EndV <= 0 {
		m.point(anchor, true, m.walls, 0, end)
	}
}
