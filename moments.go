package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the mass-property engine of docs/evaluator-design.md §4:
// decad integrating its OWN records. sketch decides topology and
// admissibility; once a region is recorded, its areas and moments are decad's
// job, computed by closed-form boundary integrals (Green's theorem) per
// segment kind — exact for line, circle and arc walks, and ErrUnsupported for
// the free-form kinds.

// Area returns the recorded region's net area — the outer loop minus its
// holes — as a [Measurement] of Kind Area (mm²): a computed quantity carries
// its Exactness and Bound (docs/api-design.md §6). The closed forms here are
// exact, so the measurement reads Exact with a zero bound; each boundary
// segment contributes its Green's-theorem integral in walk order, so a hole's
// clockwise walk subtracts without a special case.
//
// A region whose boundary contains a free-form segment kind (ellipse,
// elliptical arc, conic, spline, closed spline, fit spline, NURBS) is
// [ErrUnsupported] (docs/evaluator-design.md §11) — never approximated.
func (r ProfileRecord) Area() (Measurement, error) {
	ig, err := r.integrals()
	if err != nil {
		return Measurement{}, err
	}
	return Measurement{
		Value:     units.SquareMillimeters(ig.area),
		Exactness: Exact,
		Bound:     units.SquareMillimeters(0),
	}, nil
}

// Centroid returns the recorded region's centroid from its exact first
// moments, as a [VecMeasurement] — a computed coordinate is a measurement
// (docs/api-design.md §6). The Value is PLANE-LOCAL: (u, v, 0) in the
// region's own plane coordinates, millimetres (§5.2), not a world position —
// lift it through the profile's PlaneRecord to place it in space. The closed
// forms are exact, so it reads Exact with a zero bound.
//
// A region whose net area is zero has no centroid and is [ErrDegenerate].
func (r ProfileRecord) Centroid() (VecMeasurement, error) {
	ig, err := r.integrals()
	if err != nil {
		return VecMeasurement{}, err
	}
	if ig.area == 0 {
		return VecMeasurement{}, fmt.Errorf(`%w: a region with zero net area has no centroid`, ErrDegenerate)
	}
	return VecMeasurement{
		Value:     r3.NewVec(ig.mu/ig.area, ig.mv/ig.area, 0),
		Exactness: Exact,
		Bound:     units.Millimeters(0),
	}, nil
}

// SecondMoments is a recorded region's second moments of area about the
// plane origin, in the plane's own (u, v): every field is a Measurement of
// Kind SecondMomentOfArea (mm⁴), Exact with a zero bound — the closed forms
// admit no approximation. They are what a revolve's solid centroid is
// computed from (docs/evaluator-design.md §4/§6); to re-reference them to
// another axis, use the parallel-axis theorem with the region's Area and
// Centroid.
type SecondMoments struct {
	// UU is ∫u² dA, VV is ∫v² dA, UV is the mixed ∫uv dA.
	UU Measurement
	UV Measurement
	VV Measurement
}

// SecondMoments returns the region's exact second moments of area about the
// plane origin. The staging matches Area: a free-form boundary kind is
// [ErrUnsupported].
func (r ProfileRecord) SecondMoments() (SecondMoments, error) {
	ig, err := r.integrals()
	if err != nil {
		return SecondMoments{}, err
	}
	exact := func(x float64) Measurement {
		return Measurement{Value: units.QuarticMillimeters(x), Exactness: Exact, Bound: units.QuarticMillimeters(0)}
	}
	return SecondMoments{UU: exact(ig.muu), UV: exact(ig.muv), VV: exact(ig.mvv)}, nil
}

// regionIntegrals accumulates the boundary integrals of one region: the net
// signed area, the first moments ∫u dA and ∫v dA, and the second moments
// ∫u² dA, ∫uv dA and ∫v² dA.
type regionIntegrals struct {
	area float64
	mu   float64 // ∫u dA
	mv   float64 // ∫v dA
	muu  float64 // ∫u² dA
	muv  float64 // ∫uv dA
	mvv  float64 // ∫v² dA
}

// integrals walks the outer loop and every hole in recorded walk order and
// sums each segment's closed-form contribution. Walk order carries the sign:
// the outer loop is counter-clockwise (positive), holes are clockwise
// (negative), so the sum IS the net region integral.
func (r ProfileRecord) integrals() (regionIntegrals, error) {
	var ig regionIntegrals
	for loopIndex, loop := range append([]LoopRecord{r.Outer}, r.Holes...) {
		if err := validateMomentLoop(loop, loopIndex); err != nil {
			return regionIntegrals{}, err
		}
		for _, seg := range loop.Segments {
			if err := ig.add(seg); err != nil {
				return regionIntegrals{}, err
			}
		}
	}
	return ig, nil
}

// validateMomentLoop rejects a record that leaves any boundary junction open.
// The integrals add only the declared walks; joining near endpoints here would
// invent a closing edge and make a nonzero error bound look Exact.
func validateMomentLoop(loop LoopRecord, loopIndex int) error {
	if len(loop.Segments) == 0 {
		return nil
	}

	first, err := walkOf(loop.Segments[0])
	if err != nil {
		return fmt.Errorf(`profile loop %d segment 0: %w`, loopIndex, err)
	}
	previous := first
	for segmentIndex := 1; segmentIndex < len(loop.Segments); segmentIndex++ {
		current, err := walkOf(loop.Segments[segmentIndex])
		if err != nil {
			return fmt.Errorf(`profile loop %d segment %d: %w`, loopIndex, segmentIndex, err)
		}
		if !momentWalksJoin(previous, current) {
			return fmt.Errorf(`%w: profile loop %d has an open junction before segment %d`, ErrDegenerate, loopIndex, segmentIndex)
		}
		previous = current
	}
	if !momentWalksJoin(previous, first) {
		return fmt.Errorf(`%w: profile loop %d does not close`, ErrDegenerate, loopIndex)
	}
	return nil
}

// momentWalksJoin permits only the endpoint rounding from evaluating a recorded
// walk. This is not a geometric distance weld: a material gap remains open.
func momentWalksJoin(a, b segmentWalk) bool {
	uScale := math.Max(math.Abs(a.endU), math.Abs(b.startU))
	vScale := math.Max(math.Abs(a.endV), math.Abs(b.startV))
	if a.circular {
		uScale = math.Max(uScale, math.Abs(a.cU))
		uScale = math.Max(uScale, math.Abs(a.radius))
		vScale = math.Max(vScale, math.Abs(a.cV))
		vScale = math.Max(vScale, math.Abs(a.radius))
	}
	if b.circular {
		uScale = math.Max(uScale, math.Abs(b.cU))
		uScale = math.Max(uScale, math.Abs(b.radius))
		vScale = math.Max(vScale, math.Abs(b.cV))
		vScale = math.Max(vScale, math.Abs(b.radius))
	}
	return momentCoordinatesJoin(a.endU, b.startU, uScale) && momentCoordinatesJoin(a.endV, b.startV, vScale)
}

func momentCoordinatesJoin(a, b, scale float64) bool {
	if !finiteSegmentValue(a) || !finiteSegmentValue(b) || !finiteSegmentValue(scale) {
		return false
	}
	if a == b {
		return true
	}
	next := math.Nextafter(scale, math.Inf(1))
	if math.IsInf(next, 1) {
		// The largest finite float has no larger finite neighbour. A distinct
		// endpoint is a material gap because there is no representable rounding
		// allowance at this scale.
		return false
	}
	return math.Abs(a-b) <= 32*(next-scale)
}

// add accumulates one segment's boundary-integral contribution, in the
// segment's recorded walk direction (TStart→TEnd, which TStart > TEnd runs
// against the curve's natural sense — the sign falls out of the integral
// limits, no special case).
func (ig *regionIntegrals) add(seg CurveSegment) error {
	// The codec accepts pointer variants and normalizes them to values
	// (record.go); the integrals read segments on the same terms, so a
	// *LineSeg integrates exactly like its value and a nil pointer is
	// rejected rather than misread as an unsupported kind.
	seg, err := normalizeSegment(seg)
	if err != nil {
		return err
	}
	switch seg := seg.(type) {
	case LineSeg:
		ig.addLine(seg)
		return nil
	case CircleSeg:
		r, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return fmt.Errorf(`decad: a circle segment's radius is not a length: %w`, err)
		}
		// The record carries the walk twice — the range order, and CCW —
		// and the two must agree (a reversed walk swaps the range AND flips
		// CCW, seam §2). A record that contradicts itself is malformed, not
		// a judgement call.
		if seg.CCW != (seg.TStart < seg.TEnd) {
			return fmt.Errorf(`%w: a circle segment's CCW flag contradicts its range order`, ErrDegenerate)
		}
		// The arrangement's normalized t is the angle 2π·t from +u
		// (geom.BoundaryEdge); the recorded range order is the walk.
		ig.addCircular(seg.Center, r, 2*math.Pi*seg.TStart, 2*math.Pi*seg.TEnd)
		return nil
	case ArcSeg:
		// geom.Arc's derived readings, computed on the record's own pinned
		// points: radius from Center→Start, angles about the center, the
		// sweep CCW-wrapped into (0, 2π].
		radius := math.Hypot(seg.Start.U-seg.Center.U, seg.Start.V-seg.Center.V)
		a0 := math.Atan2(seg.Start.V-seg.Center.V, seg.Start.U-seg.Center.U)
		a1 := math.Atan2(seg.End.V-seg.Center.V, seg.End.U-seg.Center.U)
		sweep := math.Mod(a1-a0, 2*math.Pi)
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		// normalized t maps to angle = a0 + t·sweep; the range order is the walk.
		ig.addCircular(seg.Center, radius, a0+seg.TStart*sweep, a0+seg.TEnd*sweep)
		return nil
	default:
		return fmt.Errorf(`%w: this evaluator computes mass properties over line, arc and circle profile segments only; the profile has a %T segment`, ErrUnsupported, seg)
	}
}

// addLine accumulates the straight chord from the walk's start point to its
// end point. The recorded range picks the walked piece of the entity's own
// Start→End parameterization.
func (ig *regionIntegrals) addLine(seg LineSeg) {
	u0, v0 := lerp2(seg.Start, seg.End, seg.TStart)
	u1, v1 := lerp2(seg.Start, seg.End, seg.TEnd)
	// A     = ½ ∮ (u dv − v du)
	// ∫u dA = ½ ∮ u² dv
	// ∫v dA = −½ ∮ v² du
	ig.area += 0.5 * (u0*v1 - u1*v0)
	ig.mu += (v1 - v0) * (u0*u0 + u0*u1 + u1*u1) / 6
	ig.mv += -(u1 - u0) * (v0*v0 + v0*v1 + v1*v1) / 6

	// ∫u² dA = ⅓ ∮ u³ dv;  ∫v² dA = −⅓ ∮ v³ du — the cubic sums are the
	// exact ∫₀¹ of the lerp cubed.
	ig.muu += (v1 - v0) * (u0*u0*u0 + u0*u0*u1 + u0*u1*u1 + u1*u1*u1) / 12
	ig.mvv += -(u1 - u0) * (v0*v0*v0 + v0*v0*v1 + v0*v1*v1 + v1*v1*v1) / 12

	// ∫uv dA = ½ ∮ u²v dv: expand u(t)² v(t) and integrate the polynomial
	// exactly, term by term.
	du, dv := u1-u0, v1-v0
	intU2V := v0*(u0*u0+u0*du+du*du/3) + dv*(u0*u0/2+2*u0*du/3+du*du/4)
	ig.muv += 0.5 * dv * intU2V
}

// addCircular accumulates a circular path about center c with radius r, from
// angle th0 to th1 in the walk direction (th1 < th0 walks clockwise). The
// antiderivatives are exact, so a whole period, a fragment, and a reversed
// walk are all the same formula.
func (ig *regionIntegrals) addCircular(c Point2, r, th0, th1 float64) {
	sin0, cos0 := math.Sincos(th0)
	sin1, cos1 := math.Sincos(th1)
	dth := th1 - th0

	// A = ½ ∫ (u v′ − v u′) dθ = ½ [r²·θ + c_u·r·sin θ + c_v·r·cos θ]
	ig.area += 0.5 * (r*r*dth + c.U*r*(sin1-sin0) - c.V*r*(cos1-cos0))

	// ∫u dA = ½ ∮ u² dv, with u = c_u + r cos θ, dv = r cos θ dθ:
	//   ½ r [c_u²·sin θ + 2 c_u r (θ/2 + sin 2θ/4) + r²(sin θ − sin³θ/3)]
	intCos := sin1 - sin0
	intCos2 := dth/2 + (math.Sin(2*th1)-math.Sin(2*th0))/4
	intCos3 := (sin1 - sin1*sin1*sin1/3) - (sin0 - sin0*sin0*sin0/3)
	ig.mu += 0.5 * r * (c.U*c.U*intCos + 2*c.U*r*intCos2 + r*r*intCos3)

	// ∫v dA = −½ ∮ v² du, with v = c_v + r sin θ, du = −r sin θ dθ:
	//   ½ r [c_v²·(−cos θ)′... i.e. ½ r ∫ v² sin θ dθ]
	intSin := cos0 - cos1
	intSin2 := dth/2 - (math.Sin(2*th1)-math.Sin(2*th0))/4
	intSin3 := (cos0 - cos0*cos0*cos0/3) - (cos1 - cos1*cos1*cos1/3)
	ig.mv += 0.5 * r * (c.V*c.V*intSin + 2*c.V*r*intSin2 + r*r*intSin3)

	// The quartic antiderivatives for the second moments:
	//   ∫cos⁴θ dθ = 3θ/8 + sin 2θ/4 + sin 4θ/32
	//   ∫sin⁴θ dθ = 3θ/8 − sin 2θ/4 + sin 4θ/32
	intCos4 := 3*dth/8 + (math.Sin(2*th1)-math.Sin(2*th0))/4 + (math.Sin(4*th1)-math.Sin(4*th0))/32
	intSin4 := 3*dth/8 - (math.Sin(2*th1)-math.Sin(2*th0))/4 + (math.Sin(4*th1)-math.Sin(4*th0))/32

	// ∫u² dA = ⅓ ∮ u³ dv, dv = r cos θ dθ, u³ expanded about the center.
	ig.muu += r / 3 * (c.U*c.U*c.U*intCos + 3*c.U*c.U*r*intCos2 + 3*c.U*r*r*intCos3 + r*r*r*intCos4)

	// ∫v² dA = −⅓ ∮ v³ du, du = −r sin θ dθ, v³ expanded about the center.
	ig.mvv += r / 3 * (c.V*c.V*c.V*intSin + 3*c.V*c.V*r*intSin2 + 3*c.V*r*r*intSin3 + r*r*r*intSin4)

	// ∫uv dA = ½ ∮ u²v dv: u²v = c_v·u² + r sin θ·u², with
	//   ∫sin θ cos θ dθ = sin²θ/2, ∫sin θ cos²θ dθ = −cos³θ/3,
	//   ∫sin θ cos³θ dθ = −cos⁴θ/4.
	intSC := (sin1*sin1 - sin0*sin0) / 2
	intSC2 := (cos0*cos0*cos0 - cos1*cos1*cos1) / 3
	intSC3 := (cos0*cos0*cos0*cos0 - cos1*cos1*cos1*cos1) / 4
	ig.muv += 0.5 * r * (c.V*(c.U*c.U*intCos+2*c.U*r*intCos2+r*r*intCos3) +
		r*(c.U*c.U*intSC+2*c.U*r*intSC2+r*r*intSC3))
}

// lerp2 returns the point at parameter t on the segment start→end.
func lerp2(start, end Point2, t float64) (float64, float64) {
	return start.U + t*(end.U-start.U), start.V + t*(end.V-start.V)
}
