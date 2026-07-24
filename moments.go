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
// A malformed or open record is [ErrDegenerate]. A circle radius of the wrong
// kind is [ErrUnitKind], a negative radius is [ErrNegativeMagnitude], and a
// non-finite field or arithmetic result is [ErrNotFinite]. No measurement is
// returned on error.
func (r ProfileRecord) Area() (Measurement, error) {
	ig, err := r.integralsTo(momentAreaOrder)
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
// Record validation and arithmetic errors match [ProfileRecord.Area].
func (r ProfileRecord) Centroid() (VecMeasurement, error) {
	ig, err := r.integralsTo(momentFirstOrder)
	if err != nil {
		return VecMeasurement{}, err
	}
	u, v := ig.mu/ig.area, ig.mv/ig.area
	if !finiteMomentValues(u, v) {
		return VecMeasurement{}, fmt.Errorf(`%w: the region centroid is not finite`, ErrNotFinite)
	}
	return VecMeasurement{
		Value:     r3.NewVec(u, v, 0),
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
// [ErrUnsupported], and malformed or non-finite records are rejected before a
// measurement is constructed.
func (r ProfileRecord) SecondMoments() (SecondMoments, error) {
	ig, err := r.integralsTo(momentSecondOrder)
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

type momentIntegralOrder uint8

const (
	momentAreaOrder momentIntegralOrder = iota
	momentFirstOrder
	momentSecondOrder
)

func finiteMomentValues(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}

func (ig regionIntegrals) isFinite(order momentIntegralOrder) bool {
	switch order {
	case momentAreaOrder:
		return finiteMomentValues(ig.area)
	case momentFirstOrder:
		return finiteMomentValues(ig.area, ig.mu, ig.mv)
	default:
		return finiteMomentValues(ig.area, ig.mu, ig.mv, ig.muu, ig.muv, ig.mvv)
	}
}

// integrals walks the outer loop and every hole in recorded walk order and
// sums each segment's closed-form contribution. Walk order carries the sign:
// the outer loop is counter-clockwise (positive), holes are clockwise
// (negative), so the sum IS the net region integral.
func (r ProfileRecord) integrals() (regionIntegrals, error) {
	record, anchor, err := validateMomentFields(r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecord(record, anchor, momentSecondOrder)
}

func (r ProfileRecord) integralsTo(order momentIntegralOrder) (regionIntegrals, error) {
	record, anchor, err := validateMomentRecord(r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecord(record, anchor, order)
}

// evaluatorIntegrals supplies only the mass properties an evaluator needs.
// Public measurement methods use integralsTo and retain full topology and
// finiteness checks; evaluator construction must not let unused higher-order
// overflow prevent clearance verification from running.
func (r ProfileRecord) evaluatorIntegrals(order momentIntegralOrder) (regionIntegrals, error) {
	record, anchor, err := validateMomentFields(r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecord(record, anchor, order)
}

func (r ProfileRecord) evaluatorIntegralsUnchecked(order momentIntegralOrder) (regionIntegrals, error) {
	record, anchor, err := validateMomentFields(r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecordUnchecked(record, anchor, order)
}

func integrateMomentRecord(record ProfileRecord, anchor Point2, order momentIntegralOrder) (regionIntegrals, error) {
	return integrateMomentRecordMode(record, anchor, order, true)
}

func integrateMomentRecordUnchecked(record ProfileRecord, anchor Point2, order momentIntegralOrder) (regionIntegrals, error) {
	return integrateMomentRecordMode(record, anchor, order, false)
}

func integrateMomentRecordMode(record ProfileRecord, anchor Point2, order momentIntegralOrder, checkFinite bool) (regionIntegrals, error) {
	var ig regionIntegrals
	for loopIndex, loop := range append([]LoopRecord{record.Outer}, record.Holes...) {
		for segmentIndex, segment := range loop.Segments {
			shifted, err := shiftMomentSegment(segment, anchor)
			if err != nil {
				return regionIntegrals{}, err
			}
			if err := ig.addFor(shifted, order); err != nil {
				return regionIntegrals{}, err
			}
			if checkFinite && !ig.isFinite(order) {
				return regionIntegrals{}, fmt.Errorf(`%w: mass-property integration overflowed at loop %d segment %d`, ErrNotFinite, loopIndex, segmentIndex)
			}
		}
	}
	if ig.area <= 0 {
		return regionIntegrals{}, fmt.Errorf(`%w: the recorded region encloses no positive net area`, ErrDegenerate)
	}
	ig = translateMomentIntegrals(ig, anchor, order)
	if checkFinite && !ig.isFinite(order) {
		return regionIntegrals{}, fmt.Errorf(`%w: mass-property integration overflowed while restoring the profile origin`, ErrNotFinite)
	}
	return ig, nil
}

func shiftMomentSegment(segment CurveSegment, anchor Point2) (CurveSegment, error) {
	segment, err := normalizeSegment(segment)
	if err != nil {
		return nil, err
	}
	shift := func(point Point2) Point2 {
		return Point2{U: point.U - anchor.U, V: point.V - anchor.V}
	}
	switch segment := segment.(type) {
	case LineSeg:
		segment.Start = shift(segment.Start)
		segment.End = shift(segment.End)
		return segment, nil
	case CircleSeg:
		segment.Center = shift(segment.Center)
		return segment, nil
	case ArcSeg:
		segment.Center = shift(segment.Center)
		segment.Start = shift(segment.Start)
		segment.End = shift(segment.End)
		return segment, nil
	default:
		return nil, fmt.Errorf(`%w: this evaluator computes mass properties over line, arc and circle profile segments only; the profile has a %T segment`, ErrUnsupported, segment)
	}
}

func translateMomentIntegrals(ig regionIntegrals, anchor Point2, order momentIntegralOrder) regionIntegrals {
	if order == momentAreaOrder {
		return ig
	}
	mu, mv := ig.mu, ig.mv
	if order == momentSecondOrder {
		ig.muu += 2*anchor.U*mu + anchor.U*anchor.U*ig.area
		ig.muv += anchor.V*mu + anchor.U*mv + anchor.U*anchor.V*ig.area
		ig.mvv += 2*anchor.V*mv + anchor.V*anchor.V*ig.area
	}
	ig.mu += anchor.U * ig.area
	ig.mv += anchor.V * ig.area
	return ig
}

// addFor accumulates one segment's boundary-integral contribution, in the
// segment's recorded walk direction. It stops at the highest moment the
// caller requested, so Area does not fail because an unused higher moment
// would overflow.
func (ig *regionIntegrals) add(segment CurveSegment) error {
	return ig.addFor(segment, momentSecondOrder)
}

func (ig *regionIntegrals) addFor(segment CurveSegment, order momentIntegralOrder) error {
	segment, err := normalizeSegment(segment)
	if err != nil {
		return err
	}
	switch segment := segment.(type) {
	case LineSeg:
		ig.addLine(segment, order)
		return nil
	case CircleSeg:
		radius, err := segment.Radius.In(units.Millimeter)
		if err != nil {
			return fmt.Errorf(`decad: a circle segment's radius is not a length: %w`, err)
		}
		if segment.CCW != (segment.TStart < segment.TEnd) {
			return fmt.Errorf(`%w: a circle segment's CCW flag contradicts its range order`, ErrDegenerate)
		}
		ig.addCircular(
			segment.Center,
			radius,
			2*math.Pi*segment.TStart,
			2*math.Pi*segment.TEnd,
			order,
		)
		return nil
	case ArcSeg:
		radius := math.Hypot(segment.Start.U-segment.Center.U, segment.Start.V-segment.Center.V)
		a0 := math.Atan2(segment.Start.V-segment.Center.V, segment.Start.U-segment.Center.U)
		a1 := math.Atan2(segment.End.V-segment.Center.V, segment.End.U-segment.Center.U)
		sweep := math.Mod(a1-a0, 2*math.Pi)
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		ig.addCircular(
			segment.Center,
			radius,
			a0+segment.TStart*sweep,
			a0+segment.TEnd*sweep,
			order,
		)
		return nil
	default:
		return fmt.Errorf(`%w: this evaluator computes mass properties over line, arc and circle profile segments only; the profile has a %T segment`, ErrUnsupported, segment)
	}
}

// addLine accumulates the straight chord from the walk's start point to its
// end point. The recorded range picks the walked piece of the entity's own
// Start→End parameterization.
func (ig *regionIntegrals) addLine(segment LineSeg, order momentIntegralOrder) {
	u0, v0 := lerp2(segment.Start, segment.End, segment.TStart)
	u1, v1 := lerp2(segment.Start, segment.End, segment.TEnd)
	ig.area += 0.5 * (u0*v1 - u1*v0)
	if order == momentAreaOrder {
		return
	}

	ig.mu += (v1 - v0) * (u0*u0 + u0*u1 + u1*u1) / 6
	ig.mv += -(u1 - u0) * (v0*v0 + v0*v1 + v1*v1) / 6
	if order == momentFirstOrder {
		return
	}

	ig.muu += (v1 - v0) * (u0*u0*u0 + u0*u0*u1 + u0*u1*u1 + u1*u1*u1) / 12
	ig.mvv += -(u1 - u0) * (v0*v0*v0 + v0*v0*v1 + v0*v1*v1 + v1*v1*v1) / 12

	du, dv := u1-u0, v1-v0
	intU2V := v0*(u0*u0+u0*du+du*du/3) + dv*(u0*u0/2+2*u0*du/3+du*du/4)
	ig.muv += 0.5 * dv * intU2V
}

// addCircular accumulates a circular path about center c with radius r, from
// angle th0 to th1 in the walk direction (th1 < th0 walks clockwise).
func (ig *regionIntegrals) addCircular(c Point2, r, th0, th1 float64, order momentIntegralOrder) {
	sin0, cos0 := math.Sincos(th0)
	sin1, cos1 := math.Sincos(th1)
	dth := th1 - th0

	ig.area += 0.5 * (r*r*dth + c.U*r*(sin1-sin0) - c.V*r*(cos1-cos0))
	if order == momentAreaOrder {
		return
	}

	intCos := sin1 - sin0
	intCos2 := dth/2 + (math.Sin(2*th1)-math.Sin(2*th0))/4
	intCos3 := (sin1 - sin1*sin1*sin1/3) - (sin0 - sin0*sin0*sin0/3)
	ig.mu += 0.5 * r * (c.U*c.U*intCos + 2*c.U*r*intCos2 + r*r*intCos3)

	intSin := cos0 - cos1
	intSin2 := dth/2 - (math.Sin(2*th1)-math.Sin(2*th0))/4
	intSin3 := (cos0 - cos0*cos0*cos0/3) - (cos1 - cos1*cos1*cos1/3)
	ig.mv += 0.5 * r * (c.V*c.V*intSin + 2*c.V*r*intSin2 + r*r*intSin3)
	if order == momentFirstOrder {
		return
	}

	intCos4 := 3*dth/8 + (math.Sin(2*th1)-math.Sin(2*th0))/4 + (math.Sin(4*th1)-math.Sin(4*th0))/32
	intSin4 := 3*dth/8 - (math.Sin(2*th1)-math.Sin(2*th0))/4 + (math.Sin(4*th1)-math.Sin(4*th0))/32

	ig.muu += r / 3 * (c.U*c.U*c.U*intCos + 3*c.U*c.U*r*intCos2 + 3*c.U*r*r*intCos3 + r*r*r*intCos4)
	ig.mvv += r / 3 * (c.V*c.V*c.V*intSin + 3*c.V*c.V*r*intSin2 + 3*c.V*r*r*intSin3 + r*r*r*intSin4)

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
