package sectionaudit

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

func shiftPoint(point Point2) Point2 {
	return Point2{U: point.U - 0, V: point.V - 0}
}

func lerp2(start, end Point2, t float64) (float64, float64) {
	switch t {
	case 0:
		return start.U, start.V
	case 1:
		return end.U, end.V
	}
	return start.U + t*(end.U-start.U), start.V + t*(end.V-start.V)
}

// LoopSignedArea is one loop's signed area (positive counter-clockwise): the
// Green's-theorem boundary integral of its own segments.
func LoopSignedArea(budget *proofbound.WorkBudget, loop LoopRecord) (float64, error) {
	var area float64
	for _, seg := range loop.Segments {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, err
		}
		term, err := analyticSignedArea(seg)
		if err != nil {
			return 0, err
		}
		area += term
	}
	return area, nil
}

// analyticSignedArea evaluates only the area term of the same Green's-theorem
// integral used by regionIntegrals.addAnalytic. Keep its float operations in
// the same order: S8 compares the held signed areas, including near zero.
func analyticSignedArea(segment CurveSegment) (float64, error) {
	segment, err := sectionrecord.NormalizeSegment(segment)
	if err != nil {
		return 0, err
	}
	switch segment := segment.(type) {
	case LineSeg:
		start := shiftPoint(segment.Start)
		end := shiftPoint(segment.End)
		u0, v0 := lerp2(start, end, segment.TStart)
		u1, v1 := lerp2(start, end, segment.TEnd)
		return 0.5 * (u0*v1 - u1*v0), nil
	case CircleSeg:
		if segment.Radius.Kind() != units.Length {
			return 0, fmt.Errorf(`%w: a circle segment's radius must be a %s, got %s`,
				ErrUnitKind, units.Length, segment.Radius.Kind())
		}
		radius, err := segment.Radius.In(units.Millimeter)
		if err != nil {
			return 0, fmt.Errorf(`%w: a circle segment's radius is not representable: %s`, ErrNotFinite, err)
		}
		if segment.CCW != (segment.TStart < segment.TEnd) {
			return 0, fmt.Errorf(`%w: a circle segment's CCW flag contradicts its range order`, ErrDegenerate)
		}
		return circularSignedArea(shiftPoint(segment.Center), radius,
			2*math.Pi*segment.TStart, 2*math.Pi*segment.TEnd), nil
	case ArcSeg:
		center := shiftPoint(segment.Center)
		start := shiftPoint(segment.Start)
		end := shiftPoint(segment.End)
		radius := math.Hypot(start.U-center.U, start.V-center.V)
		a0 := math.Atan2(start.V-center.V, start.U-center.U)
		a1 := math.Atan2(end.V-center.V, end.U-center.U)
		sweep := math.Mod(a1-a0, 2*math.Pi)
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		return circularSignedArea(center, radius,
			a0+segment.TStart*sweep, a0+segment.TEnd*sweep), nil
	default:
		return 0, fmt.Errorf(`%w: this evaluator computes mass properties over line, arc, circle and Tier A free-form profile segments only; the profile has a %T segment`,
			ErrUnsupported, segment)
	}
}

func circularSignedArea(center Point2, radius, start, end float64) float64 {
	sin0, cos0 := math.Sincos(start)
	sin1, cos1 := math.Sincos(end)
	delta := end - start
	return 0.5 * (radius*radius*delta + center.U*radius*(sin1-sin0) - center.V*radius*(cos1-cos0))
}
