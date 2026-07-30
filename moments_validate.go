package decad

import (
	"context"
	"fmt"
	"math"
	"slices"
	"strings"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// validateMomentRecord normalizes and checks the fields the integrator reads,
// then asks sketch to decide whether those entities form the recorded region.
// decad does not carry a second planar-arrangement implementation.
func validateMomentRecord(record ProfileRecord) (ProfileRecord, Point2, error) {
	checked, anchor, err := validateMomentFields(record)
	if err != nil {
		return ProfileRecord{}, Point2{}, err
	}
	if circular, err := validateWholeCircleRegion(checked); circular {
		if err != nil {
			return ProfileRecord{}, Point2{}, err
		}
		return checked, anchor, nil
	}
	if !momentRecordMatchesSketch(checked) {
		validationRecord, err := scaleMomentRecordForValidation(checked, anchor)
		if err != nil {
			return ProfileRecord{}, Point2{}, err
		}
		if !momentRecordMatchesSketch(validationRecord) {
			return ProfileRecord{}, Point2{}, fmt.Errorf(
				`%w: the recorded segments do not form the stated closed region`,
				ErrDegenerate,
			)
		}
	}
	return checked, anchor, nil
}

func validateMomentFields(record ProfileRecord) (ProfileRecord, Point2, error) {
	return validateMomentFieldsBudget(nil, record)
}

func validateMomentFieldsBudget(budget *workBudget, record ProfileRecord) (ProfileRecord, Point2, error) {
	return validateMomentFieldsWithPoll(func() error { return wallBudgetStep(budget) }, record)
}

func validateMomentFieldsContext(ctx context.Context, record ProfileRecord) (ProfileRecord, Point2, error) {
	return validateMomentFieldsWithPoll(ctx.Err, record)
}

func validateMomentFieldsWithPoll(poll func() error, record ProfileRecord) (ProfileRecord, Point2, error) {
	loops := append([]LoopRecord{record.Outer}, record.Holes...)
	normalized := make([]LoopRecord, len(loops))
	var anchor Point2
	for loopIndex := range normalized {
		if poll != nil {
			if err := poll(); err != nil {
				return ProfileRecord{}, Point2{}, err
			}
		}
		loop := record.Outer
		if loopIndex > 0 {
			loop = record.Holes[loopIndex-1]
		}
		if len(loop.Segments) == 0 {
			return ProfileRecord{}, Point2{}, fmt.Errorf(
				`decad: profile loop %d is invalid: %w: a recorded loop holds no segments`,
				loopIndex,
				ErrDegenerate,
			)
		}
		normalized[loopIndex].Segments = make([]CurveSegment, len(loop.Segments))
		for segmentIndex, segment := range loop.Segments {
			if poll != nil {
				if err := poll(); err != nil {
					return ProfileRecord{}, Point2{}, err
				}
			}
			checked, start, err := validateMomentSegment(segment)
			if err != nil {
				return ProfileRecord{}, Point2{}, fmt.Errorf(
					`decad: profile loop %d segment %d is invalid: %w`,
					loopIndex,
					segmentIndex,
					err,
				)
			}
			normalized[loopIndex].Segments[segmentIndex] = checked
			if loopIndex == 0 && segmentIndex == 0 {
				anchor = start
			}
		}
	}
	checked := ProfileRecord{Outer: normalized[0], Holes: normalized[1:]}
	return checked, anchor, nil
}

func scaleMomentRecordForValidation(record ProfileRecord, anchor Point2) (ProfileRecord, error) {
	scale := 0.0
	grow := func(point Point2) {
		scale = math.Max(scale, math.Abs(point.U-anchor.U))
		scale = math.Max(scale, math.Abs(point.V-anchor.V))
	}
	for _, loop := range append([]LoopRecord{record.Outer}, record.Holes...) {
		for _, segment := range loop.Segments {
			switch segment := segment.(type) {
			case LineSeg:
				grow(segment.Start)
				grow(segment.End)
			case CircleSeg:
				grow(segment.Center)
				radius, _ := segment.Radius.In(units.Millimeter)
				scale = math.Max(scale, radius)
			case ArcSeg:
				grow(segment.Center)
				grow(segment.Start)
				grow(segment.End)
			case SplineSeg:
				growAll(segment.Control, grow)
			case ClosedSplineSeg:
				growAll(segment.Control, grow)
			case NURBSSeg:
				growAll(segment.Control, grow)
			}
		}
	}
	if scale == 0 {
		return ProfileRecord{}, fmt.Errorf(`%w: a recorded region has no geometric extent`, ErrDegenerate)
	}
	if !finiteMomentValues(scale) {
		return ProfileRecord{}, fmt.Errorf(`%w: the recorded region's extent is not finite`, ErrNotFinite)
	}

	transform := func(point Point2) Point2 {
		return Point2{U: (point.U - anchor.U) / scale, V: (point.V - anchor.V) / scale}
	}
	loops := append([]LoopRecord{record.Outer}, record.Holes...)
	scaled := make([]LoopRecord, len(loops))
	for loopIndex, loop := range loops {
		scaled[loopIndex].Segments = make([]CurveSegment, len(loop.Segments))
		for segmentIndex, segment := range loop.Segments {
			switch segment := segment.(type) {
			case LineSeg:
				segment.Start = transform(segment.Start)
				segment.End = transform(segment.End)
				scaled[loopIndex].Segments[segmentIndex] = segment
			case CircleSeg:
				radius, _ := segment.Radius.In(units.Millimeter)
				segment.Center = transform(segment.Center)
				segment.Radius = units.Millimeters(radius / scale)
				scaled[loopIndex].Segments[segmentIndex] = segment
			case ArcSeg:
				segment.Center = transform(segment.Center)
				segment.Start = transform(segment.Start)
				segment.End = transform(segment.End)
				scaled[loopIndex].Segments[segmentIndex] = segment
			case SplineSeg:
				// A B-spline is affine invariant, so transforming its control
				// points transforms the curve — the rescaled record states the
				// same shape at unit scale.
				segment.Control = shiftPoints(segment.Control, transform)
				scaled[loopIndex].Segments[segmentIndex] = segment
			case ClosedSplineSeg:
				segment.Control = shiftPoints(segment.Control, transform)
				scaled[loopIndex].Segments[segmentIndex] = segment
			case NURBSSeg:
				segment.Control = shiftPoints(segment.Control, transform)
				scaled[loopIndex].Segments[segmentIndex] = segment
			}
		}
	}
	return ProfileRecord{Outer: scaled[0], Holes: scaled[1:]}, nil
}

func growAll(points []Point2, grow func(Point2)) {
	for _, point := range points {
		grow(point)
	}
}

// validateMomentSegment normalizes one recorded segment and returns it beside
// the walk's own start point — the anchor the integrator re-references its
// moments to. A free-form segment resolves that point from its converted Bézier
// chain rather than through walkOf, which still refuses free-form kinds: the
// walk vocabulary carries no free-form state yet, and a free-form walk that
// merely read as non-circular would be built as a straight line.
func validateMomentSegment(segment CurveSegment) (CurveSegment, Point2, error) {
	segment, err := normalizeSegment(segment)
	if err != nil {
		return nil, Point2{}, err
	}
	if segment == nil {
		return nil, Point2{}, errNilSegment
	}
	if isFreeformSegment(segment) {
		return validateFreeformMomentSegment(segment)
	}

	switch segment := segment.(type) {
	case LineSeg:
		if !finiteMomentValues(
			segment.Start.U,
			segment.Start.V,
			segment.End.U,
			segment.End.V,
			segment.TStart,
			segment.TEnd,
		) {
			return nil, Point2{}, fmt.Errorf(`%w: a line segment field is not finite`, ErrNotFinite)
		}
		if err := validateMomentRange(segment.TStart, segment.TEnd); err != nil {
			return nil, Point2{}, err
		}
	case CircleSeg:
		radius, err := magnitudeIn(segment.Radius, units.Length, units.Millimeter, "a circle segment's radius")
		if err != nil {
			return nil, Point2{}, err
		}
		if !finiteMomentValues(segment.Center.U, segment.Center.V, segment.TStart, segment.TEnd) {
			return nil, Point2{}, fmt.Errorf(`%w: a circle segment field is not finite`, ErrNotFinite)
		}
		if err := validateMomentRange(segment.TStart, segment.TEnd); err != nil {
			return nil, Point2{}, err
		}
		if segment.CCW != (segment.TStart < segment.TEnd) {
			return nil, Point2{}, fmt.Errorf(`%w: a circle segment's CCW flag contradicts its range order`, ErrDegenerate)
		}
		segment.Radius = units.Millimeters(radius)
		return validateMomentWalk(segment)
	case ArcSeg:
		if !finiteMomentValues(
			segment.Center.U,
			segment.Center.V,
			segment.Start.U,
			segment.Start.V,
			segment.End.U,
			segment.End.V,
			segment.TStart,
			segment.TEnd,
		) {
			return nil, Point2{}, fmt.Errorf(`%w: an arc segment field is not finite`, ErrNotFinite)
		}
		if err := validateMomentRange(segment.TStart, segment.TEnd); err != nil {
			return nil, Point2{}, err
		}
		startRadius := math.Hypot(segment.Start.U-segment.Center.U, segment.Start.V-segment.Center.V)
		endRadius := math.Hypot(segment.End.U-segment.Center.U, segment.End.V-segment.Center.V)
		if !finiteMomentValues(startRadius, endRadius) {
			return nil, Point2{}, fmt.Errorf(`%w: an arc segment's derived radius is not finite`, ErrNotFinite)
		}
		if !momentCoordinateJoins(startRadius, endRadius) {
			return nil, Point2{}, fmt.Errorf(
				`%w: an arc segment's pinned start and end radii differ (%g and %g)`,
				ErrDegenerate,
				startRadius,
				endRadius,
			)
		}
	default:
		return nil, Point2{}, fmt.Errorf(
			`%w: this evaluator computes mass properties over line, arc, circle and Tier A free-form profile segments only; the profile has a %T segment`,
			ErrUnsupported,
			segment,
		)
	}
	return validateMomentWalk(segment)
}

// validateFreeformMomentSegment checks the fields the exact integrator reads on
// a Tier A free-form segment and returns the walk's own start point. The
// conversion itself is the check: it rejects a non-finite control coordinate, a
// trimmed range, a rational NURBS and every non-Tier-A kind with its own reason
// (docs/spline-design.md Table R), so no field test is duplicated here.
//
// The integration that segment will later run is charged HERE too, on the same
// counter, so an over-budget record refuses at validation. Everything between
// this step and the moments pass — momentRecordMatchesSketch reconstructing the
// entity in a private sketch and SAMPLING it a multiple of the control count —
// is work proportional to the record's own size, and the R7 ceiling exists
// because the public ProfileRecord methods take no context and so cannot be
// cancelled. A ceiling consulted only downstream fires after the work it is
// there to bound has already run.
func validateFreeformMomentSegment(segment CurveSegment) (CurveSegment, Point2, error) {
	work := &freeformWork{}
	spans, reversed, err := freeformBezierSpans(segment, work)
	if err != nil {
		return nil, Point2{}, err
	}
	if err := chargeFreeformSpans(spans, work); err != nil {
		return nil, Point2{}, err
	}
	start, _, err := freeformEndpoints(spans, reversed)
	if err != nil {
		return nil, Point2{}, err
	}
	if !finiteMomentValues(start.U, start.V) {
		return nil, Point2{}, fmt.Errorf(`%w: a free-form segment's start point is not finite`, ErrNotFinite)
	}
	if freeformDegenerate(spans) {
		return nil, Point2{}, fmt.Errorf(`%w: a free-form segment whose control points all coincide contributes no boundary`, ErrDegenerate)
	}
	return segment, start, nil
}

// freeformDegenerate reports whether every control point of the converted chain
// is the same coordinate. The convex hull property makes that the exact
// condition for the curve to be a single point, so the test is a proof rather
// than a tolerance.
func freeformDegenerate(spans []bezierSpan) bool {
	if len(spans) == 0 || len(spans[0]) == 0 {
		return true
	}
	first := spans[0][0]
	for _, span := range spans {
		for _, point := range span {
			if point.u.Cmp(first.u) != 0 || point.v.Cmp(first.v) != 0 {
				return false
			}
		}
	}
	return true
}

func validateMomentWalk(segment CurveSegment) (CurveSegment, Point2, error) {
	walk, err := walkOf(segment)
	if err != nil {
		return nil, Point2{}, err
	}
	if !finiteMomentValues(
		walk.startU,
		walk.startV,
		walk.endU,
		walk.endV,
		walk.tanInU,
		walk.tanInV,
		walk.tanOutU,
		walk.tanOutV,
		walk.length,
		walk.cU,
		walk.cV,
		walk.radius,
		walk.th0,
		walk.th1,
	) {
		return nil, Point2{}, fmt.Errorf(`%w: a segment's derived walk is not finite`, ErrNotFinite)
	}
	if walk.length <= 0 {
		return nil, Point2{}, fmt.Errorf(`%w: a zero-length segment contributes no boundary`, ErrDegenerate)
	}
	return segment, Point2{U: walk.startU, V: walk.startV}, nil
}

func validateMomentRange(start, end float64) error {
	if start < 0 || start > 1 || end < 0 || end > 1 {
		return fmt.Errorf(`%w: a segment range must stay within [0, 1]`, ErrDegenerate)
	}
	if start == end {
		return fmt.Errorf(`%w: a zero-length segment range contributes no boundary`, ErrDegenerate)
	}
	return nil
}

func momentCoordinateJoins(a, b float64) bool {
	scale := math.Max(math.Abs(a), math.Abs(b))
	if scale == 0 {
		return true
	}
	ulp := scale - math.Nextafter(scale, 0)
	return math.Abs(a-b) <= 1024*ulp
}

// validateWholeCircleRegion keeps exact whole-circle regions independent of
// sketch's proximity threshold. The topology is only containment of the outer
// disk and pairwise separation of the hole disks, so no arrangement machinery
// is needed.
func validateWholeCircleRegion(record ProfileRecord) (bool, error) {
	loops := append([]LoopRecord{record.Outer}, record.Holes...)
	circles := make([]CircleSeg, len(loops))
	for loopIndex, loop := range loops {
		if len(loop.Segments) != 1 {
			return false, nil
		}
		circle, ok := loop.Segments[0].(CircleSeg)
		if !ok {
			return false, nil
		}
		if circle.TStart != 0 || circle.TEnd != 1 {
			if circle.TStart != 1 || circle.TEnd != 0 {
				// Partial circle fragments are handled by the normal topology
				// reconstruction and integration path below.
				return false, nil
			}
		}
		wantCCW := loopIndex == 0
		if circle.CCW != wantCCW {
			return true, fmt.Errorf(`%w: profile loop %d has the wrong winding`, ErrDegenerate, loopIndex)
		}
		circles[loopIndex] = circle
	}

	outerRadius, _ := circles[0].Radius.In(units.Millimeter)
	for holeIndex, hole := range circles[1:] {
		holeRadius, _ := hole.Radius.In(units.Millimeter)
		distance := math.Hypot(hole.Center.U-circles[0].Center.U, hole.Center.V-circles[0].Center.V)
		if !finiteMomentValues(distance) {
			return true, fmt.Errorf(`%w: a circle separation is not finite`, ErrNotFinite)
		}
		if holeRadius >= outerRadius || distance >= outerRadius-holeRadius {
			return true, fmt.Errorf(`%w: profile hole %d is not contained by the outer circle`, ErrDegenerate, holeIndex)
		}
	}
	for a := 1; a < len(circles); a++ {
		radiusA, _ := circles[a].Radius.In(units.Millimeter)
		for b := a + 1; b < len(circles); b++ {
			radiusB, _ := circles[b].Radius.In(units.Millimeter)
			minimum := radiusA + radiusB
			distance := math.Hypot(
				circles[a].Center.U-circles[b].Center.U,
				circles[a].Center.V-circles[b].Center.V,
			)
			if !finiteMomentValues(minimum, distance) {
				return true, fmt.Errorf(`%w: a circle separation is not finite`, ErrNotFinite)
			}
			if distance <= minimum {
				return true, fmt.Errorf(`%w: profile holes overlap, touch or nest`, ErrDegenerate)
			}
		}
	}
	return true, nil
}

type momentEntityKey struct {
	kind   uint8
	first  Point2
	second Point2
	third  Point2
	radius float64
	// control identifies a free-form entity by its own defining data, which
	// no fixed number of Point2 fields can hold.
	control string
}

// freeformEntityKey renders a free-form segment's defining data as a key. It
// keys on the entity's OWN fields — control points, and a NURBS's degree, knots
// and weights — so two recorded segments dedupe exactly when they name the same
// entity.
func freeformEntityKey(kind uint8, points []Point2, extra ...float64) momentEntityKey {
	var b strings.Builder
	for _, point := range points {
		fmt.Fprintf(&b, "%v,%v;", point.U, point.V)
	}
	b.WriteByte('|')
	for _, value := range extra {
		fmt.Fprintf(&b, "%v;", value)
	}
	return momentEntityKey{kind: kind, control: b.String()}
}

func momentRecordMatchesSketch(record ProfileRecord) bool {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		return false
	}

	interned := make(map[Point2]*sketch.Point)
	point := func(value Point2) *sketch.Point {
		if existing, ok := interned[value]; ok {
			return existing
		}
		created := s.CreatePoint(value.U, value.V)
		interned[value] = created
		return created
	}
	points := func(values []Point2) []*sketch.Point {
		out := make([]*sketch.Point, len(values))
		for i, value := range values {
			out[i] = point(value)
		}
		return out
	}
	entities := make(map[momentEntityKey]struct{})
	for _, loop := range append([]LoopRecord{record.Outer}, record.Holes...) {
		for _, segment := range loop.Segments {
			switch segment := segment.(type) {
			case LineSeg:
				key := momentEntityKey{kind: 1, first: segment.Start, second: segment.End}
				if _, ok := entities[key]; !ok {
					s.CreateLine(point(segment.Start), point(segment.End))
					entities[key] = struct{}{}
				}
			case CircleSeg:
				radius, _ := segment.Radius.In(units.Millimeter)
				key := momentEntityKey{kind: 2, first: segment.Center, radius: radius}
				if _, ok := entities[key]; !ok {
					s.CreateCircle(point(segment.Center), radius)
					entities[key] = struct{}{}
				}
			case ArcSeg:
				key := momentEntityKey{
					kind:   3,
					first:  segment.Center,
					second: segment.Start,
					third:  segment.End,
				}
				if _, ok := entities[key]; !ok {
					s.CreateArc(point(segment.Center), point(segment.Start), point(segment.End))
					entities[key] = struct{}{}
				}
			case SplineSeg:
				key := freeformEntityKey(4, segment.Control)
				if _, ok := entities[key]; !ok {
					if _, err := s.CreateSpline(points(segment.Control)...); err != nil {
						return false
					}
					entities[key] = struct{}{}
				}
			case ClosedSplineSeg:
				key := freeformEntityKey(5, segment.Control)
				if _, ok := entities[key]; !ok {
					if _, err := s.CreateClosedSpline(points(segment.Control)...); err != nil {
						return false
					}
					entities[key] = struct{}{}
				}
			case NURBSSeg:
				extra := append([]float64{float64(segment.Degree)}, segment.Knots...)
				extra = append(extra, segment.Weights...)
				key := freeformEntityKey(6, segment.Control, extra...)
				if _, ok := entities[key]; !ok {
					if _, err := s.CreateNURBS(segment.Degree, points(segment.Control), segment.Weights, segment.Knots); err != nil {
						return false
					}
					entities[key] = struct{}{}
				}
			default:
				return false
			}
		}
	}

	for _, profile := range s.Profiles() {
		if !profile.Valid {
			continue
		}
		candidate, _, err := RecordProfile(s, profile)
		if err == nil && momentRecordsEqual(record, candidate) {
			return true
		}
	}
	return false
}

func momentRecordsEqual(a, b ProfileRecord) bool {
	if !momentLoopsEqual(a.Outer, b.Outer) || len(a.Holes) != len(b.Holes) {
		return false
	}
	matched := make([]bool, len(b.Holes))
	for _, holeA := range a.Holes {
		found := false
		for holeIndex, holeB := range b.Holes {
			if !matched[holeIndex] && momentLoopsEqual(holeA, holeB) {
				matched[holeIndex] = true
				found = true
				break
			}
		}
		if !found {
			return false
		}
	}
	return true
}

func momentLoopsEqual(a, b LoopRecord) bool {
	if len(a.Segments) != len(b.Segments) {
		return false
	}
	for offset := range b.Segments {
		equal := true
		for segmentIndex, segmentA := range a.Segments {
			if !momentSegmentsEqual(segmentA, b.Segments[(segmentIndex+offset)%len(b.Segments)]) {
				equal = false
				break
			}
		}
		if equal {
			return true
		}
	}
	return false
}

func momentSegmentsEqual(a, b CurveSegment) bool {
	switch a := a.(type) {
	case LineSeg:
		b, ok := b.(LineSeg)
		return ok && a == b
	case CircleSeg:
		b, ok := b.(CircleSeg)
		if !ok {
			return false
		}
		radiusA, _ := a.Radius.In(units.Millimeter)
		radiusB, _ := b.Radius.In(units.Millimeter)
		return a.Center == b.Center &&
			radiusA == radiusB &&
			a.CCW == b.CCW &&
			a.TStart == b.TStart &&
			a.TEnd == b.TEnd
	case ArcSeg:
		b, ok := b.(ArcSeg)
		return ok && a == b
	case SplineSeg:
		b, ok := b.(SplineSeg)
		return ok && slices.Equal(a.Control, b.Control) && a.TStart == b.TStart && a.TEnd == b.TEnd
	case ClosedSplineSeg:
		b, ok := b.(ClosedSplineSeg)
		return ok && slices.Equal(a.Control, b.Control) &&
			a.CCW == b.CCW && a.TStart == b.TStart && a.TEnd == b.TEnd
	case NURBSSeg:
		b, ok := b.(NURBSSeg)
		return ok && a.Degree == b.Degree &&
			slices.Equal(a.Control, b.Control) &&
			slices.Equal(a.Knots, b.Knots) &&
			slices.Equal(a.Weights, b.Weights) &&
			a.TStart == b.TStart && a.TEnd == b.TEnd
	default:
		return false
	}
}
