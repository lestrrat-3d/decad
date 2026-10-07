package momentinput

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/units"
)

type (
	Point2          = sectionrecord.Point2
	LoopRecord      = sectionrecord.LoopRecord
	CurveSegment    = sectionrecord.CurveSegment
	LineSeg         = sectionrecord.LineSeg
	CircleSeg       = sectionrecord.CircleSeg
	ArcSeg          = sectionrecord.ArcSeg
	SplineSeg       = sectionrecord.SplineSeg
	ClosedSplineSeg = sectionrecord.ClosedSplineSeg
	NURBSSeg        = sectionrecord.NURBSSeg
	FitSplineSeg    = sectionrecord.FitSplineSeg
)

// Profile carries the recorded loops without root package methods.
type Profile struct {
	Outer LoopRecord
	Holes []LoopRecord
}

// Plan carries one converted free-form segment into the moments pass.
type Plan struct {
	Spans    []freeform.BezierSpan
	Reversed bool
}

var (
	ErrDegenerate  = decaderr.ErrDegenerate
	ErrNotFinite   = decaderr.ErrNotFinite
	ErrUnsupported = decaderr.ErrUnsupported
)

func ScaleForValidation(profile Profile, anchor Point2) (Profile, error) {
	return scaleMomentRecordForValidation(profile, anchor)
}
func ValidateSegment(segment CurveSegment, work *freeform.FreeformWork) (CurveSegment, Point2, Plan, error) {
	return validateMomentSegment(segment, work)
}
func ValidateFreeformSegment(segment CurveSegment, work *freeform.FreeformWork) (CurveSegment, Point2, Plan, error) {
	return validateFreeformMomentSegment(segment, work)
}
func ValidateWholeCircleRegion(profile Profile) (bool, error) {
	return validateWholeCircleRegion(profile)
}
func scaleMomentRecordForValidation(record Profile, anchor Point2) (Profile, error) {
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
			case FitSplineSeg:
				growAll(segment.Fit, grow)
			}
		}
	}
	if scale == 0 {
		return Profile{}, fmt.Errorf(`%w: a recorded region has no geometric extent`, ErrDegenerate)
	}
	if !freeform.FiniteMomentValues(scale) {
		return Profile{}, fmt.Errorf(`%w: the recorded region's extent is not finite`, ErrNotFinite)
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
			case FitSplineSeg:
				// A chord-length natural cubic is equivariant under a similarity: this
				// transform is a UNIFORM scale plus a translation, so every chord
				// scales by the same factor, the parameterization is the same up to
				// that factor, and the interpolant is the image of the original curve.
				//
				// One caveat, a consequence of the 1e-12 fit-point dedup threshold
				// (spline_fit.go) being ABSOLUTE: a rescale can collapse a fit-point
				// pair the original record kept, or keep a pair it collapsed. That is
				// confined to THIS falsifier — decad always integrates the ORIGINAL
				// record's interpolant, never the rescaled one — so the worst it can
				// do is weaken this check into a no-match refusal, or into one that
				// falsifies less than it might have. It can never move a published
				// number.
				segment.Fit = shiftPoints(segment.Fit, transform)
				scaled[loopIndex].Segments[segmentIndex] = segment
			}
		}
	}
	return Profile{Outer: scaled[0], Holes: scaled[1:]}, nil
}

func growAll(points []Point2, grow func(Point2)) {
	for _, point := range points {
		grow(point)
	}
}

// validateMomentSegment normalizes one recorded segment and returns it beside
// the walk's own start point — the anchor the integrator re-references its
// moments to. A free-form segment resolves that point from its converted Bézier
// chain rather than through walkOf: the moments path needs that point before
// any tier is decided, ahead of where walkOf's own free-form arm would even
// run (validateFreeformMomentSegment), so it is read directly from the same
// conversion the build's own survey2d.WalkKind == survey2d.WalkFreeform arm reads
// (extrude.go's buildLoopSidesAs).
func validateMomentSegment(segment CurveSegment, work *freeform.FreeformWork) (CurveSegment, Point2, Plan, error) {
	segment, err := sectionrecord.NormalizeSegment(segment)
	if err != nil {
		return nil, Point2{}, Plan{}, err
	}
	if segment == nil {
		return nil, Point2{}, Plan{}, sectionrecord.ErrNilSegment
	}
	if splinebezier.IsFreeformSegment(segment) {
		return validateFreeformMomentSegment(segment, work)
	}
	checked, start, err := validateAnalyticMomentSegment(segment, work)
	return checked, start, Plan{}, err
}

// validateAnalyticMomentSegment normalizes and checks one line, circle or arc
// segment — the kinds integrated from their own closed forms, with no
// conversion and so no charge against the record's work counter.
func validateAnalyticMomentSegment(segment CurveSegment, work *freeform.FreeformWork) (CurveSegment, Point2, error) {
	switch segment := segment.(type) {
	case LineSeg:
		if !freeform.FiniteMomentValues(
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
		radius, err := sectionrecord.MagnitudeIn(segment.Radius, units.Length, units.Millimeter, "a circle segment's radius")
		if err != nil {
			return nil, Point2{}, err
		}
		if !freeform.FiniteMomentValues(segment.Center.U, segment.Center.V, segment.TStart, segment.TEnd) {
			return nil, Point2{}, fmt.Errorf(`%w: a circle segment field is not finite`, ErrNotFinite)
		}
		if err := validateMomentRange(segment.TStart, segment.TEnd); err != nil {
			return nil, Point2{}, err
		}
		if segment.CCW != (segment.TStart < segment.TEnd) {
			return nil, Point2{}, fmt.Errorf(`%w: a circle segment's CCW flag contradicts its range order`, ErrDegenerate)
		}
		segment.Radius = units.Millimeters(radius)
		return validateMomentWalk(segment, work)
	case ArcSeg:
		if !freeform.FiniteMomentValues(
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
		if !freeform.FiniteMomentValues(startRadius, endRadius) {
			return nil, Point2{}, fmt.Errorf(`%w: an arc segment's derived radius is not finite`, ErrNotFinite)
		}
		if !arcPinnedRadiiJoin(segment, startRadius, endRadius) {
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
	return validateMomentWalk(segment, work)
}

// validateFreeformMomentSegment checks the fields the exact integrator reads on
// a Tier A free-form segment and returns the walk's own start point beside the
// converted chain the moments pass will integrate. The conversion itself is the
// check: it rejects a non-finite range or control coordinate, a trimmed range, a
// rational NURBS and every non-Tier-A kind with its own reason
// (docs/spline-design.md Table R), so no field test is duplicated here.
//
// Every charge this SEGMENT owes is levied here, on the record's own counter:
// the conversion, the re-anchoring the moments pass performs, and the exact
// integration it then runs. The R7 ceiling exists because the public
// Profile methods take no context and so cannot be cancelled, and a
// ceiling consulted at the point of use fires after the work it is there to
// bound has already run. The sketch RECONSTRUCTION is charged separately, by
// the record-level preflight above: it arranges the whole scene at once, so its
// cost is a property of the record rather than of any segment in it.
func validateFreeformMomentSegment(segment CurveSegment, work *freeform.FreeformWork) (CurveSegment, Point2, Plan, error) {
	spans, reversed, err := splinebezier.FreeformBezierSpans(segment, work)
	if err != nil {
		return nil, Point2{}, Plan{}, err
	}
	if err := freeform.ChargeFreeformShift(spans, work); err != nil {
		return nil, Point2{}, Plan{}, err
	}
	if err := freeform.ChargeFreeformSpans(spans, work); err != nil {
		return nil, Point2{}, Plan{}, err
	}
	start, end, err := splinebezier.FreeformEndpoints(spans, reversed)
	if err != nil {
		return nil, Point2{}, Plan{}, err
	}
	if !freeform.FiniteMomentValues(start.U, start.V) {
		return nil, Point2{}, Plan{}, fmt.Errorf(`%w: a free-form segment's start point is not finite`, ErrNotFinite)
	}
	if freeformDegenerate(spans) {
		return nil, Point2{}, Plan{}, fmt.Errorf(`%w: a free-form segment whose control points all coincide contributes no boundary`, ErrDegenerate)
	}
	if err := requireFitSplineTerminalJoins(segment, start, end, reversed); err != nil {
		return nil, Point2{}, Plan{}, err
	}
	return segment, start, Plan{Spans: spans, Reversed: reversed}, nil
}

// requireFitSplineTerminalJoins falsifies a FitSplineSeg whose converted chain
// does not actually reach the fit point its own record names as the walk's
// natural-end coordinate — the point a following segment's Start (or, for a
// loop's own last segment, the loop's first segment's Start) is recorded
// against. sketch's fit-point dedup (fitChordEps, geom/fitspline.go) collapses
// a run of fit points closer than 1e-12, keeping only the FIRST of each run,
// so a terminal fit point sitting that close to its predecessor is dropped and
// geom.NewFitInterpolant's own chain ends one point short of what the record
// still claims — while the record's own natural-START point (always the first
// of its own run) can never be dropped this way, so only the natural-end side
// needs checking. This is the one Tier A kind that can drift here at all
// (docs/spline-design.md Table F): a clamped SplineSeg/ClosedSplineSeg/
// NURBSSeg interpolates its own recorded end control point exactly, with no
// sketch-side collapsing on the way, so a general cross-kind join check would
// only add unearned cost for those three.
//
// sketch's own reconstruction (momentRecordMatchesSketch) cannot catch this on
// its own: it rebuilds the SAME entity from the SAME recorded fit points, so a
// collapsed terminal point is dropped identically on both sides and the round
// trip matches regardless.
func requireFitSplineTerminalJoins(segment CurveSegment, start, end Point2, reversed bool) error {
	fit, ok := segment.(FitSplineSeg)
	if !ok {
		return nil
	}
	// The walk's natural-end coordinate is the recorded chain's own LAST fit
	// point; freeformEndpoints already swapped start/end into walk order for a
	// reversed range, so the natural-end side is start there instead of end.
	want := fit.Fit[len(fit.Fit)-1]
	got := end
	if reversed {
		got = start
	}
	if got == want {
		return nil
	}
	return fmt.Errorf(
		`%w: the converted fit-spline curve's own boundary reaches (%v, %v), not the recorded terminal fit point (%v, %v) — sketch's fit-point dedup collapsed it`,
		ErrDegenerate,
		got.U,
		got.V,
		want.U,
		want.V,
	)
}

// freeformDegenerate reports whether every control point of the converted chain
// is the same coordinate. The convex hull property makes that the exact
// condition for the curve to be a single point, so the test is a proof rather
// than a tolerance.
//
// It is this path's half of Table R row R14. The length bracket refuses the
// same record on its own terms — a collapsed net is the one shape whose bracket
// has zero width (internal/freeform/spline_length.go) — so the two paths agree.
func freeformDegenerate(spans []freeform.BezierSpan) bool {
	if len(spans) == 0 || len(spans[0]) == 0 {
		return true
	}
	first := spans[0][0]
	for _, span := range spans {
		for _, point := range span {
			if point.U.Cmp(first.U) != 0 || point.V.Cmp(first.V) != 0 {
				return false
			}
		}
	}
	return true
}

func validateMomentWalk(segment CurveSegment, work *freeform.FreeformWork) (CurveSegment, Point2, error) {
	walk, err := boundarywalk.WalkOf(segment, work)
	if err != nil {
		return nil, Point2{}, err
	}
	if !freeform.FiniteMomentValues(
		walk.StartU,
		walk.StartV,
		walk.EndU,
		walk.EndV,
		walk.TanInU,
		walk.TanInV,
		walk.TanOutU,
		walk.TanOutV,
		walk.Length,
		walk.CU,
		walk.CV,
		walk.Radius,
		walk.Th0,
		walk.Th1,
	) {
		return nil, Point2{}, fmt.Errorf(`%w: a segment's derived walk is not finite`, ErrNotFinite)
	}
	if walk.Length <= 0 {
		return nil, Point2{}, fmt.Errorf(`%w: a zero-length segment contributes no boundary`, ErrDegenerate)
	}
	return segment, Point2{U: walk.StartU, V: walk.StartV}, nil
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

// arcPinnedRadiiJoin reports whether an ArcSeg's two pinned radii agree to
// within coordinate rounding: 1024 ulps of the largest magnitude the record
// states — its six coordinates and the two radii — never of the radius alone.
// A fillet's or shell's corner arc rounds its centre and feet at the
// coordinate's own scale, so its radii differ by ulps of the coordinate even
// when the radius is a thousand times smaller. The check is reject-only: a
// larger difference disproves the record as an arc (ErrDegenerate); a smaller
// one proves nothing and is CHARGED by every circular bracket through
// arcEndRadialRatio (moments_circular.go), never trusted.
func arcPinnedRadiiJoin(seg ArcSeg, startRadius, endRadius float64) bool {
	scale := max(
		math.Abs(seg.Center.U), math.Abs(seg.Center.V),
		math.Abs(seg.Start.U), math.Abs(seg.Start.V),
		math.Abs(seg.End.U), math.Abs(seg.End.V),
		math.Abs(startRadius), math.Abs(endRadius),
	)
	if scale == 0 {
		return true
	}
	ulp := scale - math.Nextafter(scale, 0)
	return math.Abs(startRadius-endRadius) <= 1024*ulp
}

// validateWholeCircleRegion keeps exact whole-circle regions independent of
// sketch's proximity threshold. The topology is only containment of the outer
// disk and pairwise separation of the hole disks, so no arrangement machinery
// is needed.
func validateWholeCircleRegion(record Profile) (bool, error) {
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
		if !freeform.FiniteMomentValues(distance) {
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
			if !freeform.FiniteMomentValues(minimum, distance) {
				return true, fmt.Errorf(`%w: a circle separation is not finite`, ErrNotFinite)
			}
			if distance <= minimum {
				return true, fmt.Errorf(`%w: profile holes overlap, touch or nest`, ErrDegenerate)
			}
		}
	}
	return true, nil
}

func shiftPoints(points []Point2, shift func(Point2) Point2) []Point2 {
	out := make([]Point2, len(points))
	for i, point := range points {
		out[i] = shift(point)
	}
	return out
}
