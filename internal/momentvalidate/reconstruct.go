package momentvalidate

import (
	"fmt"
	"math"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// ReconstructionOf counts the sketch chords charged to a recorded region.
func ReconstructionOf(profile Profile) freeform.FreeformReconstruction {
	return reconstructionOf(profile)
}

// ReconstructionChords counts the chords charged to one recorded segment.
func ReconstructionChords(segment CurveSegment) uint64 {
	return reconstructionChords(segment)
}

// ChargeReconstruction charges the whole-scene arrangements before sketch runs.
func ChargeReconstruction(profile Profile, work *freeform.FreeformWork) (uint64, error) {
	return chargeReconstruction(profile, work)
}

// RecordScene reconstructs the recorded entities in a sketch.
func RecordScene(profile Profile) (*sketch.Sketch, bool) {
	return momentRecordScene(profile)
}

// RecordsEqual compares the reconstructed profile with the recorded region.
func RecordsEqual(a, b Profile) bool {
	return momentRecordsEqual(a, b)
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

// analyticEntityKey is the interning key momentRecordScene builds an analytic
// segment's entity under, read off the segment's own defining data in constant
// time. Both that scene and reconstructionOf's chord count share it, so the
// charge cannot drift from the set of entities the arrangement actually holds.
//
// It reports false for every free-form kind. Those key on freeformEntityKey,
// which walks all the control points and allocates a string per segment — a pass
// the reconstruction charge must PRECEDE rather than run — so the chord count
// leaves them un-interned and counts each fragment for itself.
func analyticEntityKey(segment CurveSegment) (momentEntityKey, bool) {
	switch segment := segment.(type) {
	case LineSeg:
		return momentEntityKey{kind: 1, first: segment.Start, second: segment.End}, true
	case CircleSeg:
		radius, _ := segment.Radius.In(units.Millimeter)
		return momentEntityKey{kind: 2, first: segment.Center, radius: radius}, true
	case ArcSeg:
		return momentEntityKey{
			kind:   3,
			first:  segment.Center,
			second: segment.Start,
			third:  segment.End,
		}, true
	default:
		return momentEntityKey{}, false
	}
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

// momentRecordScene builds the sketch entities the record names, deduplicating
// the ones several segments share. It reports whether every entity was created:
// an entity sketch declines to build is a record that does not reconstruct, so
// the answer above is a no-match rather than a failure to report.
func momentRecordScene(record Profile) (*sketch.Sketch, bool) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		return nil, false
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
				key, _ := analyticEntityKey(segment)
				if _, ok := entities[key]; !ok {
					s.CreateLine(point(segment.Start), point(segment.End))
					entities[key] = struct{}{}
				}
			case CircleSeg:
				key, _ := analyticEntityKey(segment)
				if _, ok := entities[key]; !ok {
					s.CreateCircle(point(segment.Center), key.radius)
					entities[key] = struct{}{}
				}
			case ArcSeg:
				key, _ := analyticEntityKey(segment)
				if _, ok := entities[key]; !ok {
					s.CreateArc(point(segment.Center), point(segment.Start), point(segment.End))
					entities[key] = struct{}{}
				}
			case SplineSeg:
				key := freeformEntityKey(4, segment.Control)
				if _, ok := entities[key]; !ok {
					if _, err := s.CreateSpline(points(segment.Control)...); err != nil {
						return nil, false
					}
					entities[key] = struct{}{}
				}
			case ClosedSplineSeg:
				key := freeformEntityKey(5, segment.Control)
				if _, ok := entities[key]; !ok {
					if _, err := s.CreateClosedSpline(points(segment.Control)...); err != nil {
						return nil, false
					}
					entities[key] = struct{}{}
				}
			case NURBSSeg:
				extra := append([]float64{float64(segment.Degree)}, segment.Knots...)
				extra = append(extra, segment.Weights...)
				key := freeformEntityKey(6, segment.Control, extra...)
				if _, ok := entities[key]; !ok {
					if _, err := s.CreateNURBS(segment.Degree, points(segment.Control), segment.Weights, segment.Knots); err != nil {
						return nil, false
					}
					entities[key] = struct{}{}
				}
			case FitSplineSeg:
				key := freeformEntityKey(7, segment.Fit)
				if _, ok := entities[key]; !ok {
					if _, err := s.CreateFitSpline(points(segment.Fit)...); err != nil {
						return nil, false
					}
					entities[key] = struct{}{}
				}
			default:
				return nil, false
			}
		}
	}
	return s, true
}

func momentRecordsEqual(a, b Profile) bool {
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
	case FitSplineSeg:
		b, ok := b.(FitSplineSeg)
		return ok && slices.Equal(a.Fit, b.Fit) && a.TStart == b.TStart && a.TEnd == b.TEnd
	default:
		return false
	}
}

// reconstructionOf reads that model off a checked record. Every segment counts,
// whatever its kind: the chord total is the arrangement's own element count, and
// the arrangement is global.
//
// An entity SEVERAL segments name counts once, because momentRecordScene interns
// the entities it builds and the arrangement therefore holds one set of chords
// per distinct entity. That is the ordinary shape of a recorded region — one
// crossing cuts a circle into two fragments, and both name the same circle — so
// counting per fragment squares a chord total the scene never holds and refuses
// records whose reconstruction costs milliseconds.
//
// Only the kinds momentRecordScene keys on a fixed-size struct are interned
// here. A free-form key renders every control point into a string
// (freeformEntityKey), which is a per-element pass and an allocation this charge
// exists to precede, so free-form segments stay counted per fragment. That is
// conservative in the safe direction: an over-count of the scene, never an
// under-count of it.
func reconstructionOf(record Profile) freeform.FreeformReconstruction {
	var chords uint64
	var seen map[momentEntityKey]struct{}
	for _, loop := range append([]LoopRecord{record.Outer}, record.Holes...) {
		for _, segment := range loop.Segments {
			if chords > freeform.ReconstructionChordCeiling {
				break
			}
			if key, keyed := analyticEntityKey(segment); keyed {
				if _, duplicate := seen[key]; duplicate {
					continue
				}
				if seen == nil {
					seen = make(map[momentEntityKey]struct{})
				}
				seen[key] = struct{}{}
			}
			chords = freeform.ReconstructionCostAdd(chords, reconstructionChords(segment))
		}
	}
	return freeform.FreeformReconstruction{Chords: chords, Arrangement: freeform.ReconstructionCostMul(chords, chords)}
}

// chargeReconstruction levies the record-level part of that charge — the
// scene arrangement the validation runs to list its candidate profiles, and the
// one its rescaled retry runs — and returns the per-arrangement charge the
// candidate loop then levies for itself.
func chargeReconstruction(record Profile, work *freeform.FreeformWork) (uint64, error) {
	demand := reconstructionOf(record)
	if err := work.ReconstructionStep(freeform.ReconstructionCostMul(2, demand.Arrangement)); err != nil {
		return 0, err
	}
	return demand.Arrangement, nil
}

// reconstructionChords is how many polyline chords sketch will create for the
// entity one recorded segment names. It is sampleParams's own table, keyed on
// the kind: a line is its own single chord, a free-form curve is chorded per
// control point over a floor, and everything curved and analytic is chorded per
// turn.
func reconstructionChords(segment CurveSegment) uint64 {
	switch segment := segment.(type) {
	case LineSeg:
		return 1
	case ArcSeg:
		return arcChords(segment)
	case SplineSeg:
		// An open cubic B-spline is sampled per SPAN, so the count comes off the
		// control count less the degree — the one free-form kind whose sample
		// count is not simply its control count.
		return freeform.FreeformChords(len(segment.Control) - 3)
	case ClosedSplineSeg:
		return freeform.FreeformChords(len(segment.Control))
	case NURBSSeg:
		return freeform.FreeformChords(len(segment.Control))
	case FitSplineSeg:
		return freeform.FreeformChords(len(segment.Fit))
	default:
		// A circle, an ellipse, a conic and an elliptical arc all reach the same
		// per-turn branch. A whole turn is its own upper bound, and a partial
		// sweep is charged for a whole one rather than re-deriving a sweep the
		// record states as no field of its own.
		return freeform.AnalyticChordsPerTurn
	}
}

// arcChords is the per-turn count scaled by the arc's own sweep — an ArcSeg is
// swept counter-clockwise from Start to End about Center, which is the sweep
// sketch derives from the same three pinned points. A sweep that is not a
// positive angle inside one turn is charged a whole turn, which the sampler
// itself cannot exceed.
func arcChords(seg ArcSeg) uint64 {
	sweep := math.Atan2(seg.End.V-seg.Center.V, seg.End.U-seg.Center.U) -
		math.Atan2(seg.Start.V-seg.Center.V, seg.Start.U-seg.Center.U)
	if sweep <= 0 {
		sweep += 2 * math.Pi
	}
	if !(sweep > 0) || sweep > 2*math.Pi {
		return freeform.AnalyticChordsPerTurn
	}
	chords := uint64(math.Ceil(freeform.AnalyticChordsPerTurn * sweep / (2 * math.Pi)))
	if chords < 2 {
		return 2
	}
	return chords
}
