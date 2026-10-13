package decad

import (
	"context"
	"fmt"
	"math"
	"reflect"
	"slices"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

const obliqueSectionSource = "decad.section.oblique.v1"

// The key is private. Sketch stores its value on the original owner pointer,
// without JSON serialization; the source marker makes lost certificates fail
// closed at every profile and chain entry point.
var obliqueSectionKey = sketch.NewAttachmentKey[*obliqueSectionCertificate]()

type obliqueSectionCertificate struct {
	owner    *sketch.Sketch
	revision uint64
	frame    r3.Frame
	profile  profileRecord
	delta    float64
	points   []*sketch.Point
	lines    []*sketch.Line
}

func boundedSectionSketch(s *sketch.Sketch) bool {
	if s == nil {
		return false
	}
	if obliqueSectionKey.Present(s) {
		return true
	}
	for _, point := range s.Points() {
		if point != nil && point.Source() == obliqueSectionSource {
			return true
		}
	}
	for _, entity := range s.Entities() {
		if entity != nil && entity.Source() == obliqueSectionSource {
			return true
		}
	}
	return false
}

func sectionFrameBitsEqual(a, b r3.Frame) bool {
	for _, pair := range [][2]r3.Vec{
		{a.Origin(), b.Origin()}, {a.U(), b.U()}, {a.V(), b.V()}, {a.N(), b.N()},
	} {
		if math.Float64bits(pair[0].X) != math.Float64bits(pair[1].X) ||
			math.Float64bits(pair[0].Y) != math.Float64bits(pair[1].Y) ||
			math.Float64bits(pair[0].Z) != math.Float64bits(pair[1].Z) {
			return false
		}
	}
	return true
}

func (c *obliqueSectionCertificate) live(s *sketch.Sketch) bool {
	if c == nil || c.owner != s || c.revision != s.Revision() || c.delta < 0 {
		return false
	}
	points, entities := s.Points(), s.Entities()
	for _, point := range c.points {
		if point == nil || !point.IsReference() || point.IsStale() || point.Source() != obliqueSectionSource {
			return false
		}
		if !slices.Contains(points, point) {
			return false
		}
	}
	for _, line := range c.lines {
		if line == nil || !line.IsReference() || line.IsStale() || line.Source() != obliqueSectionSource {
			return false
		}
		if !slices.Contains(entities, sketch.Entity(line)) {
			return false
		}
	}
	return true
}

// recordExtrudeProfile is the only profile seam that can consume a bounded
// Section. Every other caller uses recordProfile, which refuses the marker.
func recordExtrudeProfile(s *sketch.Sketch, p *sketch.Profile) (
	profileRecord, planeRecord, float64, *obliqueSectionCertificate, error,
) {
	if !boundedSectionSketch(s) {
		profile, plane, area, err := recordProfile(s, p)
		return profile, plane, area, nil, err
	}
	certificate, ok := obliqueSectionKey.Load(s)
	if !ok || !certificate.live(s) {
		return profileRecord{}, planeRecord{}, 0, nil,
			fmt.Errorf("%w: bounded Section source is stale", ErrUnsupported)
	}
	profile, plane, area, err := momentinput.RecordProfileWithArea(s, p)
	if err != nil {
		return profileRecord{}, planeRecord{}, 0, nil, err
	}
	if !reflect.DeepEqual(profile, certificate.profile) {
		return profileRecord{}, planeRecord{}, 0, nil,
			fmt.Errorf("%w: bounded Section profile has changed", ErrUnsupported)
	}
	frame, err := s.Plane().Frame()
	if err != nil || !sectionFrameBitsEqual(frame, certificate.frame) {
		return profileRecord{}, planeRecord{}, 0, nil,
			fmt.Errorf("%w: bounded Section frame has changed", ErrUnsupported)
	}
	return profile, plane, area, certificate, nil
}

func reconstructBoundedSection(ctx context.Context, polygon []planePoint, frame r3.Frame, delta float64) (
	*sketch.Sketch, *sketch.Profile, error,
) {
	if len(polygon) < 3 || delta < 0 {
		return nil, nil, fmt.Errorf("%w: Section has no bounded polygon", ErrUnsupported)
	}
	world := sketch.NewWorld()
	plane, err := world.CreatePlaneFromFrame(frame)
	if err != nil {
		return nil, nil, err
	}
	s, err := world.CreateSketch(plane)
	if err != nil {
		return nil, nil, err
	}
	points := make([]*sketch.Point, len(polygon))
	lines := make([]*sketch.Line, len(polygon))
	segments := make([]curveSegment, len(polygon))
	for i, point := range polygon {
		if err := ctx.Err(); err != nil {
			return nil, nil, err
		}
		points[i] = s.CreateReferencePoint(point.U, point.V, obliqueSectionSource)
	}
	for i := range polygon {
		next := (i + 1) % len(polygon)
		lines[i], err = s.CreateReferenceLine(points[i], points[next], obliqueSectionSource)
		if err != nil {
			return nil, nil, fmt.Errorf("%w: bounded Section reference line: %v", ErrUnsupported, err)
		}
		segments[i] = lineSeg{Start: polygon[i], End: polygon[next], TStart: 0, TEnd: 1}
	}
	expected := profileRecord{Outer: loopRecord{Segments: segments}}
	work := freeform.NewFreeformWork()
	if _, err := momentinput.ChargeReconstruction(expected, work); err != nil {
		return nil, nil, err
	}
	for _, profile := range s.Profiles() {
		if err := ctx.Err(); err != nil {
			return nil, nil, err
		}
		if !profile.Valid {
			continue
		}
		recorded, _, _, err := momentinput.RecordProfileWithArea(s, profile)
		if err != nil || !sectionSameLineRing(recorded, polygon) {
			continue
		}
		certificate := &obliqueSectionCertificate{
			owner: s, revision: s.Revision(), frame: frame, profile: recorded,
			delta: delta, points: points, lines: lines,
		}
		if !obliqueSectionKey.Attach(s, certificate) {
			return nil, nil, fmt.Errorf("%w: bounded Section certificate failed", ErrUnsupported)
		}
		return s, profile, nil
	}
	return nil, nil, fmt.Errorf("%w: Sketch did not certify bounded Section polygon", ErrUnsupported)
}

func sectionSameLineRing(recorded profileRecord, polygon []planePoint) bool {
	if len(recorded.Holes) != 0 || len(recorded.Outer.Segments) != len(polygon) {
		return false
	}
	used := make([]bool, len(polygon))
	for _, segment := range recorded.Outer.Segments {
		line, ok := segment.(lineSeg)
		if !ok || !((line.TStart == 0 && line.TEnd == 1) || (line.TStart == 1 && line.TEnd == 0)) {
			return false
		}
		found := false
		for i := range polygon {
			if used[i] {
				continue
			}
			a, b := polygon[i], polygon[(i+1)%len(polygon)]
			if (line.Start == a && line.End == b) || (line.Start == b && line.End == a) {
				used[i], found = true, true
				break
			}
		}
		if !found {
			return false
		}
	}
	return true
}
