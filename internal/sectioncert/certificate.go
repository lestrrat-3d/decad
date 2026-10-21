package sectioncert

import (
	"context"
	"fmt"
	"math"
	"reflect"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

type curveSegment = sectionrecord.CurveSegment

type lineSeg = sectionrecord.LineSeg

type loopRecord = sectionrecord.LoopRecord

// planePoint is the evaluator's plane-local coordinate. Point2 is its public
// spelling; both share one underlying struct, so a value assigns either way.
type planePoint = sectionrecord.Point2

type planeRecord = sectionrecord.PlaneRecord

type profileRecord = momentinput.Profile

// RecordProfile converts a sketch profile into the structural records
// the evaluator carries: the region as a profile record — the entity's own
// defining data per boundary edge, plus the recorded range — and the sketch
// plane, read through s.Plane().Frame(), as the plane record that lifts the
// plane-local region into world space. The feature calls run this conversion.
//
// Admission consumes only a fresh snapshot authenticated against sketch's own
// answers (agent-docs/api-design.md §7): p must name s (Profile.Sketch, else
// [ErrForeignProfile]), be current (Profile.IsStale, else [ErrStaleProfile]),
// and exactly match one fresh result from s.Profiles. Every boundary entity
// must be non-nil and owned by s. A caller-altered snapshot is
// [ErrInvalidProfile], and a foreign boundary entity is [ErrForeignProfile].
// A profile whose Valid is false is also never silently swept
// ([ErrInvalidProfile]). The one further rejection is not a validity judgement:
// an authenticated valid profile whose boundary cannot be recorded exactly —
// a Partial fragment sketch could not certify
// (BoundaryEdge.TExact == false), a certified range the seam's reject-only
// falsifier disproves, or a loop whose source-aware junction check finds a
// contradiction — is [ErrUnrecordableProfile]. decad never repairs, projects
// or fits a point sketch handed over, and it never solves for one.
//
// That last rejection is the one a caller meets by drawing rather than by
// hitting an upstream limit. sketch admits a region on its own proximity
// threshold, so entities whose ends miss by a fraction of a nanometre still
// arrange into one valid profile. At a whole-to-whole join, decad records each
// entity's own points verbatim, so a loop whose recorded coordinates do not
// meet bounds no region. A certified mixed join at a genuinely cut bound
// instead compares the record endpoint with sketch's evaluated node at the
// range falsifier's relative tolerance. An uncut Partial bound uses the
// record's own endpoint, so it remains an exact check. Ends merely driven
// together by a coincidence constraint remain a whole-to-whole exact check:
// the solver converges to within its residual, not to the same coordinate.
// The return value includes authenticated area so feature callers never read
// a caller-mutable field after admission.
func RecordProfile(s *sketch.Sketch, p *sketch.Profile) (profileRecord, planeRecord, float64, error) {
	if BoundedSectionSketch(s) {
		return profileRecord{}, planeRecord{}, 0,
			fmt.Errorf("%w: this bounded Section profile needs a consumer that carries its source bound", decaderr.ErrUnsupported)
	}
	return momentinput.RecordProfileWithArea(s, p)
}

const obliqueSectionSource = "decad.section.oblique.v1"

// The key is private. Sketch stores its value on the original owner pointer,
// without JSON serialization; the source marker makes lost certificates fail
// closed at every profile and chain entry point.
var obliqueSectionKey = sketch.NewAttachmentKey[*obliqueSectionCertificate]()

// obliqueSectionCertificate is the private provenance a bounded Section attaches
// to its Sketch; Frame and Delta are read by the root Extrude seam.
type obliqueSectionCertificate struct {
	owner    *sketch.Sketch
	revision uint64
	Frame    r3.Frame
	profile  profileRecord
	Delta    float64
	points   []*sketch.Point
	lines    []*sketch.Line
}

// BoundedSectionSketch reports whether s is a private bounded-section Sketch
// carrying a live oblique section certificate.
func BoundedSectionSketch(s *sketch.Sketch) bool {
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
	if c == nil || c.owner != s || c.revision != s.Revision() || c.Delta < 0 {
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

// RecordExtrudeProfile is the only profile seam that can consume a bounded
// Section. Every other caller uses RecordProfile, which refuses the marker.
func RecordExtrudeProfile(s *sketch.Sketch, p *sketch.Profile) (
	profileRecord, planeRecord, float64, *obliqueSectionCertificate, error,
) {
	if !BoundedSectionSketch(s) {
		profile, plane, area, err := RecordProfile(s, p)
		return profile, plane, area, nil, err
	}
	certificate, ok := obliqueSectionKey.Load(s)
	if !ok || !certificate.live(s) {
		return profileRecord{}, planeRecord{}, 0, nil,
			fmt.Errorf("%w: bounded Section source is stale", decaderr.ErrUnsupported)
	}
	profile, plane, area, err := momentinput.RecordProfileWithArea(s, p)
	if err != nil {
		return profileRecord{}, planeRecord{}, 0, nil, err
	}
	if !reflect.DeepEqual(profile, certificate.profile) {
		return profileRecord{}, planeRecord{}, 0, nil,
			fmt.Errorf("%w: bounded Section profile has changed", decaderr.ErrUnsupported)
	}
	frame, err := s.Plane().Frame()
	if err != nil || !sectionFrameBitsEqual(frame, certificate.Frame) {
		return profileRecord{}, planeRecord{}, 0, nil,
			fmt.Errorf("%w: bounded Section frame has changed", decaderr.ErrUnsupported)
	}
	return profile, plane, area, certificate, nil
}

// ReconstructBoundedSection rebuilds the private bounded-section Sketch and
// its profile from a recorded section polygon, frame and delta.
func ReconstructBoundedSection(ctx context.Context, polygon []planePoint, frame r3.Frame, delta float64) (
	*sketch.Sketch, *sketch.Profile, error,
) {
	if len(polygon) < 3 || delta < 0 {
		return nil, nil, fmt.Errorf("%w: Section has no bounded polygon", decaderr.ErrUnsupported)
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
			return nil, nil, fmt.Errorf("%w: bounded Section reference line: %v", decaderr.ErrUnsupported, err)
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
			owner: s, revision: s.Revision(), Frame: frame, profile: recorded,
			Delta: delta, points: points, lines: lines,
		}
		if !obliqueSectionKey.Attach(s, certificate) {
			return nil, nil, fmt.Errorf("%w: bounded Section certificate failed", decaderr.ErrUnsupported)
		}
		return s, profile, nil
	}
	return nil, nil, fmt.Errorf("%w: Sketch did not certify bounded Section polygon", decaderr.ErrUnsupported)
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
