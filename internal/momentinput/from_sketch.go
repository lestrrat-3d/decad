package momentinput

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sketchrecord"
	"github.com/lestrrat-3d/sketch"
)

// RecordProfile admits a live sketch profile and converts its boundary into
// the structural record used by the evaluator. Callers needing the area from
// the authenticated snapshot use RecordProfileWithArea.
func RecordProfile(s *sketch.Sketch, p *sketch.Profile) (Profile, sectionrecord.PlaneRecord, error) {
	profile, plane, _, err := RecordProfileWithArea(s, p)
	return profile, plane, err
}

// RecordProfileWithArea also returns the authenticated sketch area, so callers
// never read a caller-mutable profile field after admission.
func RecordProfileWithArea(s *sketch.Sketch, p *sketch.Profile) (Profile, sectionrecord.PlaneRecord, float64, error) {
	trusted, err := sketchrecord.AdmitProfile(s, p)
	if err != nil {
		return Profile{}, sectionrecord.PlaneRecord{}, 0, err
	}

	frame, err := s.Plane().Frame()
	if err != nil {
		return Profile{}, sectionrecord.PlaneRecord{}, 0,
			fmt.Errorf(`decad: failed to resolve the sketch plane: %w`, err)
	}
	plane := sectionrecord.PlaneRecord{Origin: frame.Origin(), U: frame.U(), V: frame.V()}

	outer, holes, err := sketchrecord.RecordProfileLoops(trusted)
	if err != nil {
		return Profile{}, sectionrecord.PlaneRecord{}, 0, err
	}
	return Profile{Outer: outer, Holes: holes}, plane, trusted.Area, nil
}
