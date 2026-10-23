package facetedrecord

import (
	"slices"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
)

// PointSourceCurrent checks the source identity held when LoftFromPoint
// recorded the profile. Certificate getters are checked separately at use.
func PointSourceCurrent(record *PointSectionRecord) bool {
	if record == nil || record.Source == nil || record.SourceBox != record.SourceBoxIssued ||
		!slices.Equal(record.SourceStation, record.SourceStationIssued) ||
		proofbound.IsNonFinite(record.SourceStationRound) ||
		record.SourceStationRound < 0 ||
		record.SourceStationRound != record.SourceRoundIssued {
		return false
	}
	src := record.Source
	if src.Sketch == nil || src.Profile == nil || src.Profile.Sketch() != src.Sketch ||
		src.Profile.IsStale() || src.Sketch.Revision() != src.Revision ||
		src.Profile.Revision() != src.Revision ||
		len(src.Entities) != len(record.Profile.Outer.Segments) ||
		len(src.Profile.Outer) != len(src.Entities) {
		return false
	}
	_, plane, _, err := sectioncert.RecordProfile(src.Sketch, src.Profile)
	if err != nil {
		return false
	}
	// LoftFromPoint recorded NewFrame of the profile's plane record, and NewFrame
	// is not idempotent: the sketch plane's own frame can differ from it in the
	// last bit, so the current plane is rebuilt the way the record was.
	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil || frame.Origin() != src.Frame.Origin() ||
		frame.U() != src.Frame.U() || frame.V() != src.Frame.V() {
		return false
	}
	for i, entity := range src.Entities {
		if entity == nil || src.Profile.Outer[i].Entity != entity {
			return false
		}
	}
	return true
}
