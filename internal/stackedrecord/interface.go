package stackedrecord

import (
	"reflect"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// ExclusiveHoles returns the recorded holes present on only one side of a
// slab interface. Admission has already proved their nesting.
func ExclusiveHoles(lower, upper momentinput.Profile) ([]sectionrecord.LoopRecord, []sectionrecord.LoopRecord) {
	var lowerOnly, upperOnly []sectionrecord.LoopRecord
	for _, hole := range lower.Holes {
		if !containsLoop(upper.Holes, hole) {
			lowerOnly = append(lowerOnly, hole)
		}
	}
	for _, hole := range upper.Holes {
		if !containsLoop(lower.Holes, hole) {
			upperOnly = append(upperOnly, hole)
		}
	}
	return lowerOnly, upperOnly
}

func containsLoop(loops []sectionrecord.LoopRecord, wanted sectionrecord.LoopRecord) bool {
	for _, loop := range loops {
		// Segment records have several dynamic types, including slices and
		// unexported fields. A whole-record comparison preserves nil slices.
		if reflect.DeepEqual(loop, wanted) {
			return true
		}
	}
	return false
}
