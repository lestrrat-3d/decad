package stackedrecord

import (
	"context"
	"reflect"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Exposed reverses each exclusive hole into one exposed patch.
func Exposed(ctx context.Context, holes []sectionrecord.LoopRecord) ([]momentinput.Profile, error) {
	out := make([]momentinput.Profile, 0, len(holes))
	for _, hole := range holes {
		reversed, err := offset2d.ReverseLoopRecordContext(ctx, hole)
		if err != nil {
			return nil, err
		}
		out = append(out, momentinput.Profile{Outer: reversed})
	}
	return out, nil
}

// EnclosingExposed records the material between a new, enclosing hole and
// the earlier holes it contains. The cut's sketch cells prove that nesting.
func EnclosingExposed(ctx context.Context, outer sectionrecord.LoopRecord,
	inner []sectionrecord.LoopRecord) ([]momentinput.Profile, error) {
	reversed, err := offset2d.ReverseLoopRecordContext(ctx, outer)
	if err != nil {
		return nil, err
	}
	holes := append([]sectionrecord.LoopRecord(nil), inner...)
	return []momentinput.Profile{{Outer: reversed, Holes: holes}}, nil
}

// UnionExposed records a wider region with the narrower outer as a reversed hole.
func UnionExposed(ctx context.Context, wider, narrower momentinput.Profile) ([]momentinput.Profile, error) {
	hole, err := offset2d.ReverseLoopRecordContext(ctx, narrower.Outer)
	if err != nil {
		return nil, err
	}
	return []momentinput.Profile{{Outer: wider.Outer, Holes: []sectionrecord.LoopRecord{hole}}}, nil
}

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
