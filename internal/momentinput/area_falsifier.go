package momentinput

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
)

// FalsifyRecordedArea is evaluator-design §1's one live-profile read: the
// recorded region's closed-form area is compared against sketch's own Area
// answer, and a LARGE mismatch rejects the call — the record and the profile
// disagree, which is a bug somewhere. A small residual proves nothing and
// admits nothing; the check can only reject, the same one-sided shape as the
// seam's range falsifier.
//
// It returns the record's own area integral it compared, never sketch's
// answer, so a caller that needs the record's area (Loft's chord target,
// docs/loft-gear-bounds-design.md §5) reads it here instead of integrating
// the record a second time.
func (profile Profile) FalsifyRecordedArea(sketchArea float64, work *freeform.FreeformWork) (float64, error) {
	ig, err := profile.EvaluatorIntegrals(freeform.MomentAreaOrder, work)
	if err != nil {
		return 0, err
	}
	return compareRecordedArea(ig.Area, sketchArea)
}

// FalsifyRecordedAreaContext is FalsifyRecordedArea with the record's
// integration polling ctx; a canceled ctx returns ctx.Err() unchanged.
func (profile Profile) FalsifyRecordedAreaContext(ctx context.Context, sketchArea float64,
	work *freeform.FreeformWork) (float64, error) {
	ig, err := profile.EvaluatorIntegralsContext(ctx, freeform.MomentAreaOrder, work)
	if err != nil {
		return 0, err
	}
	return compareRecordedArea(ig.Area, sketchArea)
}

// FalsifyLoftRecordedArea compares the recorded area's own reading against
// sketch's authenticated area. Loft needs only area from the integral.
func (profile Profile) FalsifyLoftRecordedArea(sketchArea float64, work *freeform.FreeformWork) (float64, error) {
	area, err := profile.EvaluatorArea(work)
	if err != nil {
		return 0, err
	}
	return compareRecordedArea(area, sketchArea)
}

// compareRecordedArea rejects a recorded area that differs from sketch's by
// more than 1e-9 of the larger of 1 and sketch's magnitude, and returns the
// recorded area otherwise.
func compareRecordedArea(recorded, sketchArea float64) (float64, error) {
	scale := math.Max(1, math.Abs(sketchArea))
	if math.Abs(recorded-sketchArea) > 1e-9*scale {
		return 0, fmt.Errorf(`%w: the recorded boundary's area %v does not reproduce sketch's %v; report upstream as a bug`,
			decaderr.ErrUnrecordableProfile, recorded, sketchArea)
	}
	return recorded, nil
}
