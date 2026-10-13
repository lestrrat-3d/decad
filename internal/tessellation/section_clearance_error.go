package tessellation

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/units"
)

// RequireLoopClearance maps the first cross-loop clearance failure to the
// caller's typed tessellation refusal.
func RequireLoopClearance(ctx context.Context, pts []SectionPoint,
	loopIdx [][]int, loopSag []float64) error {
	failure, failed, err := SectionLoopClearance(ctx, pts, loopIdx, loopSag)
	if err != nil || !failed {
		return err
	}
	gate := failure.ChordGate + failure.Floor
	msg := fmt.Sprintf(
		`cap boundary loops %d and %d have measured distance %s; distance must exceed the required clearance gate %s`,
		failure.LoopA, failure.LoopB, units.Millimeters(failure.Distance), units.Millimeters(gate),
	)
	if failure.Distance > failure.Floor && failure.ChordGate > 0 {
		msg += `; retry with a finer chord tolerance to reduce the gate`
	}
	return NewExpectedError(fmt.Errorf(`%w: %s`, decaderr.ErrDegenerate, msg))
}

// RequireWalkClearance maps the first within-loop clearance failure to the
// indexed refusal that deterministic meridian refinement reads.
func RequireWalkClearance(ctx context.Context, pts []SectionPoint,
	loopIdx [][]int, sag [][]float64) error {
	failure, failed, err := SectionWalkClearance(ctx, pts, loopIdx, sag)
	if err != nil || !failed {
		return err
	}
	gate := failure.ChordGate + failure.Floor
	msg := fmt.Sprintf(
		`chords %d and %d of section loop %d have measured distance %s; distance must exceed the required clearance gate %s`,
		failure.ChordA, failure.ChordB, failure.Loop, units.Millimeters(failure.Distance), units.Millimeters(gate),
	)
	if failure.Distance > failure.Floor && failure.ChordGate > 0 {
		msg += `; retry with a finer chord tolerance to reduce the gate`
	}
	return &SectionClearanceError{
		Err:  NewExpectedError(fmt.Errorf(`%w: %s`, decaderr.ErrDegenerate, msg)),
		Loop: failure.Loop, A: failure.ChordA, B: failure.ChordB,
	}
}

// SectionClearanceError names the two chords RequireWalkClearance refused and
// the loop they belong to, so deterministic refinement can choose them.
type SectionClearanceError struct {
	Err        error
	Loop, A, B int
}

func (e *SectionClearanceError) Error() string { return e.Err.Error() }
func (e *SectionClearanceError) Unwrap() error { return e.Err }
