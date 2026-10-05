package main

import (
	"errors"
	"fmt"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// errBodyNotInTimeline is returned by timelineTrack.At when the timeline's
// world does not hold the track's body.
var errBodyNotInTimeline = errors.New("body is not in the timeline's world")

// timelineTrack shows one body of a dynamics.Timeline on a kinetograph driven
// node (docs/multibody-dynamics-design.md §11.2). Its At samples the
// timeline's certified state at the frame time and returns the body's pose
// there. A dynamics pose maps the body as modeled, placement included, to
// world, which is the frame of the part kinetograph tessellates; with the
// driven node directly under the rig's root, the pose is the node's local
// transform as it is.
//
// timelineTrack holds no mutable state, and Timeline.Sample writes nothing and
// returns the same state for the same time, so At returns the same transform
// for the same t and is safe for kinetograph's concurrent render workers. The
// timeline must not Advance while a clip that reads it renders.
type timelineTrack struct {
	timeline *dynamics.Timeline
	body     *decad.Body
}

// At returns the body's certified pose at d. It converts d to a time label
// of float seconds, which rounds the label by at most an ulp of a second; the
// pose returned is the certified pose at that label, so no geometric claim
// moves. A time beyond the timeline's certified end, or one the replay
// refuses, returns Timeline.Sample's error, and kinetograph fails the frame:
// the gallery never holds the last certified pose and never extrapolates.
func (t timelineTrack) At(d time.Duration) (r3.Transform, error) {
	state, err := t.timeline.Sample(units.Seconds(float64(d) / 1e9))
	if err != nil {
		return r3.Transform{}, fmt.Errorf("sample timeline at %s: %w", d, err)
	}
	entry, ok := state.Body(t.body)
	if !ok {
		return r3.Transform{}, errBodyNotInTimeline
	}
	return entry.Pose, nil
}
