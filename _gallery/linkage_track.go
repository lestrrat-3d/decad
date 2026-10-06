package main

import (
	"fmt"
	"slices"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// linkageTrack shows one link of a decad.Linkage on a kinetograph driven node
// (docs/linkage-check-design.md §2.4). Its At maps the frame time t to the
// drive fraction s = t/duration, clamped to [0, 1], and returns the link's
// world pose from Linkage.PoseAt, the same call VerifyLinkage builds every
// pose it checks from. A PoseAt pose maps the link's bodies as modeled,
// placement included, to world, which is the frame of the part kinetograph
// tessellates; with the driven node directly under the rig's root, the pose
// is the node's local transform as it is.
//
// linkageTrack holds no mutable state, and PoseAt writes nothing, so At
// returns the same transform for the same t and is safe for kinetograph's
// concurrent render workers. The linkage must not gain links while a clip
// that reads it renders. PoseAt composes one transform per joint on the
// link's path, so At keeps no cache.
type linkageTrack struct {
	linkage  *decad.Linkage
	drive    decad.Drive
	index    int // the link's position in linkage.Links()
	duration time.Duration
}

// newLinkageTrack returns the track of link under drive, run once over
// duration. link must be one of linkage.Links(), and duration positive.
func newLinkageTrack(linkage *decad.Linkage, drive decad.Drive, link *decad.Link,
	duration time.Duration) (linkageTrack, error) {
	index := slices.Index(linkage.Links(), link)
	if index < 0 {
		return linkageTrack{}, fmt.Errorf("linkage track: the link is not one of the linkage's links")
	}
	if duration <= 0 {
		return linkageTrack{}, fmt.Errorf("linkage track: duration %s is not positive", duration)
	}
	return linkageTrack{linkage: linkage, drive: drive, index: index, duration: duration}, nil
}

// At returns the link's world pose at the drive fraction t/duration. A time
// before 0 holds the pose at s = 0, and a time past duration the pose at
// s = 1, so a clip may hold the drive's end. PoseAt's refusal fails the frame.
func (t linkageTrack) At(d time.Duration) (r3.Transform, error) {
	s := driveFraction(d, t.duration)
	pose, err := t.linkage.PoseAt(t.drive, units.Scalar(s))
	if err != nil {
		return r3.Transform{}, fmt.Errorf("linkage pose at s = %g: %w", s, err)
	}
	return pose.Poses[t.index], nil
}

// driveFraction is t/duration clamped to [0, 1]. Both durations are whole
// nanoseconds, exact as float64 below 2⁵³ ns (about 104 days), so the
// quotient is the correctly rounded fraction, and exact whenever the true
// fraction is a float. At 64 frames per second frame i falls at exactly
// i·15625000 ns, so over a drive of n frames, n a power of two, frame i reads
// the dyadic fraction i/n exactly.
func driveFraction(t, duration time.Duration) float64 {
	if t <= 0 {
		return 0
	}
	if t >= duration {
		return 1
	}
	return float64(t) / float64(duration)
}
