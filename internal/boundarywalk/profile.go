package boundarywalk

import (
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// Profile is a recorded outer loop and its holes.
type Profile struct {
	Outer sectionrecord.LoopRecord
	Holes []sectionrecord.LoopRecord
}

// ProfileWalks holds one profile's resolved walks and the work they charged.
type ProfileWalks struct {
	Outer               []survey2d.SegmentWalk
	Holes               [][]survey2d.SegmentWalk
	Spent               uint64
	ReconstructionSpent uint64
}

// ResolveProfile resolves each recorded segment once, in outer-then-holes
// order, and measures both work counters across those calls.
func ResolveProfile(profile Profile, work *freeform.FreeformWork) (ProfileWalks, error) {
	before, beforeRecon := WorkSpent(work)
	outer := make([]survey2d.SegmentWalk, len(profile.Outer.Segments))
	for i, seg := range profile.Outer.Segments {
		w, err := WalkOf(seg, work)
		if err != nil {
			return ProfileWalks{}, err
		}
		outer[i] = w
	}
	holes := make([][]survey2d.SegmentWalk, len(profile.Holes))
	for hi, hole := range profile.Holes {
		hw := make([]survey2d.SegmentWalk, len(hole.Segments))
		for i, seg := range hole.Segments {
			w, err := WalkOf(seg, work)
			if err != nil {
				return ProfileWalks{}, err
			}
			hw[i] = w
		}
		holes[hi] = hw
	}
	after, afterRecon := WorkSpent(work)
	return ProfileWalks{
		Outer:               outer,
		Holes:               holes,
		Spent:               after - before,
		ReconstructionSpent: afterRecon - beforeRecon,
	}, nil
}

// WorkSpent reads both counters, treating a nil counter as uncharged.
func WorkSpent(work *freeform.FreeformWork) (uint64, uint64) {
	if work == nil {
		return 0, 0
	}
	return work.Spent, work.ReconstructionSpent
}
