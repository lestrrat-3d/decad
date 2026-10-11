package tessellation

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ChordLoop resolves and chords a recorded loop once for every face that
// shares it. Resolved walks must come from the same loop, and their resolution
// cost must already have been charged to work by the caller. One work budget
// spans walk resolution, coalescing and sample emission.
func ChordLoop[F any](ctx context.Context, loop sectionrecord.LoopRecord, chord, height float64,
	work *freeform.FreeformWork, resolved *momentinput.ProfileWalks, roleLoop int,
	wallFace func(survey2d.SideWalk) (F, error),
	stationBound func(sectionrecord.CurveSegment, int, int, float64, float64) proofbound.WalkEndBound,
) (ChordSamples[F], error) {
	return ChordLoopWithCountFloor(ctx, loop, chord, height, work, resolved, roleLoop,
		wallFace, stationBound, nil, 0)
}

// ChordLoopWithCountFloor uses at least the caller's circular count for each
// resolved walk. A composite sweep also supplies one free-form sagitta target,
// so every span reproduces the same dyadic parameter stations.
func ChordLoopWithCountFloor[F any](ctx context.Context, loop sectionrecord.LoopRecord, chord, height float64,
	work *freeform.FreeformWork, resolved *momentinput.ProfileWalks, roleLoop int,
	wallFace func(survey2d.SideWalk) (F, error),
	stationBound func(sectionrecord.CurveSegment, int, int, float64, float64) proofbound.WalkEndBound,
	countFloor func(survey2d.SideWalk) int,
	freeformTarget float64,
) (ChordSamples[F], error) {
	if len(loop.Segments) == 0 {
		return ChordSamples[F]{}, fmt.Errorf(`%w: a recorded loop holds no segments`, decaderr.ErrDegenerate)
	}
	budget := proofbound.NewWorkBudget(ctx)
	var loopWalks []survey2d.SegmentWalk
	if resolved != nil {
		if !resolved.LoopMatches(roleLoop, loop) {
			return ChordSamples[F]{}, momentinput.ErrResolvedWalksMismatch
		}
		loopWalks = resolved.LoopWalks(roleLoop)
	}
	raw := make([]survey2d.SideWalk, len(loop.Segments))
	// The perimeter charge uses raw walk lengths, as the body perimeter does.
	perimeterUpper := 0.0
	for i, seg := range loop.Segments {
		if err := budget.Step(); err != nil {
			return ChordSamples[F]{}, err
		}
		var w survey2d.SegmentWalk
		if loopWalks != nil {
			w = loopWalks[i]
		} else {
			var err error
			w, err = boundarywalk.WalkOf(seg, work)
			if err != nil {
				return ChordSamples[F]{}, err
			}
		}
		perimeterUpper = proofbound.AbsSumUpper(perimeterUpper, w.Length, w.LengthBound)
		raw[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
	}
	walks, err := boundarywalk.CoalesceWalksContext(ctx, raw)
	if err != nil {
		return ChordSamples[F]{}, err
	}
	sampled, err := SampleLoopWithCountFloor(walks, loop.Segments, chord, height, work, budget,
		wallFace, stationBound, countFloor, freeformTarget)
	if err != nil {
		return ChordSamples[F]{}, err
	}
	sampled.PerimeterUpper = perimeterUpper
	return sampled, nil
}
