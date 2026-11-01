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

// ChordRectangleReplay chords one four-line rectangle under the replay budget.
// wallFace must be a bounded lookup supplied by the caller.
func ChordRectangleReplay[F any](ctx context.Context, loop sectionrecord.LoopRecord, chord, height float64,
	work *freeform.FreeformWork, resolved *momentinput.ProfileWalks, wallFace func(survey2d.SideWalk) (F, error),
	budget *proofbound.ReplayBudget) (ChordSamples[F], error) {
	if err := ctx.Err(); err != nil {
		return ChordSamples[F]{}, err
	}
	if budget == nil || wallFace == nil || len(loop.Segments) != 4 {
		return ChordSamples[F]{}, fmt.Errorf("%w: bounded prism requires a four-line rectangle", decaderr.ErrUnsupported)
	}
	var lines [4]sectionrecord.LineSeg
	var axes [4]int
	for i, segment := range loop.Segments {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return ChordSamples[F]{}, err
		}
		line, ok := segment.(sectionrecord.LineSeg)
		if !ok || line.TStart != 0 || line.TEnd != 1 {
			return ChordSamples[F]{}, fmt.Errorf("%w: bounded prism requires whole straight edges", decaderr.ErrUnsupported)
		}
		if proofbound.IsNonFinite(line.Start.U) || proofbound.IsNonFinite(line.Start.V) ||
			proofbound.IsNonFinite(line.End.U) || proofbound.IsNonFinite(line.End.V) {
			return ChordSamples[F]{}, fmt.Errorf("%w: bounded prism requires finite edges", decaderr.ErrUnsupported)
		}
		lines[i] = line
		switch {
		case line.Start.U != line.End.U && line.Start.V == line.End.V:
			axes[i] = 1
		case line.Start.U == line.End.U && line.Start.V != line.End.V:
			axes[i] = 2
		default:
			return ChordSamples[F]{}, fmt.Errorf("%w: bounded prism requires axis-aligned edges", decaderr.ErrUnsupported)
		}
	}
	for i := range lines {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return ChordSamples[F]{}, err
		}
		if lines[i].End != lines[(i+1)%len(lines)].Start || axes[i] == axes[(i+1)%len(lines)] {
			return ChordSamples[F]{}, fmt.Errorf("%w: bounded prism edges do not form a rectangle", decaderr.ErrUnsupported)
		}
	}
	var cached []survey2d.SegmentWalk
	if resolved == nil {
		return ChordSamples[F]{}, fmt.Errorf("%w: bounded rectangle requires recorded walks", decaderr.ErrUnsupported)
	}
	// LoopMatches allocates one loop record. Its comparison can visit only
	// the four line records validated here.
	if len(resolved.Profile.Holes) != 0 || len(resolved.Profile.Outer.Segments) != 4 ||
		len(resolved.Outer) != 4 || len(resolved.Holes) != 0 || !resolved.Metered ||
		resolved.Spent != 0 || resolved.ReconstructionSpent != 0 {
		return ChordSamples[F]{}, fmt.Errorf("%w: bounded prism cannot reuse these walks", decaderr.ErrUnsupported)
	}
	for i, segment := range resolved.Profile.Outer.Segments {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return ChordSamples[F]{}, err
		}
		walk := resolved.Outer[i]
		if _, ok := segment.(sectionrecord.LineSeg); !ok || walk.Kind != survey2d.WalkLine ||
			len(walk.Spans) != 0 || walk.StartU != lines[i].Start.U || walk.StartV != lines[i].Start.V ||
			walk.EndU != lines[i].End.U || walk.EndV != lines[i].End.V ||
			proofbound.IsNonFinite(walk.TanInU) || proofbound.IsNonFinite(walk.TanInV) ||
			proofbound.IsNonFinite(walk.TanOutU) || proofbound.IsNonFinite(walk.TanOutV) ||
			(axes[i] == 1 && (walk.TanInU == 0 || walk.TanOutU == 0 || walk.TanInV != 0 || walk.TanOutV != 0)) ||
			(axes[i] == 2 && (walk.TanInV == 0 || walk.TanOutV == 0 || walk.TanInU != 0 || walk.TanOutU != 0)) {
			return ChordSamples[F]{}, fmt.Errorf("%w: bounded prism cached walk is not linear", decaderr.ErrUnsupported)
		}
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return ChordSamples[F]{}, err
	}
	// Four line records contain at most 64 reflection nodes and scalar fields.
	if err := budget.SpendWork(ctx, 64); err != nil {
		return ChordSamples[F]{}, err
	}
	if !resolved.LoopMatches(0, loop) {
		return ChordSamples[F]{}, momentinput.ErrResolvedWalksMismatch
	}
	cached = resolved.LoopWalks(0)
	// Raw walks and their one-element segment lists. The coalescer's output
	// and all growing segment lists use at most n+n² records for n=4.
	if err := budget.ReserveRecords(ctx, 4+4+4+4*4); err != nil {
		return ChordSamples[F]{}, err
	}
	raw := make([]survey2d.SideWalk, 4)
	perimeter := 0.0
	for i := range loop.Segments {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return ChordSamples[F]{}, err
		}
		walk := cached[i]
		perimeter = proofbound.AbsSumUpper(perimeter, walk.Length, walk.LengthBound)
		raw[i] = survey2d.SideWalk{SegmentWalk: walk, Segs: []int{i}}
	}
	if err := budget.ReserveRecords(ctx, 4); err != nil {
		return ChordSamples[F]{}, err
	}
	shared := budget.WorkBudget(ctx)
	walks, err := boundarywalk.CoalesceWalksBudget(raw, shared)
	if err != nil {
		return ChordSamples[F]{}, err
	}
	// A rectangle emits at most four stations into each of four slices.
	// Reserving n² for each slice covers all append growth for n=4.
	if err := budget.ReserveRecords(ctx, 4*4*4); err != nil {
		return ChordSamples[F]{}, err
	}
	sampled, err := SampleLoopWithCountFloor(walks, loop.Segments, chord, height, work, shared,
		wallFace, nil, nil, 0)
	if err != nil {
		return ChordSamples[F]{}, err
	}
	sampled.PerimeterUpper = perimeter
	return sampled, nil
}
