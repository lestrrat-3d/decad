package prismcells

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sketchrecord"
	"github.com/lestrrat-3d/sketch"
)

func surfaceScene(ctx context.Context, budget *proofbound.WorkBudget, receiver, tool SurfaceOperand,
	op SurfaceOperation) (*sketch.Sketch, map[sketch.Entity]Origin, []*sketch.Profile, error) {
	segments, withinCap, err := RegionsWithinWorkCap(budget, receiver.Profile, tool.Profile)
	if err != nil {
		return nil, nil, nil, err
	}
	if !withinCap {
		switch op {
		case SurfaceTrim:
			return nil, nil, nil, fmt.Errorf(
				`%w: the trim scene charges at least %d arranger segments against this evaluator's cap of %d; simplify the receiver or tool before trimming`,
				decaderr.ErrUnsupported, segments, MaxArrangementSegments)
		case SurfaceSplit:
			return nil, nil, nil, fmt.Errorf(`%w: the split scene charges %d arranger segments against the cap of %d`,
				decaderr.ErrUnsupported, segments, MaxArrangementSegments)
		default:
			return nil, nil, nil, fmt.Errorf(`%w: the extend scene charges %d segments against the cap of %d`,
				decaderr.ErrUnsupported, segments, MaxArrangementSegments)
		}
	}
	reexpress, err := NewReexpression(receiver.Placement, tool.Placement)
	if err != nil {
		return nil, nil, nil, err
	}
	s, tags, _, err := BuildSceneRegions(budget,
		[]SceneProfile{{Outer: receiver.Profile.Outer, Holes: receiver.Profile.Holes}},
		[]SceneProfile{{Outer: tool.Profile.Outer, Holes: tool.Profile.Holes}}, reexpress)
	if err != nil {
		return nil, nil, nil, err
	}
	if err := budget.Err(); err != nil {
		return nil, nil, nil, err
	}
	profiles, err := ProfilesContext(ctx, s.Profiles)
	if err != nil {
		return nil, nil, nil, err
	}
	if err := budget.Err(); err != nil {
		return nil, nil, nil, err
	}
	return s, tags, profiles, nil
}

// ResolveSurfaceTrim selects and records the receiver's open walks from the
// private arrangement of an admitted Trim pair.
func ResolveSurfaceTrim(ctx context.Context, budget *proofbound.WorkBudget,
	receiver, tool SurfaceOperand, keepInside bool) ([]sectionrecord.ChainRecord, float64, error) {
	_, tags, profiles, err := surfaceScene(ctx, budget, receiver, tool, SurfaceTrim)
	if err != nil {
		return nil, 0, err
	}
	if len(profiles) == 0 {
		return nil, 0, fmt.Errorf(`%w: the receiver and tool's arrangement holds no bounded cell`, decaderr.ErrUnsupported)
	}
	// The structural no-crossing check runs before the classifier because
	// nested and disjoint arrangements are valid trim answers too.
	walks, err := ResolveTrimWalks(budget, tags, profiles, len(receiver.Profile.Holes), keepInside)
	if err != nil {
		return nil, 0, err
	}
	return RecordTrimWalks(budget, walks)
}

// SurfaceCell is one arranged target cell and its own section cut charge.
type SurfaceCell struct {
	Profile  momentinput.Profile
	CutDelta float64
}

// ResolveSurfaceSplit selects and records the target's separated cells.
func ResolveSurfaceSplit(ctx context.Context, budget *proofbound.WorkBudget,
	target, tool SurfaceOperand) ([]SurfaceCell, error) {
	_, tags, profiles, err := surfaceScene(ctx, budget, target, tool, SurfaceSplit)
	if err != nil {
		return nil, err
	}
	if len(profiles) == 0 {
		return nil, fmt.Errorf(`%w: the split arrangement holds no bounded cell`, decaderr.ErrUnsupported)
	}
	selected, err := ResolveSplitCells(budget, tags, profiles, len(target.Profile.Holes))
	if err != nil {
		return nil, err
	}
	result := make([]SurfaceCell, len(selected))
	for i, cell := range selected {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		profile, err := recordSurfaceCellContext(ctx, cell)
		if err != nil {
			return nil, err
		}
		cutDelta, err := SplitCellCutDelta(budget, cell)
		if err != nil {
			return nil, err
		}
		result[i] = SurfaceCell{Profile: profile, CutDelta: cutDelta}
	}
	return result, nil
}

func recordSurfaceCellContext(ctx context.Context, cell *sketch.Profile) (momentinput.Profile, error) {
	if err := ctx.Err(); err != nil {
		return momentinput.Profile{}, err
	}
	type result struct {
		profile momentinput.Profile
		err     error
	}
	done := make(chan result)
	go func() {
		outer, holes, err := sketchrecord.RecordProfileLoops(cell)
		done <- result{profile: momentinput.Profile{Outer: outer, Holes: holes}, err: err}
	}()
	select {
	case recorded := <-done:
		return recorded.profile, recorded.err
	case <-ctx.Done():
		<-done
		return momentinput.Profile{}, ctx.Err()
	}
}

// ResolveSurfaceExtend reads sketch's nearest cut on the named carrier and
// widens that carrier's recorded range in its own parameter order. The full
// carrier is recreated in ascending order, so the scene and record use the
// same parameter: a reversed LineSeg does not need a 1 − t conversion.
func ResolveSurfaceExtend(ctx context.Context, budget *proofbound.WorkBudget,
	receiver, tool SurfaceOperand, seg sectionrecord.CurveSegment, atStart bool) (sectionrecord.CurveSegment, float64, error) {
	t0, t1, err := SegmentParamRange(seg)
	if err != nil {
		return nil, 0, err
	}
	old := t1
	if atStart {
		old = t0
	}
	if old == 0 || old == 1 {
		return nil, 0, fmt.Errorf(`%w: the named section end already reaches its carrier's own domain`, decaderr.ErrUnsupported)
	}
	full, err := FullExtendSegment(seg)
	if err != nil {
		return nil, 0, err
	}
	receiver.Profile = momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{full}}}
	s, tags, profiles, err := surfaceScene(ctx, budget, receiver, tool, SurfaceExtend)
	if err != nil {
		return nil, 0, err
	}
	nearest, edge, found, err := ResolveExtendCut(budget, tags, profiles,
		func() ([]*sketch.Chain, error) { return ChainsContext(ctx, s.Chains) }, t0, t1, atStart)
	if err != nil {
		return nil, 0, err
	}
	if !found {
		return nil, 0, fmt.Errorf(
			`%w: the tool has no cut past the named end at t = %v inside the carrier's own natural domain, which is %s`,
			decaderr.ErrUnsupported, old, ExtendCarrierDomain(seg))
	}
	if edge.Entity != nil {
		if _, err := sketchrecord.RecordEdge(edge); err != nil {
			return nil, 0, err
		}
	}
	widened := ExtendSetBound(seg, atStart, nearest)
	delta, err := CutDelta(sketch.BoundaryEdge{Partial: true}, widened)
	if err != nil {
		return nil, 0, err
	}
	return widened, delta, nil
}
