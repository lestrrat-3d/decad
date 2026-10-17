package mirrorjoin

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
)

// WallRef names one selected wall in a recorded prism or stacked prism.
// A prism uses Slab -1; a stacked prism names its slab directly.
type WallRef struct {
	Slab, Loop, Segment int
}

// JoinPrismPayload runs J2-J7 over a prism receiver and returns the joined
// payload: the same frame, placement, interval and axial displacements over
// the assembled record, with δ_mirror as its section displacement.
func JoinPrismPayload(budget *proofbound.WorkBudget, pp prismrecord.Payload, walls []WallRef) (prismrecord.Payload, error) {
	region := NewRegion(pp.Profile)
	segs := make([]sectionrecord.LineSeg, 0, len(walls))
	for _, w := range walls {
		seg, err := joinWallSegment(region.Loops, w)
		if err != nil {
			return prismrecord.Payload{}, err
		}
		segs = append(segs, seg)
		region.Sel[w.Loop][w.Segment] = struct{}{}
	}
	line, err := AdmitLine(segs)
	if err != nil {
		return prismrecord.Payload{}, err
	}
	if err := line.AdmitRegions(budget, []Region{region}); err != nil {
		return prismrecord.Payload{}, err
	}
	if pp.SectionDelta != 0 {
		return prismrecord.Payload{}, errJoinDisplaced
	}
	joined, delta, err := line.Assemble(budget, region)
	if err != nil {
		return prismrecord.Payload{}, err
	}
	out := pp
	out.Profile = joined
	out.SectionDelta = delta
	out.Walks = nil
	out.BlendSegs = nil
	out.BlendKind = ""
	return out, nil
}

// JoinStackedPayload runs J2-J7 over every slab's region of a stacked
// receiver, rewrites each region by the same splice, and re-derives every
// interface's exposed records from the rewritten regions. Equal loops rewrite
// to equal loops, so the outer wall stays one column and each interface stays
// monotone; stackedrecord.Build's Falsify call re-checks both.
func JoinStackedPayload(ctx context.Context, budget *proofbound.WorkBudget, sp stackedrecord.Payload, walls []WallRef) (stackedrecord.Payload, error) {
	// J1: the join re-derives each interface from exclusive holes, which a
	// union-built stack's changing outer loop does not record.
	if len(sp.OuterRuns()) != 1 {
		return stackedrecord.Payload{}, fmt.Errorf(`%w: a union-built stack changes its outer loop between slabs, which the join does not rewrite; join the operands before the union (J1)`, decaderr.ErrUnsupported)
	}
	regions := make([]Region, len(sp.Slabs))
	for k, slab := range sp.Slabs {
		if len(slab.Regions) != 1 {
			return stackedrecord.Payload{}, fmt.Errorf(`%w: slab %d has no single region`, decaderr.ErrUnsupported, k)
		}
		regions[k] = NewRegion(slab.Regions[0])
	}
	segs := make([]sectionrecord.LineSeg, 0, len(walls))
	for _, w := range walls {
		if w.Slab < 0 || w.Slab >= len(regions) {
			return stackedrecord.Payload{}, fmt.Errorf(`%w: a selected wall names slab %d, which the receiver does not have`, decaderr.ErrDegenerate, w.Slab)
		}
		seg, err := joinWallSegment(regions[w.Slab].Loops, w)
		if err != nil {
			return stackedrecord.Payload{}, err
		}
		segs = append(segs, seg)
		// The wall's face spans its whole column: mark the same loop and
		// segment wherever an equal loop appears.
		wallLoop := regions[w.Slab].Loops[w.Loop]
		for _, region := range regions {
			for li, loop := range region.Loops {
				equal, err := momentinput.ExactLoopEqual(budget, wallLoop, loop)
				if err != nil {
					return stackedrecord.Payload{}, err
				}
				if equal {
					region.Sel[li][w.Segment] = struct{}{}
				}
			}
		}
	}
	line, err := AdmitLine(segs)
	if err != nil {
		return stackedrecord.Payload{}, err
	}
	if err := line.AdmitRegions(budget, regions); err != nil {
		return stackedrecord.Payload{}, err
	}
	if sp.SectionDelta != 0 {
		return stackedrecord.Payload{}, errJoinDisplaced
	}
	out := sp
	out.Slabs = make([]stackedrecord.Slab, len(sp.Slabs))
	for k, region := range regions {
		joined, delta, err := line.Assemble(budget, region)
		if err != nil {
			return stackedrecord.Payload{}, err
		}
		out.Slabs[k] = sp.Slabs[k]
		out.Slabs[k].Regions = []momentinput.Profile{joined}
		out.SectionDelta = math.Max(out.SectionDelta, delta)
	}
	interfaces, err := stackedrecord.Derive(ctx, out.Slabs, sp.Interfaces)
	if err != nil {
		return stackedrecord.Payload{}, err
	}
	out.Interfaces = interfaces
	return out, nil
}

// errJoinDisplaced is J7: the §5.3 audit proves closure on the recorded
// coordinates, and a displaced record does not state the section it denotes.
var errJoinDisplaced = fmt.Errorf(`%w: the receiver's section carries a displacement from the section it denotes, so the join cannot prove its splice closes; join a receiver drawn directly (J7)`, decaderr.ErrUnsupported)

// joinWallSegment is J2's record half: the selected wall's segment, which
// must be a line. A curved wall names no mirror plane.
func joinWallSegment(loops []sectionrecord.LoopRecord, w WallRef) (sectionrecord.LineSeg, error) {
	if w.Loop < 0 || w.Loop >= len(loops) || w.Segment < 0 || w.Segment >= len(loops[w.Loop].Segments) {
		return sectionrecord.LineSeg{}, fmt.Errorf(`%w: a selected wall names segment (%d, %d), which the receiver's record does not have`, decaderr.ErrDegenerate, w.Loop, w.Segment)
	}
	seg, err := sectionrecord.NormalizeSegment(loops[w.Loop].Segments[w.Segment])
	if err != nil {
		return sectionrecord.LineSeg{}, err
	}
	line, ok := seg.(sectionrecord.LineSeg)
	if !ok {
		return sectionrecord.LineSeg{}, fmt.Errorf(`%w: selected wall (%d, %d) is a %T, and a curved wall names no mirror plane`, decaderr.ErrDegenerate, w.Loop, w.Segment, seg)
	}
	return line, nil
}
