package patternassembly

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/patternrecord"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// frameOf reads the frame, placement and regions a frame-keeping instance
// shares with its receiver.
func frameOf(payload topo.Payload) (r3.Frame, r3.Transform, []momentinput.Profile, bool) {
	switch p := payload.(type) {
	case prismrecord.Payload:
		return p.Frame, p.Xform, []momentinput.Profile{p.Profile}, true
	case stackedrecord.Payload:
		regions := make([]momentinput.Profile, 0, len(p.Slabs))
		for _, slab := range p.Slabs {
			regions = append(regions, slab.Regions...)
		}
		return p.Frame, p.Xform, regions, true
	default:
		return r3.Frame{}, r3.Transform{}, nil, false
	}
}

// KeepsFrame decides §4.3's frame-keeping arm: a prism or stacked receiver
// whose every segment is a line, arc or circle with no trimmed arc or circle,
// under a co-directional motion. Co-direction is read on the composed world
// normal xform.ApplyDir(frame.N()), as prism-boolean G3 reads it: a linear
// Dir whose exact rational dot product with that normal is zero, or a
// circular Axis bit-identical to ±that normal.
func KeepsFrame(budget *proofbound.WorkBudget, rp patternrecord.Spec, payload topo.Payload) (bool, error) {
	frame, xform, regions, ok := frameOf(payload)
	if !ok {
		return false, nil
	}
	for _, region := range regions {
		for _, loop := range append([]sectionrecord.LoopRecord{region.Outer}, region.Holes...) {
			for _, seg := range loop.Segments {
				switch seg.(type) {
				case sectionrecord.LineSeg, sectionrecord.ArcSeg, sectionrecord.CircleSeg:
				default:
					return false, nil
				}
			}
		}
		trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, region.Outer, region.Holes)
		if err != nil {
			return false, err
		}
		if trimmed {
			return false, nil
		}
	}
	n := xform.ApplyDir(frame.N())
	if rp.Circular {
		return rp.Axis == n || rp.Axis == n.Scale(-1), nil
	}
	return patternrecord.Perpendicular(rp.Dir, n), nil
}

func moveRegion(budget *proofbound.WorkBudget, region momentinput.Profile,
	mv patternrecord.PointMotion) (momentinput.Profile, float64, error) {
	moved, delta, err := patternrecord.MoveRegion(budget,
		patternrecord.Region{Outer: region.Outer, Holes: region.Holes}, mv)
	if err != nil {
		return momentinput.Profile{}, 0, err
	}
	return momentinput.Profile{Outer: moved.Outer, Holes: moved.Holes}, delta, nil
}

// FrameKeepingInstance builds instance i on the receiver's frame and
// placement, moving its record in the plane and charging the motion's
// displacement. fallback asks root to use its Copy arm for a rounded
// union-built multi-slab stack.
func FrameKeepingInstance(ctx context.Context, env topo.Env, rp patternrecord.Spec,
	payload topo.Payload, i int) (*topo.Body, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	frame, xform, _, _ := frameOf(payload)
	mv, err := rp.Motion(frame, xform, i)
	if err != nil {
		return nil, false, err
	}
	switch p := payload.(type) {
	case prismrecord.Payload:
		moved, delta, err := moveRegion(budget, p.Profile, mv)
		if err != nil {
			return nil, false, err
		}
		p.Profile = moved
		p.SectionDelta = patternrecord.WithDelta(p.SectionDelta, delta)
		p.Walks = nil
		body, err := prismrecord.Build(ctx, env, p, freeform.NewFreeformWork())
		return body, false, err
	case stackedrecord.Payload:
		slabs := make([]stackedrecord.Slab, len(p.Slabs))
		delta := 0.0
		for k, slab := range p.Slabs {
			slabs[k] = slab
			slabs[k].Regions = make([]momentinput.Profile, len(slab.Regions))
			for r, region := range slab.Regions {
				moved, charge, err := moveRegion(budget, region, mv)
				if err != nil {
					return nil, false, err
				}
				slabs[k].Regions[r] = moved
				delta = math.Max(delta, charge)
			}
		}
		if delta != 0 {
			runs := p.OuterRuns()
			if len(runs) != 1 {
				// A union-built stack's narrower outer sits inside the wider
				// one by construction, which no audit re-proves. A motion that
				// rounds could move the two outers apart by its own rounding,
				// so only an exact motion keeps the frame; the rest copy.
				return nil, true, nil
			}
		}
		interfaces, err := stackedrecord.Derive(ctx, slabs, p.Interfaces)
		if err != nil {
			return nil, false, err
		}
		p.Slabs, p.Interfaces = slabs, interfaces
		p.SectionDelta = patternrecord.WithDelta(p.SectionDelta, delta)
		body, err := stackedrecord.Build(ctx, env, p, stackedrecord.OwnPlan{})
		return body, false, err
	default:
		return nil, false, fmt.Errorf(`%w: a %T receiver has no frame-keeping pattern instance`,
			decaderr.ErrUnsupported, payload)
	}
}

// Group applies §6.1's rules 1 and 2 on the frame-keeping arm. For a prism
// or a prism group it moves every region into every instance (§6.2) and asks
// prismcells.ProveGroupDisjoint whether sketch's structural read proves the
// outers pairwise disjoint; proven, the group is the one-slab stacked prism
// over all of them (ok), and not proven is a silent miss to rule 3. A
// multi-slab stack proven disjoint the same way, over its one outer loop, is
// rule 2's ErrUnsupported. A union-built stack, whose outer changes between
// slabs, is not tried.
//
// The group's section displacement is the receiver's own plus the largest
// instance δ_pattern, and at least the disjointness scene's walk charge, as a
// group Union's is (prism_group.go).
func Group(ctx context.Context, rp patternrecord.Spec, payload topo.Payload) (stackedrecord.Payload, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	frame, xform, _, _ := frameOf(payload)
	var regions []momentinput.Profile
	var slab stackedrecord.Slab
	var sectionDelta float64
	multiSlab := false
	switch p := payload.(type) {
	case prismrecord.Payload:
		regions = []momentinput.Profile{p.Profile}
		slab = stackedrecord.Slab{Z0: p.Z0, Z1: p.Z1, Z0Delta: p.Z0Delta, Z1Delta: p.Z1Delta}
		sectionDelta = p.SectionDelta
	case stackedrecord.Payload:
		if p.IsGroup() {
			regions = p.Slabs[0].Regions
			slab = p.Slabs[0]
			sectionDelta = p.SectionDelta
			break
		}
		runs := p.OuterRuns()
		if len(runs) != 1 {
			return stackedrecord.Payload{}, false, nil
		}
		regions = []momentinput.Profile{{Outer: p.Slabs[0].Regions[0].Outer}}
		multiSlab = true
	default:
		return stackedrecord.Payload{}, false, nil
	}
	all := append([]momentinput.Profile(nil), regions...)
	delta := 0.0
	for i := 1; i < rp.Count; i++ {
		mv, err := rp.Motion(frame, xform, i)
		if err != nil {
			return stackedrecord.Payload{}, false, err
		}
		for _, region := range regions {
			moved, charge, err := moveRegion(budget, region, mv)
			if err != nil {
				return stackedrecord.Payload{}, false, err
			}
			all = append(all, moved)
			delta = math.Max(delta, charge)
		}
	}
	disjoint, walk, err := prismcells.ProveGroupDisjoint(ctx, budget, all)
	if err != nil || !disjoint {
		return stackedrecord.Payload{}, false, err
	}
	if multiSlab {
		return stackedrecord.Payload{}, false, fmt.Errorf(`%w: the pattern's instances are disjoint stacked prisms, and no payload holds several lumps of a multi-slab stack; use PatternCopies to keep them as separate bodies`, decaderr.ErrUnsupported)
	}
	slab.Regions = all
	return stackedrecord.Payload{
		Slabs: []stackedrecord.Slab{slab},
		Frame: frame, Xform: xform,
		SectionDelta: math.Max(patternrecord.WithDelta(sectionDelta, delta), walk),
	}, true, nil
}
