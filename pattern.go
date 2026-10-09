package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/patternrecord"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file is PatternCopies (docs/mirror-pattern-design.md §4.3, §6.2):
// repeating a body along a line or about an axis, each instance a new live
// body. Where the receiver is a straight or stacked prism and the motion
// keeps it co-directional, an instance keeps the receiver's frame and
// placement and moves its RECORD in the plane, with the motion's rounding
// charged into its section displacement; every other instance is a
// PlacedCopy under the composed rigid motion.

// PatternSpec is how a pattern lays out its instances: a [LinearPattern] or
// a [CircularPattern]. The set is sealed.
type PatternSpec = patternrecord.PatternSpec

// LinearPattern places instance i at i·Step along Dir, i = 0..Count-1.
// Instance 0 is the receiver itself. Dir is a direction (dimensionless,
// non-zero, not normalised by the caller); Step is a length magnitude, and
// its sense is Dir's.
type LinearPattern = patternrecord.LinearPattern

// CircularPattern places instance i rotated by i/Count of a turn, right-handed
// about the axis through Center along Axis, i = 0..Count-1. The step angle is
// denoted by Count, never by a float, so a quarter turn is exactly a quarter
// turn. Center need not lie on the receiver's own sweep axis.
type CircularPattern = patternrecord.CircularPattern

// PatternCopies returns Count−1 new live bodies, instances 1..Count−1 of
// spec in order, and leaves the receiver live (docs/mirror-pattern-design.md
// §4.3).
//
// The gates are PlacedCopy's — a nil context is ErrDegenerate, a body this
// evaluator did not build is ErrUnsupported, a retired or foreign receiver is
// refused — plus the spec's own: a nil spec or a Count below 2 is
// ErrDegenerate; a non-finite Dir, Axis or Center is ErrNotFinite and a zero
// Dir or Axis ErrDegenerate; a Step that is not a length is ErrUnitKind, a
// negative one ErrNegativeMagnitude and a zero one ErrDegenerate. Every
// instance is built before any is registered, so a refusal or a canceled
// context leaves the document unchanged.
//
// A straight or stacked prism patterned along a direction exactly
// perpendicular to its sweep, or about an axis bit-identical to its sweep
// normal, keeps one frame: each instance is the receiver's own record moved
// in its plane, so a pattern of integer-millimetre holes stays Exact. Its
// readings carry the motion's rounding as a section displacement.
func (b *Body) PatternCopies(ctx context.Context, spec PatternSpec) ([]*Body, error) {
	d, rp, keeping, err := b.patternReceiver(ctx, spec)
	if err != nil {
		return nil, err
	}
	out, err := rp.buildInstances(ctx, d, d.nextProducerID(), b.payload, keeping)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commitMany(out)
	return out, nil
}

// patternReceiver runs the gates PatternCopies and Patterned share, then
// decides §4.3's frame-keeping arm for the receiver.
func (b *Body) patternReceiver(ctx context.Context, spec PatternSpec) (*Document, resolvedPattern, bool, error) {
	if ctx == nil {
		return nil, resolvedPattern{}, false, fmt.Errorf(`%w: a nil context cannot control a pattern`, ErrDegenerate)
	}
	if b == nil || b.doc == nil {
		return nil, resolvedPattern{}, false, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := b.doc
	if err := d.requireLive(b); err != nil {
		return nil, resolvedPattern{}, false, err
	}
	resolved, err := patternrecord.Resolve(spec)
	if err != nil {
		return nil, resolvedPattern{}, false, err
	}
	rp := resolvedPattern{Spec: resolved}
	if b.payload == nil {
		return nil, resolvedPattern{}, false, fmt.Errorf(`%w: this evaluator cannot copy a body it did not build`, ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return nil, resolvedPattern{}, false, err
	}
	keeping, err := rp.keepsFrame(proofbound.NewWorkBudget(ctx), b.payload)
	if err != nil {
		return nil, resolvedPattern{}, false, err
	}
	return d, rp, keeping, nil
}

// buildInstances builds instances 1..Count−1 under the producer identities
// base, base+1, ..., registering none of them.
func (rp resolvedPattern) buildInstances(ctx context.Context, d *Document, base producerID, payload featurePayload, keeping bool) ([]*Body, error) {
	out := make([]*Body, 0, rp.Count-1)
	for i := 1; i < rp.Count; i++ {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		ref := base + producerID(i-1)
		var body *Body
		var err error
		if keeping {
			body, err = rp.frameKeepingInstance(ctx, d, ref, payload, i)
		} else {
			body, err = rp.placedInstance(ctx, d, ref, payload, i)
		}
		if err != nil {
			return nil, err
		}
		out = append(out, body)
	}
	return out, nil
}

// resolvedPattern adds body construction to a validated neutral pattern spec.
type resolvedPattern struct{ patternrecord.Spec }

// placedInstance is the PlacedCopy arm: the payload re-evaluated under its
// placement composed with instance i's motion.
func (rp resolvedPattern) placedInstance(ctx context.Context, d *Document, ref producerID, payload featurePayload, i int) (*Body, error) {
	m, err := rp.WorldMotion(i)
	if err != nil {
		return nil, fmt.Errorf(`%w: pattern instance %d has no rigid motion: %s`, ErrDegenerate, i, err)
	}
	composed, err := payload.transform().Then(m)
	if err != nil {
		return nil, fmt.Errorf(`decad: composing the placement failed: %w`, err)
	}
	return payload.placed(ctx, d, ref, composed)
}

// patternFrame is the frame and placement a frame-keeping instance shares
// with its receiver, and the regions it moves.
func patternFrameOf(payload featurePayload) (r3.Frame, r3.Transform, []profileRecord, bool) {
	switch p := payload.(type) {
	case prismPayload:
		return p.frame, p.xform, []profileRecord{p.profile}, true
	case stackedPrismPayload:
		regions := make([]profileRecord, 0, len(p.slabs))
		for _, slab := range p.slabs {
			regions = append(regions, slab.regions...)
		}
		return p.frame, p.xform, regions, true
	default:
		return r3.Frame{}, r3.Transform{}, nil, false
	}
}

// keepsFrame decides §4.3's frame-keeping arm: a prism or stacked receiver
// whose every segment is a line, arc or circle with no trimmed arc or circle,
// under a co-directional motion. Co-direction is read on the composed world
// normal xform.ApplyDir(frame.N()), as prism-boolean G3 reads it: a linear
// Dir whose exact rational dot product with that normal is zero, or a
// circular Axis bit-identical to ±that normal.
func (rp resolvedPattern) keepsFrame(budget *proofbound.WorkBudget, payload featurePayload) (bool, error) {
	frame, xform, regions, ok := patternFrameOf(payload)
	if !ok {
		return false, nil
	}
	for _, region := range regions {
		for _, loop := range append([]loopRecord{region.Outer}, region.Holes...) {
			for _, seg := range loop.Segments {
				switch seg.(type) {
				case lineSeg, arcSeg, circleSeg:
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

// pointMotion is the plane-local mapping used by the root payload builders.
type pointMotion = patternrecord.PointMotion

func moveRegion(budget *proofbound.WorkBudget, region profileRecord, mv pointMotion) (profileRecord, float64, error) {
	moved, delta, err := patternrecord.MoveRegion(budget,
		patternrecord.Region{Outer: region.Outer, Holes: region.Holes}, mv)
	if err != nil {
		return profileRecord{}, 0, err
	}
	return profileRecord{Outer: moved.Outer, Holes: moved.Holes}, delta, nil
}

// frameKeepingInstance builds instance i of the frame-keeping arm: the
// receiver's own frame, placement, interval and axial displacements over its
// record moved in the plane, charged δ_pattern into the section displacement.
func (rp resolvedPattern) frameKeepingInstance(ctx context.Context, d *Document, ref producerID, payload featurePayload, i int) (*Body, error) {
	budget := proofbound.NewWorkBudget(ctx)
	frame, xform, _, _ := patternFrameOf(payload)
	mv, err := rp.Motion(frame, xform, i)
	if err != nil {
		return nil, err
	}
	switch p := payload.(type) {
	case prismPayload:
		moved, delta, err := moveRegion(budget, p.profile, mv)
		if err != nil {
			return nil, err
		}
		p.profile = moved
		p.sectionDelta = patternrecord.WithDelta(p.sectionDelta, delta)
		p.walks = nil
		return evalPrismContext(ctx, d, ref, p, freeform.NewFreeformWork())
	case stackedPrismPayload:
		slabs := make([]prismSlab, len(p.slabs))
		delta := 0.0
		for k, slab := range p.slabs {
			slabs[k] = slab
			slabs[k].regions = make([]profileRecord, len(slab.regions))
			for r, region := range slab.regions {
				moved, charge, err := moveRegion(budget, region, mv)
				if err != nil {
					return nil, err
				}
				slabs[k].regions[r] = moved
				delta = math.Max(delta, charge)
			}
		}
		if delta != 0 {
			runs := p.outerRuns()
			if len(runs) != 1 {
				// A union-built stack's narrower outer sits inside the wider
				// one by construction, which no audit re-proves. A motion that
				// rounds could move the two outers apart by its own rounding,
				// so only an exact motion keeps the frame; the rest copy.
				return rp.placedInstance(ctx, d, ref, payload, i)
			}
		}
		interfaces, err := stackedInterfaces(ctx, slabs, p.interfaces)
		if err != nil {
			return nil, err
		}
		p.slabs, p.interfaces = slabs, interfaces
		p.sectionDelta = patternrecord.WithDelta(p.sectionDelta, delta)
		return evalStackedContext(ctx, d, ref, p)
	default:
		return nil, fmt.Errorf(`%w: a %T receiver has no frame-keeping pattern instance`, ErrUnsupported, payload)
	}
}

// Patterned returns ONE body holding every instance of spec, the receiver
// included, and retires the receiver (docs/mirror-pattern-design.md §4.3,
// §6.1). Its gates are PatternCopies'. The instances combine by the first
// rule that applies:
//
//  1. A straight prism or a prism group on the frame-keeping arm whose
//     instances sketch proves pairwise disjoint becomes one prism group: a
//     one-slab stacked prism holding every instance's regions on the
//     receiver's frame and interval. No boolean runs.
//  2. Any other receiver whose instances are proven disjoint is
//     ErrUnsupported: no payload holds disjoint lumps of it. Use
//     PatternCopies and keep the instances as separate bodies.
//  3. Otherwise the instances combine by Union in index order,
//     Union(Union(b0, b1), b2) …, each with that boolean's own gates and
//     refusals, and a refusal is returned as that Union's error.
//
// Every instance and every intermediate result is built before anything is
// registered, so a refusal or a canceled context leaves the document
// unchanged and the receiver live.
func (b *Body) Patterned(ctx context.Context, spec PatternSpec) (*Body, error) {
	d, rp, keeping, err := b.patternReceiver(ctx, spec)
	if err != nil {
		return nil, err
	}
	base := d.nextProducerID()
	if keeping && b.Kind() != BodySheet {
		sp, ok, err := rp.patternGroup(ctx, b.payload)
		if err != nil {
			return nil, err
		}
		if ok {
			body, err := evalStackedContext(ctx, d, base, sp)
			if err != nil {
				return nil, err
			}
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			d.commitSpan(body, 1, b)
			return body, nil
		}
	}
	instances, err := rp.buildInstances(ctx, d, base, b.payload, keeping)
	if err != nil {
		return nil, err
	}
	ref := base + producerID(len(instances))
	acc := b
	for _, inst := range instances {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		acc, err = booleanBody(ctx, meshbool.OpUnion, acc, inst, ref)
		if err != nil {
			return nil, err
		}
		ref++
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commitSpan(acc, ref-base, b)
	return acc, nil
}

// patternGroup is §6.1's rules 1 and 2 on the frame-keeping arm. For a prism
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
func (rp resolvedPattern) patternGroup(ctx context.Context, payload featurePayload) (stackedPrismPayload, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	frame, xform, _, _ := patternFrameOf(payload)
	var regions []profileRecord
	var slab prismSlab
	var sectionDelta float64
	multiSlab := false
	switch p := payload.(type) {
	case prismPayload:
		regions = []profileRecord{p.profile}
		slab = prismSlab{z0: p.z0, z1: p.z1, z0Delta: p.z0Delta, z1Delta: p.z1Delta}
		sectionDelta = p.sectionDelta
	case stackedPrismPayload:
		if p.isGroup() {
			regions = p.slabs[0].regions
			slab = p.slabs[0]
			sectionDelta = p.sectionDelta
			break
		}
		runs := p.outerRuns()
		if len(runs) != 1 {
			return stackedPrismPayload{}, false, nil
		}
		regions = []profileRecord{{Outer: p.slabs[0].regions[0].Outer}}
		multiSlab = true
	default:
		return stackedPrismPayload{}, false, nil
	}
	all := append([]profileRecord(nil), regions...)
	delta := 0.0
	for i := 1; i < rp.Count; i++ {
		mv, err := rp.Motion(frame, xform, i)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		for _, region := range regions {
			moved, charge, err := moveRegion(budget, region, mv)
			if err != nil {
				return stackedPrismPayload{}, false, err
			}
			all = append(all, moved)
			delta = math.Max(delta, charge)
		}
	}
	disjoint, walk, err := prismcells.ProveGroupDisjoint(ctx, budget, all)
	if err != nil || !disjoint {
		return stackedPrismPayload{}, false, err
	}
	if multiSlab {
		return stackedPrismPayload{}, false, fmt.Errorf(`%w: the pattern's instances are disjoint stacked prisms, and no payload holds several lumps of a multi-slab stack; use PatternCopies to keep them as separate bodies`, ErrUnsupported)
	}
	slab.regions = all
	return stackedPrismPayload{
		slabs: []prismSlab{slab},
		frame: frame, xform: xform,
		sectionDelta: math.Max(patternrecord.WithDelta(sectionDelta, delta), walk),
	}, true, nil
}
