package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/patternassembly"
	"github.com/lestrrat-3d/decad/internal/patternrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is PatternCopies (agent-docs/mirror-pattern-design.md §4.3, §6.2):
// repeating a body along a line or about an axis, each instance a new live
// body. Where the receiver is a straight or stacked prism and the motion
// keeps it co-directional, an instance keeps the receiver's frame and
// placement and moves its RECORD in the plane, with the motion's rounding
// charged into its section displacement; every other instance is a
// Copy under the composed rigid motion.

// PatternSpec is how a pattern lays out its instances: a [LinearPattern] or
// a [CircularPattern]. The set is sealed.
type PatternSpec interface{ patternSpec() }

// LinearPattern places instance i at i·Step along Dir, i = 0..Count-1.
// Instance 0 is the receiver itself. Dir is a direction (dimensionless,
// non-zero, not normalised by the caller); Step is a length magnitude, and
// its sense is Dir's.
type LinearPattern struct {
	Dir   r3.Vec
	Step  units.Value
	Count int
}

// CircularPattern places instance i rotated by i/Count of a turn, right-handed
// about the axis through Center along Axis, i = 0..Count-1. The step angle is
// denoted by Count, never by a float, so a quarter turn is exactly a quarter
// turn. Center need not lie on the receiver's own sweep axis.
type CircularPattern struct {
	Center, Axis r3.Vec
	Count        int
}

func (LinearPattern) patternSpec()   {}
func (CircularPattern) patternSpec() {}

func resolvePatternSpec(spec PatternSpec) (patternrecord.Spec, error) {
	errNil := fmt.Errorf(`%w: a nil pattern spec names no pattern`, ErrDegenerate)
	switch s := spec.(type) {
	case *LinearPattern:
		if s == nil {
			return patternrecord.Spec{}, errNil
		}
		value := *s
		return patternrecord.ResolveLinear(value.Dir, value.Step, value.Count)
	case *CircularPattern:
		if s == nil {
			return patternrecord.Spec{}, errNil
		}
		value := *s
		return patternrecord.ResolveCircular(value.Center, value.Axis, value.Count)
	case LinearPattern:
		return patternrecord.ResolveLinear(s.Dir, s.Step, s.Count)
	case CircularPattern:
		return patternrecord.ResolveCircular(s.Center, s.Axis, s.Count)
	default:
		return patternrecord.Spec{}, errNil
	}
}

// PatternCopies returns Count−1 new live bodies, instances 1..Count−1 of
// spec in order, and leaves b live (agent-docs/mirror-pattern-design.md §4.3).
//
// The gates are Copy's — a nil context or a nil b is ErrDegenerate, a body
// this evaluator did not build is ErrUnsupported, a retired or foreign b is
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
func PatternCopies(ctx context.Context, b *Body, spec PatternSpec) ([]*Body, error) {
	d, rp, keeping, err := b.patternReceiver(ctx, spec)
	if err != nil {
		return nil, err
	}
	out, err := rp.buildInstances(ctx, d, d.nextProducerID(), b.n.Payload, keeping)
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
	if b == nil || b.n.Doc == nil {
		return nil, resolvedPattern{}, false, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, resolvedPattern{}, false, err
	}
	resolved, err := resolvePatternSpec(spec)
	if err != nil {
		return nil, resolvedPattern{}, false, err
	}
	rp := resolvedPattern{Spec: resolved}
	if b.n.Payload == nil {
		return nil, resolvedPattern{}, false, fmt.Errorf(`%w: this evaluator cannot copy a body it did not build`, ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return nil, resolvedPattern{}, false, err
	}
	keeping, err := patternassembly.KeepsFrame(proofbound.NewWorkBudget(ctx), rp.Spec, b.n.Payload)
	if err != nil {
		return nil, resolvedPattern{}, false, err
	}
	return d, rp, keeping, nil
}

// buildInstances builds instances 1..Count−1 under the producer identities
// base, base+1, ..., registering none of them.
func (rp resolvedPattern) buildInstances(ctx context.Context, d *Document, base producerID, payload topo.Payload, keeping bool) ([]*Body, error) {
	out := make([]*Body, 0, rp.Count-1)
	for i := 1; i < rp.Count; i++ {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		ref := base + producerID(i-1)
		var body *Body
		var err error
		if keeping {
			var node *topo.Body
			var fallback bool
			node, fallback, err = patternassembly.FrameKeepingInstance(ctx,
				envOf(documentNode(d), ref), rp.Spec, payload, i)
			if err != nil {
				return nil, err
			}
			if fallback {
				body, err = rp.placedInstance(ctx, d, ref, payload, i)
			} else {
				body, err = bodyResult(node, nil)
			}
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

// placedInstance is the Copy arm: the payload re-evaluated under its
// placement composed with instance i's motion.
func (rp resolvedPattern) placedInstance(ctx context.Context, d *Document, ref producerID, payload topo.Payload, i int) (*Body, error) {
	m, err := rp.WorldMotion(i)
	if err != nil {
		return nil, fmt.Errorf(`%w: pattern instance %d has no rigid motion: %s`, ErrDegenerate, i, err)
	}
	composed, err := payload.Transform().Then(m)
	if err != nil {
		return nil, fmt.Errorf(`decad: composing the placement failed: %w`, err)
	}
	return placePayload(ctx, payload, d, ref, composed)
}

// Patterned returns ONE body holding every instance of spec, b included,
// and retires b (agent-docs/mirror-pattern-design.md §4.3, §6.1). Its gates are PatternCopies'. The instances combine by the first
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
// unchanged and b live.
func Patterned(ctx context.Context, b *Body, spec PatternSpec) (*Body, error) {
	d, rp, keeping, err := b.patternReceiver(ctx, spec)
	if err != nil {
		return nil, err
	}
	base := d.nextProducerID()
	if keeping && b.Kind() != BodySheet {
		sp, ok, err := patternassembly.Group(ctx, rp.Spec, b.n.Payload)
		if err != nil {
			return nil, err
		}
		if ok {
			body, err := bodyResult(stackedrecord.Build(ctx, envOf(documentNode(d), base), sp, stackedrecord.OwnPlan{}))
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
	instances, err := rp.buildInstances(ctx, d, base, b.n.Payload, keeping)
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
