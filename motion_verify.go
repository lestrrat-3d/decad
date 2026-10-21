package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/motionengine"
	"github.com/lestrrat-3d/decad/internal/motionoption"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/report"
)

// This file is Document.VerifyMotion's gate (agent-docs/motion-check-design.md §3,
// §5-§8): it validates the moving set, the motion and the options and hands
// the nodes to internal/motionengine, the engine VerifyLinkage and
// VerifyJointBox share (agent-docs/linkage-check-design.md §6).

// resolveMovers validates the moving set (agent-docs/motion-check-design.md §3, §8).
func (d *Document) resolveMovers(moving []*Body) error {
	if len(moving) == 0 {
		return fmt.Errorf(`%w: an empty moving set moves nothing`, ErrDegenerate)
	}
	seen := make(map[*Body]struct{}, len(moving))
	for _, b := range moving {
		if err := d.requireLive(b); err != nil {
			return err
		}
		if _, dup := seen[b]; dup {
			return fmt.Errorf(`%w: a body is listed twice in the moving set`, ErrDegenerate)
		}
		seen[b] = struct{}{}
		if b.n.Payload == nil {
			return fmt.Errorf(`%w: this evaluator cannot move a body it did not build`, ErrUnsupported)
		}
	}
	return nil
}

// VerifyMotion checks whether the rigid set moving, carried along m, meets
// any other live body of d anywhere on the path (agent-docs/motion-check-design.md).
// It never changes the document: each pose re-evaluates a mover's payload
// under the composed motion as a transient body that is never committed, so
// Document.Bodies(), every body's live state and the next producer identity
// are what they were before the call.
//
// Every pair result at a pose is Verify's own row or diagnostic. Between two
// adjacent poses the interval certificate (§5.2) proves the path clear only
// when every (mover, static) pair's two proven lower bounds together exceed
// the farthest any mover point can travel across the interval; an interval it
// cannot certify down to the resolution is IntervalUndecided and the report
// reads Suspect, never Sound. A pose publishes a Collision only for an overlap
// that survives the pose's own deviation from the ideal motion (§5.1).
//
// moving MUST be non-empty, hold live bodies of d, and list no body twice.
// A nil context returns ErrDegenerate before validation. Other validation
// errors precede cancellation; after validation a canceled context returns
// ctx.Err() and no report.
func (d *Document) VerifyMotion(ctx context.Context, moving []*Body, m Motion, opts ...MotionOption) (*MotionReport, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control motion verification`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if err := d.resolveMovers(moving); err != nil {
		return nil, err
	}
	spec, err := resolveMotion(m)
	if err != nil {
		return nil, err
	}
	encodedOpts, err := decodeMotionOptions(opts)
	if err != nil {
		return nil, err
	}
	cfg, err := motionoption.Resolve(encodedOpts, spec.Domain)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	run := motionengine.NewRun(ctx, motionHooks(), &d.n, d.n.Live(), cfg)
	run.Spec = spec
	run.Setup(bodyNodes(moving))
	poses, spans, err := run.Refine()
	if err != nil {
		return nil, err
	}
	return report.LiftMotionReport(&verifyLift, run.Publish(poses, spans), m), nil
}

// motionHooks are the services the motion engine takes from the document: the
// handle allocator, the public Booleans' operand mesher, payload placement in
// a transient document, and that document's constructor.
func motionHooks() motionengine.Hooks {
	return motionengine.Hooks{
		Alloc:   handleAlloc{},
		Operand: pairMeshes{},
		Place:   placePayloadNode,
		NewDocument: func(doc *topo.Document, live []*topo.Body) *topo.Document {
			return &bornDocument(&Document{n: topo.Document{
				NextLevel:    doc.NextLevel,
				NextCurve:    doc.NextCurve,
				NextProducer: doc.NextProducer,
				LiveSet:      live,
			}}).n
		},
	}
}
