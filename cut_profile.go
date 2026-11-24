package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/sketch"
)

// CutExtrude removes from target the volume that sweeping profiles of s along
// the sketch normal per e would fill, and returns the result, retiring target
// (agent-docs/cut-profile-design.md). It is the feature form of Extrude
// followed by Cut: the swept tool exists only inside the call, is never
// registered, and leaves no body in the document.
//
// Every profile obeys Extrude's gates and options, and the extent resolves
// exactly as Extrude resolves it, so a ToFace or ThroughAll stop reads live
// bodies, target among them. Several profiles are one feature: their tools are
// united with one another and target is cut once. Profiles whose tools touch
// or overlap are therefore removed as one volume.
//
// target MUST be a live solid. A nil context, target or sketch, or an empty
// or nil profile list, is ErrDegenerate. A tool that removes everything is
// ErrBooleanFailed, and every other refusal is Extrude's or Cut's own. Every
// failure leaves target live and the document unchanged. The history records
// one OperationCutExtrude whose consumed input is target.
func CutExtrude(ctx context.Context, target *Body, s *sketch.Sketch, profiles []*sketch.Profile, e Extent,
	opts ...ExtrudeOption) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationCutExtrude, retErr) }()
	return cutProfiles(ctx, OperationCutExtrude, target, s, profiles,
		func(toolCtx context.Context, d *Document, p *sketch.Profile) (*Body, error) {
			return Extrude(toolCtx, d, s, p, e, opts...)
		})
}

// CutRevolve removes from target the volume that revolving profiles of s about
// axis through a would fill, and returns the result, retiring target
// (agent-docs/cut-profile-design.md). It is the feature form of Revolve
// followed by Cut, with the gates, grouping and failure behaviour of
// [CutExtrude]. The history records one OperationCutRevolve.
func CutRevolve(ctx context.Context, target *Body, s *sketch.Sketch, profiles []*sketch.Profile, axis Axis,
	a AngularExtent, opts ...RevolveOption) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationCutRevolve, retErr) }()
	return cutProfiles(ctx, OperationCutRevolve, target, s, profiles,
		func(toolCtx context.Context, d *Document, p *sketch.Profile) (*Body, error) {
			return Revolve(toolCtx, d, s, p, axis, a, opts...)
		})
}

// cutProfiles builds one private tool per profile with build and cuts target
// by their union in one commit. It stays in root because it reads the body's
// private document handle and owns the private tool bodies' registration and
// removal; the cut itself is combineTools.
func cutProfiles(ctx context.Context, kind OperationKind, target *Body, s *sketch.Sketch,
	profiles []*sketch.Profile, build func(context.Context, *Document, *sketch.Profile) (*Body, error)) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a cut`, ErrDegenerate)
	}
	if target == nil || target.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	if s == nil {
		return nil, fmt.Errorf(`%w: a profile cut requires a non-nil sketch`, ErrDegenerate)
	}
	if len(profiles) == 0 {
		return nil, fmt.Errorf(`%w: a profile cut needs at least one profile`, ErrDegenerate)
	}
	d := documentHandle(target.n.Doc)
	if err := d.requireLive(target); err != nil {
		return nil, err
	}
	if target.Kind() != BodySolid {
		return nil, fmt.Errorf(`%w: a profile cut needs a solid target`, ErrNotSolid)
	}
	toolCtx, capture := privateConstructionContext(ctx)
	var tools []*Body
	defer func() { removeTools(tools) }()
	for _, p := range profiles {
		if p == nil {
			return nil, fmt.Errorf(`%w: a profile cut was given a nil profile`, ErrDegenerate)
		}
		tool, err := build(toolCtx, d, p)
		if err != nil {
			return nil, err
		}
		tools = append(tools, tool)
	}
	return combineTools(ctx, d, kind, target, meshbool.OpCut, tools, len(tools) > 1, nil, capture)
}
