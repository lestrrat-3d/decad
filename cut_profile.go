package decad

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
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
func CutExtrude(target *Body, s *sketch.Sketch, profiles []*sketch.Profile, e Extent) *CutExtrudeCommand {
	c := &CutExtrudeCommand{target: target, s: s, profiles: slices.Clone(profiles), e: copyExtent(e)}
	if target == nil {
		c.state.Record("target", errNilBody)
	}
	return c
}

// CutExtrudeCommand removes the volume a profile extrude would fill. Build it
// with [CutExtrude]. It has no SurfaceResult: a cut has no sheet form.
type CutExtrudeCommand struct {
	state    opcommand.State             // consumed flag and first recorded fault
	target   *Body                       // stored as given
	s        *sketch.Sketch              // stored as given
	profiles []*sketch.Profile           // copied at CutExtrude
	e        Extent                      // copied at CutExtrude, with the selector of a ToFace stop
	taper    opcommand.Slot[units.Value] // set by Taper
}

// Taper leans the tool's walls as [ExtrudeCommand.Taper] does for each
// profile. A second call replaces the first.
func (c *CutExtrudeCommand) Taper(a units.Value) *CutExtrudeCommand {
	c.taper.Set(a, featureoption.ValidateTaper(a))
	return c
}

// Do cuts the target and retires it, or returns the error of the first gate
// that refuses, leaving the target live and the document unchanged. A nil ctx
// is ErrDegenerate, and a second call returns ErrConsumed.
func (c *CutExtrudeCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationCutExtrude, c.run)
}

func (c *CutExtrudeCommand) run(ctx context.Context) (*Body, error) {
	st := extrudeSettings{taper: c.taper, taperSubject: "Taper"}
	return cutProfiles(ctx, OperationCutExtrude, c.target, c.s, c.profiles,
		func(toolCtx context.Context, d *Document, p *sketch.Profile) (*Body, error) {
			return extrudeTool(toolCtx, d, c.s, p, c.e, st)
		})
}

// CutRevolve removes from target the volume that revolving profiles of s about
// axis through a would fill, and returns the result, retiring target
// (agent-docs/cut-profile-design.md). It is the feature form of Revolve
// followed by Cut, with the gates, grouping and failure behaviour of
// [CutExtrude]. The history records one OperationCutRevolve. It has no
// SurfaceResult: a cut has no sheet form.
func CutRevolve(target *Body, s *sketch.Sketch, profiles []*sketch.Profile, axis Axis,
	a AngularExtent) *CutRevolveCommand {
	c := &CutRevolveCommand{target: target, s: s, profiles: slices.Clone(profiles), axis: copyAxis(axis),
		a: copyAngularExtent(a)}
	if target == nil {
		c.state.Record("target", errNilBody)
	}
	return c
}

// CutRevolveCommand removes the volume a profile revolve would fill. Build it
// with [CutRevolve]. It has no method.
type CutRevolveCommand struct {
	state    opcommand.State   // consumed flag and first recorded fault
	target   *Body             // stored as given
	s        *sketch.Sketch    // stored as given
	profiles []*sketch.Profile // copied at CutRevolve
	axis     Axis              // copied at CutRevolve, with the selector of an EdgeAxis
	a        AngularExtent     // copied at CutRevolve, with the selector of a ToFaceAngular stop
}

// Do cuts the target and retires it, or returns the error of the first gate
// that refuses, leaving the target live and the document unchanged. A nil ctx
// is ErrDegenerate, and a second call returns ErrConsumed.
func (c *CutRevolveCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationCutRevolve, c.run)
}

func (c *CutRevolveCommand) run(ctx context.Context) (*Body, error) {
	return cutProfiles(ctx, OperationCutRevolve, c.target, c.s, c.profiles,
		func(toolCtx context.Context, d *Document, p *sketch.Profile) (*Body, error) {
			return revolve(toolCtx, d, c.s, p, c.axis, c.a, false)
		})
}

// cutProfiles builds one private tool per profile with build and cuts target
// by their union in one commit. It stays in root because it reads the body's
// private document handle and owns the private tool bodies' registration and
// removal; the cut itself is combineTools.
func cutProfiles(ctx context.Context, kind OperationKind, target *Body, s *sketch.Sketch,
	profiles []*sketch.Profile, build func(context.Context, *Document, *sketch.Profile) (*Body, error)) (*Body, error) {
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
