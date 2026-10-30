package revolvebore

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Payload holds the certified source records for a hybrid bore result.
type Payload struct {
	Candidate Candidate
	placement placementBase
}

// placementBase keeps the original host and tool transforms so each
// re-placement derives both source records from the same relative placement.
type placementBase struct {
	revolve r3.Transform
	tool    r3.Transform
	set     bool
}

// Transform returns the accumulated Revolve placement.
func (p Payload) Transform() r3.Transform { return p.Candidate.Revolve.Xform }

// Placed rebuilds the certified records after a rigid placement.
func (p Payload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	base := p.placement
	if !base.set {
		base = placementBase{revolve: p.Candidate.Revolve.Xform, tool: p.Candidate.OriginalTool.Xform, set: true}
	}
	inverse, err := base.revolve.Inverse()
	if err != nil {
		return nil, err
	}
	delta, err := inverse.Then(composed)
	if err != nil {
		return nil, err
	}
	toolPlacement, err := base.tool.Then(delta)
	if err != nil {
		return nil, err
	}
	p.placement = base
	p.Candidate.Revolve.Xform = composed
	p.Candidate.OriginalTool.Xform = toolPlacement
	mapped, ok, err := MapTool(ctx, p.Candidate.Revolve, p.Candidate.Meridian, p.Candidate.OriginalTool)
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, fmt.Errorf("%w: placed bore no longer has an exact shared axis", decaderr.ErrUnsupported)
	}
	p.Candidate.Tool = mapped
	return Build(ctx, env, p)
}

// Build creates fresh topology and bounded scalar readings. Its caller must
// also prove a verification mesh before publishing the result.
func Build(ctx context.Context, env topo.Env, p Payload) (*topo.Body, error) {
	if !p.placement.set {
		p.placement = placementBase{
			revolve: p.Candidate.Revolve.Xform,
			tool:    p.Candidate.OriginalTool.Xform,
			set:     true,
		}
	}
	body, err := revolveplan.Build(ctx, env, p.Candidate.Revolve, freeform.NewFreeformWork())
	if err != nil {
		return nil, err
	}
	if _, err := AddBoundary(ctx, env, body, p.Candidate); err != nil {
		return nil, err
	}
	tool, err := prismrecord.Build(ctx, env, p.Candidate.Tool, freeform.NewFreeformWork())
	if err != nil {
		return nil, err
	}
	if err := ApplyMeasurements(body, tool, p.Candidate.Meridian); err != nil {
		return nil, err
	}
	body.Payload = p
	return body, nil
}
