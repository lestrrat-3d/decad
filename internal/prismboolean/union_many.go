package prismboolean

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// StraightLoftUnionMany converts only authenticated straight-source
// Lofts before asking the existing prism evaluator to arrange one scene. Each
// Loft must overlap the first prism over positive area, so a tangent contact
// cannot enter this route merely because another operand joins the scene.
func StraightLoftUnionMany(ctx context.Context, env topo.Env,
	operands []*topo.Body) (*topo.Body, bool, error) {
	if _, ok := operands[0].Payload.(prismrecord.Payload); !ok {
		return nil, false, nil
	}
	converted := make([]*topo.Body, len(operands))
	converted[0] = operands[0]
	lofts := 0
	for i, operand := range operands[1:] {
		if err := ctx.Err(); err != nil {
			return nil, true, err
		}
		if _, ok := operand.Payload.(prismrecord.Payload); ok {
			converted[i+1] = operand
			continue
		}
		loft, ok := operand.Payload.(loftrecord.Payload)
		if !ok {
			return nil, false, nil
		}
		prism, ok, err := loftrecord.ExactAxialPrism(ctx, loft)
		if err != nil || !ok {
			return nil, false, err
		}
		converted[i+1] = &topo.Body{Payload: prism}
		_, overlap, err := Try(ctx, meshbool.OpIntersect, converted[0].Payload, prism)
		if err != nil || !overlap {
			return nil, false, err
		}
		lofts++
	}
	if lofts == 0 {
		return nil, false, nil
	}
	return UnionMany(ctx, env, converted)
}

// UnionMany arranges all sections in one Sketch scene. Its entry
// admits whole-line, hole-free prisms on the identical recorded plane over
// intervals G5 proves equal (prismplacement.UnionZIntervalMatches). These records have no incoming section displacement, so the
// result charges only the arrangement's new cut parameters.
func UnionMany(ctx context.Context, env topo.Env,
	operands []*topo.Body) (*topo.Body, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, true, err
	}
	profiles := make([]momentinput.Profile, len(operands))
	type sourceLine struct {
		owner int
		line  sectionrecord.LineSeg
	}
	var lines []sourceLine
	var first prismrecord.Payload
	z0Delta, z1Delta := 0.0, 0.0
	for i, operand := range operands {
		if err := budget.Step(); err != nil {
			return nil, true, err
		}
		p, ok := operand.Payload.(prismrecord.Payload)
		if !ok {
			return nil, false, nil
		}
		if i == 0 {
			first = p
		} else if p.Frame != first.Frame || p.Xform != first.Xform ||
			!prismplacement.UnionZIntervalMatches(first.PlacementOperand(), p.PlacementOperand()) {
			return nil, false, nil
		}
		if p.SectionDelta != 0 || len(p.Profile.Holes) != 0 || len(p.Profile.Outer.Segments) == 0 {
			return nil, false, nil
		}
		for _, segment := range p.Profile.Outer.Segments {
			if err := budget.Step(); err != nil {
				return nil, true, err
			}
			line, ok := segment.(sectionrecord.LineSeg)
			if !ok || !prismcells.WholeSegmentRange(line.TStart, line.TEnd) {
				return nil, false, nil
			}
			lines = append(lines, sourceLine{owner: i, line: line})
		}
		profiles[i] = p.Profile
		z0Delta = max(z0Delta, p.Z0Delta)
		z1Delta = max(z1Delta, p.Z1Delta)
	}
	segments, withinCap, err := prismcells.RegionsWithinWorkCap(budget, profiles...)
	if err != nil {
		return nil, true, err
	}
	if !withinCap {
		return nil, true, prismcells.WithWorkLimit(fmt.Errorf("%w: Union's one-scene prism arrangement needs %d segments, above its %d-segment cap",
			decaderr.ErrUnsupported, segments, prismcells.MaxArrangementSegments), segments)
	}
	for i, line := range lines {
		for _, prior := range lines[:i] {
			if prior.owner == line.owner {
				continue
			}
			if err := budget.Step(); err != nil {
				return nil, true, err
			}
			if prismcells.SameLineCarrier(prior.line, line.line) {
				return nil, true, fmt.Errorf("%w: Union's one-scene prism class needs distinct source line carriers",
					decaderr.ErrUnsupported)
			}
		}
	}
	s, tags, charge, err := prismcells.BuildSceneRegions(budget, prismcells.SceneProfiles(profiles), nil,
		&prismcells.Reexpression{Identity: true})
	if err != nil {
		return nil, true, err
	}
	if charge.A != 0 || charge.B != 0 {
		return nil, true, fmt.Errorf("%w: Union's prism scene moved a source boundary", decaderr.ErrUnsupported)
	}
	cells, err := prismcells.ProfilesContext(ctx, s.Profiles)
	if err != nil {
		return nil, true, err
	}
	cells, resolved, err := prismcells.SplitRuns(budget, cells)
	if err != nil {
		return nil, true, err
	}
	if !resolved || len(cells) == 0 {
		return nil, true, fmt.Errorf("%w: Union's prism scene has no resolved material cells", decaderr.ErrUnsupported)
	}
	voidFree, err := prismcells.CellsHaveNoVoid(budget, tags, cells)
	if err != nil {
		return nil, true, err
	}
	if !voidFree {
		return nil, true, fmt.Errorf("%w: Union's prism scene cannot classify an enclosed cell", decaderr.ErrUnsupported)
	}
	loops, cutDelta, resolved, err := prismcells.MergeLoops(budget, cells, "union")
	if err != nil {
		return nil, true, err
	}
	if !resolved || len(loops) == 0 {
		return nil, true, fmt.Errorf("%w: Union's prism boundary does not close", decaderr.ErrUnsupported)
	}
	regions := make([]momentinput.Profile, len(loops))
	for i, loop := range loops {
		regions[i] = momentinput.Profile{Outer: loop}
		if err := prismcells.AuditMergeSection(budget, first, regions[i]); err != nil {
			return nil, true, err
		}
	}
	// Every operand's levels denote first's (UnionZIntervalMatches above), so
	// the shared levels keep them.
	z0Denoted := prismplacement.DenoteLevel(first.DenotedZ0(), first.Z0, z0Delta)
	z1Denoted := prismplacement.DenoteLevel(first.DenotedZ1(), first.Z1, z1Delta)
	if len(regions) == 1 {
		p := prismrecord.Payload{Profile: regions[0], Frame: first.Frame, Xform: first.Xform,
			Z0: first.Z0, Z1: first.Z1, Z0Delta: z0Delta, Z1Delta: z1Delta,
			Z0Denoted: z0Denoted, Z1Denoted: z1Denoted, SectionDelta: cutDelta}
		body, err := AnalyticBody(ctx, env, p)
		return body, true, err
	}
	disjoint, disjointWalk, err := prismcells.ProveGroupApart(ctx, budget, regions)
	if err != nil {
		return nil, true, err
	}
	if !disjoint {
		// Loops that touch at a point, such as a triangle's apex on a square's
		// side, are no prism group (agent-docs/general-boolean-design.md §3 A5).
		// A miss lets the bodies join two at a time, where the mesh path
		// takes the touching pair.
		return nil, false, nil
	}
	sp := stackedrecord.Payload{
		Slabs: []stackedrecord.Slab{{Regions: regions, Z0: first.Z0, Z1: first.Z1,
			Z0Delta: z0Delta, Z1Delta: z1Delta, Z0Denoted: z0Denoted, Z1Denoted: z1Denoted}},
		Frame: first.Frame, Xform: first.Xform,
		SectionDelta: proofbound.AbsSumUpper(disjointWalk, cutDelta),
	}
	if err := stackedrecord.Falsify(ctx, stackedrecord.Record{Slabs: sp.Slabs, Interfaces: sp.Interfaces}); err != nil {
		return nil, true, err
	}
	body, err := AnalyticBody(ctx, env, sp)
	return body, true, err
}

// AnalyticBody evaluates an analytic boolean's result payload under the
// environment's producer identity, without committing it.
func AnalyticBody(ctx context.Context, env topo.Env, payload topo.Payload) (*topo.Body, error) {
	switch p := payload.(type) {
	case prismrecord.Payload:
		return prismrecord.Build(ctx, env, p, freeform.NewFreeformWork())
	case stackedrecord.Payload:
		return stackedrecord.Build(ctx, env, p, stackedrecord.OwnPlan{})
	case breprecord.Payload:
		return breprecord.Build(ctx, env, p)
	default:
		return nil, fmt.Errorf(`%w: an analytic boolean produced a %T payload`, decaderr.ErrUnsupported, payload)
	}
}
