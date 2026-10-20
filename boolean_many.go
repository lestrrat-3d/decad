package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismboolean"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// performUnionMany validates every input before evaluation and commits the
// single result only after its complete proof and cancellation check. The
// certified pattern evaluators construct all tools in one operation.
func performUnionMany(ctx context.Context, operands []*Body) (*Body, error) {
	a := operands[0]
	if a == nil || a.n.Doc == nil {
		return nil, fmt.Errorf("%w: the first operand belongs to no document", ErrDegenerate)
	}
	d := documentHandle(a.n.Doc)
	seen := make(map[*Body]struct{}, len(operands))
	for _, operand := range operands {
		if err := d.requireLive(operand); err != nil {
			return nil, err
		}
		if _, duplicate := seen[operand]; duplicate {
			return nil, fmt.Errorf("%w: Union needs distinct bodies", ErrDegenerate)
		}
		seen[operand] = struct{}{}
	}
	for _, operand := range operands {
		if err := refuseSheetOperand(operand, "Union"); err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ref := d.nextProducerID()
	if body, matched, err := tryPrismUnionMany(ctx, d, ref, operands); matched || err != nil {
		if err != nil {
			return nil, err
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		d.commit(body, operands...)
		return body, nil
	}
	tools := operands[1:]
	node, matched, err := loftcylinder.MirrorPairs(ctx, envOf(documentNode(d), ref), verifyAllMesher{},
		bodyNode(a), bodyNodes(tools), nil)
	body := bodyHandle(node)
	if !matched && err == nil {
		body, matched, err = tryRevolveLoftSectorUnionMany(ctx, d, ref, operands)
	}
	if !matched && err == nil {
		body, matched, err = tryPrismPatternUnionMany(ctx, d, ref, operands)
	}
	if !matched && err == nil {
		body, matched, err = tryStraightLoftPrismUnionMany(ctx, d, ref, operands)
	}
	span := producerID(1)
	if !matched && err == nil {
		body, span, matched, err = tryPrismUnionChain(ctx, ref, operands)
	}
	if !matched && err == nil {
		body, err = spiralCombineBody(ctx, d, ref, a, tools)
	}
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commitSpan(body, span, operands...)
	return body, nil
}

// tryStraightLoftPrismUnionMany converts only authenticated straight-source
// Lofts before asking the existing prism evaluator to arrange one scene. Each
// Loft must overlap the first prism over positive area, so a tangent contact
// cannot enter this route merely because another operand joins the scene.
func tryStraightLoftPrismUnionMany(ctx context.Context, d *Document, ref producerID,
	operands []*Body) (*Body, bool, error) {
	if _, ok := operands[0].n.Payload.(prismPayload); !ok {
		return nil, false, nil
	}
	converted := make([]*Body, len(operands))
	converted[0] = operands[0]
	lofts := 0
	for i, operand := range operands[1:] {
		if err := ctx.Err(); err != nil {
			return nil, true, err
		}
		if _, ok := operand.n.Payload.(prismPayload); ok {
			converted[i+1] = operand
			continue
		}
		loft, ok := operand.n.Payload.(loftPayload)
		if !ok {
			return nil, false, nil
		}
		prism, ok, err := loftrecord.ExactAxialPrism(ctx, loft)
		if err != nil || !ok {
			return nil, false, err
		}
		converted[i+1] = bornBody(&Body{n: topo.Body{Payload: prism}})
		_, overlap, err := prismboolean.Try(ctx, meshbool.OpIntersect, converted[0].n.Payload, prism)
		if err != nil || !overlap {
			return nil, false, err
		}
		lofts++
	}
	if lofts == 0 {
		return nil, false, nil
	}
	return tryPrismUnionMany(ctx, d, ref, converted)
}

// tryPrismUnionChain gives prism inputs and exact straight-source Loft inputs
// the same pair evaluator as two-body Union. Every intermediate body is
// transient. The caller commits only the final certified result and reserves
// every producer identity used by its intermediate features.
func tryPrismUnionChain(ctx context.Context, ref producerID, operands []*Body) (*Body, producerID, bool, error) {
	firstPrism := false
	for i, operand := range operands {
		if _, ok := operand.n.Payload.(prismPayload); ok {
			if i < 2 {
				firstPrism = true
			}
			continue
		}
		loft, ok := operand.n.Payload.(loftPayload)
		if !ok {
			return nil, 0, false, nil
		}
		if _, ok, err := loftrecord.ExactAxialPrism(ctx, loft); err != nil {
			return nil, 0, true, err
		} else if !ok {
			return nil, 0, false, nil
		}
	}
	if !firstPrism {
		return nil, 0, false, nil
	}
	result := operands[0]
	for i, operand := range operands[1:] {
		if err := ctx.Err(); err != nil {
			return nil, 0, true, err
		}
		var err error
		result, err = booleanBody(ctx, meshbool.OpUnion, result, operand, ref+producerID(i))
		if err != nil {
			return nil, 0, true, err
		}
	}
	return result, producerID(len(operands) - 1), true, nil
}

// tryPrismUnionMany arranges all sections in one Sketch scene. Its entry
// admits whole-line, hole-free prisms on the identical recorded plane and
// interval. These records have no incoming section displacement, so the
// result charges only the arrangement's new cut parameters.
func tryPrismUnionMany(ctx context.Context, d *Document, ref producerID,
	operands []*Body) (*Body, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, true, err
	}
	profiles := make([]profileRecord, len(operands))
	type sourceLine struct {
		owner int
		line  lineSeg
	}
	var lines []sourceLine
	var first prismPayload
	z0Delta, z1Delta := 0.0, 0.0
	for i, operand := range operands {
		if err := budget.Step(); err != nil {
			return nil, true, err
		}
		p, ok := operand.n.Payload.(prismPayload)
		if !ok {
			return nil, false, nil
		}
		if i == 0 {
			first = p
		} else if p.Frame != first.Frame || p.Xform != first.Xform ||
			p.Z0 != first.Z0 || p.Z1 != first.Z1 {
			return nil, false, nil
		}
		if p.SectionDelta != 0 || len(p.Profile.Holes) != 0 || len(p.Profile.Outer.Segments) == 0 {
			return nil, false, nil
		}
		for _, segment := range p.Profile.Outer.Segments {
			if err := budget.Step(); err != nil {
				return nil, true, err
			}
			line, ok := segment.(lineSeg)
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
		return nil, true, fmt.Errorf("%w: Union's one-scene prism arrangement needs %d segments, above its %d-segment cap",
			ErrUnsupported, segments, prismcells.MaxArrangementSegments)
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
					ErrUnsupported)
			}
		}
	}
	s, tags, charge, err := prismcells.BuildSceneRegions(budget, prismcells.SceneProfiles(profiles), nil,
		&prismcells.Reexpression{Identity: true})
	if err != nil {
		return nil, true, err
	}
	if charge.A != 0 || charge.B != 0 {
		return nil, true, fmt.Errorf("%w: Union's prism scene moved a source boundary", ErrUnsupported)
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
		return nil, true, fmt.Errorf("%w: Union's prism scene has no resolved material cells", ErrUnsupported)
	}
	voidFree, err := prismcells.CellsHaveNoVoid(budget, tags, cells)
	if err != nil {
		return nil, true, err
	}
	if !voidFree {
		return nil, true, fmt.Errorf("%w: Union's prism scene cannot classify an enclosed cell", ErrUnsupported)
	}
	loops, cutDelta, resolved, err := prismcells.MergeLoops(budget, cells, "union")
	if err != nil {
		return nil, true, err
	}
	if !resolved || len(loops) == 0 {
		return nil, true, fmt.Errorf("%w: Union's prism boundary does not close", ErrUnsupported)
	}
	regions := make([]profileRecord, len(loops))
	for i, loop := range loops {
		regions[i] = profileRecord{Outer: loop}
		if err := prismcells.AuditMergeSection(budget, first, regions[i]); err != nil {
			return nil, true, err
		}
	}
	if len(regions) == 1 {
		p := prismrecord.Payload{Profile: regions[0], Frame: first.Frame, Xform: first.Xform,
			Z0: first.Z0, Z1: first.Z1, Z0Delta: z0Delta, Z1Delta: z1Delta,
			SectionDelta: cutDelta}
		body, err := analyticBooleanBody(ctx, d, ref, p)
		return body, true, err
	}
	disjoint, disjointWalk, err := prismcells.ProveGroupDisjoint(ctx, budget, regions)
	if err != nil {
		return nil, true, err
	}
	if !disjoint {
		return nil, true, fmt.Errorf("%w: Union's prism regions are not proven disjoint", ErrUnsupported)
	}
	sp := stackedPrismPayload{
		Slabs: []stackedrecord.Slab{{Regions: regions, Z0: first.Z0, Z1: first.Z1,
			Z0Delta: z0Delta, Z1Delta: z1Delta}},
		Frame: first.Frame, Xform: first.Xform,
		SectionDelta: proofbound.AbsSumUpper(disjointWalk, cutDelta),
	}
	if err := stackedrecord.Falsify(ctx, stackedrecord.Record{Slabs: sp.Slabs, Interfaces: sp.Interfaces}); err != nil {
		return nil, true, err
	}
	body, err := analyticBooleanBody(ctx, d, ref, sp)
	return body, true, err
}
