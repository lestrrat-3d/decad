package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
)

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
			if samePrismCarrier(prior.line, line.line) {
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
		if err := prismcells.AuditMergeSection(budget, first.Payload, regions[i]); err != nil {
			return nil, true, err
		}
	}
	if len(regions) == 1 {
		p := prismPayload{Payload: prismrecord.Payload{Profile: regions[0], Frame: first.Frame, Xform: first.Xform,
			Z0: first.Z0, Z1: first.Z1, Z0Delta: z0Delta, Z1Delta: z1Delta,
			SectionDelta: cutDelta}}
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
		slabs: []stackedrecord.Slab{{Regions: regions, Z0: first.Z0, Z1: first.Z1,
			Z0Delta: z0Delta, Z1Delta: z1Delta}},
		frame: first.Frame, xform: first.Xform,
		sectionDelta: proofbound.AbsSumUpper(disjointWalk, cutDelta),
	}
	if err := stackedrecord.Falsify(ctx, stackedrecord.Record{Slabs: sp.slabs, Interfaces: sp.interfaces}); err != nil {
		return nil, true, err
	}
	body, err := analyticBooleanBody(ctx, d, ref, sp)
	return body, true, err
}

// samePrismCarrier compares the two complete supporting lines in exact
// dyadic arithmetic. This path has no proof for a span shared by inputs.
func samePrismCarrier(a, b lineSeg) bool {
	dx := lfSub(lfF(a.End.U), lfF(a.Start.U))
	dy := lfSub(lfF(a.End.V), lfF(a.Start.V))
	for _, endpoint := range []planePoint{b.Start, b.End} {
		ex := lfSub(lfF(endpoint.U), lfF(a.Start.U))
		ey := lfSub(lfF(endpoint.V), lfF(a.Start.V))
		if lfSub(lfMul(dx, ey), lfMul(dy, ex)).Sign() != 0 {
			return false
		}
	}
	return true
}
