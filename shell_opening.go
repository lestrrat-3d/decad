package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// This file is the prism and one-line Sweep side opening of
// docs/shell-opening-design.md: Shell on an exact straight prism reduction
// that removes one proper connected run R of its outer
// side faces, with or without its caps. The kept walks K are offset as an open
// chain (internal/offset2d/open_chain.go) whose two ends close by Table RO's rim, the removed
// walk's own carrier cut by the offset. Three regions follow (§3): the wall
// section W, the cavity section C and the cap slabs' region (P inward, the
// outer region O outward), and each faces modify §5's audit and an exact area
// identity before shell_opening.go builds the body from them. Every walk
// of the section must be a line, along a section axis or oblique, or a
// circular arc. internal/prismshell builds the three sections and audits them.

// classifyRemovedFaces sorts a prism shell's removed faces (§5, stage 2):
// the caps by their roles, and every other face by the side(0,j) roles of the
// receiver it carries, j being the recorded outer-loop segment the face
// sweeps. It reports whether each cap is removed and the removed outer-loop
// segments. A face that is neither a cap nor a side face of the outer loop —
// a hole loop's wall — is SO6, ErrUnsupported.
func classifyRemovedFaces(b *Body, caps prismCaps, removed []*Face) (bool, bool, map[int]struct{}, error) {
	var start, end bool
	sides := map[int]struct{}{}
	for _, f := range removed {
		switch {
		case caps.start != nil && f == caps.start:
			start = true
			continue
		case caps.end != nil && f == caps.end:
			end = true
			continue
		}
		named := false
		for _, o := range f.n.Origins {
			var li, j int
			if o.Producer != b.n.Origin.Producer {
				continue
			}
			if n, _ := fmt.Sscanf(o.Role, "side(%d,%d)", &li, &j); n != 2 {
				continue
			}
			if li != 0 {
				return false, false, nil, fmt.Errorf(`%w: a side opening removes faces of the section's outer loop only, and this face lines a hole (modify-reach SX8, shell-opening SO6)`, ErrUnsupported)
			}
			sides[j] = struct{}{}
			named = true
		}
		if !named {
			return false, false, nil, fmt.Errorf(`%w: a removed face is neither a cap nor a side face of this prism (shell-opening SO6)`, ErrUnsupported)
		}
	}
	return start, end, sides, nil
}

// classifyRemovedSweepFaces maps the published one-line Sweep wall role
// side(0,0,j) to segment j of its stored exact prism. The first zero names
// the only path span; the second names the section's outer loop. No role from
// another span or a hole loop is a side-opening target.
func classifyRemovedSweepFaces(b *Body, caps prismCaps, pp prismPayload, removed []*Face) (bool, bool, map[int]struct{}, error) {
	var start, end bool
	sides := map[int]struct{}{}
	for _, f := range removed {
		switch {
		case caps.start != nil && f == caps.start:
			start = true
			continue
		case caps.end != nil && f == caps.end:
			end = true
			continue
		}
		if len(f.n.Origins) != 1 || f.n.Origins[0].Producer != b.n.Origin.Producer {
			return false, false, nil, fmt.Errorf(`%w: a selected Sweep wall has no single source segment (shell-opening SO6)`, ErrUnsupported)
		}
		role := f.n.Origins[0].Role
		var span, loop, segment int
		n, err := fmt.Sscanf(role, "side(%d,%d,%d)", &span, &loop, &segment)
		if err != nil || n != 3 || fmt.Sprintf("side(%d,%d,%d)", span, loop, segment) != role ||
			span != 0 || segment < 0 || segment >= len(pp.Profile.Outer.Segments) {
			return false, false, nil, fmt.Errorf(`%w: a selected face does not map to this Sweep's stored prism (shell-opening SO6)`, ErrUnsupported)
		}
		if loop != 0 {
			return false, false, nil, fmt.Errorf(`%w: a side opening removes faces of the section's outer loop only (shell-opening SO6)`, ErrUnsupported)
		}
		sides[segment] = struct{}{}
	}
	return start, end, sides, nil
}

// shellSideOpening is Shell's side opening on a hole-free straight
// prism (docs/shell-opening-design.md): sides names the removed outer-loop
// segments, and removedStart/removedEnd the removed caps. The regions are
// built and audited first (prismshell.SideOpeningRegions); both caps removed records W
// as a prism over the receiver's sweep (BO1), and otherwise the slabs are
// stated as a brepPayload (BO2, breprecord.SideOpening).
func (b *Body) shellSideOpening(ctx context.Context, pp prismPayload, removedStart, removedEnd bool, sides map[int]struct{}, s float64, t units.Value, tmm, tDelta float64) (*Body, error) {
	d := documentHandle(b.n.Doc)
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	keptCaps := 0
	for _, removed := range []bool{removedStart, removedEnd} {
		if !removed {
			keptCaps++
		}
	}
	sec, err := prismshell.SideOpeningRegions(budget, prismshell.SideOpeningInput{
		Profile: pp.Profile, Height: pp.Z1 - pp.Z0, Sides: sides,
		KeptCaps: keptCaps, Sense: s, Thickness: t, HeldThickness: tmm,
		ThicknessDelta: tDelta, Tolerance: shellTol,
	}, offset2d.AuditOffsetSectionBudget)
	if err != nil {
		return nil, shellCancelCause(err)
	}
	ref := d.nextProducerID()
	var body *Body
	if keptCaps == 0 {
		body, err = evalSideOpeningPrism(ctx, d, ref, pp, sec)
	} else {
		var bp brepPayload
		bp, err = breprecord.SideOpening(ctx, budget, pp, sec, removedStart, removedEnd, s, tmm, tDelta)
		if err == nil {
			body, err = bodyResult(breprecord.Build(ctx, envOf(documentNode(d), ref), bp))
		}
	}
	if err != nil {
		return nil, shellCancelCause(err)
	}
	return commitModifyResult(ctx, b, body)
}

// This file records a prism side opening's BO1 case (docs/shell-opening-design.md
// §4): with both caps removed the wall section W swept over the receiver's own
// sweep is a prism. Otherwise the result is a stack of at most three slabs — a
// kept cap's slab over the cap region, the wall slab over W — which
// breprecord.SideOpening restates into a brepPayload (BO2) with flush walls
// merged into one planar face, the removed face's own plane holding the
// opening as its hole. Every loop is decad's own record, so no sketch scene
// is built; a topology the engine does not cover is SO5, never a fallback.

// evalSideOpeningPrism is BO1: W over the receiver's interval, frame and
// placement, carrying the offset's section displacement and, where a rim on
// a removed arc names its cut at a float parameter, three times that cut's
// gap on top (§4.4).
func evalSideOpeningPrism(ctx context.Context, d *Document, ref producerID, pp prismPayload, sec prismshell.Section) (*Body, error) {
	delta := sec.Delta
	if sec.CutGap > 0 {
		delta = proofbound.AbsSumUpper(delta, proofbound.ProductUpper(3, sec.CutGap))
		if proofbound.IsNonFinite(delta) {
			return nil, offset2d.ErrUnbounded
		}
	}
	return bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), prismrecord.Payload{
		Profile:      sec.Wall,
		Frame:        pp.Frame,
		Z0:           pp.Z0,
		Z1:           pp.Z1,
		Z0Delta:      pp.Z0Delta,
		Z1Delta:      pp.Z1Delta,
		Xform:        pp.Xform,
		SectionDelta: delta,
	}, freeform.NewFreeformWork()))
}
