package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// This file is the prism side opening of docs/shell-opening-design.md: Shell
// on a straight prism that removes one proper connected run R of its outer
// side faces, with or without its caps. The kept walks K are offset as an open
// chain (shell_chain.go) whose two ends close by Table RO's rim, the removed
// walk's own carrier cut by the offset. Three regions follow (§3): the wall
// section W, the cavity section C and the cap slabs' region (P inward, the
// outer region O outward), and each faces modify §5's audit and an exact area
// identity before shell_opening_brep.go builds the body from them. Every walk
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
		for _, o := range f.origins {
			var li, j int
			if o.producer != b.origin.producer {
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

// shellSideOpening is Body.Shell's side opening on a hole-free straight
// prism (docs/shell-opening-design.md): sides names the removed outer-loop
// segments, and removedStart/removedEnd the removed caps. The regions are
// built and audited first (prismshell.SideOpeningRegions); both caps removed records W
// as a prism over the receiver's sweep (BO1), and otherwise the slabs are
// stated as a brepPayload (BO2, shell_opening_brep.go).
func (b *Body) shellSideOpening(ctx context.Context, pp prismPayload, removedStart, removedEnd bool, sides map[int]struct{}, s float64, t units.Value, tmm, tDelta float64) (*Body, error) {
	d := b.doc
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
		Profile: pp.profile, Height: pp.z1 - pp.z0, Sides: sides,
		KeptCaps: keptCaps, Sense: s, Thickness: t, HeldThickness: tmm,
		ThicknessDelta: tDelta, Tolerance: shellTol,
	}, auditOffsetSectionBudget)
	if err != nil {
		return nil, shellCancelCause(err)
	}
	ref := d.nextProducerID()
	var body *Body
	if keptCaps == 0 {
		body, err = evalSideOpeningPrism(ctx, d, ref, pp, sec)
	} else {
		var bp brepPayload
		bp, err = sideOpeningBrep(ctx, budget, pp, sec, removedStart, removedEnd, s, tmm, tDelta)
		if err == nil {
			body, err = evalBrepContext(ctx, d, ref, bp)
		}
	}
	if err != nil {
		return nil, shellCancelCause(err)
	}
	return commitModifyResult(ctx, b, body)
}
