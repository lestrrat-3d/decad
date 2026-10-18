package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/throughshell"
	"github.com/lestrrat-3d/units"
)

// This file is route S of docs/modify-general-design.md ("modify-general §N"
// below): the shell of a brep or stacked receiver that reads as a prism A
// along a reference axis k cut by through tools T₁…Tₙ along other axes
// (Table TC, §3.1), with one or both of A's caps, one run of A's walls, or
// both removed (§3.2), or with one circular tool wall removed (§3.1d).
// For cap and outer-wall removals, the shell is the erosion
// (A \ ⋃Tᵢ) ⊖ t = (A ⊖ t) \ ⋃(Tᵢ ⊕ t): A ⊖ t is the eroded section — S ⊖ t,
// or with a removed wall run the side opening's cavity section C
// (shell_opening.go) — swept over the cup's interval, each Tᵢ ⊕ t the tool's
// dilated section swept t past both walls it pierces, and the difference is
// class B's own Cut (classbgeom.OfPayloads), run privately. The result record is
// the receiver's kept faces, the cavity's faces reversed, and the rim at each
// removed face (§3.3, internal/throughshell/). The circular tool-wall route
// builds exact stacked sections (brep_shell_curved.go). Every test below
// either compares recorded floats exactly or refuses; none admits on a residual.

// brepShellCall is what Shell hands route S beside the brep request: the
// faces to remove, the wall sense, and the thickness as given, in
// millimetres, with its conversion bound (extent.MagnitudeInBounded).
type brepShellCall struct {
	removed     []*Face
	sense       ShellSense
	t           units.Value
	tmm, tDelta float64
}

// removedFaces maps the selected live faces to record indices before SG4 and
// SG5 read them. The connected-run check remains with prismshell.SideOpeningRegions.
func removedThroughFaces(tc throughshell.Cut, b *Body, bp brepPayload, removed []*Face) (throughshell.Removal, error) {
	index := map[*Face]int{}
	if _, ok := b.n.Payload.(brepPayload); ok {
		roles := facesByRole(b)
		for fi, f := range bp.Faces {
			if face := roles[f.Role]; face != nil {
				index[face] = fi
			}
		}
	}
	indices := make([]int, len(removed))
	for i, face := range removed {
		fi, ok := index[face]
		if !ok {
			fi = -1
		}
		indices[i] = fi
	}
	return throughshell.ReadRemoval(prismFaceRecords(bp), tc.Embeds, tc.Kinds, tc.Caps, tc.Axis, indices)
}

// shellThroughCut is route S (modify-general §3.3) for a receiver route P
// did not take. refusal is route P's classification refusal where some axis
// read as a prism, and nil where none did; it leads the SG3 refusal when no
// axis reads as a through-cut record either. The result is a brepPayload
// body the caller commits; the receiver is untouched on every refusal.
func shellThroughCut(ctx context.Context, b *Body, bp brepPayload, call brepShellCall, refusal error) (*Body, error) {
	// Stage 2c: SG1, then Table TC along every axis (SG3), then the removed
	// faces against the reading (SG4, SG5).
	if call.sense == Outward {
		return nil, fmt.Errorf(`%w: this evaluator shells a brep or stacked receiver inward only; an outward wall rounds the receiver's convex edges into tori and spheres, which its record does not hold (modify-general SG1)`, ErrUnsupported)
	}
	tc, err := throughshell.ReadAnyAxis(ctx, bp, refusal)
	if err != nil {
		return nil, err
	}
	if out, matched, err := shellThroughRoundPort(ctx, b, bp, tc, call); matched || err != nil {
		return out, err
	}
	rm, err := removedThroughFaces(tc, b, bp, call.removed)
	if err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}

	// Stage 3: A ⊖ t's section (eroded, with its proven displacement for a
	// removed wall run), then S11a on each tool as its dilation is built.
	input := throughshell.ShellInput{
		Thickness: call.t, HeldThickness: call.tmm,
		ThicknessDelta: call.tDelta, Tolerance: shellTol,
	}
	var sec throughshell.Section
	if len(rm.Walls) == 0 {
		sec, err = tc.ErodeSection(budget, rm, input)
	} else {
		sec, err = tc.OpeningSection(budget, rm, input)
	}
	if err != nil {
		return nil, err
	}
	dilated := make([]profileRecord, len(tc.Tools))
	for i, tool := range tc.Tools {
		dilated[i], err = offsetProfile(budget, tool.Prism.Profile, -1, call.tmm)
		if err != nil {
			return nil, err
		}
	}

	// Stage 4: the offset audit on S ⊖ t (the side opening ran its own) and on
	// each Tᵢ ⊕ t, then TC7.
	if len(rm.Walls) == 0 {
		if err := offset2d.AuditOffsetSectionBudget(budget, tc.Caps.Section, sec.Eroded); err != nil {
			return nil, shellCancelCause(err)
		}
	}
	for i, tool := range tc.Tools {
		if err := offset2d.AuditOffsetSectionBudget(budget, tool.Prism.Profile, dilated[i]); err != nil {
			return nil, shellCancelCause(err)
		}
	}
	for i := range tc.Tools {
		if err := tc.RequireStripsClear(bp, i, dilated[i], call.tmm, call.tDelta); err != nil {
			return nil, err
		}
	}

	// The offsets' proven displacements (modify §9): the cavity's faces and
	// the rims carry the largest of them.
	delta := sec.Delta
	if len(rm.Walls) == 0 {
		if delta, err = offsetSectionDelta(budget, tc.Caps.Section, 1, call.tmm, call.tDelta); err != nil {
			return nil, shellCancelCause(err)
		}
	}
	for _, tool := range tc.Tools {
		d, err := offsetSectionDelta(budget, tool.Prism.Profile, -1, call.tmm, call.tDelta)
		if err != nil {
			return nil, shellCancelCause(err)
		}
		delta = max(delta, d)
	}

	// Stage 5: the cavity, (A ⊖ t) \ ⋃(Tᵢ ⊕ t), through class B's own Cut.
	cavity, err := tc.Cavity(ctx, bp, sec.Eroded, dilated, rm, input, delta)
	if err != nil {
		return nil, err
	}

	// Stage 6: the rims, SG7, and the rim audit.
	out, err := tc.Rims(ctx, budget, bp, cavity, sec.Eroded, dilated, rm)
	if err != nil {
		return nil, err
	}

	// Stage 7: closure. breprecord.Build runs breprecord.Falsify and
	// breprecord.TopologyContext (an unpaired edge is ErrUnsupported) and measures
	// the body (a non-positive volume is ErrDegenerate).
	out.AssignRoles()
	return bodyResult(breprecord.Build(ctx, envOf(b.n.Doc, b.n.Doc.NextProducer), out))
}
