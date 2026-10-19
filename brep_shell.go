package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/stackedshell"
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
// builds exact stacked sections (brep_shell.go). Every test below
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

// stackedShellCall is call as the plain data internal/stackedshell reads. It
// stays in root because it matches the live removed face against source's
// cap-end and cap-start faces (facesByRole).
func stackedShellCall(source *Body, call brepShellCall) stackedshell.Call {
	sc := stackedshell.Call{Inward: call.sense == Inward, Removed: len(call.removed), T: call.tmm, TDelta: call.tDelta}
	if len(call.removed) == 1 {
		roles := facesByRole(source)
		sc.CapEnd = call.removed[0] == roles[provenance.RoleCapEnd]
		sc.CapStart = call.removed[0] == roles[provenance.RoleCapStart]
	}
	return sc
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
		dilated[i], err = offset2d.OffsetProfile(budget, tool.Prism.Profile, -1, call.tmm, shellTol)
		if err != nil {
			return nil, err
		}
	}

	// Stage 4: the offset audit on S ⊖ t (the side opening ran its own) and on
	// each Tᵢ ⊕ t, then TC7.
	if len(rm.Walls) == 0 {
		if err := offset2d.AuditOffsetSectionBudget(budget, tc.Caps.Section, sec.Eroded); err != nil {
			return nil, prismshell.CancelCause(err)
		}
	}
	for i, tool := range tc.Tools {
		if err := offset2d.AuditOffsetSectionBudget(budget, tool.Prism.Profile, dilated[i]); err != nil {
			return nil, prismshell.CancelCause(err)
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
		if delta, err = offset2d.OffsetSectionDelta(budget, tc.Caps.Section, 1, call.tmm, call.tDelta, shellTol); err != nil {
			return nil, prismshell.CancelCause(err)
		}
	}
	for _, tool := range tc.Tools {
		d, err := offset2d.OffsetSectionDelta(budget, tool.Prism.Profile, -1, call.tmm, call.tDelta, shellTol)
		if err != nil {
			return nil, prismshell.CancelCause(err)
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

// shellThroughRoundPort shells a rectangular prism cut by one whole-circle
// through tool when the tool's cylindrical wall is the sole removed face.
// The tool wall is replaced by the two short cylindrical rims through the
// exterior wall thickness. Between them the inset rectangular cavity opens
// into the port. All three sections are exact stack records, so the stack's
// analytic mass and mesh proofs apply without a new surface kind.
func shellThroughRoundPort(ctx context.Context, source *Body, bp brepPayload, tc throughshell.Cut,
	call brepShellCall) (*Body, bool, error) {
	if len(call.removed) != 1 || len(tc.Tools) != 1 || len(tc.Tools[0].Walls) != 1 {
		return nil, false, nil
	}
	tool := tc.Tools[0]
	wall := bp.Faces[tool.Walls[0]]
	if call.removed[0] != facesByRole(source)[wall.Role] {
		return nil, false, nil
	}
	refuse := func(reason string) (*Body, bool, error) {
		return nil, true, fmt.Errorf(`%w: the removed circular through wall %s %s (modify-general SG4)`,
			ErrUnsupported, wall.Role, reason)
	}
	// Seven source faces and the two rectangular cap readings certify that
	// the one tool passes through a box. No other hole or curved outer wall
	// can be discarded by the three-section rewrite.
	if len(bp.Faces) != 7 || len(tc.Caps.Section.Holes) != 0 || len(bp.Faces[tool.LowerFace].Region.Holes) != 1 ||
		len(bp.Faces[tool.UpperFace].Region.Holes) != 1 {
		return refuse("needs one rectangular box and one through hole")
	}
	if _, ok := prismshell.RecordedBossRect(tc.Caps.Section); !ok {
		return refuse("needs a rectangular box section")
	}
	for _, face := range bp.Faces {
		if face.Delta != 0 || face.Z0Delta != 0 || face.Z1Delta != 0 {
			return refuse("needs exact source faces and levels")
		}
	}
	if call.tDelta != 0 {
		return refuse("needs an exactly converted thickness")
	}
	outer, ok := brepgeom.NewPrismMap(tc.Embeds[tool.LowerFace], tool.FrameEmbed).Loop(bp.Faces[tool.LowerFace].Region.Outer)
	if !ok {
		return refuse("cannot restate the pierced rectangle in the tool frame")
	}
	rect, ok := prismshell.RecordedBossRect(profileRecord{Outer: outer})
	if !ok {
		return refuse("needs a rectangular pierced wall")
	}
	circle, ok := prismshell.RecordedBossCircle(tool.Prism.Profile)
	if !ok {
		return refuse("needs one exact whole-circle through tool")
	}
	t := call.tmm
	inner := rect.Inset(t)
	loInner, hiInner := tool.Lo+t, tool.Hi-t
	if !(loInner < hiInner && inner.Width() > 0 && inner.Height() > 0 &&
		circle.Center.U-circle.Radius > inner.U0 && circle.Center.U+circle.Radius < inner.U1 &&
		circle.Center.V-circle.Radius > inner.V0 && circle.Center.V+circle.Radius < inner.V1) {
		return refuse("has too little wall thickness or port clearance")
	}
	if !prismshell.BossRectArithmeticExact(rect, t) ||
		proofbound.ExactSumRound(loInner, tool.Lo, t) != 0 ||
		proofbound.ExactSumRound(hiInner, tool.Hi, -t) != 0 ||
		proofbound.ExactSumRound(tool.Hi-tool.Lo, tool.Hi, -tool.Lo) != 0 {
		return refuse("needs exactly represented offset coordinates")
	}
	for _, c := range []float64{circle.Center.U, circle.Center.V} {
		if proofbound.ExactSumRound(c-circle.Radius, c, -circle.Radius) != 0 ||
			proofbound.ExactSumRound(c+circle.Radius, c, circle.Radius) != 0 {
			return refuse("needs exactly represented circle extents")
		}
	}
	// The box around the circle is a strict recorded-coordinate proof that
	// the circle is inside the inset rectangle. The interface ring has no
	// crossings or tangency, so its outer and inner loops bound one face.
	port, err := offset2d.ReverseLoopRecordContext(ctx, circle.Outer.Outer)
	if err != nil {
		return nil, true, err
	}
	cavity, err := offset2d.ReverseLoopRecordContext(ctx, inner.Profile().Outer)
	if err != nil {
		return nil, true, err
	}
	end := profileRecord{Outer: outer, Holes: []loopRecord{port}}
	middle := profileRecord{Outer: outer, Holes: []loopRecord{cavity}}
	ring, err := stackedrecord.EnclosingExposed(ctx, cavity, []loopRecord{port})
	if err != nil {
		return nil, true, err
	}
	sp := stackedPrismPayload{Frame: tool.Prism.Frame, Xform: bp.Xform,
		Slabs: []stackedrecord.Slab{
			{Regions: []profileRecord{end}, Z0: tool.Lo, Z1: loInner},
			{Regions: []profileRecord{middle}, Z0: loInner, Z1: hiInner},
			{Regions: []profileRecord{end}, Z0: hiInner, Z1: tool.Hi},
		},
		Interfaces: []stackedrecord.Interface{{LowerExposed: ring}, {UpperExposed: ring}},
	}
	result, err := bodyResult(stackedrecord.Build(ctx, envOf(source.n.Doc, source.n.Doc.NextProducer), sp, stackedrecord.OwnPlan{}))
	return result, true, err
}
