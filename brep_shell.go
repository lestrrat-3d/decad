package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/shellsurvey"
	"github.com/lestrrat-3d/decad/internal/throughshell"
	"github.com/lestrrat-3d/units"
)

// This file is route S of docs/modify-general-design.md ("modify-general §N"
// below): the shell of a brep or stacked receiver that reads as a prism A
// along a reference axis k cut by through tools T₁…Tₙ along other axes
// (Table TC, §3.1), with one or both of A's caps, one run of A's walls, or
// both removed (§3.2). The shell is the erosion
// (A \ ⋃Tᵢ) ⊖ t = (A ⊖ t) \ ⋃(Tᵢ ⊕ t): A ⊖ t is the eroded section — S ⊖ t,
// or with a removed wall run the side opening's cavity section C
// (shell_opening.go) — swept over the cup's interval, each Tᵢ ⊕ t the tool's
// dilated section swept t past both walls it pierces, and the difference is
// class B's own Cut (classBOfPayloads), run privately. The result record is
// the receiver's kept faces, the cavity's faces reversed, and the rim at each
// removed face (§3.3, brep_shell_rim.go). Every test below either compares
// recorded floats exactly or refuses; none admits on a residual.

// brepShellCall is what Shell hands route S beside the brep request: the
// faces to remove, the wall sense, and the thickness as given, in
// millimetres, with its conversion bound (extent.MagnitudeInBounded).
type brepShellCall struct {
	removed     []*Face
	sense       ShellSense
	t           units.Value
	tmm, tDelta float64
}

// throughFace is what Table TC reads one record face as.
type throughFace = brepgeom.ThroughFace

const (
	throughCap      = brepgeom.ThroughCap      // TC1: A's bottom or top
	throughWall     = brepgeom.ThroughWall     // TC2(a): a wall of A along k
	throughHoleWall = brepgeom.ThroughHoleWall // TC2(a) claiming a segment of a hole of S
	throughPierced  = brepgeom.ThroughPierced  // TC2(b): a planar wall of A across j ≠ k
	throughTool     = brepgeom.ThroughTool     // TC2(c): a wall of a through tool
)

// throughCut is one Table TC reading of a record along reference axis k: the
// prism A (its caps, section and levels, brepgeom.ReadPrismCaps's reading), the level
// displacements A's walls state at each cap level, the through tools, what
// each face of the record is, and the record's face embeds.
type throughCut struct {
	k                  int
	caps               brepgeom.PrismCaps
	zloDelta, zhiDelta float64
	tools              []throughToolRead
	kinds              []throughFace
	embeds             []brepEmbed
}

// throughToolRead is one through tool (TC4): the prism over hole loop loop0
// of face w0 at the lower level lo along reference axis j, through to hole
// loop loop1 of the face w1 at hi (each 1 + the index in the face's Holes),
// its section stated in brepgeom.AxisFrame(ref, j) (whose embed is
// frameEmb), and the tool walls between them.
type throughToolRead struct {
	j            int
	w0, w1       int
	loop0, loop1 int
	lo, hi       float64
	prism        prismPayload
	walls        []int
	frameEmb     brepEmbed
}

// readThroughCut reads the record as Table TC's through-cut record along
// reference axis k (modify-general §3.1), or reports false with the reason,
// which names the first face the table does not take. The error is a context
// error only.
func readThroughCut(ctx context.Context, bp brepPayload, embeds []brepEmbed, k int) (throughCut, string, bool, error) {
	record, reason, err := throughshell.ReadThroughFaces(ctx, prismFaceRecords(bp), embeds, k)
	if err != nil || reason != "" {
		return throughCut{}, reason, false, err
	}
	tc := throughCut{k: k, caps: record.Caps, kinds: record.Kinds, embeds: embeds,
		zloDelta: record.ZloDelta, zhiDelta: record.ZhiDelta}

	// TC4 and TC5: the tools, read through the record's own pairing.
	topo, err := brepTopologyContext(ctx, bp)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return throughCut{}, "", false, ctxErr
		}
		return throughCut{}, "the record does not pair its edges", false, nil
	}
	faces := make([]brepgeom.ThroughToolFace, len(bp.faces))
	for i, f := range bp.faces {
		faces[i] = brepgeom.ThroughToolFace{
			Region: f.region, Z0: f.z0, Z1: f.z1, Outward: f.outward, Role: f.role,
		}
	}
	reads, reason, err := brepgeom.ReadThroughTools(ctx,
		&brepgeom.Topology{Uses: topo.uses, Edges: topo.edges, EdgeOf: topo.edgeOf, FaceUses: topo.faceUses},
		faces, embeds, tc.kinds, bp.faces[0].frame)
	if err != nil || reason != "" {
		return throughCut{}, reason, false, err
	}
	for _, read := range reads {
		tc.tools = append(tc.tools, throughToolRead{
			j: read.Axis, w0: read.LowerFace, w1: read.UpperFace,
			loop0: read.LowerLoop, loop1: read.UpperLoop,
			lo: read.Lo, hi: read.Hi, walls: read.Walls, frameEmb: read.FrameEmbed,
			prism: prismPayload{profile: profileRecord{Outer: read.Outer},
				frame: read.Frame, z0: read.Lo, z1: read.Hi, xform: bp.xform},
		})
	}
	return tc, "", true, nil
}

type throughRemoval = throughshell.Removal

// removedFaces maps the selected live faces to record indices before SG4 and
// SG5 read them. The connected-run check remains with sideOpeningRegions.
func (tc throughCut) removedFaces(b *Body, bp brepPayload, removed []*Face) (throughRemoval, error) {
	index := map[*Face]int{}
	if _, ok := b.payload.(brepPayload); ok {
		roles := facesByRole(b)
		for fi, f := range bp.faces {
			if face := roles[f.role]; face != nil {
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
	return throughshell.ReadRemoval(prismFaceRecords(bp), tc.embeds, tc.kinds, tc.caps, tc.k, indices)
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
	tc, err := readThroughCutAnyAxis(ctx, bp, refusal)
	if err != nil {
		return nil, err
	}
	rm, err := tc.removedFaces(b, bp, call.removed)
	if err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}

	// Stage 3: A ⊖ t's section (eroded, with its proven displacement for a
	// removed wall run), then S11a on each tool as its dilation is built.
	var sec throughSection
	if len(rm.Walls) == 0 {
		sec, err = tc.erodeThroughSection(budget, rm, call)
	} else {
		sec, err = tc.openingThroughSection(budget, bp, rm, call)
	}
	if err != nil {
		return nil, err
	}
	dilated := make([]profileRecord, len(tc.tools))
	for i, tool := range tc.tools {
		dilated[i], err = offsetProfile(budget, tool.prism.profile, -1, call.tmm)
		if err != nil {
			return nil, err
		}
	}

	// Stage 4: the offset audit on S ⊖ t (the side opening ran its own) and on
	// each Tᵢ ⊕ t, then TC7.
	if len(rm.Walls) == 0 {
		if err := auditOffsetSectionBudget(budget, tc.caps.Section, sec.eroded); err != nil {
			return nil, shellCancelCause(err)
		}
	}
	for i, tool := range tc.tools {
		if err := auditOffsetSectionBudget(budget, tool.prism.profile, dilated[i]); err != nil {
			return nil, shellCancelCause(err)
		}
	}
	for i := range tc.tools {
		if err := tc.requireStripsClear(bp, i, dilated[i], call.tmm, call.tDelta); err != nil {
			return nil, err
		}
	}

	// The offsets' proven displacements (modify §9): the cavity's faces and
	// the rims carry the largest of them.
	delta := sec.delta
	if len(rm.Walls) == 0 {
		if delta, err = offsetSectionDelta(budget, tc.caps.Section, 1, call.tmm, call.tDelta); err != nil {
			return nil, shellCancelCause(err)
		}
	}
	for _, tool := range tc.tools {
		d, err := offsetSectionDelta(budget, tool.prism.profile, -1, call.tmm, call.tDelta)
		if err != nil {
			return nil, shellCancelCause(err)
		}
		delta = max(delta, d)
	}

	// Stage 5: the cavity, (A ⊖ t) \ ⋃(Tᵢ ⊕ t), through class B's own Cut.
	cavity, err := tc.cavity(ctx, bp, sec.eroded, dilated, rm, call, delta)
	if err != nil {
		return nil, err
	}

	// Stage 6: the rims, SG7, and the rim audit.
	out, err := throughCutRims(ctx, budget, bp, tc, cavity, sec.eroded, dilated, rm)
	if err != nil {
		return nil, err
	}

	// Stage 7: closure. evalBrepContext runs falsifyBrepPayload and
	// brepTopologyContext (an unpaired edge is ErrUnsupported) and measures
	// the body (a non-positive volume is ErrDegenerate).
	out.assignRoles()
	return evalBrepContext(ctx, b.doc, b.doc.nextProducerID(), out)
}

// throughSection is A ⊖ t's section: eroded, and for a removed wall run the
// side opening's proven section displacement (shell-opening §4.4).
type throughSection struct {
	eroded profileRecord
	delta  float64
}

// erodeThroughSection is stage 3 on S with caps alone removed (§3.3 step 1):
// S18 and S10's section limit, S10's height limit for a kept cap, then S11a
// as offsetProfile(S, +1, t) is built. The caller audits the offset and
// proves its displacement.
func (tc throughCut) erodeThroughSection(budget *proofbound.WorkBudget, rm throughRemoval, call brepShellCall) (throughSection, error) {
	section := tc.caps.Section
	inradius, enough, err := shellsurvey.SectionInradius(budget, boundarywalk.Profile(section), call.tmm, call.tDelta, shellTol)
	if err != nil {
		return throughSection{}, err
	}
	if err := requireSectionCavity(call.t, call.tmm, inradius, enough); err != nil {
		return throughSection{}, err
	}
	h := tc.caps.Zhi - tc.caps.Zlo
	if maxT := h - shellTol*math.Max(1, h); rm.KeptCaps() > 0 && call.tmm >= maxT {
		return throughSection{}, fmt.Errorf(`%w: the shell thickness %s meets or exceeds the accepted maximum %s (the sweep height %s less the evaluator's rounding tolerance); use a thickness strictly below the accepted maximum`, ErrDegenerate, call.t, units.Millimeters(max(maxT, 0)), units.Millimeters(h))
	}
	eroded, err := offsetProfile(budget, section, 1, call.tmm)
	if err != nil {
		return throughSection{}, err
	}
	return throughSection{eroded: eroded}, nil
}

// openingThroughSection is stage 3 on S with a removed wall run (§3.2's
// second row): A ⊖ t's section is the side opening's cavity section C over
// the kept chain, cut at each end by the rim rule of
// docs/shell-opening-design.md Table RO, built and audited by
// sideOpeningRegions in that document's gate order (SO6, SO3's height half,
// S11a and SO1, SO2, SO4 per end, modify §5's audit of W and C, the exact area
// identity), whose refusals keep their codes. No section limit runs: the
// cavity opens through the removed walls (shell-opening §5). The displacement
// is the chain's figure plus three times the cut gap, as evalSideOpeningPrism
// charges a section that publishes C. A cut that runs back along the removed
// walk's carrier (a reflex end) leaves a rim inside the material that is no
// piece of a removed face, which route S does not state: SG5.
func (tc throughCut) openingThroughSection(budget *proofbound.WorkBudget, bp brepPayload, rm throughRemoval, call brepShellCall) (throughSection, error) {
	pp := prismPayload{profile: tc.caps.Section, frame: tc.caps.Frame, xform: bp.xform, z0: tc.caps.Zlo, z1: tc.caps.Zhi}
	sec, err := sideOpeningRegions(budget, pp, rm.Sides, rm.KeptCaps(), 1, call.t, call.tmm, call.tDelta)
	if err != nil {
		return throughSection{}, shellCancelCause(err)
	}
	if len(sec.corners) > 0 {
		return throughSection{}, fmt.Errorf(`%w: the removed wall run meets a kept wall at a reflex corner, where the rim runs back along the removed wall's carrier into the material, a face this evaluator does not state for a through-cut record (modify-general SG5)`, ErrUnsupported)
	}
	delta := sec.delta
	if sec.cutGap > 0 {
		delta = proofbound.AbsSumUpper(delta, proofbound.ProductUpper(3, sec.cutGap))
	}
	return throughSection{eroded: sec.cavity, delta: delta}, nil
}

// readThroughCutAnyAxis reads Table TC along reference axes 0, 1, 2 in order
// and returns the first reading. Where none reads, the refusal is SG3, led by
// route P's refusal (brep-modify SB3) where some axis read as a prism and by
// SB10 where none did. It names the first face Table TC does not take along
// the first axis whose caps TC1 reads, else along the first axis holding more
// than two planar faces across it, else along axis 0.
func readThroughCutAnyAxis(ctx context.Context, bp brepPayload, refusal error) (throughCut, error) {
	reason := "its face frames have no exact map onto one reference frame"
	if embeds, err := brepEmbeds(bp.faces); err == nil {
		reasons := make([]string, 3)
		rank := make([]int, 3)
		for k := range 3 {
			tc, why, ok, err := readThroughCut(ctx, bp, embeds, k)
			if err != nil {
				return throughCut{}, err
			}
			if ok {
				return tc, nil
			}
			reasons[k] = fmt.Sprintf("along reference axis %d, %s", k, why)
			if _, capsRead := brepgeom.ReadPrismCaps(prismFaceRecords(bp), embeds, k); capsRead {
				rank[k] = 2
			} else if throughCapsCount(bp, embeds, k) > 2 {
				rank[k] = 1
			}
		}
		best := 0
		for k := range 3 {
			if rank[k] > rank[best] {
				best = k
			}
		}
		reason = reasons[best]
	}
	if refusal != nil {
		return throughCut{}, fmt.Errorf(`%w; this body reads as a prism along a reference axis, and no such prism takes the removed faces as its caps (brep-modify SB3); nor does it read as a prism cut by through tools: %s (modify-general SG3)`, refusal, reason)
	}
	return throughCut{}, fmt.Errorf(`%w: this evaluator shells a brep or stacked receiver only where it reads as a prism along a reference axis (brep-modify SB10) or as one cut by through tools, and this one reads as neither: %s (modify-general SG3)`, ErrUnsupported, reason)
}

// throughCapsCount is the number of planar faces across reference axis k.
func throughCapsCount(bp brepPayload, embeds []brepEmbed, k int) int {
	n := 0
	for fi, f := range bp.faces {
		if f.planar() && embeds[fi].Axis[2] == k {
			n++
		}
	}
	return n
}

// requireStripsClear is TC7 (modify-general §3.3 step 3) for tool i: beyond
// each wall it pierces, the dilated tool reaches t into the slab
// {j ∈ [L0 − t, L0)} (and {j ∈ (L1, L1 + t]}) over its section's m-extent
// widened by t. With A a prism along k, the question is two-dimensional in
// S's plane: every segment of S must have an outward box separated from both
// strips along some axis, by exact rational comparison. The strips are
// widened by the thickness's conversion bound and cover the dilated record's
// own box. A box that is not separated is SG6. The test only refuses.
func (tc throughCut) requireStripsClear(bp brepPayload, i int, dilated profileRecord, tmm, tDelta float64) error {
	tool := tc.tools[i]
	separated, err := throughshell.StripsClear(throughshell.StripInput{
		Tool: tool.prism.profile, Dilated: dilated, Receiver: tc.caps.Section,
		ToolFrame: tool.frameEmb, ReceiverFrame: tc.caps.Embed,
		SweepAxis: tc.k, PiercedAxis: tool.j, Lo: tool.lo, Hi: tool.hi,
		Thickness: tmm, ThicknessDelta: tDelta,
	})
	if err != nil || separated {
		return err
	}
	return fmt.Errorf(`%w: the tool through %s, dilated by the shell thickness, reaches past the wall it pierces into the material of the receiver, or this evaluator cannot separate it from there (modify-general SG6)`, ErrUnsupported, bp.faces[tool.w0].role)
}

// cavity builds A' \ T₁' \ … \ Tₙ' (modify-general §3.3 step 4): A' is
// A ⊖ t's section (eroded) over the cup's interval, a kept cap's level moved
// by t, and
// Tᵢ' the dilated tool section over [L0 − t, L1 + t]. Each private Cut is
// class B's own entry over payloads, so it builds no Body and touches no
// document; a pair class B does not take is SG6, naming the tool.
//
// Class B admits no section displacement (general-boolean B5), so the cuts
// run on the offsets' recorded floats with every displacement zero, and every
// face of the cavity takes the displacement afterwards: delta, the largest
// offset displacement, as its section displacement, and the largest of delta
// and the cap levels' displacements on both levels. The largest bound covers
// each face's own, so the charge is an upper bound for every face. Each cut
// is told that charge: class B widens every box it compares by it, so a
// recorded gap within it refuses, and it refuses its crossing reach while the
// charge is positive, since that reach charges its crossings nothing for it.
func (tc throughCut) cavity(ctx context.Context, bp brepPayload, eroded profileRecord, dilated []profileRecord, rm throughRemoval, call brepShellCall, delta float64) (brepPayload, error) {
	step := func(from, d, by float64) (float64, float64) {
		to := from + by
		return to, proofbound.AbsSumUpper(d, call.tDelta, proofarith.AddRoundError(from, by, to))
	}
	z0, z0Delta := tc.caps.Zlo, max(tc.zloDelta, bp.faces[tc.caps.Bottom].z0Delta)
	z1, z1Delta := tc.caps.Zhi, max(tc.zhiDelta, bp.faces[tc.caps.Top].z0Delta)
	if !rm.Bottom {
		z0, z0Delta = step(z0, z0Delta, call.tmm)
	}
	if !rm.Top {
		z1, z1Delta = step(z1, z1Delta, -call.tmm)
	}
	charge := max(delta, z0Delta, z1Delta)
	var cut featurePayload = prismPayload{profile: eroded, frame: tc.caps.Frame, z0: z0, z1: z1, xform: bp.xform}
	for i, tool := range tc.tools {
		lo, loDelta := step(tool.lo, 0, -call.tmm)
		hi, hiDelta := step(tool.hi, 0, call.tmm)
		tp := prismPayload{profile: dilated[i], frame: tool.prism.frame, xform: bp.xform,
			z0: lo, z1: hi, z0Delta: loDelta, z1Delta: hiDelta}
		out, ok, err := classBOfPayloads(ctx, meshbool.OpCut, cut, tp, bp.xform, charge)
		if err != nil {
			if ctxErr := ctx.Err(); ctxErr != nil {
				return brepPayload{}, ctxErr
			}
			if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
				return brepPayload{}, err
			}
			return brepPayload{}, fmt.Errorf(`%w; the shell's cavity is no class-B cut of the eroded prism by the tool through %s dilated by the thickness (modify-general SG6)`, err, bp.faces[tool.w0].role)
		}
		if _, isBrep := out.(brepPayload); !ok || !isBrep {
			return brepPayload{}, fmt.Errorf(`%w: the shell's cavity is no class-B cut of the eroded prism by the tool through %s dilated by the thickness: two dilated tools meet, or a dilated tool reaches a cap of the cavity (modify-general SG6)`, ErrUnsupported, bp.faces[tool.w0].role)
		}
		cut = out
	}
	cavity, ok := cut.(brepPayload)
	if !ok {
		pp, isPrism := cut.(prismPayload)
		if !isPrism {
			return brepPayload{}, fmt.Errorf(`%w: the shell's cavity is a %T, not a face record`, ErrUnsupported, cut)
		}
		var err error
		cavity, err = brepOfPrism(pp)
		if err != nil {
			return brepPayload{}, err
		}
	}
	faces := make([]brepFace, len(cavity.faces))
	for i, f := range cavity.faces {
		f.delta = max(f.delta, delta)
		f.z0Delta, f.z1Delta = max(f.z0Delta, charge), max(f.z1Delta, charge)
		faces[i] = f
	}
	cavity.faces = faces
	return cavity, nil
}
