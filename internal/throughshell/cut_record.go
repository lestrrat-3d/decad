package throughshell

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/shellsurvey"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type ShellInput struct {
	Thickness                                units.Value
	HeldThickness, ThicknessDelta, Tolerance float64
}

func cancelCause(err error) error {
	if errors.Is(err, context.Canceled) {
		return context.Canceled
	}
	if errors.Is(err, context.DeadlineExceeded) {
		return context.DeadlineExceeded
	}
	return err
}

func prismFaces(bp breprecord.Payload) []brepgeom.PrismRectFace {
	faces := make([]brepgeom.PrismRectFace, len(bp.Faces))
	for i, f := range bp.Faces {
		faces[i] = brepgeom.PrismRectFace{
			Frame: f.Frame, Region: f.Region, Wall: f.Wall, Z0: f.Z0, Z1: f.Z1,
			Z0Delta: f.Z0Delta, Z1Delta: f.Z1Delta,
			Split0: len(f.Side0) != 0, Split1: len(f.Side1) != 0,
			Outward: f.Outward, Role: f.Role,
		}
	}
	return faces
}

// Cut is one Table TC reading of a record along reference axis k: the
// prism A (its caps, section and levels, brepgeom.ReadPrismCaps's reading), the level
// displacements A's walls state at each cap level, the through tools, what
// each face of the record is, and the record's face embeds.
type Cut struct {
	Axis               int
	Caps               brepgeom.PrismCaps
	ZloDelta, ZhiDelta float64
	Tools              []Tool
	Kinds              []brepgeom.ThroughFace
	Embeds             []brepgeom.Embed
}

// Tool is one through tool (TC4): the prism over hole loop loop0
// of face w0 at the lower level lo along reference axis j, through to hole
// loop loop1 of the face w1 at hi (each 1 + the index in the face's Holes),
// its section stated in brepgeom.AxisFrame(ref, j) (whose embed is
// frameEmb), and the tool walls between them.
type Tool struct {
	Axis                 int
	LowerFace, UpperFace int
	LowerLoop, UpperLoop int
	Lo, Hi               float64
	Prism                prismrecord.Payload
	Walls                []int
	FrameEmbed           brepgeom.Embed
}

// ReadCut reads the record as Table TC's through-cut record along
// reference axis k (modify-general §3.1), or reports false with the reason,
// which names the first face the table does not take. The error is a context
// error only.
func ReadCut(ctx context.Context, bp breprecord.Payload, embeds []brepgeom.Embed, k int) (Cut, string, bool, error) {
	record, reason, err := ReadThroughFaces(ctx, prismFaces(bp), embeds, k)
	if err != nil || reason != "" {
		return Cut{}, reason, false, err
	}
	tc := Cut{Axis: k, Caps: record.Caps, Kinds: record.Kinds, Embeds: embeds,
		ZloDelta: record.ZloDelta, ZhiDelta: record.ZhiDelta}

	// TC4 and TC5: the tools, read through the record's own pairing.
	btopo, err := breprecord.TopologyContext(ctx, bp)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return Cut{}, "", false, ctxErr
		}
		return Cut{}, "the record does not pair its edges", false, nil
	}
	faces := make([]brepgeom.ThroughToolFace, len(bp.Faces))
	for i, f := range bp.Faces {
		faces[i] = brepgeom.ThroughToolFace{
			Region: f.Region, Z0: f.Z0, Z1: f.Z1, Outward: f.Outward, Role: f.Role,
		}
	}
	reads, reason, err := brepgeom.ReadThroughTools(ctx,
		&brepgeom.Topology{Uses: btopo.Uses, Edges: btopo.Edges, EdgeOf: btopo.EdgeOf, FaceUses: btopo.FaceUses},
		faces, embeds, tc.Kinds, bp.Faces[0].Frame)
	if err != nil || reason != "" {
		return Cut{}, reason, false, err
	}
	for _, read := range reads {
		tc.Tools = append(tc.Tools, Tool{
			Axis: read.Axis, LowerFace: read.LowerFace, UpperFace: read.UpperFace,
			LowerLoop: read.LowerLoop, UpperLoop: read.UpperLoop,
			Lo: read.Lo, Hi: read.Hi, Walls: read.Walls, FrameEmbed: read.FrameEmbed,
			Prism: prismrecord.Payload{Profile: momentinput.Profile{Outer: read.Outer},
				Frame: read.Frame, Z0: read.Lo, Z1: read.Hi, Xform: bp.Xform},
		})
	}
	return tc, "", true, nil
}

// ReadAnyAxis reads Table TC along reference axes 0, 1, 2 in order
// and returns the first reading. Where none reads, the refusal is SG3, led by
// route P's refusal (brep-modify SB3) where some axis read as a prism and by
// SB10 where none did. It names the first face Table TC does not take along
// the first axis whose caps TC1 reads, else along the first axis holding more
// than two planar faces across it, else along axis 0.
func ReadAnyAxis(ctx context.Context, bp breprecord.Payload, refusal error) (Cut, error) {
	reason := "its face frames have no exact map onto one reference frame"
	if embeds, err := breprecord.Embeds(bp.Faces); err == nil {
		reasons := make([]string, 3)
		rank := make([]int, 3)
		for k := range 3 {
			tc, why, ok, err := ReadCut(ctx, bp, embeds, k)
			if err != nil {
				return Cut{}, err
			}
			if ok {
				return tc, nil
			}
			reasons[k] = fmt.Sprintf("along reference axis %d, %s", k, why)
			if _, capsRead := brepgeom.ReadPrismCaps(prismFaces(bp), embeds, k); capsRead {
				rank[k] = 2
			} else if breprecord.ThroughCapsCount(bp, embeds, k) > 2 {
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
		return Cut{}, fmt.Errorf(`%w; this body reads as a prism along a reference axis, and no such prism takes the removed faces as its caps (brep-modify SB3); nor does it read as a prism cut by through tools: %s (modify-general SG3)`, refusal, reason)
	}
	return Cut{}, fmt.Errorf(`%w: this evaluator shells a brep or stacked receiver only where it reads as a prism along a reference axis (brep-modify SB10) or as one cut by through tools, and this one reads as neither: %s (modify-general SG3)`, decaderr.ErrUnsupported, reason)
}

// Section is A ⊖ t's section: eroded, and for a removed wall run the
// side opening's proven section displacement (shell-opening §4.4).
type Section struct {
	Eroded momentinput.Profile
	Delta  float64
}

// ErodeSection is stage 3 on S with caps alone removed (§3.3 step 1):
// S18 and S10's section limit, S10's height limit for a kept cap, then S11a
// as offset2d.OffsetProfile(S, +1, t) is built. The caller audits the offset and
// proves its displacement.
func (tc Cut) ErodeSection(budget *proofbound.WorkBudget, rm Removal, call ShellInput) (Section, error) {
	section := tc.Caps.Section
	inradius, enough, err := shellsurvey.SectionInradius(budget, boundarywalk.Profile(section), call.HeldThickness, call.ThicknessDelta, call.Tolerance)
	if err != nil {
		return Section{}, err
	}
	if err := shellsurvey.RequireSectionCavity(call.Thickness, call.HeldThickness, inradius, enough, call.Tolerance); err != nil {
		return Section{}, err
	}
	h := tc.Caps.Zhi - tc.Caps.Zlo
	if maxT := h - call.Tolerance*math.Max(1, h); rm.KeptCaps() > 0 && call.HeldThickness >= maxT {
		return Section{}, fmt.Errorf(`%w: the shell thickness %s meets or exceeds the accepted maximum %s (the sweep height %s less the evaluator's rounding tolerance); use a thickness strictly below the accepted maximum`, decaderr.ErrDegenerate, call.Thickness, units.Millimeters(max(maxT, 0)), units.Millimeters(h))
	}
	eroded, err := offset2d.OffsetProfile(budget, section, 1, call.HeldThickness, call.Tolerance)
	if err != nil {
		return Section{}, err
	}
	return Section{Eroded: eroded}, nil
}

// OpeningSection is stage 3 on S with a removed wall run (§3.2's
// second row): A ⊖ t's section is the side opening's cavity section C over
// the kept chain, cut at each end by the rim rule of
// docs/shell-opening-design.md Table RO, built and audited by
// prismshell.SideOpeningRegions in that document's gate order (SO6, SO3's height half,
// S11a and SO1, SO2, SO4 per end, modify §5's audit of W and C, the exact area
// identity), whose refusals keep their codes. No section limit runs: the
// cavity opens through the removed walls (shell-opening §5). The displacement
// is the chain's figure plus three times the cut gap, as evalSideOpeningPrism
// charges a section that publishes C. A cut that runs back along the removed
// walk's carrier (a reflex end) leaves a rim inside the material that is no
// piece of a removed face, which route S does not state: SG5.
func (tc Cut) OpeningSection(budget *proofbound.WorkBudget, rm Removal, call ShellInput) (Section, error) {
	sec, err := prismshell.SideOpeningRegions(budget, prismshell.SideOpeningInput{
		Profile: tc.Caps.Section, Height: tc.Caps.Zhi - tc.Caps.Zlo, Sides: rm.Sides,
		KeptCaps: rm.KeptCaps(), Sense: 1, Thickness: call.Thickness, HeldThickness: call.HeldThickness,
		ThicknessDelta: call.ThicknessDelta, Tolerance: call.Tolerance,
	}, offset2d.AuditOffsetSectionBudget)
	if err != nil {
		return Section{}, cancelCause(err)
	}
	if len(sec.Corners) > 0 {
		return Section{}, fmt.Errorf(`%w: the removed wall run meets a kept wall at a reflex corner, where the rim runs back along the removed wall's carrier into the material, a face this evaluator does not state for a through-cut record (modify-general SG5)`, decaderr.ErrUnsupported)
	}
	delta := sec.Delta
	if sec.CutGap > 0 {
		delta = proofbound.AbsSumUpper(delta, proofbound.ProductUpper(3, sec.CutGap))
	}
	return Section{Eroded: sec.Cavity, Delta: delta}, nil
}

// RequireStripsClear is TC7 (modify-general §3.3 step 3) for tool i: beyond
// each wall it pierces, the dilated tool reaches t into the slab
// {j ∈ [L0 − t, L0)} (and {j ∈ (L1, L1 + t]}) over its section's m-extent
// widened by t. With A a prism along k, the question is two-dimensional in
// S's plane: every segment of S must have an outward box separated from both
// strips along some axis, by exact rational comparison. The strips are
// widened by the thickness's conversion bound and cover the dilated record's
// own box. A box that is not separated is SG6. The test only refuses.
func (tc Cut) RequireStripsClear(bp breprecord.Payload, i int, dilated momentinput.Profile, tmm, tDelta float64) error {
	tool := tc.Tools[i]
	separated, err := StripsClear(StripInput{
		Tool: tool.Prism.Profile, Dilated: dilated, Receiver: tc.Caps.Section,
		ToolFrame: tool.FrameEmbed, ReceiverFrame: tc.Caps.Embed,
		SweepAxis: tc.Axis, PiercedAxis: tool.Axis, Lo: tool.Lo, Hi: tool.Hi,
		Thickness: tmm, ThicknessDelta: tDelta,
	})
	if err != nil || separated {
		return err
	}
	return fmt.Errorf(`%w: the tool through %s, dilated by the shell thickness, reaches past the wall it pierces into the material of the receiver, or this evaluator cannot separate it from there (modify-general SG6)`, decaderr.ErrUnsupported, bp.Faces[tool.LowerFace].Role)
}

// Cavity builds A' \ T₁' \ … \ Tₙ' (modify-general §3.3 step 4): A' is
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
func (tc Cut) Cavity(ctx context.Context, bp breprecord.Payload, eroded momentinput.Profile, dilated []momentinput.Profile, rm Removal, call ShellInput, delta float64) (breprecord.Payload, error) {
	step := func(from, d, by float64) (float64, float64) {
		to := from + by
		return to, proofbound.AbsSumUpper(d, call.ThicknessDelta, proofarith.AddRoundError(from, by, to))
	}
	z0, z0Delta := tc.Caps.Zlo, max(tc.ZloDelta, bp.Faces[tc.Caps.Bottom].Z0Delta)
	z1, z1Delta := tc.Caps.Zhi, max(tc.ZhiDelta, bp.Faces[tc.Caps.Top].Z0Delta)
	if !rm.Bottom {
		z0, z0Delta = step(z0, z0Delta, call.HeldThickness)
	}
	if !rm.Top {
		z1, z1Delta = step(z1, z1Delta, -call.HeldThickness)
	}
	charge := max(delta, z0Delta, z1Delta)
	var cut topo.Payload = prismrecord.Payload{Profile: eroded, Frame: tc.Caps.Frame, Z0: z0, Z1: z1, Xform: bp.Xform}
	for i, tool := range tc.Tools {
		lo, loDelta := step(tool.Lo, 0, -call.HeldThickness)
		hi, hiDelta := step(tool.Hi, 0, call.HeldThickness)
		tp := prismrecord.Payload{Profile: dilated[i], Frame: tool.Prism.Frame, Xform: bp.Xform,
			Z0: lo, Z1: hi, Z0Delta: loDelta, Z1Delta: hiDelta}
		out, ok, err := classbgeom.OfPayloads(ctx, meshbool.OpCut, cut, tp, bp.Xform, charge)
		if err != nil {
			if ctxErr := ctx.Err(); ctxErr != nil {
				return breprecord.Payload{}, ctxErr
			}
			if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
				return breprecord.Payload{}, err
			}
			return breprecord.Payload{}, fmt.Errorf(`%w; the shell's cavity is no class-B cut of the eroded prism by the tool through %s dilated by the thickness (modify-general SG6)`, err, bp.Faces[tool.LowerFace].Role)
		}
		if _, isBrep := out.(breprecord.Payload); !ok || !isBrep {
			return breprecord.Payload{}, fmt.Errorf(`%w: the shell's cavity is no class-B cut of the eroded prism by the tool through %s dilated by the thickness: two dilated tools meet, or a dilated tool reaches a cap of the cavity (modify-general SG6)`, decaderr.ErrUnsupported, bp.Faces[tool.LowerFace].Role)
		}
		cut = out
	}
	cavity, ok := cut.(breprecord.Payload)
	if !ok {
		pp, isPrism := cut.(prismrecord.Payload)
		if !isPrism {
			return breprecord.Payload{}, fmt.Errorf(`%w: the shell's cavity is a %T, not a face record`, decaderr.ErrUnsupported, cut)
		}
		var err error
		cavity, err = breprecord.OfPrism(pp)
		if err != nil {
			return breprecord.Payload{}, err
		}
	}
	faces := make([]breprecord.Face, len(cavity.Faces))
	for i, f := range cavity.Faces {
		f.Delta = max(f.Delta, delta)
		f.Z0Delta, f.Z1Delta = max(f.Z0Delta, charge), max(f.Z1Delta, charge)
		faces[i] = f
	}
	cavity.Faces = faces
	return cavity, nil
}

// This file is route S's rim assembly (docs/modify-general-design.md §3.3
// steps 5 and 6, "modify-general §N" below). Each removed face R is stated as
// a planar face, and its rim is R's region less the cavity's trace in R's
// plane: R's outer loop holding every cavity face there as a hole, and one
// band per hole of R between that hole and the cavity hole it partners.
// Where the cavity's trace runs along R's own boundary — a removed wall run
// beside a removed cap, each rim reaching the other's edge — the coincident
// pieces cancel and the rest chain into the rim's loops. Every comparison is
// exact on recorded floats, and every loop the assembly cannot state refuses
// (SG7).

// Rims assembles the shell's record (modify-general §3.3 steps 5
// and 6): the receiver's faces but the removed ones, verbatim; the cavity's
// faces reversed, except those lying in a removed face's plane; and, per
// removed face R, its rim: R's region less every cavity face Q in R's plane
// (RimRegions), and for each hole h of R the band between h and the
// hole of some Q that partners it — at a cap the hole of the eroded section
// of h's index, at a pierced wall the section of the tool through h dilated
// by t. A loop with no partner is SG7. Every rim region and band runs modify
// §5's audit (S8, S7, S9). A cap's rim carries the cavity's section
// displacement; a wall's rim, whose region states the cavity's levels and the
// receiver's cap levels as in-plane coordinates, carries the largest of that
// and every cavity level's displacement.
func (tc Cut) Rims(ctx context.Context, budget *proofbound.WorkBudget, bp breprecord.Payload, cavity breprecord.Payload, eroded momentinput.Profile, dilated []momentinput.Profile, rm Removal) (breprecord.Payload, error) {
	frames := make([]r3.Frame, 0, 1+len(cavity.Faces))
	frames = append(frames, bp.Faces[0].Frame)
	for _, f := range cavity.Faces {
		frames = append(frames, f.Frame)
	}
	embeds, err := brepgeom.Embeds(frames, decaderr.ErrUnsupported)
	if err != nil {
		return breprecord.Payload{}, err
	}
	cavEmbeds := embeds[1:]
	cavFaces := prismFaces(cavity)
	joined, _, err := breprecord.JoinProfile(eroded)
	if err != nil {
		return breprecord.Payload{}, err
	}
	charge := cavity.SectionDelta()
	for _, f := range cavity.Faces {
		charge = max(charge, f.Z0Delta, f.Z1Delta)
	}

	var removed []int
	if rm.Bottom {
		removed = append(removed, tc.Caps.Bottom)
	}
	if rm.Top {
		removed = append(removed, tc.Caps.Top)
	}
	removed = append(removed, rm.Walls...)
	rimTools := make([]RimTool, len(tc.Tools))
	for i, tool := range tc.Tools {
		rimTools[i] = RimTool{W0: tool.LowerFace, W1: tool.UpperFace,
			Loop0: tool.LowerLoop, Loop1: tool.UpperLoop, FrameEmbed: tool.FrameEmbed}
	}
	gone := map[int]struct{}{}
	inPlane := map[int]struct{}{}
	var rims []breprecord.Face
	for _, fi := range removed {
		if err := ctx.Err(); err != nil {
			return breprecord.Payload{}, err
		}
		gone[fi] = struct{}{}
		rp, err := tc.rimPlane(bp, fi)
		if err != nil {
			return breprecord.Payload{}, err
		}
		r := rp.face
		partner, reason, err := NewRimPartner(ctx, RimPartnerInput{
			FaceIndex: fi, Wall: rp.wall, Holes: r.Region.Holes, Embed: rp.e,
			Caps: tc.Caps, Joined: joined, Tools: rimTools, Dilated: dilated,
		})
		if err != nil {
			return breprecord.Payload{}, err
		}
		if reason != "" {
			return breprecord.Payload{}, rimError(r, reason)
		}
		trace, reason, err := TraceRimFaces(ctx, RimTraceInput{
			Faces: cavFaces, Embeds: cavEmbeds, Holes: r.Region.Holes,
			Embed: rp.e, Axis: rp.axis, Level: rp.level, Wall: rp.wall,
			SweepAxis: tc.Axis, CapsEmbed: tc.Caps.Embed, Partner: partner,
		})
		if err != nil {
			return breprecord.Payload{}, err
		}
		if reason != "" {
			return breprecord.Payload{}, rimError(r, reason)
		}
		for _, ci := range trace.InPlane {
			inPlane[ci] = struct{}{}
		}
		regions, reason, err := RimRegions(ctx, budget, r.Region.Outer, trace.Outers)
		if err != nil {
			return breprecord.Payload{}, cancelCause(err)
		}
		if reason != "" {
			return breprecord.Payload{}, rimError(r, reason)
		}
		delta := cavity.SectionDelta()
		if rp.wall {
			delta = charge
		}
		for _, region := range append(regions, trace.Bands...) {
			if err := auditRim(budget, region); err != nil {
				return breprecord.Payload{}, err
			}
			region := region
			rims = append(rims, breprecord.Face{Frame: r.Frame, Region: &region, Outward: r.Outward, Sweep: r.Sweep,
				Z0: r.Z0, Z1: r.Z0, Z0Delta: r.Z0Delta, Z1Delta: r.Z0Delta, Delta: delta})
		}
	}

	out := breprecord.Payload{Xform: bp.Xform}
	for fi, f := range bp.Faces {
		if _, removed := gone[fi]; removed {
			continue
		}
		out.Faces = append(out.Faces, f)
	}
	for ci, f := range cavity.Faces {
		if _, in := inPlane[ci]; in {
			continue
		}
		out.Faces = append(out.Faces, f.Reversed())
	}
	out.Faces = append(out.Faces, rims...)
	return out, nil
}

// rimPlaneRecord is one removed face stated as a planar face: face is the
// statement (frame, region, outward, level, sweep), e its frame's embed, axis
// and level its plane in reference coordinates, and wall whether it is a wall
// of A rather than a cap.
type rimPlaneRecord struct {
	face  breprecord.Face
	e     brepgeom.Embed
	axis  int
	level float64
	wall  bool
}

// rimPlane states removed face fi as a planar face. A cap and a pierced wall
// are planar faces already and are taken as recorded. A wall along k is a
// straight wall along a section axis (removedFaces admits no other): its
// statement is the rectangle it sweeps over [zlo, zhi] in the signed
// permutation frame across its normal axis whose normal is its outward
// normal, restating a wall along k as a planar face does (breprecord.Face.sweep).
func (tc Cut) rimPlane(bp breprecord.Payload, fi int) (rimPlaneRecord, error) {
	f, e := bp.Faces[fi], tc.Embeds[fi]
	if f.Planar() {
		return rimPlaneRecord{face: f, e: e, axis: e.Axis[2], level: e.Sign[2]*f.Z0 + 0, wall: tc.Kinds[fi] == brepgeom.ThroughPierced}, nil
	}
	wall, reason := RestateWall(f.Wall, e, bp.Faces[0].Frame, tc.Axis, tc.Caps.Zlo, tc.Caps.Zhi)
	if reason != "" {
		return rimPlaneRecord{}, rimError(f, reason)
	}
	z := wall.Embed.Sign[2]*wall.Level + 0
	face := breprecord.Face{Frame: wall.Frame, Region: &wall.Region, Outward: true,
		Sweep: tc.Caps.Frame.N(), Z0: z, Z1: z, Role: f.Role}
	return rimPlaneRecord{face: face, e: wall.Embed, axis: wall.Axis, level: wall.Level, wall: true}, nil
}

// rimError is SG7, naming the removed face.
func rimError(r breprecord.Face, what string) error {
	return fmt.Errorf(`%w: the shell's rim at %s does not partner the cavity's trace: %s (modify-general SG7)`, decaderr.ErrUnsupported, r.Role, what)
}

// auditRim is modify §5's audit on one rim region or band: S8 (the
// outer loop walks counter-clockwise and every hole clockwise, each with a
// non-zero area), then S7 (no crossing or contact) and S9 (nesting) through
// the shared audit.
func auditRim(budget *proofbound.WorkBudget, region momentinput.Profile) error {
	for li, loop := range append([]sectionrecord.LoopRecord{region.Outer}, region.Holes...) {
		area, err := sectionaudit.LoopSignedArea(budget, loop)
		if err != nil {
			return cancelCause(err)
		}
		if (li == 0 && !(area > 0)) || (li > 0 && !(area < 0)) {
			return fmt.Errorf(`%w: the shell's rim loop %d is walked against its material`, decaderr.ErrDegenerate, li)
		}
	}
	if err := offset2d.AuditOffsetSectionBudget(budget, region, region); err != nil {
		return cancelCause(err)
	}
	return nil
}
