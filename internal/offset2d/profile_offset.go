package offset2d

import (
	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file adapts the exact section offset of agent-docs/modify-design.md §7 and
// audits its record for a shell. The offset is per-feature and topology-preserving:
// a line offsets to a parallel line, a circle to a concentric circle, a corner
// closes with a miter or an arc of radius t about the corner point or, at a G1
// join, with the corner moved t along the shared normal (§7). Every piece
// is a line or an arc, so P ⊖ t / P ⊕ t is again a ProfileRecord in the
// line-and-arc vocabulary (§2). The offset is decad's OWN synthesized geometry,
// proven by exact closed-form tests, never a residual (§5, CLAUDE.md's
// falsify-only rule governs only what sketch hands over).
//
// A feature the offset DROPS or a miter that does not close is a feature-set
// change the evaluator cannot build without a trimmed-offset kernel — S11a /
// S11, ErrUnsupported, caught here as the offset is constructed and so
// antecedent to the audit (§4). A drop takes two shapes, both caught here: a
// circular segment whose offset radius reaches zero (OffsetRadius),
// and a whole loop whose erosion is empty: a polygonal hole narrower than 2t
// offset outward, whose corner joins overshoot every walk so each runs backward
// (WalkConsumed). The eroded loop keeps its walk sense, so its signed
// area does not change sign and the §5 audit's S8 cannot see it — which is why
// the drop is decided per walk, here, before there is any section to audit. A
// merge the audit catches later is S11b.

// ProfileCornerLoopsBudget resolves every loop of a recorded section into its
// coalesced corner walk in the section's own plane-local coordinates. A prism
// reads its section through it, and a revolve its meridian
// (internal/brepmodify/revolve_blend.go): the corner rewrite is the same 2D construction for both.
func ProfileCornerLoopsBudget(budget *proofbound.WorkBudget, profile momentinput.Profile) ([]survey2d.CornerLoop, error) {
	walks, err := boundarywalk.ModifyLoopsBudget(budget, boundarywalk.Profile(profile))
	if err != nil {
		return nil, err
	}
	out := make([]survey2d.CornerLoop, len(walks))
	for i, loop := range walks {
		out[i] = survey2d.CornerLoop{Walks: loop}
	}
	return out, nil
}

// RewriteProfileBudget rebuilds the loops of a recorded profile with their
// selected corner blends and returns the connector positions in each loop.
func RewriteProfileBudget(budget *proofbound.WorkBudget, orig momentinput.Profile, loops []survey2d.CornerLoop, blendAt []map[int]*Blend) (momentinput.Profile, []map[int]struct{}, error) {
	origLoops := append([]sectionrecord.LoopRecord{orig.Outer}, orig.Holes...)
	newLoops := make([]sectionrecord.LoopRecord, len(origLoops))
	blendSegs := make([]map[int]struct{}, len(origLoops))
	for li := range origLoops {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return momentinput.Profile{}, nil, err
		}
		blendSegs[li] = map[int]struct{}{}
		if len(blendAt[li]) == 0 {
			newLoops[li] = sectionrecord.CloneLoopRecord(origLoops[li])
			continue
		}
		segs, connectors, err := RewriteLoop(budget, loops[li].Walks, blendAt[li])
		if err != nil {
			return momentinput.Profile{}, nil, err
		}
		newLoops[li] = sectionrecord.LoopRecord{Segments: segs}
		blendSegs[li] = connectors
	}
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return momentinput.Profile{}, nil, err
	}
	return momentinput.Profile{Outer: newLoops[0], Holes: newLoops[1:]}, blendSegs, nil
}

// AuditRewriteBudget converts the root corner records into the neutral
// section audit's ordered rewrite inputs.
func AuditRewriteBudget(budget *proofbound.WorkBudget, orig, rewritten momentinput.Profile,
	loops []survey2d.CornerLoop, blendAt []map[int]*Blend) error {
	records := make([]sectionaudit.RewriteLoop, len(loops))
	for li, loop := range loops {
		cutbacks := make(map[int]sectionaudit.Cutback, len(blendAt[li]))
		for ci, blend := range blendAt[li] {
			if blend != nil {
				cutbacks[ci] = sectionaudit.Cutback{A: blend.CutbackA, B: blend.CutbackB}
			}
		}
		records[li] = sectionaudit.RewriteLoop{Walks: loop.Walks, Cutbacks: cutbacks}
	}
	return sectionaudit.AuditRewrite(budget, orig, rewritten, records)
}

// OffsetProfile computes the topology-preserving offset of a section: P ⊖ t
// (inward, s = +1) or P ⊕ t (outward, s = −1), each loop offset in its own
// sense (agent-docs/modify-design.md §7). A dropped feature is S11a (ErrDrop);
// a non-closing miter is S11 (ErrTopology), naming the loop and the
// corner (CornerTopologyError). Both are ErrUnsupported. tol is the shell
// tolerance BuildLoop classifies with.
func OffsetProfile(budget *proofbound.WorkBudget, profile momentinput.Profile, s, t, tol float64) (momentinput.Profile, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return momentinput.Profile{}, err
	}
	loops, err := ProfileCornerLoopsBudget(budget, profile)
	if err != nil {
		return momentinput.Profile{}, err
	}
	out := make([]sectionrecord.LoopRecord, len(loops))
	for i, loop := range loops {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return momentinput.Profile{}, err
		}
		segs, err := BuildLoop(budget, loop.Walks, s, t, tol)
		if err != nil {
			return momentinput.Profile{}, InLoop(err, i)
		}
		out[i] = sectionrecord.LoopRecord{Segments: segs}
	}
	return momentinput.Profile{Outer: out[0], Holes: out[1:]}, nil
}

// AuditOffsetSectionBudget runs the shared §5 audit (AuditRewriteBudget) on a
// shell's offset section, in §4's order: S8 (orientation — an offset loop turned
// inside out is ErrDegenerate), the crossing test (a crossing or boundary contact
// of offset loops — S11b, ErrUnsupported), then S9 (nesting). A shell mints no
// cutback, so the empty fillet map makes the S6 trim test a no-op — the one test
// that cannot fire on an offset (§8).
func AuditOffsetSectionBudget(budget *proofbound.WorkBudget, orig, offset momentinput.Profile) error {
	loops, err := ProfileCornerLoopsBudget(budget, offset)
	if err != nil {
		return err
	}
	empty := make([]map[int]*Blend, len(loops))
	for i := range empty {
		empty[i] = map[int]*Blend{}
	}
	return AuditRewriteBudget(budget, orig, offset, loops, empty)
}

// OffsetSectionDelta is the cup's offset displacement (agent-docs/modify-design.md
// §9): a proven upper bound on how far any boundary point of the offset
// section OffsetProfile records for (profile, s, t) sits from the boundary of
// the offset the shell DENOTES — P ⊖ t* inward, P ⊕ t* outward — where t* is
// the caller's thickness in exact millimetres, anywhere within tDelta of the
// held t (extent.MagnitudeInBounded's conversion bound). tol is the shell
// tolerance SectionDeltaFromWalks classifies with.
func OffsetSectionDelta(budget *proofbound.WorkBudget, profile momentinput.Profile, s, t, tDelta, tol float64) (float64, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return 0, err
	}
	amount, err := OffsetAmount(s, t, tDelta)
	if err != nil {
		return 0, err
	}
	loops, err := ProfileCornerLoopsBudget(budget, profile)
	if err != nil {
		return 0, err
	}
	walks := make([][]survey2d.SideWalk, len(loops))
	for i, loop := range loops {
		walks[i] = loop.Walks
	}
	return SectionDeltaFromWalks(budget, walks, s, t, amount, tol)
}
