package decad

import (
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file adapts the exact section offset of docs/modify-design.md §7 and
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
// circular segment whose offset radius reaches zero (offset2d.OffsetRadius),
// and a whole loop whose erosion is empty: a polygonal hole narrower than 2t
// offset outward, whose corner joins overshoot every walk so each runs backward
// (offset2d.WalkConsumed). The eroded loop keeps its walk sense, so its signed
// area does not change sign and the §5 audit's S8 cannot see it — which is why
// the drop is decided per walk, here, before there is any section to audit. A
// merge the audit catches later is S11b.

// offsetProfile computes the topology-preserving offset of a section: P ⊖ t
// (inward, s = +1) or P ⊕ t (outward, s = −1), each loop offset in its own
// sense (docs/modify-design.md §7). A dropped feature is S11a (offset2d.ErrDrop);
// a non-closing miter is S11 (offset2d.ErrTopology), naming the loop and the
// corner (offset2d.CornerTopologyError). Both are ErrUnsupported.
func offsetProfile(budget *proofbound.WorkBudget, profile ProfileRecord, s, t float64) (ProfileRecord, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return ProfileRecord{}, err
	}
	loops, err := prismCornerLoopsBudget(budget, prismPayload{profile: profile})
	if err != nil {
		return ProfileRecord{}, err
	}
	out := make([]LoopRecord, len(loops))
	for i, loop := range loops {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return ProfileRecord{}, err
		}
		segs, err := offset2d.BuildLoop(budget, loop.walks, s, t, shellTol)
		if err != nil {
			return ProfileRecord{}, offset2d.InLoop(err, i)
		}
		out[i] = LoopRecord{Segments: segs}
	}
	return ProfileRecord{Outer: out[0], Holes: out[1:]}, nil
}

// offsetJoinsBudget resolves every corner join of one coalesced loop of two or
// more walks, in walk order: corner i sits at walk i's start (== walk i−1's
// end). It is the one place the corner rule of docs/modify-design.md §7 is
// decided by offset2d.SectionJoinsBudget. The build (offset2d.BuildLoop) and
// its displacement proof (offsetSectionDelta) read those same joins.
func offsetJoinsBudget(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, s, t float64) ([]cornerJoin, error) {
	result, err := offset2d.SectionJoinsBudget(budget, walks, s, t, shellTol)
	if err != nil {
		return nil, err
	}
	joins := make([]cornerJoin, len(result))
	for i, j := range result {
		joins[i] = cornerJoin{
			arc: j.Arc, g1: j.G1, vU: j.VertU, vV: j.VertV,
			m:  Point2{U: j.M.U, V: j.M.V},
			pA: Point2{U: j.PA.U, V: j.PA.V},
			pB: Point2{U: j.PB.U, V: j.PB.V},
		}
	}
	return joins, nil
}

// cornerJoin is one corner's resolved offset join: a miter point m, or an arc
// of radius t about (vU, vV) from pA (the arriving walk's offset end) to pB (the
// leaving walk's offset start).
type cornerJoin struct {
	arc bool
	// g1 marks a G1 join (modify §7): m is the leaving walk's offset start
	// v + s·t·n̂, not a carrier intersection, and the corner ruling v→m is the
	// exact affine locus.
	g1     bool
	vU, vV float64
	m      Point2
	pA, pB Point2
}

// auditOffsetSectionBudget runs the shared §5 audit (fillet_audit.go) on a shell's
// offset section, in §4's order: S8 (orientation — an offset loop turned inside
// out is ErrDegenerate), the crossing test (a crossing or boundary contact of
// offset loops — S11b, ErrUnsupported), then S9 (nesting). A shell mints no
// cutback, so the empty fillet map makes the S6 trim test a no-op — the one test
// that cannot fire on an offset (§8).
func auditOffsetSectionBudget(budget *proofbound.WorkBudget, orig, offset ProfileRecord) error {
	loops, err := prismCornerLoopsBudget(budget, prismPayload{profile: offset})
	if err != nil {
		return err
	}
	empty := make([]map[int]*cornerBlend, len(loops))
	for i := range empty {
		empty[i] = map[int]*cornerBlend{}
	}
	return auditRewriteBudget(budget, orig, offset, loops, empty)
}

// offsetSectionDelta is the cup's offset displacement (docs/modify-design.md
// §9): a proven upper bound on how far any boundary point of the offset
// section offsetProfile records for (profile, s, t) sits from the boundary of
// the offset the shell DENOTES — P ⊖ t* inward, P ⊕ t* outward — where t* is
// the caller's thickness in exact millimetres, anywhere within tDelta of the
// held t (magnitudeInBounded's conversion bound).
func offsetSectionDelta(budget *proofbound.WorkBudget, profile ProfileRecord, s, t, tDelta float64) (float64, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return 0, err
	}
	amount, err := offset2d.OffsetAmount(s, t, tDelta)
	if err != nil {
		return 0, err
	}
	loops, err := prismCornerLoopsBudget(budget, prismPayload{profile: profile})
	if err != nil {
		return 0, err
	}
	walks := make([][]survey2d.SideWalk, len(loops))
	for i, loop := range loops {
		walks[i] = loop.walks
	}
	return offset2d.SectionDeltaFromWalks(budget, walks, s, t, amount, shellTol)
}
