package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/r3"
)

// This file is the chamfer half of docs/modify-reach-design.md PR E (§8.3),
// scoped to §8.3's cap-loop chamfer only: capBlendPayload, the receiver/
// selection classification RX1's second class needs (every geometric edge of
// one or more COMPLETE prism cap loops, never mixed with lateral edges — S4),
// gates SX4/SX6/SX7/SX10/SX12, the BX3 roles, and the build. The cap-loop
// FILLET (§8.2, Cylinder/Torus/Sphere patches) is a separate PR (E2) and is
// not implemented here; neither is WithAsymmetricChamfer (PR A) — this PR
// covers the equal-setback case only, dc = ds = d (§8.3).
//
// The reduction mirrors modify-design §2's lateral-edge one: the selected
// cap loop's boundary is offset dc = d into the material (the "cap contour",
// still in the cap plane — shell_offset.go's exact per-feature offset, reused
// unchanged) while the ORIGINAL loop is held at its own (u, v) and moved
// axially ds = d into the material (the "side contour"). The band between the
// two contours is a ruled surface: a Plane for a line wall, a Cone for a
// circular wall (concentric with the wall — the offset preserves the center),
// and a Cone whose apex is the ORIGINAL corner point for a reflex corner's
// extra offset arc (the offset loop's own miter carries no extra patch: two
// neighboring patches meet directly along the shared slanted edge from the
// offset corner down to the original corner, so a convex corner needs no
// third patch). The prism's own side wall for a chamfered loop is trimmed to
// the interval below the band, built with the unmodified buildLoopSidesAs —
// the wall's near-cap edge IS the band's side-level boundary, shared, never
// re-derived.

// capBlendPayload is the evaluator's own record of a complete-cap-loop
// chamfer result (docs/modify-reach-design.md §8.3, BX3): the receiver's
// unrewritten section (unselected loops build exactly as an ordinary prism;
// selected loops are chamfered per below), the plane frame, sweep interval,
// accumulated placement, the single equal setback d, and which loop indices
// (into append(profile.Outer, profile.Holes...)) are chamfered on which cap.
// It is evaluator-private: the recipe records only the selector and d, never
// the rewritten geometry (modify §11's role rule — a role indexes the record
// it labels, so a result's roles are minted from the result's own record,
// never inherited).
type capBlendPayload struct {
	profile    ProfileRecord
	frame      r3.Frame
	z0, z1     float64
	xform      r3.Transform
	d          float64
	startLoops map[int]bool // loop index -> chamfered on the z0 cap
	endLoops   map[int]bool // loop index -> chamfered on the z1 cap
}

// transform is the accumulated rigid placement.
func (cbp capBlendPayload) transform() r3.Transform { return cbp.xform }

// placed re-evaluates the same record under the composed motion (evaluator
// §8): every gate this PR enforces is a closed-form fact of the RECORD, so a
// placed body re-derives the identical chamfer.
func (cbp capBlendPayload) placed(ctx context.Context, d *Document, ref StepRef, composed r3.Transform) (*Body, error) {
	cbp.xform = composed
	return evalCapBlendContext(ctx, d, ref, cbp)
}

// prismLike returns a prismPayload sharing the receiver's frame and
// placement over [z0, z1], so the point/dir machinery already built for
// prisms serves the cap-blend build too.
func (cbp capBlendPayload) prismLike(z0, z1 float64) prismPayload {
	return prismPayload{frame: cbp.frame, z0: z0, z1: z1, xform: cbp.xform}
}

// extentAlong is the cap-blend body's extent interval along an arbitrary
// world direction g (Table DX, DX5): the unrewritten receiver's own exact
// extent, widened by d on both ends. Every chamfer point lies within d of the
// original prism's own boundary (a convex reduction stays strictly inside
// it; a reflex corner's apex patch bulges by at most d), so this is a sound
// — if not razor-tight — envelope, evaluated directly rather than reused from
// a tighter but unproven closed form.
func (cbp capBlendPayload) extentAlong(g r3.Vec) (float64, float64, error) {
	pp := prismPayload{profile: cbp.profile, frame: cbp.frame, z0: cbp.z0, z1: cbp.z1, xform: cbp.xform}
	lo, hi, err := pp.extentAlong(g)
	if err != nil {
		return 0, 0, err
	}
	pad := cbp.d * g.Len()
	return lo - pad, hi + pad, nil
}

// requireNotCapBlendReceiver is SX10 (docs/modify-reach-design.md Table SX):
// another modify op on a capBlendPayload receiver is staged — the body
// exists, but composing another feature onto a cap-blend result is not built.
// Called before the ordinary prismPayload cast in Fillet/Chamfer/Shell so the
// more specific reason leads the generic "not a prism" refusal.
func requireNotCapBlendReceiver(payload featurePayload, op string) error {
	if _, ok := payload.(capBlendPayload); ok {
		return fmt.Errorf(`%w: this evaluator does not yet compose another modify op onto a cap-loop chamfer result; %s a receiver this evaluator built directly`, ErrUnsupported, op)
	}
	return nil
}

// capLoops returns the receiver's loops as append(Outer, Holes...), the same
// index space Table BX's roles and the prism's own side(i,j) roles use.
func (cbp capBlendPayload) loops() []LoopRecord {
	return append([]LoopRecord{cbp.profile.Outer}, cbp.profile.Holes...)
}

// classifyChamferSelection is RX1's second class plus SX4 (Table SX):
// distinguishes a lateral-edge selection (the base modify-design §7 case,
// left to the existing code path unchanged) from a complete-cap-loop
// selection, and refuses a selection that mixes the two classes or covers
// only part of a cap loop. It returns the loop indices selected per cap when
// the selection is a clean cap-loop selection; lateral is true when every
// selected edge is instead an ordinary lateral edge (the caller then runs the
// base path).
func classifyChamferSelection(ctx context.Context, pp prismPayload, b *Body, sel EdgeSelector, edges []*Edge) (startLoops, endLoops map[int]bool, lateral bool, err error) {
	budget := newWorkBudget(ctx)
	cornerLoops, err := prismCornerLoopsBudget(budget, pp)
	if err != nil {
		return nil, nil, false, err
	}

	// Index every rim edge of every complete cap loop by (cap, loop).
	type capKey struct {
		start bool
		loop  int
	}
	capEdgeOf := map[*Edge]capKey{}
	capLoopSize := map[capKey]int{}
	for _, f := range b.Faces() {
		var start bool
		var isCap bool
		for _, o := range f.Origins() {
			if o.Role == roleCapStart {
				start, isCap = true, true
			}
			if o.Role == roleCapEnd {
				start, isCap = false, true
			}
		}
		if !isCap {
			continue
		}
		for li, l := range f.Loops() {
			key := capKey{start: start, loop: li}
			for _, e := range l.Edges() {
				capEdgeOf[e] = key
				capLoopSize[key]++
			}
		}
	}

	lateralCount, capCount := 0, 0
	touched := map[capKey]map[*Edge]bool{}
	var unmatched *Edge
	for ei, e := range edges {
		if _, _, found, mErr := matchCornerBudget(budget, pp, cornerLoops, e); mErr != nil {
			return nil, nil, false, mErr
		} else if found {
			lateralCount++
			continue
		}
		if key, ok := capEdgeOf[e]; ok {
			capCount++
			if touched[key] == nil {
				touched[key] = map[*Edge]bool{}
			}
			touched[key][e] = true
			continue
		}
		if unmatched == nil {
			unmatched = edges[ei]
		}
	}

	switch {
	case lateralCount > 0 && capCount == 0 && unmatched == nil:
		return nil, nil, true, nil
	case lateralCount > 0 && capCount > 0:
		return nil, nil, false, fmt.Errorf(`%w: the selection mixes lateral edges with prism cap-loop edges in one call; selector %s`, ErrUnsupported, sel)
	case capCount == 0:
		if unmatched != nil {
			return nil, nil, false, fmt.Errorf(`%w: a chamfer of a cap edge is the vertex-blend problem, not yet supported; selector %s`, ErrUnsupported, sel)
		}
		return nil, nil, false, fmt.Errorf(`%w: the selector matched no edges to chamfer`, ErrNoMatch)
	}

	startLoops = map[int]bool{}
	endLoops = map[int]bool{}
	for key, set := range touched {
		if len(set) != capLoopSize[key] {
			return nil, nil, false, fmt.Errorf(`%w: the selection covers only part of a cap loop; every geometric edge of a complete loop must be selected; selector %s`, ErrUnsupported, sel)
		}
		if key.start {
			startLoops[key.loop] = true
		} else {
			endLoops[key.loop] = true
		}
	}
	return startLoops, endLoops, false, nil
}

// buildCapBlend runs the existence and constructed-geometry gates (SX6, SX7,
// SX12) and, once every gate passes, builds the body. It is the shared entry
// ChamferContext calls once a clean cap-loop selection is classified.
func buildCapBlend(ctx context.Context, doc *Document, ref StepRef, pp prismPayload, d float64, startLoops, endLoops map[int]bool) (*Body, error) {
	height := pp.z1 - pp.z0
	loops := append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...)

	// SX7 (band-meeting): a loop chamfered on both caps needs both bands to
	// fit without meeting; a loop chamfered on one cap needs its own band to
	// fit within the sweep. Existence-first: this is a fact about the sweep
	// interval alone, decided before the 2D offset is built.
	for li := range loops {
		reach := 0.0
		if startLoops[li] {
			reach += d
		}
		if endLoops[li] {
			reach += d
		}
		if reach >= height {
			return nil, fmt.Errorf(`%w: the chamfer band(s) on loop %d reach or pass the opposite end of the sweep; a merging kernel is not available`, ErrUnsupported, li)
		}
	}

	// SX6 + SX7/SX12: build the "mixed" profile — every selected loop offset
	// d into the material (the cap contour at whichever cap it is selected
	// on; when a loop is selected on BOTH caps the two contours are equal —
	// the offset does not depend on which cap — so one offset serves both),
	// every unselected loop left unchanged — and run the existing exact
	// offset + §5 audit machinery on it. SX6 is the offset's own drop
	// refusal (errOffsetDrop, wrapping ErrUnsupported); SX7/SX12 are the
	// audit's crossing/contact/nesting refusals. Because dc = ds = d in this
	// PR, the ruled patch at axial fraction s meets the parallel section as
	// exactly the loop offset by s*d (docs/modify-reach-design.md §8.3): the
	// offset distance to any fixed feature is monotone non-increasing as the
	// offset grows from 0, so a crossing anywhere in the family occurs no
	// later than it occurs at the full offset d — proving the family
	// disjoint at s=1 certifies every s in [0, 1].
	budget := newWorkBudget(ctx)
	mixed, err := mixedOffsetProfile(budget, pp.profile, d, startLoops, endLoops)
	if err != nil {
		return nil, err
	}
	if err := auditOffsetSectionBudget(budget, pp.profile, mixed); err != nil {
		return nil, wrapCapBlendAuditError(err)
	}

	cbp := capBlendPayload{
		profile:    pp.profile,
		frame:      pp.frame,
		z0:         pp.z0,
		z1:         pp.z1,
		xform:      pp.xform,
		d:          d,
		startLoops: startLoops,
		endLoops:   endLoops,
	}
	return evalCapBlendContext(ctx, doc, ref, cbp)
}

// mixedOffsetProfile offsets exactly the loops named in startLoops/endLoops
// (their union — a loop chamfered on either or both caps takes the same
// in-plane offset) by d into the material, leaving every other loop
// unchanged. It reuses offsetLoopBudget's per-feature offset unmodified.
func mixedOffsetProfile(budget *workBudget, profile ProfileRecord, d float64, startLoops, endLoops map[int]bool) (ProfileRecord, error) {
	loops, err := prismCornerLoopsBudget(budget, prismPayload{profile: profile})
	if err != nil {
		return ProfileRecord{}, err
	}
	orig := append([]LoopRecord{profile.Outer}, profile.Holes...)
	out := make([]LoopRecord, len(orig))
	for li := range orig {
		if err := wallBudgetStep(budget); err != nil {
			return ProfileRecord{}, err
		}
		if !startLoops[li] && !endLoops[li] {
			out[li] = cloneLoopRecord(orig[li])
			continue
		}
		segs, err := offsetLoopBudget(budget, loops[li], 1, d)
		if err != nil {
			return ProfileRecord{}, err
		}
		out[li] = LoopRecord{Segments: segs}
	}
	return ProfileRecord{Outer: out[0], Holes: out[1:]}, nil
}

// wrapCapBlendAuditError relabels the shared offset audit's refusal with
// SX7/SX12's own wording: SX6 (drop) has already been decided by
// mixedOffsetProfile's own call to offsetLoopBudget before the audit runs, so
// every refusal reaching here is the audit's crossing/contact/nesting one —
// modify-reach §8.3's ruled-patch intersection question, §4's SX7/SX12.
func wrapCapBlendAuditError(err error) error {
	if errors.Is(err, context.Canceled) {
		return context.Canceled
	}
	if errors.Is(err, context.DeadlineExceeded) {
		return context.DeadlineExceeded
	}
	return fmt.Errorf(`%w: the cap-loop chamfer's ruled patches cannot be certified disjoint from a non-adjacent boundary (%v); a trimming kernel is not available`, ErrUnsupported, err)
}
