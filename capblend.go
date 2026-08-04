package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
)

// This file is docs/modify-reach-design.md PR E (§14), which lands §8.3's
// cap-loop chamfer: capBlendPayload, the receiver/selection classification
// RX1's second class needs (every geometric edge of one or more COMPLETE
// prism cap loops, never mixed with lateral edges — S4), gates
// SX4/SX6/SX7/SX10/SX12/SX13, the BX3 roles, and the build. The cap-loop FILLET
// (§8.2, Cylinder/Torus/Sphere patches) is in row E's staged column and is
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
	// patches carries every chamferCap(...) role beside its plane-local
	// geometry (capPatchGeom), populated once at build time
	// (evalCapBlendContext) and reused by the DX7/DX8 surveys —
	// placement-invariant, since capPatchGeom is entirely plane-local
	// (u, v, z), unaffected by xform. It is a SLICE in Table BX row BX3's own
	// deterministic patch order (loop index, then the chamfered cap, then the
	// patch's index within that band), because a survey that walks it emits
	// public output a caller may diff: a map hands Go's randomized start order
	// to every reader, and sorting the role strings is not this order either —
	// it puts patch 10 ahead of patch 2.
	patches []capPatch
}

// capPatch is one built chamfer patch's role paired with the geometry that
// role labels.
type capPatch struct {
	role string
	geom capPatchGeom
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

// extentAlong is the cap-blend body's exact extent interval along an
// arbitrary world direction g (Table DX, DX5: "analytic patch extrema"). A
// linear functional over a compact solid attains its extremes on the
// boundary, and every piece of THIS boundary is ruled between two level
// curves: a trimmed side wall between the original loop at its own two
// straight-slab levels, a chamfer patch (a quad Plane, a trimmed Cone, a
// corner-apex Cone) between the original loop at the side level and the
// offset cap contour at the cap level, and each cap face inside its own
// boundary. A linear functional on a segment is extremized at an endpoint, so
// extremizing the level curves themselves is complete — and every candidate
// is a point the body actually holds, so the interval is attained, not an
// envelope.
//
// The receiver prism's own extent is NOT reusable in either form. Padding it
// by d is unsound as a claim of attainment and never needed: the cap contour
// offsets INTO the material and [z0, z1] is unchanged, so the body is
// contained in the receiver prism. Reusing it unpadded is still wrong,
// because along a diagonal the prism's maximum can sit at the very corner the
// chamfer removes. A stop reads this as an exact endpoint AND as the "is this
// body in the sweep's path" test (stops.go), so an approximation here both
// overruns the sweep and fabricates a dependency on a body it never meets.
//
// That is why this method REFUSES rather than approximates wherever the
// extreme is held by the cap contour with a direction that reads the contour's
// own in-plane displacement (capblend_contour.go). The contour is a computed
// section, not a recorded one, so an extreme it holds is only known to a
// proven displacement — and a stop has no bound to widen, exactly as
// prismPayload's own section displacement leaves ThroughAll/ThroughAllSide
// ErrUnsupported. Every direction whose extreme is held by a recorded
// coordinate, and every direction that reads the contour with no in-plane
// weight at all (the sweep's own axis, where the contour sits at the exact cap
// level), still answers the exact interval it always did.
func (cbp capBlendPayload) extentAlong(g r3.Vec) (float64, float64, error) {
	return cbp.extentAlongContext(context.Background(), g)
}

func (cbp capBlendPayload) extentAlongContext(ctx context.Context, g r3.Vec) (float64, float64, error) {
	return cbp.extentAlongWork(ctx, g, newFreeformWork())
}

// extentAlongWork is the same reading against a caller-owned free-form work
// counter, so a caller already holding one for this record (the bounds pass,
// which asks for three directions in a row) spends what is left of the R7
// ceiling rather than opening a fresh one per axis.
func (cbp capBlendPayload) extentAlongWork(ctx context.Context, g r3.Vec, work *freeformWork) (float64, float64, error) {
	lo, hi, bound, err := cbp.extentBoundedAlong(ctx, g, work)
	if err != nil {
		return 0, 0, err
	}
	if bound != 0 {
		return 0, 0, fmt.Errorf(`%w: a cap-loop chamfer's extent along this direction is held by its own computed cap contour, which is known only to a displacement of %v mm; a stop reads this coordinate as exact and has no bound to widen`, ErrUnsupported, bound)
	}
	return lo, hi, nil
}

// extentBoundedAlong is the reading itself: the interval AND the proven
// displacement of its two endpoints. The displacement is per CANDIDATE and the
// endpoints keep only what survives the extremization, which is what keeps the
// answer exact wherever a recorded coordinate wins: the true minimum lies
// between the minimum of the candidates' lower ends and the minimum of their
// upper ends, so a candidate that loses by more than its own displacement
// contributes nothing to the reported bound. Two mechanisms displace a
// candidate and no others do — the cap contour's own in-plane displacement,
// weighted by how much of the direction lies in the plane, and the rounding of
// a chamfered end's trimmed straight level, weighted by the axial component.
func (cbp capBlendPayload) extentBoundedAlong(ctx context.Context, g r3.Vec, work *freeformWork) (float64, float64, float64, error) {
	pl := cbp.prismLike(0, 0)
	base := cbp.xform.Apply(cbp.frame.Origin()).Dot(g)
	gu := pl.dir(1, 0, 0).Dot(g)
	gv := pl.dir(0, 1, 0).Dot(g)
	gz := pl.dir(0, 0, 1).Dot(g)
	inPlane := upRound(math.Hypot(gu, gv))
	axial := math.Abs(gz)
	lo, hi := math.Inf(1), math.Inf(-1)
	loLower, loUpper := math.Inf(1), math.Inf(1)
	hiLower, hiUpper := math.Inf(-1), math.Inf(-1)
	// A candidate with no displacement at all keeps its exact value at both
	// ends: widening it by a directed rounding would mint a bound out of an
	// arithmetic that committed nothing, which is precisely the claim this
	// reading exists to avoid making in the other direction.
	outward := func(v, allow float64, up bool) float64 {
		if allow == 0 {
			return v
		}
		if up {
			return upRound(v + allow)
		}
		return downRound(v - allow)
	}
	take := func(l, h, z, allow float64) {
		lv, hv := l+z*gz, h+z*gz
		lo = math.Min(lo, lv)
		hi = math.Max(hi, hv)
		loLower = math.Min(loLower, outward(lv, allow, false))
		loUpper = math.Min(loUpper, outward(lv, allow, true))
		hiLower = math.Max(hiLower, outward(hv, allow, false))
		hiUpper = math.Max(hiUpper, outward(hv, allow, true))
	}
	for li, loop := range cbp.loops() {
		if err := ctx.Err(); err != nil {
			return 0, 0, 0, err
		}
		onStart, onEnd := cbp.startLoops[li], cbp.endLoops[li]
		zLo, zHi := cbp.z0, cbp.z1
		loAllow, hiAllow := 0.0, 0.0
		if onStart {
			zLo = cbp.z0 + cbp.d
			loAllow = productUpper(axial, addRoundError(cbp.z0, cbp.d, zLo))
		}
		if onEnd {
			zHi = cbp.z1 - cbp.d
			hiAllow = productUpper(axial, addRoundError(cbp.z1, -cbp.d, zHi))
		}
		l, h, err := boundaryExtremesContext(ctx, ProfileRecord{Outer: loop}, gu, gv, work)
		if err != nil {
			return 0, 0, 0, err
		}
		// The original loop bounds the straight slab at both its own levels;
		// a chamfered end's level is also that band's side-level directrix, and
		// that level is a float sum whose rounding moves the candidate.
		take(l, h, zLo, loAllow)
		take(l, h, zHi, hiAllow)
		if !onStart && !onEnd {
			continue
		}
		// The cap contour is the band's cap-level directrix and the chamfered
		// cap face's own boundary — the same offset loop the build emits, and
		// the same displacement the band's own vertices and edges carry.
		contour, err := capLoopBoundary(ctx, loop, cbp.d)
		if err != nil {
			return 0, 0, 0, err
		}
		delta, err := loopContourDelta(ctx, loop, cbp.d)
		if err != nil {
			return 0, 0, 0, err
		}
		cl, ch, err := boundaryExtremesContext(ctx, ProfileRecord{Outer: contour}, gu, gv, work)
		if err != nil {
			return 0, 0, 0, err
		}
		// The contour sits at the cap's own recorded level, so only the
		// in-plane part of the direction reads its displacement at all.
		contourAllow := productUpper(inPlane, delta)
		if onStart {
			take(cl, ch, cbp.z0, contourAllow)
		}
		if onEnd {
			take(cl, ch, cbp.z1, contourAllow)
		}
	}
	if math.IsInf(lo, 1) {
		return 0, 0, 0, fmt.Errorf(`%w: the recorded region has no boundary`, ErrDegenerate)
	}
	bound := absSumUpper(math.Max(
		math.Max(loUpper-lo, lo-loLower),
		math.Max(hiUpper-hi, hi-hiLower),
	))
	return base + lo, base + hi, bound, nil
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
// SX12, and SX13's axial half) and, once every gate passes, builds the body. It
// is the shared entry ChamferContext calls once a clean cap-loop selection is
// classified. SX13's radial half is decided per circular wall as the band is
// constructed, in capblend_geom.go's capBandRadius.
func buildCapBlend(ctx context.Context, doc *Document, ref StepRef, pp prismPayload, d float64, startLoops, endLoops map[int]bool) (*Body, error) {
	height := pp.z1 - pp.z0
	loops := append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...)

	// SX6 + SX7/SX12: build the "mixed" profile — every selected loop offset
	// d into the material (the cap contour at whichever cap it is selected
	// on; when a loop is selected on BOTH caps the two contours are equal —
	// the offset does not depend on which cap — so one offset serves both),
	// every unselected loop left unchanged — and run the existing exact
	// offset + §5 audit machinery on it. SX6 is the offset's own drop
	// refusal, re-sentinelled here (wrapCapBlendDropError); the audit's own
	// refusals keep the sentinel their §4 stage-6 row decided
	// (wrapCapBlendAuditError) — SX7/SX12 for a crossing or contact, the base
	// S8/S9 ErrDegenerate for broken nesting. Because dc = ds = d in this
	// PR, the ruled patch at axial fraction s meets the parallel section as
	// exactly the loop offset by s*d (docs/modify-reach-design.md §8.3): the
	// offset distance to any fixed feature is monotone non-increasing as the
	// offset grows from 0, so a crossing anywhere in the family occurs no
	// later than it occurs at the full offset d — proving the family
	// disjoint at s=1 certifies every s in [0, 1].
	budget := newWorkBudget(ctx)
	mixed, err := mixedOffsetProfile(budget, pp.profile, d, startLoops, endLoops)
	if err != nil {
		return nil, wrapCapBlendDropError(err)
	}

	// SX7 (band-meeting): a loop chamfered on both caps needs both bands to
	// fit without meeting; a loop chamfered on one cap needs its own band to
	// fit within the sweep.
	//
	// It runs AFTER SX6, which docs/modify-reach-design.md puts in stage 5 while
	// this row is stage 6 ("SX6 precedes SX7"). The two rows answer the same §4
	// existence test in opposite directions, and stage order is how §4 keeps one
	// input from having two sentinels: a setback that empties the cap contour
	// names a body that does not exist at any sweep height, so it is SX6's
	// ErrDegenerate whether the sweep is short enough for the bands to meet or
	// not. Deciding the band reach first would let the sweep height alone pick
	// which sentinel that one nonexistent body reports.
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

	if err := auditOffsetSectionBudget(budget, pp.profile, mixed); err != nil {
		return nil, wrapCapBlendAuditError(err)
	}
	if err := requireCapBlendLevelsSeparate(pp.z0, pp.z1, d, startLoops, endLoops); err != nil {
		return nil, err
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

// requireCapBlendLevelsSeparate is SX13's AXIAL half (Table SX,
// docs/modify-reach-design.md §4 stage 6 and §8.3), the sibling of
// capblend_geom.go's capBandRadius. The band has two directrices and the
// setback displaces both: `d` in the plane, which capBandRadius proves survived
// float64 at each circular wall's own radius, and `d` along the sweep, which
// carries the original loop from the cap level to the side level. A tall enough
// sweep puts `d` under the float64 spacing of that coordinate, and `z1 - d`
// rounds back onto `z1` (or `z0 + d` onto `z0`).
//
// What that returns is not a coarse body but a wrong one, for the same reason
// the radial collapse is. Every patch of the band comes out flat IN the cap
// plane, and a Plane carries its normal with no bound, so each of those faces
// asserts the chamfer's 45-degree taper as a fact about geometry that has none —
// which is exactly what the DX7 undercut survey reads off the surface. (The
// volume is not what refuses the call: the collapsed level reports the correctly
// rounded volume of the true chamfered solid, and its bound honestly charges the
// collapse.) The requested body exists — a real prism with a real, if tiny,
// chamfer — and only float64 cannot name its side level at that sweep
// coordinate, which is §4's ErrUnsupported side of the existence test.
//
// The axial half is a fact about the sweep interval and the setback alone, so it
// is decided once per chamfered cap rather than per wall. SX7's band-reach gate
// above is the opposite failure on the same two numbers and never overlaps this
// one: SX7 refuses a setback so LARGE beside the sweep that the band passes the
// far end, this one a setback so SMALL beside the sweep's own coordinates that
// the level it displaces does not move.
func requireCapBlendLevelsSeparate(z0, z1, d float64, startLoops, endLoops map[int]bool) error {
	refuse := func(which string, level float64) error {
		return fmt.Errorf(`%w: the chamfer setback %v mm is below the float64 spacing of the %s cap's own sweep level %v mm, so the band's side level rounds back onto the cap level and every patch is emitted flat in the cap plane; a wider setback or a shorter sweep states a chamfer this evaluator can build`, ErrUnsupported, d, which, level)
	}
	if anyLoopSelected(startLoops) && z0+d == z0 {
		return refuse(`start`, z0)
	}
	if anyLoopSelected(endLoops) && z1-d == z1 {
		return refuse(`end`, z1)
	}
	return nil
}

// anyLoopSelected reports whether a per-cap loop set names at least one loop.
func anyLoopSelected(loops map[int]bool) bool {
	for _, on := range loops {
		if on {
			return true
		}
	}
	return false
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

// wrapCapBlendDropError re-sentinels the shared offset's own drop refusal as
// SX6 (docs/modify-reach-design.md Table SX). The offset code is Shell's, and
// there its drop is S11a — ErrUnsupported, because the shell body exists and
// only a trimmed-offset kernel is missing. A CAP-LOOP CHAMFER asks a different
// question of the same machinery: the drop says the requested cap contour is
// the empty set (offsetting a radius-4 circle inward by 4 leaves nothing), so
// the body the caller named does not exist at all. §4's existence test puts
// that in stage 5 with ErrDegenerate. The translation lives here, at the one
// call site that asks the existence question, never on errOffsetDrop itself —
// Shell's own sentinel is correct for Shell. The result does not wrap
// errOffsetDrop: carrying its ErrUnsupported along would leave the refusal
// answering to both sentinels, and a caller branching on either would be
// right, which is the ambiguity §4's one-sentinel rule exists to prevent.
func wrapCapBlendDropError(err error) error {
	if !errors.Is(err, errOffsetDrop) {
		return err
	}
	return fmt.Errorf(`%w: the cap-loop offset drops a section feature, so the requested cap contour is empty`, ErrDegenerate)
}

// wrapCapBlendAuditError relabels the shared offset audit's refusal with the
// wording of the stage-6 row it came from, and preserves the sentinel that row
// already decided. SX6 (drop) is settled by mixedOffsetProfile's own call to
// offsetLoopBudget before the audit runs, so a refusal reaching here is one of
// the audit's own: the base S8/S9 nesting rows, which say the offset section is
// decidably broken and therefore that no such body exists (ErrDegenerate), or
// SX7/SX12, which say the body exists and this evaluator cannot trim or merge
// it (ErrUnsupported). Those are opposite existence claims (§4's one-sentinel
// rule), so an S9 refusal keeps ErrDegenerate through %w and errors.Is keeps
// branching on it; only the ErrUnsupported family is restated as SX7/SX12.
func wrapCapBlendAuditError(err error) error {
	if errors.Is(err, context.Canceled) {
		return context.Canceled
	}
	if errors.Is(err, context.DeadlineExceeded) {
		return context.DeadlineExceeded
	}
	if errors.Is(err, ErrDegenerate) {
		return fmt.Errorf(`%w; the section that broke is the cap-loop chamfer's own offset of the selected loop(s) by d`, err)
	}
	return fmt.Errorf(`%w: the cap-loop chamfer's ruled patches cannot be certified disjoint from a non-adjacent boundary (%v); a trimming kernel is not available`, ErrUnsupported, err)
}
