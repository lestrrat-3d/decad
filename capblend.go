package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// This file is docs/modify-reach-design.md PR E (§14), which lands §8.3's
// cap-loop chamfer: capBlendPayload, the receiver/selection classification
// RX1's second class needs (every geometric edge of one or more COMPLETE
// prism cap loops, never mixed with lateral edges — S4), gates
// SX4/SX6/SX7/SX10/SX12/SX13, the BX3 roles, and the build. The cap-loop FILLET
// (§8.2, Cylinder/Torus/Sphere patches) is in row E's staged column and is
// not implemented here. Each chamfered cap carries its own two setbacks
// (capSetback): dc = ds = d for an equal chamfer, and the pair §8.3.1 assigns
// from WithAsymmetricChamfer's reference face for a two-distance one.
//
// The reduction mirrors modify-design §2's lateral-edge one: the selected
// cap loop's boundary is offset dc into the material (the "cap contour",
// still in the cap plane — shell_offset.go's exact per-feature offset, reused
// unchanged) while the ORIGINAL loop is held at its own (u, v) and moved
// axially ds into the material (the "side contour"). The band between the
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
// accumulated placement, each chamfered cap's two setbacks, and which loop
// indices (into append(profile.Outer, profile.Holes...)) are chamfered on which
// cap. It is evaluator-private: the public call supplies only the selector and
// its distances, never
// the rewritten geometry (modify §11's role rule — a role indexes the record
// it labels, so a result's roles are minted from the result's own record,
// never inherited).
type capBlendPayload struct {
	profile    ProfileRecord
	frame      r3.Frame
	z0, z1     float64
	z0Delta    float64
	z1Delta    float64
	xform      r3.Transform
	start, end capSetback   // the z0 and z1 caps' own setbacks
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
	// bandDelta is each built band's own cap-contour displacement
	// (capBandResult.delta, capband.ContourDisplacement), keyed by the
	// (loop, cap) that band sits on. buildCapBand already computes it once for
	// every cap-level vertex, edge and area reading of that band; storing it here
	// lets a later reader — the tessellator, docs/tessellation-reach-design.md
	// §7 — charge the SAME number those readings did rather than re-derive a
	// second one from the same offset. It is plane-local and
	// placement-invariant, exactly as patches is.
	bandDelta map[capBandKey]float64
}

// capSetback is one chamfered cap's two setbacks (docs/modify-reach-design.md
// §8.3.1): dc across the cap face — the in-plane offset of the cap contour —
// and ds down the side wall — the axial distance from the cap level to the
// side level — each beside the rounding its own unit conversion committed.
// dsDelta is charged by every level built from ds. dcDelta is charged by the
// cap contour's displacement (capband.ContourDisplacement), which encloses the contour
// over every offset amount within dcDelta of dc, and by every cap-level
// length that reads dc as a radius. An equal chamfer holds dc == ds == d.
type capSetback struct {
	dc, dcDelta, ds, dsDelta float64
}

// axialUpper is a proven upper bound on the axial rise the band denotes: the
// side setback the caller stated, which lies within dsDelta of ds. A setback
// stated in millimetres converts exactly, and the bound is |ds| itself.
func (s capSetback) axialUpper() float64 {
	if s.dsDelta > 0 {
		return proofbound.AbsSumUpper(s.ds, s.dsDelta)
	}
	return math.Abs(s.ds)
}

// setbackAt returns the setbacks of the cap a band with material sense
// matSign sits on: positive is the start cap, negative the end cap.
func (cbp capBlendPayload) setbackAt(matSign float64) capSetback {
	if matSign > 0 {
		return cbp.start
	}
	return cbp.end
}

// loopSetback is the setbacks of whichever cap loop li is chamfered on. A loop
// chamfered on both caps takes one pair on both: resolveCapSetbacks refuses a
// loop whose two caps pick its setbacks differently.
func (cbp capBlendPayload) loopSetback(li int) capSetback {
	if cbp.startLoops[li] {
		return cbp.start
	}
	return cbp.end
}

// loopBandDelta is the larger contour displacement of loop li's chamfer bands
// (evalCapBlendContext's bandDelta), zero where it is chamfered on neither cap.
func (cbp capBlendPayload) loopBandDelta(li int) float64 {
	delta := 0.0
	for _, start := range [...]bool{true, false} {
		if d, ok := cbp.bandDelta[capBandKey{loop: li, start: start}]; ok {
			delta = math.Max(delta, d)
		}
	}
	return delta
}

// loopOffset is loop li's own in-plane offset, its loopSetback's dc.
func (cbp capBlendPayload) loopOffset(li int) float64 { return cbp.loopSetback(li).dc }

// capBandKey names one chamfer band: the loop it belongs to (an index into
// loops(), the same index space Table BX's roles use) and which cap it sits on.
type capBandKey struct {
	loop  int
	start bool
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
func (cbp capBlendPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	cbp.xform = composed
	return evalCapBlendContext(ctx, d, ref, cbp)
}

// prismLike returns a prismPayload sharing the receiver's frame and
// placement over [z0, z1], so the point/dir machinery already built for
// prisms serves the cap-blend build too.
func (cbp capBlendPayload) prismLike(z0, z1 float64) prismPayload {
	return prismPayload{
		frame: cbp.frame, z0: z0, z1: z1,
		z0Delta: cbp.z0Delta, z1Delta: cbp.z1Delta,
		xform: cbp.xform,
	}
}

// capBandLevel preserves the selected end's axial displacement while a
// chamfer band derives its cap and side levels. The start band has positive
// material sense; the end band has negative material sense.
func (cbp capBlendPayload) capBandLevel(capZ, matSign float64) proofbound.BoundedScalar {
	capDelta := cbp.z1Delta
	if matSign > 0 {
		capDelta = cbp.z0Delta
	}
	return proofbound.MeasuredScalar(capZ, capDelta)
}

// axialDelta is the larger sweep-level displacement a body-relative stop must
// preserve when it resolves against this cap blend. A chamfered end also
// carries the setback conversion and float-sum rounding of its level.
func (cbp capBlendPayload) axialDelta() float64 {
	z0Delta, z1Delta := cbp.z0Delta, cbp.z1Delta
	if len(cbp.startLoops) != 0 {
		s := cbp.start
		z0Delta = proofbound.AbsSumUpper(z0Delta, s.dsDelta, proofarith.AddRoundError(cbp.z0, s.ds, cbp.z0+s.ds))
	}
	if len(cbp.endLoops) != 0 {
		s := cbp.end
		z1Delta = proofbound.AbsSumUpper(z1Delta, s.dsDelta, proofarith.AddRoundError(cbp.z1, -s.ds, cbp.z1-s.ds))
	}
	return math.Max(z0Delta, z1Delta)
}

// extentAlong is the through-all stop's reading of the cap-blend body
// (stops.go): its extent interval along an arbitrary world direction g, beside
// the proven displacement extentBoundedAlong states for its two ends (Table DX,
// DX5: "analytic patch extrema"). A
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
// chamfer removes. A stop reads this interval both as the level it records and
// as the "is this body in the sweep's path" test, so an approximation with no
// stated displacement would overrun the sweep and fabricate a dependency on a
// body it never meets.
//
// That is why this method publishes the displacement instead of hiding it. The
// cap contour is a computed section, not a recorded one (capblend_contour.go),
// so an extreme it holds is known only to a proven displacement — which the
// stop charges to the level it resolves, and outside which it decides its own
// in-path test. On an unplaced body every direction whose extreme is held by an
// exact recorded coordinate still answers the exact interval it always did,
// with a zero displacement; under a placement the reading's own arithmetic can
// round even there, and charges what it commits.
// A direction along the sweep ignores contour displacement but
// still reads the cap level's own inherited axial displacement.
func (cbp capBlendPayload) extentAlong(g r3.Vec) (float64, float64, float64, error) {
	return cbp.extentBoundedAlong(context.Background(), g, freeform.NewFreeformWork())
}

// extentBoundedAlong is the reading itself: the interval AND the proven
// displacement of its two endpoints. The displacement is per CANDIDATE and the
// endpoints keep only what survives the extremization, which is what keeps the
// answer exact wherever a recorded coordinate wins: the true minimum lies
// between the minimum of the candidates' lower ends and the minimum of their
// upper ends, so a candidate that loses by more than its own displacement
// contributes nothing to the reported bound. Three mechanisms displace a
// candidate: an inherited end displacement, the cap contour's own in-plane
// displacement, and the rounding of a chamfered end's trimmed straight level.
// A fourth, prismPlacementCoeffAllow, displaces every candidate the SAME way
// — the frame and placement's own rounding of base/gu/gv/gz — and so composes
// outward with the per-candidate maximum rather than folding into it. A fifth
// does the same one step later: the reading's own summation of base with the
// extremized candidate, charged exactly by proofbound.ExactSumRound (internal/proofbound/bounds.go), since a
// placement can leave every coefficient exactly right and still round when the
// terms are added.
func (cbp capBlendPayload) extentBoundedAlong(ctx context.Context, g r3.Vec, work *freeform.FreeformWork) (float64, float64, float64, error) {
	pl := cbp.prismLike(0, 0)
	base := cbp.xform.Apply(cbp.frame.Origin()).Dot(g)
	gu := pl.dir(1, 0, 0).Dot(g)
	gv := pl.dir(0, 1, 0).Dot(g)
	gz := pl.dir(0, 0, 1).Dot(g)
	inPlane := proofbound.UpRound(math.Hypot(gu, gv))
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
			return proofbound.UpRound(v + allow)
		}
		return freeform.DownRound(v - allow)
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
		loAllow := proofbound.ProductUpper(axial, cbp.z0Delta)
		hiAllow := proofbound.ProductUpper(axial, cbp.z1Delta)
		if onStart {
			s := cbp.start
			zLo = cbp.z0 + s.ds
			loAllow = proofbound.AbsSumUpper(loAllow, proofbound.ProductUpper(axial, proofbound.AbsSumUpper(s.dsDelta, proofarith.AddRoundError(cbp.z0, s.ds, zLo))))
		}
		if onEnd {
			s := cbp.end
			zHi = cbp.z1 - s.ds
			hiAllow = proofbound.AbsSumUpper(hiAllow, proofbound.ProductUpper(axial, proofbound.AbsSumUpper(s.dsDelta, proofarith.AddRoundError(cbp.z1, -s.ds, zHi))))
		}
		// The boundary scan states its own displacement per candidate — nonzero
		// wherever a circular candidate's apex is not exactly representable
		// (extrude.go's circularExtremeInterval) — and it displaces the reading
		// IN PLANE, so it composes with the axial terms above rather than
		// replacing either.
		l, h, planeAllow, err := boundaryExtremesBoundedContext(ctx, ProfileRecord{Outer: loop}, gu, gv, work, nil)
		if err != nil {
			return 0, 0, 0, err
		}
		// The original loop bounds the straight slab at both its own levels;
		// a chamfered end's level is also that band's side-level directrix, and
		// that level is a float sum whose rounding moves the candidate.
		take(l, h, zLo, proofbound.AbsSumUpper(loAllow, planeAllow))
		take(l, h, zHi, proofbound.AbsSumUpper(hiAllow, planeAllow))
		if !onStart && !onEnd {
			continue
		}
		// The cap contour is the band's cap-level directrix and the chamfered
		// cap face's own boundary — the same offset loop the build emits, and
		// the same displacement the band's own vertices and edges carry.
		contour, err := capLoopBoundary(ctx, loop, cbp.loopOffset(li))
		if err != nil {
			return 0, 0, 0, err
		}
		setback := cbp.loopSetback(li)
		delta, err := loopContourDelta(ctx, loop, setback.dc, setback.dcDelta)
		if err != nil {
			return 0, 0, 0, err
		}
		cl, ch, contourPlaneAllow, err := boundaryExtremesBoundedContext(ctx, ProfileRecord{Outer: contour}, gu, gv, work, nil)
		if err != nil {
			return 0, 0, 0, err
		}
		// The contour sits at its cap level, so its in-plane displacement and
		// that level's inherited axial displacement compose independently. The
		// scan's own candidate displacement is a third in-plane term.
		contourAllow := proofbound.AbsSumUpper(proofbound.ProductUpper(inPlane, delta), contourPlaneAllow)
		if onStart {
			take(cl, ch, cbp.z0, proofbound.AbsSumUpper(contourAllow, proofbound.ProductUpper(axial, cbp.z0Delta)))
		}
		if onEnd {
			take(cl, ch, cbp.z1, proofbound.AbsSumUpper(contourAllow, proofbound.ProductUpper(axial, cbp.z1Delta)))
		}
	}
	if math.IsInf(lo, 1) {
		return 0, 0, 0, fmt.Errorf(`%w: the recorded region has no boundary`, ErrDegenerate)
	}
	bound := proofbound.AbsSumUpper(math.Max(
		math.Max(loUpper-lo, lo-loLower),
		math.Max(hiUpper-hi, hi-hiLower),
	))
	coordUpper, err := profileCoordinateEnvelope(cbp.profile, work, nil)
	if err != nil {
		return 0, 0, 0, err
	}
	zUpper := math.Max(math.Abs(cbp.z0), math.Abs(cbp.z1))
	placeAllow := prismPlacementCoeffAllow(pl, g, base, gu, gv, gz, coordUpper, zUpper)
	// A FIFTH mechanism displaces both ends and composes outward with the rest:
	// the reading's own recombination base + lo (and base + hi), charged exactly
	// against those two terms by proofbound.ExactSumRound (internal/proofbound/bounds.go). It is the one term
	// prismPlacementCoeffAllow's coefficient check cannot reach — a pure
	// translation leaves base/gu/gv/gz each exactly right and the addition that
	// follows still rounds — and it is charged per END, since the two ends are
	// summed from different terms.
	loEnd, hiEnd := base+lo, base+hi
	sumAllow := math.Max(
		proofbound.ExactSumRound(loEnd, base, lo),
		proofbound.ExactSumRound(hiEnd, base, hi),
	)
	bound = proofbound.AbsSumUpper(bound, placeAllow, sumAllow)
	return loEnd, hiEnd, bound, nil
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
// base path). caps names the two cap faces whose loops are the cap loops: the
// receiver's own capStart/capEnd faces, or a brep receiver's route P caps
// (docs/brep-modify-design.md §4.2), each face loop mapped to its section
// loop.
func classifyChamferSelection(ctx context.Context, pp prismPayload, caps prismCaps, sel EdgeSelector, edges []*Edge) (startLoops, endLoops map[int]bool, lateral bool, err error) {
	budget := proofbound.NewWorkBudget(ctx)
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
	for _, c := range []struct {
		face  *Face
		start bool
	}{{caps.start, true}, {caps.end, false}} {
		if c.face == nil {
			continue
		}
		for li, l := range c.face.Loops() {
			key := capKey{start: c.start, loop: caps.sectionLoop(c.start, li)}
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
// is the shared entry Chamfer calls once a clean cap-loop selection is
// classified. SX13's radial half is decided per circular wall as the band is
// constructed, in capband.BandRadius. start and end are the two
// caps' own setbacks (§8.3.1); a cap with no selected loop reads neither.
func buildCapBlend(ctx context.Context, doc *Document, ref producerID, pp prismPayload, start, end capSetback, startLoops, endLoops map[int]bool) (*Body, error) {
	height := pp.z1 - pp.z0
	loops := append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...)
	cbp := capBlendPayload{
		profile:    pp.profile,
		frame:      pp.frame,
		z0:         pp.z0,
		z1:         pp.z1,
		z0Delta:    pp.z0Delta,
		z1Delta:    pp.z1Delta,
		xform:      pp.xform,
		start:      start,
		end:        end,
		startLoops: startLoops,
		endLoops:   endLoops,
	}

	// SX6 + SX7/SX12: build the "mixed" profile — every selected loop offset
	// its own dc into the material (the cap contour at whichever cap it is
	// selected on; when a loop is selected on BOTH caps the two contours are
	// equal — resolveCapSetbacks gives both caps of such a loop one dc — so
	// one offset serves both),
	// every unselected loop left unchanged — and run the existing exact
	// offset + §5 audit machinery on it. SX6 is the offset's own drop
	// refusal, re-sentinelled here (wrapCapBlendDropError); the audit's own
	// refusals keep the sentinel their §4 stage-6 row decided
	// (wrapCapBlendAuditError) — SX7/SX12 for a crossing or contact, the base
	// S8/S9 ErrDegenerate for broken nesting. SX12 audits the exact offset
	// FAMILY, not the ruled patch the body builds: at axial fraction s, the
	// denoted miter locus is the parallel section offset by s*dc
	// (docs/modify-reach-design.md §8.3) — a fact about that family alone,
	// independent of which surface later reports the body's volume — and the
	// offset distance to any fixed feature is monotone non-increasing as the
	// offset grows from 0, so a crossing anywhere in the family occurs no
	// later than it occurs at the full offset dc — proving the family
	// disjoint at s=1 certifies every s in [0, 1].
	budget := proofbound.NewWorkBudget(ctx)
	mixed, err := mixedOffsetProfile(budget, cbp)
	if err != nil {
		return nil, wrapCapBlendDropError(err)
	}

	// SX7 (band-meeting): a loop chamfered on both caps needs both bands to
	// fit without meeting; a loop chamfered on one cap needs its own band to
	// fit within the sweep. A band reaches its own cap's ds along the sweep
	// (§8.3.1), whatever its dc.
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
			reach += start.ds
		}
		if endLoops[li] {
			reach += end.ds
		}
		if reach >= height {
			return nil, fmt.Errorf(`%w: the chamfer band(s) on loop %d reach or pass the opposite end of the sweep; a merging kernel is not available`, ErrUnsupported, li)
		}
	}

	if err := auditOffsetSectionBudget(budget, pp.profile, mixed); err != nil {
		return nil, wrapCapBlendAuditError(err)
	}
	if err := auditCapBlendSetbackSpan(budget, pp.profile, cbp); err != nil {
		return nil, err
	}
	if err := requireCapBlendLevelsSeparate(cbp); err != nil {
		return nil, err
	}
	return evalCapBlendContext(ctx, doc, ref, cbp)
}

// requireCapBlendLevelsSeparate is SX13's AXIAL half (Table SX,
// docs/modify-reach-design.md §4 stage 6 and §8.3), the sibling of
// capband.BandRadius. The band has two directrices and the
// setback displaces both: `d` in the plane, which capband.BandRadius proves survived
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
// The axial half is a fact about the sweep interval and the cap's own side
// setback ds alone (§8.3.1), so it is decided once per chamfered cap rather
// than per wall. SX7's band-reach gate above is the opposite failure on the
// same two numbers and never overlaps this one: SX7 refuses a setback so LARGE
// beside the sweep that the band passes the far end, this one a setback so
// SMALL beside the sweep's own coordinates that the level it displaces does not
// move.
func requireCapBlendLevelsSeparate(cbp capBlendPayload) error {
	refuse := func(which string, ds, level float64) error {
		return fmt.Errorf(`%w: the chamfer's side setback %v mm is below the float64 spacing of the %s cap's own sweep level %v mm, so the band's side level rounds back onto the cap level and every patch is emitted flat in the cap plane; a wider setback or a shorter sweep states a chamfer this evaluator can build`, ErrUnsupported, ds, which, level)
	}
	if ds := cbp.start.ds; anyLoopSelected(cbp.startLoops) && cbp.z0+ds == cbp.z0 {
		return refuse(`start`, ds, cbp.z0)
	}
	if ds := cbp.end.ds; anyLoopSelected(cbp.endLoops) && cbp.z1-ds == cbp.z1 {
		return refuse(`end`, ds, cbp.z1)
	}
	return nil
}

// auditCapBlendSetbackSpan runs SX6 and the stage-6 offset audit again at the
// top of each cap's setback span (docs/modify-reach-design.md §8.3.1). A
// setback stated in a unit other than millimetres is held as a float dc some
// dcDelta away from the value the caller stated, so the stated value is any
// point of [dc − dcDelta, dc + dcDelta]. The audit at dc certifies every
// offset up to dc and no further: SX12's monotonicity argument (a crossing or
// contact anywhere in the offset family occurs no later than at its largest
// member) runs the other way only. Auditing the offset at the span's top, the
// smallest float at or above dc + dcDelta, certifies every offset the span
// holds. A setback stated in millimetres has a one-point span and runs no
// second audit.
//
// A refusal here means the held dc builds and some point of the span does
// not, so this evaluator cannot decide whether the stated setback names a
// body: it is ErrUnsupported whichever sentinel the audit itself raised, and
// the audit's own error is folded in with %v so the refusal answers to one
// sentinel only.
func auditCapBlendSetbackSpan(budget *proofbound.WorkBudget, profile ProfileRecord, cbp capBlendPayload) error {
	top, ok := cbp.setbackSpanTop()
	if !ok {
		return nil
	}
	mixed, err := mixedOffsetProfile(budget, top)
	if err == nil {
		err = auditOffsetSectionBudget(budget, profile, mixed)
	}
	if err == nil {
		return nil
	}
	if errors.Is(err, context.Canceled) {
		return context.Canceled
	}
	if errors.Is(err, context.DeadlineExceeded) {
		return context.DeadlineExceeded
	}
	return fmt.Errorf(`%w: the chamfer setback across the cap converts to a float within its own rounding of the stated value, and at the top of that span the cap contour fails the offset audit (%v); this evaluator cannot decide whether the stated setback builds`, ErrUnsupported, err)
}

// setbackSpanTop is cbp with each cap's dc moved to the top of its setback
// span: the smallest float at or above dc + dcDelta, formed exactly. ok is
// false when neither cap's dc carries a conversion rounding, so the span is
// the held point and nothing differs.
func (cbp capBlendPayload) setbackSpanTop() (capBlendPayload, bool) {
	top := func(s capSetback) (capSetback, bool) {
		if !(s.dcDelta > 0) {
			return s, false
		}
		rd, rw := proofarith.FloatRat(s.dc), proofarith.FloatRat(s.dcDelta)
		if rd == nil || rw == nil {
			s.dc = math.Inf(1)
			return s, true
		}
		s.dc = proofbound.RatFloatUp(new(big.Rat).Add(rd, rw))
		return s, true
	}
	out := cbp
	var okStart, okEnd bool
	out.start, okStart = top(cbp.start)
	out.end, okEnd = top(cbp.end)
	return out, okStart || okEnd
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

// mixedOffsetProfile offsets exactly the loops cbp chamfers (the union of its
// startLoops/endLoops — a loop chamfered on either or both caps takes one
// in-plane offset, loopOffset's dc) into the material, leaving every other
// loop unchanged. It reuses offset2d.BuildLoop's per-feature offset unmodified.
func mixedOffsetProfile(budget *proofbound.WorkBudget, cbp capBlendPayload) (ProfileRecord, error) {
	profile := cbp.profile
	loops, err := prismCornerLoopsBudget(budget, prismPayload{profile: profile})
	if err != nil {
		return ProfileRecord{}, err
	}
	orig := append([]LoopRecord{profile.Outer}, profile.Holes...)
	out := make([]LoopRecord, len(orig))
	for li := range orig {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return ProfileRecord{}, err
		}
		if !cbp.startLoops[li] && !cbp.endLoops[li] {
			out[li] = cloneLoopRecord(orig[li])
			continue
		}
		segs, err := offset2d.BuildLoop(budget, loops[li].walks, 1, cbp.loopOffset(li), shellTol)
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
// question of the same machinery: the drop says the selected loop has no
// regular radius-d envelope — the contour is empty (offsetting a radius-4
// circle inward by 4 leaves nothing) or merely irregular (a fillet arc whose
// offset radius reaches zero collapses to a sharp corner) — so the body the
// caller named does not exist as a cap-loop chamfer. §4's existence test puts
// that in stage 5 with ErrDegenerate. The translation lives here, at the one
// call site that asks the existence question, never on offset2d.ErrDrop itself —
// Shell's own sentinel is correct for Shell. The result does not wrap
// offset2d.ErrDrop: carrying its ErrUnsupported along would leave the refusal
// answering to both sentinels, and a caller branching on either would be
// right, which is the ambiguity §4's one-sentinel rule exists to prevent.
func wrapCapBlendDropError(err error) error {
	if !errors.Is(err, offset2d.ErrDrop) {
		return err
	}
	return fmt.Errorf(`%w: the cap-loop offset drops a section feature (a circular wall's offset radius reaches zero, or a walk is consumed), so the selected loop has no regular cap contour at this setback`, ErrDegenerate)
}

// wrapCapBlendAuditError relabels the shared offset audit's refusal with the
// wording of the stage-6 row it came from, and preserves the sentinel that row
// already decided. SX6 (drop) is settled by mixedOffsetProfile's own call to
// offset2d.BuildLoop before the audit runs, so a refusal reaching here is one of
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
		return fmt.Errorf(`%w; the section that broke is the cap-loop chamfer's own offset of the selected loop(s) by the cap setback`, err)
	}
	return fmt.Errorf(`%w: the cap-loop chamfer's ruled patches cannot be certified disjoint from a non-adjacent boundary (%v); a trimming kernel is not available`, ErrUnsupported, err)
}
