package capblend

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismextent"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// This file is the cap-loop chamfer payload docs/modify-reach-design.md PR E
// (§14) states: the record Build evaluates over topology nodes and a placement
// re-evaluates. It implements topo.Payload (docs/evaluator-design.md §8).
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
// the interval below the band, built with the unmodified prismrecord.BuildLoopSidesAs —
// the wall's near-cap edge IS the band's side-level boundary, shared, never
// re-derived.

// Payload is the evaluator's own record of a complete-cap-loop
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
type Payload struct {
	Profile    momentinput.Profile
	Frame      r3.Frame
	Z0, Z1     float64
	Z0Delta    float64
	Z1Delta    float64
	Xform      r3.Transform
	Start, End Setback      // the z0 and z1 caps' own setbacks
	StartLoops map[int]bool // loop index -> chamfered on the z0 cap
	EndLoops   map[int]bool // loop index -> chamfered on the z1 cap
	// patches carries every chamferCap(...) role beside its plane-local
	// geometry (capband.Patch), populated once at build time
	// (Build) and reused by the DX7/DX8 surveys —
	// placement-invariant, since capband.Patch is entirely plane-local
	// (u, v, z), unaffected by xform. It is a SLICE in Table BX row BX3's own
	// deterministic patch order (loop index, then the chamfered cap, then the
	// patch's index within that band), because a survey that walks it emits
	// public output a caller may diff: a map hands Go's randomized start order
	// to every reader, and sorting the role strings is not this order either —
	// it puts patch 10 ahead of patch 2.
	Patches []Patch
	// bandDelta is each built band's own cap-contour displacement
	// (BandResult.delta, capband.ContourDisplacement), keyed by the
	// (loop, cap) that band sits on. BuildBand already computes it once for
	// every cap-level vertex, edge and area reading of that band; storing it here
	// lets a later reader — the tessellator, docs/tessellation-reach-design.md
	// §7 — charge the SAME number those readings did rather than re-derive a
	// second one from the same offset. It is plane-local and
	// placement-invariant, exactly as patches is.
	BandDelta map[BandKey]float64
	// draft marks the view a draft body builds its wall band through
	// (draftrecord.Payload.Band): its corners take the sharp offset
	// rule (offset2d.SharpJoinsBudget) rather than the reflex-corner arc, its
	// wall patches carry the prism's side(i, j) roles, and its rim and ruling
	// edges take the prism's convexity (docs/draft-design.md §2, §6). A
	// cap-loop chamfer never sets it.
	Draft bool
	// fillet marks a route L view whose inward circular walk may collapse to
	// one pole. A chamfer or draft never uses that contour rule.
	Fillet bool
	// draftKept is a draft view's draftPayload.Kept: the recorded segments of
	// the walls a subset draft leaves in place, which every per-walk reading
	// of the band offsets by zero (walkAmounts). Empty moves every wall.
	DraftKept map[offset2d.DraftWall]struct{}
	// loftSource marks the cap band built over an axial loft's held outer
	// station polygon. Its fit-spline allowances remain attached to the
	// resulting body and mesh; the bore remains an exact circular hole.
	LoftSource *LoftSource
}

// Setback is one chamfered cap's two setbacks (docs/modify-reach-design.md
// §8.3.1): dc across the cap face — the in-plane offset of the cap contour —
// and ds down the side wall — the axial distance from the cap level to the
// side level — each beside the rounding its own unit conversion committed.
// dsDelta is charged by every level built from ds. dcDelta is charged by the
// cap contour's displacement (capband.ContourDisplacement), which encloses the contour
// over every offset amount within dcDelta of dc, and by every cap-level
// length that reads dc as a radius. An equal chamfer holds dc == ds == d.
type Setback struct {
	Dc, DcDelta, Ds, DsDelta float64
}

// axialUpper is a proven upper bound on the axial rise the band denotes: the
// side setback the caller stated, which lies within dsDelta of ds. A setback
// stated in millimetres converts exactly, and the bound is |ds| itself.
func (s Setback) AxialUpper() float64 {
	if s.DsDelta > 0 {
		return proofbound.AbsSumUpper(s.Ds, s.DsDelta)
	}
	return math.Abs(s.Ds)
}

// setbackAt returns the setbacks of the cap a band with material sense
// matSign sits on: positive is the start cap, negative the end cap.
func (cbp Payload) SetbackAt(matSign float64) Setback {
	if matSign > 0 {
		return cbp.Start
	}
	return cbp.End
}

// loopSetback is the setbacks of whichever cap loop li is chamfered on. A loop
// chamfered on both caps takes one pair on both: resolveCapSetbacks refuses a
// loop whose two caps pick its setbacks differently.
func (cbp Payload) LoopSetback(li int) Setback {
	if cbp.StartLoops[li] {
		return cbp.Start
	}
	return cbp.End
}

// loopBandDelta is the larger contour displacement of loop li's chamfer bands
// (Build's bandDelta), zero where it is chamfered on neither cap.
func (cbp Payload) LoopBandDelta(li int) float64 {
	delta := 0.0
	for _, start := range [...]bool{true, false} {
		if d, ok := cbp.BandDelta[BandKey{Loop: li, Start: start}]; ok {
			delta = math.Max(delta, d)
		}
	}
	return delta
}

// walkAmounts is the in-plane offset of each coalesced walk of loop li when
// the band's setback is d: d for every walk of a chamfer or a draft of every
// wall, and zero for a walk a subset draft keeps (draftKept).
func (cbp Payload) WalkAmounts(li int, walks []survey2d.SideWalk, d float64) []float64 {
	return offset2d.DraftWalkAmounts(cbp.DraftKept, li, walks, d)
}

// loopOffset is loop li's own in-plane offset, its loopSetback's dc.
func (cbp Payload) LoopOffset(li int) float64 { return cbp.LoopSetback(li).Dc }

// DraftNearLevel is the near cap's level in a draft band view beside its own
// axial displacement: the cap no loop is chamfered on.
func (cbp Payload) DraftNearLevel() proofbound.BoundedScalar {
	if len(cbp.EndLoops) > 0 {
		return proofbound.MeasuredScalar(cbp.Z0, cbp.Z0Delta)
	}
	return proofbound.MeasuredScalar(cbp.Z1, cbp.Z1Delta)
}

// DraftBandHeightUpper bounds the axial extent of a draft body: the held
// sweep height with both ends' displacements and subtraction rounding.
func (cbp Payload) DraftBandHeightUpper() float64 {
	h := proofbound.BoundedSub(proofbound.MeasuredScalar(cbp.Z1, cbp.Z1Delta), proofbound.MeasuredScalar(cbp.Z0, cbp.Z0Delta))
	return proofbound.AbsSumUpper(math.Abs(h.Value), h.Bound)
}

// noun names the body a refusal over this payload speaks of: the draft view's
// tapered extrude, or the cap-loop chamfer.
func (cbp Payload) Noun() string {
	if cbp.Draft {
		return "tapered extrude"
	}
	return "cap-loop chamfer"
}

// BandKey names one chamfer band: the loop it belongs to (an index into
// loops(), the same index space Table BX's roles use) and which cap it sits on.
type BandKey struct {
	Loop  int
	Start bool
}

// Patch is one built chamfer patch's role paired with the geometry that
// role labels.
type Patch struct {
	Role string
	Geom capband.Patch
}

// Transform is the accumulated rigid placement.
func (cbp Payload) Transform() r3.Transform { return cbp.Xform }

// Placed re-evaluates the same record under the composed motion (evaluator
// §8): every gate this PR enforces is a closed-form fact of the RECORD, so a
// placed body re-derives the identical chamfer.
func (cbp Payload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	cbp.Xform = composed
	return Build(ctx, env, cbp)
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (internal/reportvocab): the root type name this payload had before it moved.
func (Payload) SurveyClass() string { return "capBlendPayload" }

// prismLike returns a prismrecord.Payload sharing the receiver's frame and
// placement over [z0, z1], so the point/dir machinery already built for
// prisms serves the cap-blend build too.
func (cbp Payload) PrismLike(z0, z1 float64) prismrecord.Payload {
	return prismrecord.Payload{
		Frame: cbp.Frame, Z0: z0, Z1: z1,
		Z0Delta: cbp.Z0Delta, Z1Delta: cbp.Z1Delta,
		Xform: cbp.Xform,
	}
}

// capBandLevel preserves the selected end's axial displacement while a
// chamfer band derives its cap and side levels. The start band has positive
// material sense; the end band has negative material sense.
func (cbp Payload) CapBandLevel(capZ, matSign float64) proofbound.BoundedScalar {
	capDelta := cbp.Z1Delta
	if matSign > 0 {
		capDelta = cbp.Z0Delta
	}
	return proofbound.MeasuredScalar(capZ, capDelta)
}

// AxialDelta is the larger sweep-level displacement a body-relative stop must
// preserve when it resolves against this cap blend. A chamfered end also
// carries the setback conversion and float-sum rounding of its level.
func (cbp Payload) AxialDelta() float64 {
	z0Delta, z1Delta := cbp.Z0Delta, cbp.Z1Delta
	if len(cbp.StartLoops) != 0 {
		s := cbp.Start
		z0Delta = proofbound.AbsSumUpper(z0Delta, s.DsDelta, proofarith.AddRoundError(cbp.Z0, s.Ds, cbp.Z0+s.Ds))
	}
	if len(cbp.EndLoops) != 0 {
		s := cbp.End
		z1Delta = proofbound.AbsSumUpper(z1Delta, s.DsDelta, proofarith.AddRoundError(cbp.Z1, -s.Ds, cbp.Z1-s.Ds))
	}
	return math.Max(z0Delta, z1Delta)
}

// ExtentAlong is the through-all stop's reading of the cap-blend body
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
// cap contour is a computed section, not a recorded one (payload.go's LoopContourDelta),
// so an extreme it holds is known only to a proven displacement — which the
// stop charges to the level it resolves, and outside which it decides its own
// in-path test. On an unplaced body every direction whose extreme is held by an
// exact recorded coordinate still answers the exact interval it always did,
// with a zero displacement; under a placement the reading's own arithmetic can
// round even there, and charges what it commits.
// A direction along the sweep ignores contour displacement but
// still reads the cap level's own inherited axial displacement.
func (cbp Payload) ExtentAlong(g r3.Vec) (float64, float64, float64, error) {
	return cbp.ExtentBoundedAlong(context.Background(), g, freeform.NewFreeformWork())
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
// A fourth, prismrecord.PlacementCoeffAllow, displaces every candidate the SAME way
// — the frame and placement's own rounding of base/gu/gv/gz — and so composes
// outward with the per-candidate maximum rather than folding into it. A fifth
// does the same one step later: the reading's own summation of base with the
// extremized candidate, charged exactly by proofbound.ExactSumRound (internal/proofbound/bounds.go), since a
// placement can leave every coefficient exactly right and still round when the
// terms are added.
func (cbp Payload) ExtentBoundedAlong(ctx context.Context, g r3.Vec, work *freeform.FreeformWork) (float64, float64, float64, error) {
	pl := cbp.PrismLike(0, 0)
	base := cbp.Xform.Apply(cbp.Frame.Origin()).Dot(g)
	gu := pl.Dir(1, 0, 0).Dot(g)
	gv := pl.Dir(0, 1, 0).Dot(g)
	gz := pl.Dir(0, 0, 1).Dot(g)
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
	for li, loop := range cbp.Loops() {
		if err := ctx.Err(); err != nil {
			return 0, 0, 0, err
		}
		onStart, onEnd := cbp.StartLoops[li], cbp.EndLoops[li]
		zLo, zHi := cbp.Z0, cbp.Z1
		loAllow := proofbound.ProductUpper(axial, cbp.Z0Delta)
		hiAllow := proofbound.ProductUpper(axial, cbp.Z1Delta)
		if onStart {
			s := cbp.Start
			zLo = cbp.Z0 + s.Ds
			loAllow = proofbound.AbsSumUpper(loAllow, proofbound.ProductUpper(axial, proofbound.AbsSumUpper(s.DsDelta, proofarith.AddRoundError(cbp.Z0, s.Ds, zLo))))
		}
		if onEnd {
			s := cbp.End
			zHi = cbp.Z1 - s.Ds
			hiAllow = proofbound.AbsSumUpper(hiAllow, proofbound.ProductUpper(axial, proofbound.AbsSumUpper(s.DsDelta, proofarith.AddRoundError(cbp.Z1, -s.Ds, zHi))))
		}
		// The boundary scan states its own displacement per candidate — nonzero
		// wherever a circular candidate's apex is not exactly representable
		// (extrude.go's circularExtremeInterval) — and it displaces the reading
		// IN PLANE, so it composes with the axial terms above rather than
		// replacing either.
		l, h, planeAllow, err := prismextent.ProfileBoundaryExtremes(ctx, momentinput.Profile{Outer: loop}, gu, gv, work, nil)
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
		contour, err := cbp.ContourOf(ctx, li, loop, cbp.LoopOffset(li))
		if err != nil {
			return 0, 0, 0, err
		}
		setback := cbp.LoopSetback(li)
		delta, err := cbp.LoopContourDelta(ctx, li, loop, setback.Dc, setback.DcDelta)
		if err != nil {
			return 0, 0, 0, err
		}
		cl, ch, contourPlaneAllow, err := prismextent.ProfileBoundaryExtremes(ctx, momentinput.Profile{Outer: contour}, gu, gv, work, nil)
		if err != nil {
			return 0, 0, 0, err
		}
		// The contour sits at its cap level, so its in-plane displacement and
		// that level's inherited axial displacement compose independently. The
		// scan's own candidate displacement is a third in-plane term.
		contourAllow := proofbound.AbsSumUpper(proofbound.ProductUpper(inPlane, delta), contourPlaneAllow)
		if onStart {
			take(cl, ch, cbp.Z0, proofbound.AbsSumUpper(contourAllow, proofbound.ProductUpper(axial, cbp.Z0Delta)))
		}
		if onEnd {
			take(cl, ch, cbp.Z1, proofbound.AbsSumUpper(contourAllow, proofbound.ProductUpper(axial, cbp.Z1Delta)))
		}
	}
	if math.IsInf(lo, 1) {
		return 0, 0, 0, fmt.Errorf(`%w: the recorded region has no boundary`, decaderr.ErrDegenerate)
	}
	bound := proofbound.AbsSumUpper(math.Max(
		math.Max(loUpper-lo, lo-loLower),
		math.Max(hiUpper-hi, hi-hiLower),
	))
	coordUpper, err := momentinput.CoordinateEnvelope(cbp.Profile, work, nil)
	if err != nil {
		return 0, 0, 0, err
	}
	zUpper := math.Max(math.Abs(cbp.Z0), math.Abs(cbp.Z1))
	placeAllow := prismrecord.PlacementCoeffAllow(pl, g, base, gu, gv, gz, coordUpper, zUpper)
	// A FIFTH mechanism displaces both ends and composes outward with the rest:
	// the reading's own recombination base + lo (and base + hi), charged exactly
	// against those two terms by proofbound.ExactSumRound (internal/proofbound/bounds.go). It is the one term
	// prismrecord.PlacementCoeffAllow's coefficient check cannot reach — a pure
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

// capLoops returns the receiver's loops as append(Outer, Holes...), the same
// index space Table BX's roles and the prism's own side(i,j) roles use.
func (cbp Payload) Loops() []sectionrecord.LoopRecord {
	return append([]sectionrecord.LoopRecord{cbp.Profile.Outer}, cbp.Profile.Holes...)
}

// offsetJoins is capband.OffsetJoins read under cbp's corner rule for loop li. A
// draft view (docs/draft-design.md §2) checks the band radius of each circular
// wall it moves exactly as capband.OffsetJoins does and then resolves every corner
// by the sharp rule, each walk at its own amount (offset2d.DraftWalkAmounts, §10.2): a
// reflex corner is mitered, a corner at a circular walk must be a G1 join
// between two walks moved alike (SD4), and two moved lines that do not meet
// are a cusp (SD15).
func (cbp Payload) OffsetJoins(budget *proofbound.WorkBudget, li int, cl survey2d.CornerLoop, d float64) ([]offset2d.Join, error) {
	if cbp.Fillet {
		return filletband.OffsetJoins(budget, cl.Walks, d, cbp.LoopSetback(li).DcDelta, offset2d.ShellTol)
	}
	if !cbp.Draft {
		return capband.OffsetJoins(budget, cl.Walks, d, offset2d.ShellTol)
	}
	if len(cl.Walks) == 0 {
		return nil, fmt.Errorf(`%w: a draft loop holds no walks`, decaderr.ErrDegenerate)
	}
	amounts := cbp.WalkAmounts(li, cl.Walks, d)
	for i, w := range cl.Walks {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		if w.IsCircular() {
			if _, err := capband.WallRadius(w, amounts[i], offset2d.ShellTol); err != nil {
				return nil, offset2d.DraftOffsetError(err)
			}
		}
	}
	return offset2d.DraftSharpJoins(budget, cl.Walks, amounts, offset2d.ShellTol)
}

// loopContourDelta is the package function of the same name read under
// cbp's corner rule for loop li: a draft view's contour is the sharp offset
// (cbp.offsetJoins), each walk at its own amount (walkAmounts), so its
// displacement is enclosed over the sharp joins, never over the reflex-corner
// arcs a chamfer's contour holds.
func (cbp Payload) LoopContourDelta(ctx context.Context, li int, loop sectionrecord.LoopRecord, d, dDelta float64) (float64, error) {
	if cbp.Fillet {
		return filletband.LoopContourDelta(ctx, loop, d, dDelta, offset2d.ShellTol)
	}
	if !cbp.Draft {
		return capband.LoopContourDelta(ctx, loop, d, dDelta, offset2d.ShellTol)
	}
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	cl, err := OneLoopCornerLoop(budget, loop, work)
	if err != nil {
		return 0, err
	}
	amounts := cbp.WalkAmounts(li, cl.Walks, d)
	if len(cl.Walks) == 1 && cl.Walks[0].Closed {
		a, aDelta := amounts[0], dDelta
		if a == 0 {
			aDelta = 0
		}
		return capband.WallCircleDisplacement(cl.Walks[0], a, aDelta, offset2d.ShellTol)
	}
	joins, err := cbp.OffsetJoins(budget, li, cl, d)
	if err != nil {
		return 0, err
	}
	return capband.AmountsContourDisplacement(cl.Walks, capband.ContourJoins(joins), amounts, dDelta, offset2d.ShellTol)
}

// contourOf is capband.LoopBoundary read under cbp's corner rule for loop li: a
// draft view's far section is the loop's sharp offset
// (offset2d.BuildSharpLoop), every corner mitered and each walk moved its own
// amount (walkAmounts), where a chamfer's cap contour closes a reflex corner
// with an arc.
func (cbp Payload) ContourOf(ctx context.Context, li int, loop sectionrecord.LoopRecord, d float64) (sectionrecord.LoopRecord, error) {
	if !cbp.Draft {
		return capband.LoopBoundary(ctx, loop, d, offset2d.ShellTol)
	}
	budget := proofbound.NewWorkBudget(ctx)
	cl, err := OneLoopCornerLoop(budget, loop, freeform.NewFreeformWork())
	if err != nil {
		return sectionrecord.LoopRecord{}, err
	}
	segs, _, err := offset2d.BuildSharpLoop(budget, cl.Walks, cbp.WalkAmounts(li, cl.Walks, d), offset2d.ShellTol)
	if err != nil {
		return sectionrecord.LoopRecord{}, offset2d.DraftOffsetError(err)
	}
	return sectionrecord.LoopRecord{Segments: segs}, nil
}

// LoftSource retains the fit-spline section and its already certified
// approximation. The modifier acts on the loft's held cap edges, so its new
// band is polygonal while the original spline remains available as the flank
// source and its displacement and mass allowances remain attached.
type LoftSource struct {
	Profile momentinput.Profile
	Proof   loftmesh.MeshProof
}
