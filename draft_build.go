package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file builds a draft body (docs/draft-design.md §7): the gates of Table
// SD in §5's stage order, the near cap over the recorded section P, the far
// cap over its sharp offset Q, and one ruled wall per walk between them. The
// walls are the cap-loop band of docs/modify-reach-design.md §8.3 with no
// straight slab below it: draftPayload.band hands buildCapBand a view whose
// side level is the sketch plane and whose cap contour is Q, so every wall
// patch, ruling and far-cap edge, and every bound they carry, comes from that
// one construction. The measurements are draft_moments.go's.

// draftAngle is SD2 (docs/draft-design.md Table SD) on a nonzero taper that
// has passed SD1: the angle in radians beside its conversion rounding, or
// ErrDegenerate for an angle at or past a right angle to the sweep. The test
// reads the stated angle in degrees as well as in radians, because a stated
// 90 degrees converts to a radian float just below π/2.
func draftAngle(taper units.Value) (float64, float64, error) {
	alpha, alphaDelta, err := extent.DisplacementIn(taper, units.Angle, units.Radian, "the taper")
	if err != nil {
		return 0, 0, err
	}
	if deg, err := taper.In(units.Degree); err != nil || math.Abs(deg) >= 90 || math.Abs(alpha) >= math.Pi/2 {
		return 0, 0, fmt.Errorf(`%w: a taper of %s leans the walls at or past a right angle to the sweep, so they sweep no solid; a draft angle lies strictly between -90 and 90 degrees`, ErrDegenerate, taper)
	}
	return alpha, alphaDelta, nil
}

// extrudeDraft is Extrude's path for a nonzero taper alpha (radians, within
// alphaDelta of the stated angle, past SD1 and SD2): the stage-2 gates SD11
// and SD12 on the extent and the option, then the build. Every gate runs
// before the document changes.
func (d *Document) extrudeDraft(profile ProfileRecord, frame r3.Frame, e Extent, alpha, alphaDelta float64, surfaceResult bool) (*Body, error) {
	if _, ok := e.(Distance); !ok {
		return nil, fmt.Errorf(`%w: this evaluator tapers a Distance extent only; a Symmetric, TwoSided, ThroughAll or ToFace extent with a nonzero taper needs a two-sided draft record or a stop-face level (draft SD11)`, ErrUnsupported)
	}
	if surfaceResult {
		return nil, fmt.Errorf(`%w: this evaluator builds a tapered extrude as a solid only; omit WithSurfaceResult or the taper (draft SD12)`, ErrUnsupported)
	}
	sweep, err := d.resolveLinearExtent(e, frame)
	if err != nil {
		return nil, err
	}
	ref := d.nextProducerID()
	body, err := evalDraftContext(context.Background(), d, ref, draftPayload{
		profile:    profile,
		frame:      frame,
		z0:         sweep.z0,
		z1:         sweep.z1,
		z0Delta:    sweep.z0Delta,
		z1Delta:    sweep.z1Delta,
		nearStart:  sweep.z0 == 0,
		taper:      alpha,
		taperDelta: alphaDelta,
		xform:      r3.Identity(),
	})
	if err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}

// evalDraftContext runs the record gates of Table SD in stage order and, once
// every one passes, builds the body. Stage 3 resolves every loop's walks (SD3)
// and corners (SD4, SD15); stage 4 offsets them (SD5, SD6, SD7, SD13); stage 5
// audits the far section (SD6, SD8, SD9); stage 6 audits it again at the top of
// d's span (SD10); stage 7 builds the patches (SD14, SD16). Nothing is made
// before stage 7, and nothing is committed here.
func evalDraftContext(ctx context.Context, doc *Document, ref producerID, dp draftPayload) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	height := proofbound.BoundedSub(proofbound.MeasuredScalar(dp.z1, dp.z1Delta), proofbound.MeasuredScalar(dp.z0, dp.z0Delta))
	if !(height.Value > 0) {
		return nil, fmt.Errorf(`%w: the sweep interval is empty`, ErrDegenerate)
	}
	off, offDelta, err := draftOffset(height.Value, height.Bound, dp.taper, dp.taperDelta)
	if err != nil {
		return nil, err
	}
	dp.d, dp.dDelta = off, offDelta

	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	walks, err := draftSectionWalks(budget, dp.profile, work)
	if err != nil {
		return nil, err
	}
	far, err := draftFarSection(budget, walks, dp.d)
	if err != nil {
		return nil, err
	}
	if err := auditOffsetSectionBudget(budget, dp.profile, far); err != nil {
		return nil, wrapDraftAuditError(err)
	}
	if err := auditDraftSpan(budget, dp.profile, walks, dp.d, dp.dDelta); err != nil {
		return nil, err
	}
	dp.far = far
	return buildDraftBody(ctx, doc, ref, dp, work)
}

// draftSectionWalks is stage 3: every loop of the section resolved into its
// coalesced walks, each walk analytic (SD3), and every corner classified by the
// sharp rule at a unit offset (SD4 for a corner at a circular walk that is not
// a G1 join, SD15 for a cusp). The classification does not depend on the
// amount, so stage 4 reads the same corners at d.
func draftSectionWalks(budget *proofbound.WorkBudget, profile ProfileRecord, work *freeform.FreeformWork) ([][]survey2d.SideWalk, error) {
	loops := append([]LoopRecord{profile.Outer}, profile.Holes...)
	out := make([][]survey2d.SideWalk, len(loops))
	for li, loop := range loops {
		raw := make([]survey2d.SideWalk, len(loop.Segments))
		for i, seg := range loop.Segments {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return nil, err
			}
			w, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				return nil, err
			}
			if err := boundarywalk.RequireAnalyticWalk(w, "a tapered extrude"); err != nil {
				return nil, fmt.Errorf(`%w; the offset of a free-form wall is not a recorded kind (draft SD3)`, err)
			}
			raw[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
		}
		walks, err := boundarywalk.CoalesceWalksBudget(raw, budget)
		if err != nil {
			return nil, err
		}
		if len(walks) == 0 {
			return nil, fmt.Errorf(`%w: a recorded loop holds no segments`, ErrDegenerate)
		}
		if len(walks) > 1 || !walks[0].Closed {
			if _, err := offset2d.SharpJoinsBudget(budget, walks, 1, 1, shellTol); err != nil {
				return nil, wrapDraftOffsetError(err)
			}
		}
		out[li] = walks
	}
	return out, nil
}

// draftFarSection is stage 4: every loop offset sharply by t, its walk sense
// kept (offset2d.BuildSharpLoop). A circular wall whose radius collapses is
// SD5 and a wall its own corners consume is SD7, both ErrUnsupported; an outer
// loop shrunk until every one of its walls is consumed is SD6, ErrDegenerate,
// because the taper has consumed the region before the far end. Then SD13: an
// amount that rounds to zero, a far corner bit-identical to its near corner,
// or a far radius bit-identical to its near radius names a far section
// float64 cannot tell from the near one.
func draftFarSection(budget *proofbound.WorkBudget, walks [][]survey2d.SideWalk, t float64) (ProfileRecord, error) {
	if t == 0 {
		return ProfileRecord{}, errDraftAmountRoundsAway(t)
	}
	loops := make([]LoopRecord, len(walks))
	for li, ws := range walks {
		segs, joins, err := offset2d.BuildSharpLoop(budget, ws, 1, t, shellTol)
		if errors.Is(err, offset2d.ErrLoopConsumed) && li == 0 && t > 0 {
			return ProfileRecord{}, fmt.Errorf(`%w: the taper offsets every wall of the outer loop past its neighbours before the far end, so the region is consumed and no solid reaches that far; a smaller taper or a shorter sweep states a body (draft SD6)`, ErrDegenerate)
		}
		if err != nil {
			return ProfileRecord{}, wrapDraftOffsetError(err)
		}
		for _, j := range joins {
			if j.M.U == j.VertU && j.M.V == j.VertV {
				return ProfileRecord{}, errDraftAmountRoundsAway(t)
			}
		}
		for _, w := range ws {
			if !w.IsCircular() {
				continue
			}
			if rr, ok := offset2d.OffsetRadius(w, 1, t, shellTol); ok && rr == w.Radius {
				return ProfileRecord{}, errDraftAmountRoundsAway(t)
			}
		}
		loops[li] = LoopRecord{Segments: segs}
	}
	return ProfileRecord{Outer: loops[0], Holes: loops[1:]}, nil
}

// errDraftAmountRoundsAway is SD13: the requested body exists, a real if tiny
// taper, but float64 cannot name its far section at the section's own scale,
// and a far section equal to the near one would publish walls with no lean.
func errDraftAmountRoundsAway(t float64) error {
	return fmt.Errorf(`%w: the taper's offset amount %v mm is below the float64 spacing of the section's own coordinates, so the far section rounds back onto the near one; a larger taper or a longer sweep states a draft this evaluator can build (draft SD13)`, ErrUnsupported, t)
}

// auditDraftSpan is SD10: stages 4 and 5 again at the top of d's span, the
// amount farthest from zero that the stated sweep and taper can denote. The
// audit at d certifies every smaller amount (the crossing and contact tests
// are monotone in the amount, modify-reach §8.3.1) and none larger, so a far
// section that builds at d and fails at the span's top leaves this evaluator
// unable to decide whether the stated draft builds: ErrUnsupported, with the
// audit's own error folded in with %v so the refusal answers to one sentinel.
func auditDraftSpan(budget *proofbound.WorkBudget, profile ProfileRecord, walks [][]survey2d.SideWalk, d, dDelta float64) error {
	if !(dDelta > 0) {
		return nil
	}
	rd, rw := proofarith.FloatRat(math.Abs(d)), proofarith.FloatRat(dDelta)
	if rd == nil || rw == nil {
		return errDraftOffsetUnbounded
	}
	top := proofbound.RatFloatUp(new(big.Rat).Add(rd, rw))
	if d < 0 {
		top = -top
	}
	far, err := draftFarSection(budget, walks, top)
	if err == nil {
		err = auditOffsetSectionBudget(budget, profile, far)
	}
	switch {
	case err == nil:
		return nil
	case errors.Is(err, context.Canceled):
		return context.Canceled
	case errors.Is(err, context.DeadlineExceeded):
		return context.DeadlineExceeded
	}
	return fmt.Errorf(`%w: the taper's offset amount is known only within the span its sweep and tangent enclosures state, and at the top of that span the far section fails its audit (%v); this evaluator cannot decide whether the stated draft builds (draft SD10)`, ErrUnsupported, err)
}

// sharpOffsetJoinsBudget is the draft band's corner rule
// (capBlendPayload.offsetJoins): every corner of one coalesced loop resolved
// by offset2d.SharpJoinsBudget at amount d, in walk order, as the cornerJoin
// records buildCapBand reads. No join is an arc.
func sharpOffsetJoinsBudget(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, d float64) ([]cornerJoin, error) {
	result, err := offset2d.SharpJoinsBudget(budget, walks, 1, d, shellTol)
	if err != nil {
		return nil, wrapDraftOffsetError(err)
	}
	joins := make([]cornerJoin, len(result))
	for i, j := range result {
		joins[i] = cornerJoin{g1: j.G1, vU: j.VertU, vV: j.VertV, m: Point2{U: j.M.U, V: j.M.V}}
	}
	return joins, nil
}

// wrapDraftOffsetError states an offset refusal as the Table SD row it is.
// Each keeps the sentinel its row decides: SD4 and SD15 are ErrUnsupported, as
// are SD5 and SD7, which offset2d.ErrDrop already carries.
func wrapDraftOffsetError(err error) error {
	switch {
	case errors.Is(err, offset2d.ErrCircularMiter):
		return fmt.Errorf(`%w: a circular wall meets its neighbour other than tangentially, and that corner moves along a conic as the taper offsets it, which no line-and-arc far section records; join the wall tangentially (draft SD4)`, ErrUnsupported)
	case errors.Is(err, offset2d.ErrTopology):
		return fmt.Errorf(`%w; two walls meet at a cusp, so their tapered carriers do not intersect (draft SD15)`, err)
	case errors.Is(err, offset2d.ErrLoopConsumed):
		return fmt.Errorf(`%w: the taper consumes every wall of a loop before the far end, so the section changes topology there (draft SD7)`, ErrUnsupported)
	case errors.Is(err, offset2d.ErrDrop):
		return fmt.Errorf(`%w; the taper's far section loses a circular wall whose radius collapses (draft SD5) or a wall its own corners consume (draft SD7)`, err)
	}
	return err
}

// wrapDraftAuditError names the far section as the section the shared offset
// audit refused, keeping the audit's sentinel: S8's ErrDegenerate (SD6),
// the crossing test's ErrUnsupported (SD8) and the nesting rows' (SD9).
func wrapDraftAuditError(err error) error {
	if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
		return err
	}
	return fmt.Errorf(`%w; the section that failed is the tapered extrude's far section (draft SD6, SD8, SD9)`, err)
}

// buildDraftBody is stage 7 and the assembly (docs/draft-design.md §7 steps
// 7–9): per loop the near rim at the sketch plane, then buildCapBand's walls
// and far rim over it; the two caps over the near and far rims; one shell,
// one lump. Every edge bounds exactly two faces: a near rim edge its wall and
// the near cap, a far rim edge its wall and the far cap, a ruling its two
// walls. draft_moments.go then measures the body.
func buildDraftBody(ctx context.Context, doc *Document, ref producerID, dp draftPayload, work *freeform.FreeformWork) (*Body, error) {
	cbp := dp.band()
	loops := cbp.loops()
	nearZ, nearDelta, farZ, matSign := dp.levels()
	body := &Body{doc: doc, origin: FeatureRef{producer: ref, Role: roleBody}, solid: true}
	pl := cbp.prismLike(0, 0)

	var faces []*Face
	nearLoops := make([]*Loop, len(loops))
	farLoops := make([]*Loop, len(loops))
	bands := make([]capBandResult, len(loops))
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		nearCo, err := draftNearRim(ctx, pl, li, loop, nearZ, nearDelta, work)
		if err != nil {
			return nil, err
		}
		band, err := buildCapBand(ctx, body, ref, cbp, li, loop, farZ, matSign, nearCo, work)
		if err != nil {
			return nil, fmt.Errorf(`the tapered extrude's walls: %w`, err)
		}
		faces = append(faces, band.patches...)
		nearLoops[li] = &Loop{coedges: nearCo, outer: li == 0}
		farLoops[li] = &Loop{coedges: band.capCo, outer: li == 0}
		bands[li] = band
		dp.farDelta = math.Max(dp.farDelta, band.delta)
	}

	startLoops, endLoops := farLoops, nearLoops
	if dp.nearStart {
		startLoops, endLoops = nearLoops, farLoops
	}
	startFrame, err := capFrame(pl, dp.z0, true)
	if err != nil {
		return nil, err
	}
	endFrame, err := capFrame(pl, dp.z1, false)
	if err != nil {
		return nil, err
	}
	capStart := &Face{
		surface:       Plane{Frame: startFrame},
		origins:       []FeatureRef{{producer: ref, Role: roleCapStart}},
		body:          body,
		loops:         startLoops,
		axialDelta:    dp.z0Delta,
		hasAxialDelta: true,
	}
	capEnd := &Face{
		surface:       Plane{Frame: endFrame},
		origins:       []FeatureRef{{producer: ref, Role: roleCapEnd}},
		body:          body,
		loops:         endLoops,
		axialDelta:    dp.z1Delta,
		hasAxialDelta: true,
	}
	if err := attachFaceLoopsContext(ctx, []*Face{capStart, capEnd}); err != nil {
		return nil, err
	}
	faces = append(faces, capStart, capEnd)
	body.lumps = []*Lump{{shells: []*Shell{{faces: faces}}}}

	nearCap, farCap := capStart, capEnd
	if !dp.nearStart {
		nearCap, farCap = capEnd, capStart
	}
	if err := measureDraftBody(ctx, body, dp, cbp, bands, nearCap, farCap, work); err != nil {
		return nil, err
	}
	body.payload = dp
	return body, nil
}

// draftNearRim builds one loop's near rim at level z: a vertex at every
// junction of the coalesced walks (one seam vertex for a lone closed walk) and
// one rim edge per walk, in walk order, exactly the bottom rim
// buildLoopSidesAs builds for a prism's wall at the same level. Each vertex
// carries its junction bound, the level's own displacement and its frame lift
// rounding; each edge the walk's own length and convexity. buildCapBand reads
// these as the band's side directrix, so wall i's near edge is coedge i.
func draftNearRim(ctx context.Context, pl prismPayload, li int, loop LoopRecord, z, zDelta float64, work *freeform.FreeformWork) ([]coedge, error) {
	if len(loop.Segments) == 0 {
		return nil, fmt.Errorf(`%w: a recorded loop holds no segments`, ErrDegenerate)
	}
	raw := make([]survey2d.SideWalk, len(loop.Segments))
	for i, seg := range loop.Segments {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		seg, err := normalizeSegment(seg)
		if err != nil {
			return nil, err
		}
		w, err := boundarywalk.WalkOf(seg, work)
		if err != nil {
			return nil, err
		}
		raw[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
	}
	walks, err := boundarywalk.CoalesceWalksContext(ctx, raw)
	if err != nil {
		return nil, err
	}
	n := len(walks)
	level := pl
	level.z0, level.z1 = z, z
	rimVertex := func(u, v, extra float64) *Vertex {
		held, lift := pl.liftedVertex(u, v, z)
		return &Vertex{position: held, bound: units.Millimeters(proofbound.AbsSumUpper(zDelta, lift, extra))}
	}
	singleClosed := n == 1 && walks[0].Closed
	verts := make([]*Vertex, n)
	for i, w := range walks {
		prev := walks[(i+n-1)%n]
		u, v, extra := junctionVertexAt(loop.Segments, prev, w)
		verts[i] = rimVertex(u, v, extra)
	}
	co := make([]coedge, n)
	for i, w := range walks {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		start, end := verts[i], verts[(i+1)%n]
		if singleClosed {
			end = start
		}
		convex, err := rimConvexity(ctx, w, li != 0, work)
		if err != nil {
			return nil, err
		}
		edge, _, _, _, err := buildWallGeometry(level, w, convex, singleClosed, start, end, start, end)
		if err != nil {
			return nil, err
		}
		co[i] = coedge{edge: edge, forward: true}
	}
	return co, nil
}
