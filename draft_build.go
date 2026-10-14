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
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
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
// alphaDelta of the stated angle, past SD1 and SD2): the SD12 surface-result
// gate, extent resolution, then one or two draft slabs. A one-sided stop
// resolves its far level and axial bound before the single-slab build.
// ctx is checked between those steps and polled inside the slab builds; a
// canceled ctx returns ctx.Err() before the commit. Every gate runs before the
// document changes.
func (d *Document) extrudeDraft(ctx context.Context, profile profileRecord, frame r3.Frame, e Extent, alpha, alphaDelta float64, surfaceResult bool) (*Body, error) {
	switch e.(type) {
	case Distance, ThroughAll, ToFace, Symmetric, TwoSided:
	default:
		return nil, fmt.Errorf(`%w: a tapered extrude does not support extent %T`, ErrUnsupported, e)
	}
	if surfaceResult {
		return nil, fmt.Errorf(`%w: this evaluator builds a tapered extrude as a solid only; omit WithSurfaceResult or the taper (draft SD12)`, ErrUnsupported)
	}
	sweep, err := d.resolveLinearExtent(e, frame)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ref := d.nextProducerID()
	base := draftPayload{
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
	}
	var body *Body
	if sweep.z0 < 0 && sweep.z1 > 0 {
		negative, positive := base, base
		negative.z1, negative.z1Delta, negative.nearStart = 0, 0, false
		positive.z0, positive.z0Delta, positive.nearStart = 0, 0, true
		body, err = evalTwoSidedDraftContext(ctx, d, ref, twoSidedDraftPayload{
			negative: negative, positive: positive,
		})
	} else {
		body, err = evalDraftContext(ctx, d, ref, base)
	}
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}

// wrapStageError names the stage of a build that failed. A canceled or
// expired context passes through unwrapped, so a caller reads ctx.Err() itself.
func wrapStageError(stage string, err error) error {
	if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
		return err
	}
	return fmt.Errorf(`%s: %w`, stage, err)
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
	walks, err := offset2d.DraftSectionWalks(budget, dp.profile, dp.kept, work, shellTol)
	if err != nil {
		return nil, err
	}
	far, err := offset2d.DraftFarSection(budget, walks, dp.kept, dp.d, shellTol)
	if err != nil {
		return nil, err
	}
	if err := offset2d.AuditOffsetSectionBudget(budget, dp.profile, far); err != nil {
		return nil, offset2d.DraftAuditError(err)
	}
	if err := auditDraftSpan(budget, dp.profile, walks, dp.kept, dp.d, dp.dDelta); err != nil {
		return nil, err
	}
	dp.far = far
	return buildDraftBody(ctx, doc, ref, dp, work)
}

// auditDraftSpan is SD10: stages 4 and 5 again at the top of d's span, the
// amount farthest from zero that the stated sweep and taper can denote. The
// audit at d certifies every smaller amount (the crossing and contact tests
// are monotone in the amount, modify-reach §8.3.1) and none larger, so a far
// section that builds at d and fails at the span's top leaves this evaluator
// unable to decide whether the stated draft builds: ErrUnsupported, with the
// audit's own error folded in with %v so the refusal answers to one sentinel.
func auditDraftSpan(budget *proofbound.WorkBudget, profile profileRecord, walks [][]survey2d.SideWalk, kept map[offset2d.DraftWall]struct{}, d, dDelta float64) error {
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
	far, err := offset2d.DraftFarSection(budget, walks, kept, top, shellTol)
	if err == nil {
		err = offset2d.AuditOffsetSectionBudget(budget, profile, far)
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
	body := bornBody(&Body{n: topo.Body{Solid: true, Origin: provenance.Ref{Producer: ref, Role: provenance.RoleBody}, Doc: documentNode(doc)}})
	pl := cbp.prismLike(0, 0)

	var faces []*Face
	nearLoops := make([]*Loop, len(loops))
	farLoops := make([]*Loop, len(loops))
	bands := make([]capBandResult, len(loops))
	dp.patches = nil
	dp.bandDelta = make(map[capBandKey]float64, len(loops))
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		nearCo, err := draftNearRim(ctx, pl, li, loop, nearZ, nearDelta, work)
		if err != nil {
			return nil, err
		}
		band, err := buildCapBand(ctx, body, ref, cbp, li, loop, farZ, matSign, nearCo, nil, work)
		if err != nil {
			return nil, wrapStageError(`the tapered extrude's walls`, err)
		}
		faces = append(faces, band.patches...)
		nearLoops[li] = bornLoop(&Loop{n: topo.Loop{Outer: li == 0, CoEdges: nearCo}})
		farLoops[li] = bornLoop(&Loop{n: topo.Loop{Outer: li == 0, CoEdges: band.capCo}})
		bands[li] = band
		dp.farDelta = math.Max(dp.farDelta, band.delta)
		dp.bandDelta[capBandKey{loop: li, start: !dp.nearStart}] = band.delta
		if len(band.patches) != len(band.geom) {
			return nil, fmt.Errorf(`%w: the tapered extrude's wall band on loop %d built %d walls over %d patch geometries`, ErrDegenerate, li, len(band.patches), len(band.geom))
		}
		for p, f := range band.patches {
			dp.patches = append(dp.patches, capPatch{role: f.n.Origins[0].Role, geom: band.geom[p]})
		}
	}

	startLoops, endLoops := farLoops, nearLoops
	if dp.nearStart {
		startLoops, endLoops = nearLoops, farLoops
	}
	startFrame, err := pl.CapFrame(dp.z0, true)
	if err != nil {
		return nil, err
	}
	endFrame, err := pl.CapFrame(dp.z1, false)
	if err != nil {
		return nil, err
	}
	capStart := bornFace(&Face{
		n: topo.Face{
			AxialDelta:    dp.z0Delta,
			HasAxialDelta: true,
			Surface:       surfacegeom.Plane{Frame: startFrame},
			CapRecord:     &topo.CapRecord{Frame: pl.Frame, Xform: pl.Xform, Z: dp.z0},
			Origins:       []provenance.Ref{{Producer: ref, Role: provenance.RoleCapStart}},
			Body:          bodyNode(body),
			Loops:         loopNodes(startLoops),
		},
	})
	capEnd := bornFace(&Face{
		n: topo.Face{
			AxialDelta:    dp.z1Delta,
			HasAxialDelta: true,
			Surface:       surfacegeom.Plane{Frame: endFrame},
			CapRecord:     &topo.CapRecord{Frame: pl.Frame, Xform: pl.Xform, Z: dp.z1},
			Origins:       []provenance.Ref{{Producer: ref, Role: provenance.RoleCapEnd}},
			Body:          bodyNode(body),
			Loops:         loopNodes(endLoops),
		},
	})
	if err := attachFaceLoopsContext(ctx, []*Face{capStart, capEnd}); err != nil {
		return nil, err
	}
	faces = append(faces, capStart, capEnd)
	body.n.Lumps = []*topo.Lump{lumpNode(bornLump(&Lump{n: topo.Lump{Shells: []*topo.FaceShell{faceShellNode(bornFaceShell(&FaceShell{n: topo.FaceShell{Faces: faceNodes(faces)}}))}}}))}

	nearCap, farCap := capStart, capEnd
	if !dp.nearStart {
		nearCap, farCap = capEnd, capStart
	}
	if err := measureDraftBody(ctx, body, dp, cbp, bands, nearCap, farCap, work); err != nil {
		return nil, err
	}
	body.n.Payload = dp
	return body, nil
}

// draftNearRim builds one loop's near rim at level z: a vertex at every
// junction of the coalesced walks (one seam vertex for a lone closed walk) and
// one rim edge per walk, in walk order, exactly the bottom rim
// buildLoopSidesAs builds for a prism's wall at the same level. Each vertex
// carries its junction bound, the level's own displacement and its frame lift
// rounding; each edge the walk's own length and convexity. buildCapBand reads
// these as the band's side directrix, so wall i's near edge is coedge i.
func draftNearRim(ctx context.Context, pl prismPayload, li int, loop loopRecord, z, zDelta float64, work *freeform.FreeformWork) ([]topo.CoEdge, error) {
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
	level.Z0, level.Z1 = z, z
	rimVertex := func(u, v, extra float64) *Vertex {
		held, lift := pl.LiftedVertex(u, v, z)
		return bornVertex(&Vertex{n: topo.Vertex{Position: held, Bound: units.Millimeters(proofbound.AbsSumUpper(zDelta, lift, extra))}})
	}
	singleClosed := n == 1 && walks[0].Closed
	verts := make([]*Vertex, n)
	for i, w := range walks {
		prev := walks[(i+n-1)%n]
		u, v, extra := boundarywalk.JunctionVertexAt(loop.Segments, prev, w)
		verts[i] = rimVertex(u, v, extra)
	}
	co := make([]topo.CoEdge, n)
	for i, w := range walks {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		start, end := verts[i], verts[(i+1)%n]
		if singleClosed {
			end = start
		}
		convex, err := boundarywalk.RimConvexity(ctx, w, li != 0, work)
		if err != nil {
			return nil, err
		}
		edge, _, _, _, err := buildWallGeometry(level, w, convex, singleClosed, start, end, start, end)
		if err != nil {
			return nil, err
		}
		co[i] = topo.CoEdge{Edge: edgeNode(edge), Forward: true}
	}
	return co, nil
}
