package draftrecord

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/units"
)

// This file builds a draft body over topology nodes (agent-docs/draft-design.md
// §7): the gates of Table SD in §5's stage order, the near cap over the
// recorded section P, the far cap over its sharp offset Q, and one ruled wall
// per walk between them. The walls are the cap-loop band of
// agent-docs/modify-reach-design.md §8.3 with no straight slab below it:
// Payload.Band hands capblend.BuildBand a view whose side level is the sketch
// plane and whose cap contour is Q, so every wall patch, ruling and far-cap
// edge, and every bound they carry, comes from that one construction. The
// measurements are measure.go's.

// Build runs the record gates of Table SD in stage order and, once every one
// passes, builds the body under env. Stage 3 resolves every loop's walks (SD3)
// and corners (SD4, SD15); stage 4 offsets them (SD5, SD6, SD7, SD13); stage 5
// audits the far section (SD6, SD8, SD9); stage 6 audits it again at the top of
// d's span (SD10); stage 7 builds the patches (SD14, SD16). Nothing is made
// before stage 7, and nothing is committed here.
func Build(ctx context.Context, env topo.Env, dp Payload) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	height := proofbound.BoundedSub(proofbound.MeasuredScalar(dp.Z1, dp.Z1Delta), proofbound.MeasuredScalar(dp.Z0, dp.Z0Delta))
	if !(height.Value > 0) {
		return nil, fmt.Errorf(`%w: the sweep interval is empty`, decaderr.ErrDegenerate)
	}
	off, offDelta, err := Offset(height.Value, height.Bound, dp.Taper, dp.TaperDelta)
	if err != nil {
		return nil, err
	}
	dp.D, dp.DDelta = off, offDelta

	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	walks, err := offset2d.DraftSectionWalks(budget, dp.Profile, dp.Kept, work, offset2d.ShellTol)
	if err != nil {
		return nil, err
	}
	far, err := offset2d.DraftFarSection(budget, walks, dp.Kept, dp.D, offset2d.ShellTol)
	if err != nil {
		return nil, err
	}
	if err := offset2d.AuditOffsetSectionBudget(budget, dp.Profile, far); err != nil {
		return nil, offset2d.DraftAuditError(err)
	}
	if err := auditSpan(budget, dp.Profile, walks, dp.Kept, dp.D, dp.DDelta); err != nil {
		return nil, err
	}
	dp.Far = far
	return buildBody(ctx, env, dp, work)
}

// auditSpan is SD10: stages 4 and 5 again at the top of d's span, the
// amount farthest from zero that the stated sweep and taper can denote. The
// audit at d certifies every smaller amount (the crossing and contact tests
// are monotone in the amount, modify-reach §8.3.1) and none larger, so a far
// section that builds at d and fails at the span's top leaves this evaluator
// unable to decide whether the stated draft builds: ErrUnsupported, with the
// audit's own error folded in with %v so the refusal answers to one sentinel.
func auditSpan(budget *proofbound.WorkBudget, profile momentinput.Profile, walks [][]survey2d.SideWalk, kept map[offset2d.DraftWall]struct{}, d, dDelta float64) error {
	if !(dDelta > 0) {
		return nil
	}
	rd, rw := proofarith.FloatRat(math.Abs(d)), proofarith.FloatRat(dDelta)
	if rd == nil || rw == nil {
		return errOffsetUnbounded
	}
	top := proofbound.RatFloatUp(new(big.Rat).Add(rd, rw))
	if d < 0 {
		top = -top
	}
	far, err := offset2d.DraftFarSection(budget, walks, kept, top, offset2d.ShellTol)
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
	return fmt.Errorf(`%w: the taper's offset amount is known only within the span its sweep and tangent enclosures state, and at the top of that span the far section fails its audit (%v); this evaluator cannot decide whether the stated draft builds (draft SD10)`, decaderr.ErrUnsupported, err)
}

// buildBody is stage 7 and the assembly (agent-docs/draft-design.md §7 steps
// 7–9): per loop the near rim at the sketch plane, then capblend.BuildBand's walls
// and far rim over it; the two caps over the near and far rims; one shell,
// one lump. Every edge bounds exactly two faces: a near rim edge its wall and
// the near cap, a far rim edge its wall and the far cap, a ruling its two
// walls. measure.go then measures the body.
func buildBody(ctx context.Context, env topo.Env, dp Payload, work *freeform.FreeformWork) (*topo.Body, error) {
	cbp := dp.Band()
	loops := cbp.Loops()
	nearZ, nearDelta, farZ, matSign := dp.Levels()
	ref := env.Ref
	body := env.Alloc.NewBody()
	body.Solid = true
	body.Origin = provenance.Ref{Producer: ref, Role: provenance.RoleBody}
	body.Doc = env.Doc
	pl := cbp.PrismLike(0, 0)

	var faces []*topo.Face
	nearLoops := make([]*topo.Loop, len(loops))
	farLoops := make([]*topo.Loop, len(loops))
	bands := make([]capblend.BandResult, len(loops))
	dp.Patches = nil
	dp.BandDelta = make(map[capblend.BandKey]float64, len(loops))
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		nearCo, err := nearRim(ctx, env.Alloc, pl, li, loop, nearZ, nearDelta, work)
		if err != nil {
			return nil, err
		}
		band, err := capblend.BuildBand(ctx, env,
			body, cbp, li, loop, farZ, matSign, nearCo, nil, work)
		if err != nil {
			return nil, compositesweep.WrapStageError(`the tapered extrude's walls`, err)
		}
		faces = append(faces, band.Patches...)
		nearLoops[li] = topobuild.NewLoop(env.Alloc, topo.Loop{Outer: li == 0, CoEdges: nearCo})
		farLoops[li] = topobuild.NewLoop(env.Alloc, topo.Loop{Outer: li == 0, CoEdges: band.CapCo})
		bands[li] = band
		dp.FarDelta = math.Max(dp.FarDelta, band.Delta)
		dp.BandDelta[capblend.BandKey{Loop: li, Start: !dp.NearStart}] = band.Delta
		if len(band.Patches) != len(band.Geom) {
			return nil, fmt.Errorf(`%w: the tapered extrude's wall band on loop %d built %d walls over %d patch geometries`, decaderr.ErrDegenerate, li, len(band.Patches), len(band.Geom))
		}
		for p, f := range band.Patches {
			dp.Patches = append(dp.Patches, capblend.Patch{Role: f.Origins[0].Role, Geom: band.Geom[p]})
		}
	}

	startLoops, endLoops := farLoops, nearLoops
	if dp.NearStart {
		startLoops, endLoops = nearLoops, farLoops
	}
	startFrame, err := pl.CapFrame(dp.Z0, true)
	if err != nil {
		return nil, err
	}
	endFrame, err := pl.CapFrame(dp.Z1, false)
	if err != nil {
		return nil, err
	}
	capStart := topobuild.NewFace(env.Alloc, topo.Face{
		AxialDelta:    dp.Z0Delta,
		HasAxialDelta: true,
		Surface:       surfacegeom.Plane{Frame: startFrame},
		CapRecord:     &topo.CapRecord{Frame: pl.Frame, Xform: pl.Xform, Z: dp.Z0},
		Origins:       []provenance.Ref{{Producer: ref, Role: provenance.RoleCapStart}},
		Body:          body,
		Loops:         startLoops,
	})
	capEnd := topobuild.NewFace(env.Alloc, topo.Face{
		AxialDelta:    dp.Z1Delta,
		HasAxialDelta: true,
		Surface:       surfacegeom.Plane{Frame: endFrame},
		CapRecord:     &topo.CapRecord{Frame: pl.Frame, Xform: pl.Xform, Z: dp.Z1},
		Origins:       []provenance.Ref{{Producer: ref, Role: provenance.RoleCapEnd}},
		Body:          body,
		Loops:         endLoops,
	})
	if err := topobuild.AttachFaceLoops(ctx, []*topo.Face{capStart, capEnd}); err != nil {
		return nil, err
	}
	faces = append(faces, capStart, capEnd)
	shell := env.Alloc.NewFaceShell()
	shell.Faces = faces
	lump := env.Alloc.NewLump()
	lump.Shells = []*topo.FaceShell{shell}
	body.Lumps = []*topo.Lump{lump}

	nearCap, farCap := capStart, capEnd
	if !dp.NearStart {
		nearCap, farCap = capEnd, capStart
	}
	if err := measureBody(ctx, body, dp, cbp, bands, nearCap, farCap, work); err != nil {
		return nil, err
	}
	body.Payload = dp
	return body, nil
}

// nearRim builds one loop's near rim at level z: a vertex at every
// junction of the coalesced walks (one seam vertex for a lone closed walk) and
// one rim edge per walk, in walk order, exactly the bottom rim
// prismrecord.BuildLoopSidesAs builds for a prism's wall at the same level. Each vertex
// carries its junction bound, the level's own displacement and its frame lift
// rounding; each edge the walk's own length and convexity. capblend.BuildBand reads
// these as the band's side directrix, so wall i's near edge is coedge i.
func nearRim(ctx context.Context, alloc topo.Alloc, pl prismrecord.Payload, li int, loop sectionrecord.LoopRecord, z, zDelta float64, work *freeform.FreeformWork) ([]topo.CoEdge, error) {
	if len(loop.Segments) == 0 {
		return nil, fmt.Errorf(`%w: a recorded loop holds no segments`, decaderr.ErrDegenerate)
	}
	raw := make([]survey2d.SideWalk, len(loop.Segments))
	for i, seg := range loop.Segments {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		seg, err := sectionrecord.NormalizeSegment(seg)
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
	rimVertex := func(u, v, extra float64) *topo.Vertex {
		held, lift := pl.LiftedVertex(u, v, z)
		return topobuild.NewVertex(alloc, topo.Vertex{Position: held, Bound: units.Millimeters(proofbound.AbsSumUpper(zDelta, lift, extra))})
	}
	singleClosed := n == 1 && walks[0].Closed
	verts := make([]*topo.Vertex, n)
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
		edge, _, _, _, err := prismrecord.BuildWallGeometry(alloc, level, w, convex, singleClosed, start, end, start, end) //nolint:contextcheck // The ordinary wall builder has no replay context or budget.
		if err != nil {
			return nil, err
		}
		co[i] = topo.CoEdge{Edge: edge, Forward: true}
	}
	return co, nil
}
