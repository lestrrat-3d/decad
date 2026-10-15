package patchchain

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/units"
)

// RolePatch is a Patch face's own Origin() role (docs/surface-design.md §5).
const RolePatch = "patch"

// Build builds the patch body from the payload: one planar face carrying one
// loop per recorded loop, the free edges that bound it, and the measurements
// the finished body publishes. It mirrors prismrecord.Build's own order
// (internal/prismrecord/build.go) without ever building a solid.
func Build(ctx context.Context, env topo.Env, pp Payload, work *freeform.FreeformWork) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ig, err := pp.Profile.EvaluatorIntegralsContext(ctx, freeform.MomentAreaOrder, work)
	if err != nil {
		return nil, err
	}
	if ig.Area <= 0 {
		return nil, fmt.Errorf(`%w: the recorded region encloses no area`, decaderr.ErrDegenerate)
	}

	body := env.Alloc.NewBody()
	body.Solid = false
	body.Origin = provenance.Ref{Producer: env.Ref, Role: provenance.RoleBody}
	body.Kind = topo.BodySheet
	body.Doc = env.Doc
	prismView := pp.Prism()

	// pw resolves every boundary segment's walk exactly once for this whole
	// build (internal/momentinput/profile_walks.go), read back by every
	// buildLoop call below instead of each one resolving afresh — the same
	// reuse prismrecord.Build runs for its own loops.
	pw := pp.Walks
	if pw.Reusable(pp.Profile) {
		if err := pw.Charge(work); err != nil {
			return nil, err
		}
	} else {
		resolved, err := momentinput.ResolveProfileWalks(pp.Profile, work)
		if err != nil {
			return nil, err
		}
		pw = resolved
	}

	loops := append([]sectionrecord.LoopRecord{pp.Profile.Outer}, pp.Profile.Holes...)
	var faceLoops []*topo.Loop
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		coedges, err := buildLoop(ctx, env.Alloc, prismView, li, loop, work, pw)
		if err != nil {
			return nil, err
		}
		faceLoops = append(faceLoops, topobuild.NewLoop(env.Alloc, topo.Loop{Outer: li == 0, CoEdges: coedges}))
	}

	// The positive side is the sketch plane's own normal — the sense
	// Direction.Along names for that plane (docs/surface-design.md §5.1) —
	// so the face's frame is Payload.CapFrame at zero height with no flip, unlike a
	// prism's start cap.
	frame, err := prismView.CapFrame(0, false)
	if err != nil {
		return nil, err
	}
	face := topobuild.NewFace(env.Alloc, topo.Face{
		Area:          ig.Area,
		AreaBound:     ig.AreaBound,
		AxialDelta:    0,
		HasAxialDelta: true,
		Surface:       surfacegeom.Plane{Frame: frame},
		CapRecord:     &topo.CapRecord{Frame: prismView.Frame, Xform: prismView.Xform, Z: 0},
		Origins:       []provenance.Ref{{Producer: env.Ref, Role: RolePatch}},
		Body:          body,
		Loops:         faceLoops,
	})
	if err := topobuild.AttachFaceLoops(ctx, []*topo.Face{face}); err != nil {
		return nil, err
	}

	shell := env.Alloc.NewFaceShell()
	shell.Open = true
	shell.Faces = []*topo.Face{face}
	lump := env.Alloc.NewLump()
	lump.Shells = []*topo.FaceShell{shell}
	body.Lumps = []*topo.Lump{lump}

	body.Area = measurement.Measurement{
		Value:     units.SquareMillimeters(ig.Area),
		Exactness: measurement.ExactnessOf(ig.AreaBound),
		Bound:     units.SquareMillimeters(ig.AreaBound),
	}
	// volume and centroid stay at their zero value: finite, so
	// topobuild.ValidateAnalyticMeasurements below passes, and neither is
	// reachable through Body.Volume/Body.Centroid while solid is false
	// (docs/surface-design.md §8).
	bounds, err := prismrecord.Bounds(ctx, prismView, work, pw)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	body.Bounds = bounds
	if err := prismrecord.ChargeMap(body, pp.Frame, pp.Xform); err != nil {
		return nil, err
	}
	if err := topobuild.ValidateAnalyticMeasurements(body); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	pp.Walks = pw
	body.Payload = pp
	return body, nil
}

// buildLoop builds one recorded loop's free boundary: one vertex per
// junction, one edge per coalesced walk, and exactly one coedge per edge
// (forward: true) — a patch has no side wall and no cap to share an edge
// with, so every edge it mints is free by construction. It reuses
// prismrecord.BuildLoopSidesAs's own per-kind curve construction and its circular
// edgeSign rule (internal/prismrecord/build.go), which prismrecord.BuildLoopSidesAs's own doc comment
// notes is a template rather than a callable for exactly this reason: a
// patch's loop has no bottom/top pair, no vertical edge and no per-walk face.
//
// pp is Payload.Prism()'s zero-height view. holeLoop is li != 0, the
// same convention prismrecord.BuildLoopSides derives it by. resolved is pp.Profile's
// pre-resolved segment walks, or nil to resolve each segment through walkOf
// as before (internal/momentinput/profile_walks.go).
func buildLoop(ctx context.Context, alloc topo.Alloc, pp prismrecord.Payload, li int, loop sectionrecord.LoopRecord, work *freeform.FreeformWork, resolved *momentinput.ProfileWalks) ([]topo.CoEdge, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(loop.Segments) == 0 {
		return nil, fmt.Errorf(`%w: a recorded loop holds no segments`, decaderr.ErrDegenerate)
	}
	holeLoop := li != 0

	var loopWalks []survey2d.SegmentWalk
	if resolved != nil {
		if !resolved.LoopMatches(li, loop) {
			return nil, momentinput.ErrResolvedWalksMismatch
		}
		loopWalks = resolved.LoopWalks(li)
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
		var w survey2d.SegmentWalk
		if loopWalks != nil {
			w = loopWalks[i]
		} else {
			w, err = boundarywalk.WalkOf(seg, work)
			if err != nil {
				return nil, err
			}
		}
		raw[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
	}
	walks, err := boundarywalk.CoalesceWalksContext(ctx, raw)
	if err != nil {
		return nil, err
	}
	n := len(walks)
	singleClosed := n == 1 && walks[0].Closed

	// A patch's own record is its own denotation (Payload carries no
	// section displacement), so the section and level terms below are always
	// zero; they are composed anyway, exactly as prismrecord.BuildLoopSidesAs composes its
	// rim vertices, so a later field this payload gains is charged the same way
	// rather than silently skipped. Beside them every rim vertex carries its
	// own exact frame lift and placement rounding (prismrecord.Payload.LiftedVertex),
	// zero wherever that lift is exact for the coordinates at hand.
	rimVertex := func(u, v, extra float64) *topo.Vertex {
		held, lift := pp.LiftedVertex(u, v, pp.Z0)
		base := proofbound.AbsSumUpper(pp.SectionDelta, pp.Z0Delta, lift)
		return topobuild.NewVertex(alloc, topo.Vertex{Position: held, Bound: units.Millimeters(proofbound.AbsSumUpper(base, extra))})
	}
	var seam *topo.Vertex
	var verts []*topo.Vertex
	if singleClosed {
		w := walks[0]
		seam = rimVertex(boundarywalk.JunctionVertexAt(loop.Segments, w, w))
	} else {
		verts = make([]*topo.Vertex, n)
		for i, w := range walks {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			prev := walks[(i+n-1)%n]
			verts[i] = rimVertex(boundarywalk.JunctionVertexAt(loop.Segments, prev, w))
		}
	}

	coedges := make([]topo.CoEdge, 0, n)
	for i, w := range walks {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		var start, end *topo.Vertex
		if singleClosed {
			start, end = seam, seam
		} else {
			start, end = verts[i], verts[(i+1)%n]
		}
		convex, err := boundarywalk.RimConvexity(ctx, w, holeLoop, work)
		if err != nil {
			return nil, err
		}
		var curve surfacegeom.Curve
		switch w.Kind {
		case survey2d.WalkCircular:
			// The same circular edgeSign rule prismrecord.BuildLoopSidesAs's survey2d.WalkCircular
			// arm applies to its own rim edges: an Arc3/Circle3 is CCW from
			// start to end about its axis, and a clockwise walk (or a
			// reflected placement) inverts that sense.
			clockwise := w.Th1 < w.Th0
			edgeSign := 1.0
			if clockwise {
				edgeSign = -1
			}
			if pp.Reflected() {
				edgeSign = -edgeSign
			}
			edgeAxis := pp.Dir(0, 0, 1).Scale(edgeSign)
			center := pp.Point(w.CU, w.CV, pp.Z0)
			radius := units.Millimeters(w.Radius)
			if singleClosed {
				curve = surfacegeom.Circle3{Center: center, Axis: edgeAxis, Radius: radius}
			} else {
				curve = surfacegeom.Arc3{Center: center, Axis: edgeAxis, Radius: radius}
			}
		case survey2d.WalkFreeform:
			curve = surfacegeom.NURBSCurve{}
		default:
			curve = surfacegeom.Line3{}
		}
		edge := topobuild.NewEdge(alloc, topo.Edge{Convex: convex, Length: w.Length, LengthBound: w.LengthBound, Curve: curve, Start: start, End: end})
		switch c := curve.(type) {
		case surfacegeom.Circle3:
			edge.CurveBound, edge.CurveBounded = pp.CircleCurveBound(w.CU, w.CV, pp.Z0, pp.Z0Delta, 0, w.Radius, w.RadiusBound, c.Center, c.Axis)
		case surfacegeom.Arc3:
			edge.CurveBound, edge.CurveBounded = pp.CircleCurveBound(w.CU, w.CV, pp.Z0, pp.Z0Delta, 0, w.Radius, w.RadiusBound, c.Center, c.Axis)
		}
		coedges = append(coedges, topo.CoEdge{Edge: edge, Forward: true})
	}
	return coedges, nil
}
