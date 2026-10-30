package breprecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/units"
)

// Build builds the body a Payload records (§4.2, §4.3). Every
// edge is shared by exactly two faces, identified by record identity in the
// reference frame, so the body closes by construction and the count proves it.
// Roles are the record's own face(k)/wall(k) under ref; no capStart or capEnd
// is minted and no operand provenance is carried. A route L band's boundary
// edges bound one record face each, and the band's patches, attached after
// the record's faces, bound their other side (band.go). Every node comes from
// env.Alloc, and the body links env.Doc under producer env.Ref.
func Build(ctx context.Context, env topo.Env, bp Payload) (*topo.Body, error) {
	// Point-query Cut sources describe only the A7 body first built from them.
	// Rebuilds must not carry them across edits or placement changes.
	bp.PointQueryCut = nil
	ref, alloc := env.Ref, env.Alloc
	btopo, err := TopologyContext(ctx, bp)
	if err != nil {
		return nil, err
	}
	body := alloc.NewBody()
	body.Solid = true
	body.Origin = provenance.Ref{Producer: ref, Role: provenance.RoleBody}
	body.Kind = topo.BodySolid
	body.Doc = env.Doc
	refView := bp.RefView()

	// Vertices are shared by reference coordinates. Each carries the largest
	// displacement any use placing it states — its face's section displacement,
	// its level's, and the bound from its walk end to the point the use's
	// recorded segment denotes there (brepgeom.Use's StartBound and EndBound,
	// which add an arc's radial residual at its natural t = 1 end) — beside
	// the vertex's own exact frame and placement lift rounding
	// (prismrecord.Payload.LiftedVertex), measured once per vertex when it is first
	// placed. Every use meeting at a vertex holds the same reference
	// coordinates, so the largest of them reaches every neighbour's denoted
	// end, as boundarywalk.JunctionVertex's bound does for a prism.
	type placedVertex struct {
		v    *topo.Vertex
		lift float64
	}
	vertices := map[[3]float64]placedVertex{}
	// placeVertex returns the vertex at reference coordinates c, widening its
	// bound to cover one more use: faceDelta and levelDelta are that use's
	// section and level displacements and endAllow its denoted-end bound.
	placeVertex := func(c [3]float64, faceDelta, levelDelta, endAllow float64) *topo.Vertex {
		pv, ok := vertices[c]
		if !ok {
			held, lift := refView.LiftedVertex(c[0], c[1], c[2])
			// The bound starts as zero millimetres, so an Exact vertex
			// publishes a length as every other vertex does.
			pv = placedVertex{v: topobuild.NewVertex(alloc, topo.Vertex{Position: held, Bound: units.Millimeters(0)}), lift: lift}
			vertices[c] = pv
		}
		if b := proofbound.AbsSumUpper(proofbound.AbsSumUpper(faceDelta, levelDelta, pv.lift), endAllow); b > pv.v.Bound.Base() {
			pv.v.Bound = units.Millimeters(b)
		}
		return pv.v
	}
	for ui, u := range btopo.Uses {
		// A whole circle's seam is its rim's: a loop that walks the same
		// circle from another seam places no vertex of its own. A band's
		// whole circle places its seam with its band edge (openEdges).
		closedOpen := u.Key.Closed && btopo.EdgeOf[ui] < 0
		if (u.Part != brepgeom.LoopSeg && !brepgeom.IsRim(u.Part)) || (u.Part == brepgeom.LoopSeg && u.Key.Closed) || closedOpen {
			continue
		}
		f := bp.Faces[u.Face]
		placeVertex(u.DirFrom, f.Delta, u.LevelDelta, proofbound.WalkEndBoundAllow(u.StartBound()))
		placeVertex(u.DirTo, f.Delta, u.LevelDelta, proofbound.WalkEndBoundAllow(u.EndBound()))
	}
	// vertexAt is the lookup buildEdge reads: every vertex it names was already
	// placed by a use above, so it adds no displacement of its own.
	vertexAt := func(c [3]float64) *topo.Vertex { return placeVertex(c, 0, 0, 0) }

	edges := make([]*topo.Edge, len(btopo.Edges))
	for ei, pair := range btopo.Edges {
		edge, err := buildEdge(ctx, alloc, bp, btopo, pair, vertexAt)
		if err != nil {
			return nil, err
		}
		edges[ei] = edge
	}
	open, err := openEdges(ctx, alloc, bp, btopo, placeVertex)
	if err != nil {
		return nil, err
	}
	coedgeOf := func(ui int) topo.CoEdge {
		if ei := btopo.EdgeOf[ui]; ei >= 0 {
			return topo.CoEdge{Edge: edges[ei], Forward: btopo.Forward(ui)}
		}
		return open.coedge[ui]
	}

	faces := make([]*topo.Face, len(bp.Faces))
	for fi, f := range bp.Faces {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		face, err := faceBody(ctx, alloc, bp, btopo, fi, body, ref)
		if err != nil {
			return nil, err
		}
		uses := btopo.FaceUses[fi]
		switch {
		case f.Planar():
			loops := make([]*topo.Loop, len(btopo.Planar[fi]))
			for li := range loops {
				loops[li] = topobuild.NewLoop(alloc, topo.Loop{Outer: li == 0})
			}
			for _, ui := range uses {
				l := loops[btopo.Uses[ui].Loop]
				l.CoEdges = append(l.CoEdges, coedgeOf(ui))
			}
			face.Loops = loops
		case len(uses) == 2:
			face.Loops = []*topo.Loop{
				topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{coedgeOf(uses[0])}}),
				topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{coedgeOf(uses[1])}}),
			}
		default:
			l := topobuild.NewLoop(alloc, topo.Loop{Outer: true})
			for _, ui := range uses {
				l.CoEdges = append(l.CoEdges, coedgeOf(ui))
			}
			face.Loops = []*topo.Loop{l}
		}
		faces[fi] = face
	}
	if err := topobuild.AttachFaceLoops(ctx, faces); err != nil {
		return nil, err
	}
	bands, err := attachLoopBands(ctx, alloc, body, ref, bp, open)
	if err != nil {
		return nil, err
	}
	if bp.BossShell != nil {
		patches, err := attachBossShellBand(ctx, alloc, body, ref, bp, open)
		if err != nil {
			return nil, err
		}
		bands.patches = append(bands.patches, patches...)
	}
	if bp.PocketShell != nil {
		patches, err := attachPocketShellBand(ctx, alloc, body, ref, bp, open)
		if err != nil {
			return nil, err
		}
		bands.patches = append(bands.patches, patches...)
	}
	allFaces := append(faces, bands.patches...)
	if bp.BossShell != nil && bp.BossShell.Closed {
		var lumps []*topo.Lump
		lumps, err = closedBossShellLumps(alloc, allFaces, bp.BossShell.OuterFaceCount)
		body.Lumps = lumps
		if err != nil {
			return nil, err
		}
	} else {
		body.Lumps = topobuild.SheetLumps(alloc, allFaces)
	}
	if bp.BossShell != nil {
		err = measureBossShell(ctx, bp, body)
	} else if bp.PocketShell != nil {
		err = measurePocketShell(ctx, bp, body)
	} else {
		err = measureBody(ctx, bp, btopo, body, bands.mass)
	}
	if err != nil {
		return nil, err
	}
	// Every face frame is a signed permutation of the first (Embeds), so
	// each face's map has the first face's determinant and defect.
	if err := prismrecord.ChargeMap(body, bp.Faces[0].Frame, bp.Xform); err != nil {
		return nil, err
	}
	bp.LoopPatches = bands.geom
	body.Payload = bp
	return body, nil
}

// BuildWithPointQueryCut builds bp and attaches its transient exact source
// prisms to the resulting body. Generic Build calls intentionally discard them.
func BuildWithPointQueryCut(ctx context.Context, env topo.Env, bp Payload) (*topo.Body, error) {
	cut := bp.PointQueryCut
	bp.PointQueryCut = nil
	body, err := Build(ctx, env, bp)
	if err != nil || cut == nil {
		return body, err
	}
	built, ok := body.Payload.(Payload)
	if !ok {
		return nil, fmt.Errorf("%w: BRep build did not retain its payload", decaderr.ErrUnsupported)
	}
	built.PointQueryCut = cut
	body.Payload = built
	return body, nil
}

// buildEdge builds one edge from its owner use: a rim takes its wall's curve and
// walk length, a side line runs bottom to top over the wall's bounded height,
// and a loop segment shared by two planar faces is a line of its walk's
// length. Convexity reads evaluator §3's walked boundary (edgeConvex).
func buildEdge(ctx context.Context, alloc topo.Alloc, bp Payload, btopo *Topology, pair [2]int, vertexAt func([3]float64) *topo.Vertex) (*topo.Edge, error) {
	owner := btopo.Uses[pair[0]]
	f := bp.Faces[owner.Face]
	start := vertexAt(owner.DirFrom)
	end := vertexAt(owner.DirTo)
	convex, err := edgeConvex(ctx, btopo, pair)
	if err != nil {
		return nil, err
	}
	switch owner.Part {
	case brepgeom.Side0, brepgeom.Side1:
		// One piece of the side line: its own two levels, read along the
		// sweep axis in reference coordinates.
		axis := btopo.Embeds[owner.Face].Axis[2]
		lo, hi := owner.DirFrom[axis], owner.DirTo[axis]
		if lo > hi {
			lo, hi = hi, lo
		}
		height := proofbound.BoundedSub(proofbound.MeasuredScalar(hi, owner.SideDelta[1]), proofbound.MeasuredScalar(lo, owner.SideDelta[0]))
		return topobuild.NewEdge(alloc, topo.Edge{
			Convex:      convex,
			Length:      hi - lo,
			LengthBound: height.Bound,
			Curve:       surfacegeom.Line3{},
			Start:       start,
			End:         end,
		}), nil
	case brepgeom.Rim0, brepgeom.Rim1:
		w := btopo.Walls[owner.Face]
		pp := f.View(bp.Xform)
		bottom, top, _, _, err := prismrecord.BuildWallGeometry(alloc, pp, survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}}, convex, w.Closed, start, end, start, end)
		if err != nil {
			return nil, err
		}
		edge := bottom
		if owner.Part == brepgeom.Rim1 {
			edge = top
		}
		edge.LengthBound = proofbound.AbsSumUpper(w.LengthBound, proofbound.SectionDisplacementLength(f.Delta, 1))
		return edge, nil
	default:
		w := owner.Walk
		return topobuild.NewEdge(alloc, topo.Edge{
			Convex:      convex,
			Length:      w.Length,
			LengthBound: proofbound.AbsSumUpper(w.LengthBound, proofbound.SectionDisplacementLength(f.Delta, 1)),
			Curve:       surfacegeom.Line3{},
			Start:       start,
			End:         end,
		}), nil
	}
}

// edgeConvex decides an edge's walked-boundary convexity (root topology.go's
// Edge.IsConvex). A rim reads the wall it runs along: a circular wall by its
// own turn, a straight wall by the role of the loop that wall belongs to,
// which the planar face's loop states — its own role when it walks the rim the
// way the wall does, the other role when it walks it the opposite way (a
// stacked floor is a reversed hole). A line two walls of one sweep share is
// a junction, convex when the walk turns left there about the sweep axis: a
// side line between two swept walls or between a swept wall and a planar face
// restating a wall along the same axis, and a line two such planar faces
// share (Face.sweep). Any other line reads the owner planar face's loop
// role: outer convex, hole concave.
func edgeConvex(ctx context.Context, btopo *Topology, pair [2]int) (bool, error) {
	if err := ctx.Err(); err != nil {
		return false, err
	}
	return brepgeom.Convex(pair, btopo.Uses, btopo.Embeds, btopo.Walls, decaderr.ErrUnsupported)
}

// faceBody builds one face's own geometry, role and area; the caller
// attaches its loops. A planar face's plane is the cap frame its view states
// at its level, turned to its outward normal; a swept face's surface is the
// prism wall's own (prismrecord.BuildWallGeometry).
func faceBody(ctx context.Context, alloc topo.Alloc, bp Payload, btopo *Topology, fi int, body *topo.Body, ref provenance.Producer) (*topo.Face, error) {
	f := bp.Faces[fi]
	pp := f.View(bp.Xform)
	origins := []provenance.Ref{{Producer: ref, Role: f.Role}}
	if f.Blend != "" {
		origins = append(origins, provenance.Ref{Producer: ref, Role: fmt.Sprintf("%s(%d)", f.Blend, fi)})
	}
	if f.Planar() {
		frame, err := pp.CapFrame(f.Z0, !f.Outward)
		if err != nil {
			return nil, err
		}
		region, err := btopo.Region(ctx, bp, fi)
		if err != nil {
			return nil, err
		}
		return topobuild.NewFace(alloc, topo.Face{
			Area:          region.Published.Value,
			AreaBound:     region.Published.Bound,
			AxialDelta:    f.Z0Delta,
			HasAxialDelta: true,
			Surface:       surfacegeom.Plane{Frame: frame},
			CapRecord:     &topo.CapRecord{Frame: pp.Frame, Xform: pp.Xform, Z: f.Z0},
			Origins:       origins,
			Body:          body,
		}), nil
	}
	w := btopo.Walls[fi]
	_, _, surf, reversed, err := prismrecord.BuildWallGeometry(alloc, pp, survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}}, false, w.Closed, nil, nil, nil, nil)
	if err != nil {
		return nil, err
	}
	area := brepgeom.WallArea(f.Delta, f.Z0, f.Z1, f.Z0Delta, f.Z1Delta, w)
	return topobuild.NewFace(alloc, topo.Face{
		Area:      area.Value,
		AreaBound: area.Bound,
		Reversed:  reversed,
		Surface:   surf,
		Origins:   origins,
		Body:      body,
	}), nil
}
