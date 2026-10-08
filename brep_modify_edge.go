package decad

import (
	"context"
	"errors"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file is route E of docs/brep-modify-design.md ("brep-modify §N"): a
// Fillet or Chamfer of straight brep edges that run along one reference
// axis (§5). Table EB admits each edge; the corner blend is computed in both
// end faces, which must agree (SB9); the end faces take the corner rewrite,
// the two faces beside the edge are trimmed to the blend's feet, one swept
// blend face is appended per edge, and the rewritten record proves its own
// closure by pairing every edge again (§5.3).
//
// A swept straight wall the construction needs as a plane — an end face, or
// a wall holding the edge as a rim — is first restated as the planar
// rectangle it sweeps (§5.2, brepgeom.Restate), once for the whole call, and
// every edge is then classified against the restated record.

// brepEdgeSide classifies one face beside an admitted edge (Table EB, EB4).
type brepEdgeSide int

const (
	brepSideSwept  brepEdgeSide = iota // (sw): a swept face along the edge's axis whose side line is the edge
	brepSidePlanar                     // (pl): a planar face that holds the edge as one loop segment
)

// brepEdgeEnd is one end face's corner at an admitted edge's vertex: the
// loop and corner of the face's corner walk whose junction is the vertex
// (brepCornerAt), and which of the edge's two adjacent faces the arriving
// walk there pairs with.
type brepEdgeEnd struct {
	face     int
	loop     int
	corner   int
	arriving int
}

// brepEdgeBlend is one selected edge through route E: its use pair in the
// receiver's topology, its axis, its two vertices in reference coordinates
// (V0 at the lower axis coordinate), its two adjacent faces with their
// classes, its two end faces, and the blend computed in each end face.
type brepEdgeBlend struct {
	ordinal int
	edge    *Edge
	pair    [2]int
	axis    int
	v       [2][3]float64
	adj     [2]int
	side    [2]brepEdgeSide
	end     [2]brepEdgeEnd
	blend   [2]*cornerBlend
}

// brepEdgeRoute holds one route E call's readings of the receiver: its
// record, topology, and each planar face's corner walk, read once.
type brepEdgeRoute struct {
	bp     brepPayload
	topo   *brepTopology
	call   brepModifyRequest
	budget *proofbound.WorkBudget
	loops  map[int][]cornerLoop
	// loopUse maps a planar face's (face, loop, segment) to its use index.
	loopUse map[[3]int]int
}

// newBrepEdgeRoute reads one record for route E: its topology's loop-segment
// uses are indexed once, and corner walks are read on demand.
func newBrepEdgeRoute(bp brepPayload, topo *brepTopology, call brepModifyRequest, budget *proofbound.WorkBudget) *brepEdgeRoute {
	r := &brepEdgeRoute{bp: bp, topo: topo, call: call, budget: budget,
		loops: map[int][]cornerLoop{}, loopUse: map[[3]int]int{}}
	for ui, u := range topo.uses {
		if u.Part == brepLoopSeg {
			r.loopUse[[3]int{u.Face, u.Loop, u.Seg}] = ui
		}
	}
	return r
}

// brepBlendEdges is route E (brep-modify §5): it builds the Fillet or
// Chamfer of call.edges on the receiver record bp, or refuses in Table SB's
// gate order (§6): EB1/SB4 per edge, EB7/SB5 over the set; then EB2/SB6
// over every edge; then the restatement passes (§5.2), EB3/SB7 over every
// edge's end faces and SB8 over every wall holding an edge as a rim; then,
// against the restated record, EB4/SB8, EB5/SB8 and EB6 over every edge;
// then each edge's corner blend (S4, S5) in both end faces and SB9; then the
// audit of every rewritten face (S8, S6, S7, S9) and trimmed wall (S6); then
// the closure.
func brepBlendEdges(ctx context.Context, d *Document, bp brepPayload, call brepModifyRequest) (*Body, error) {
	topo, err := brepTopologyContext(ctx, bp)
	if err != nil {
		return nil, err
	}
	r := newBrepEdgeRoute(bp, topo, call, proofbound.NewWorkBudget(ctx))

	// Stage 2c: each edge is a straight line along one reference axis (EB1),
	// and no two share a vertex (EB7).
	blends := make([]*brepEdgeBlend, len(call.edges))
	for ei, e := range call.edges {
		eb, err := r.admitEdge(ei, e)
		if err != nil {
			return nil, err
		}
		blends[ei] = eb
	}
	vertexOf := map[[3]float64]*brepEdgeBlend{}
	for _, eb := range blends {
		for _, v := range eb.v {
			if other, ok := vertexOf[v]; ok {
				return nil, r.refuse(eb, "SB5", fmt.Sprintf(`it shares the vertex at (%s) with %s; a vertex blend is not supported`,
					r.render(v), selectedEdgeContext(other.ordinal, other.edge)))
			}
			vertexOf[v] = eb
		}
	}

	// Stage 3: the edge topology, gate by gate over every edge.
	incident := r.incidence()
	for _, eb := range blends {
		if err := r.requireThreeEdges(eb, incident); err != nil {
			return nil, err
		}
	}
	// The restatement passes (§5.2) collect every wall the call needs as a
	// plane, end faces first (SB7), then rim-adjacent walls (SB8); a wall one
	// edge needs as an end face and another beside it is one plane for both.
	restated := map[int]brepRestated{}
	for _, eb := range blends {
		if err := r.findEndFaces(eb, incident, restated); err != nil {
			return nil, err
		}
	}
	for _, eb := range blends {
		if err := r.restateRims(eb, restated); err != nil {
			return nil, err
		}
	}
	if len(restated) > 0 {
		if r, err = r.withRestated(ctx, restated); err != nil {
			return nil, err
		}
		// The restated record keeps every face index and every edge, but
		// numbers its uses afresh: each edge is matched again, keeping the
		// end faces the first pass found.
		for ei, eb := range blends {
			again, err := r.admitEdge(eb.ordinal, eb.edge)
			if err != nil {
				return nil, err
			}
			again.end = eb.end
			blends[ei] = again
		}
	}
	for _, eb := range blends {
		if err := r.classifySides(eb); err != nil {
			return nil, err
		}
	}
	for _, eb := range blends {
		if err := r.requireNaturalFaces(eb); err != nil {
			return nil, err
		}
	}
	for _, eb := range blends {
		if err := r.locateCorners(eb); err != nil {
			return nil, err
		}
	}

	// Stage 4: the construction's own gates per edge — S4 and S5 in G0, then
	// the recomputation in G1 and SB9.
	for _, eb := range blends {
		if err := r.computeBlends(eb); err != nil {
			return nil, err
		}
	}

	out, err := r.rewrite(blends)
	if err != nil {
		return nil, err
	}

	// Stage 6: the closure. The body build pairs every edge of the rewritten
	// record again; an edge that does not pair refuses there.
	body, err := evalBrepContext(ctx, d, d.nextProducerID(), out)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, ctxErr
		}
		return nil, fmt.Errorf(`%w; the %s's rewritten brep record (brep-modify §5.3 step 7); selector %s matched [%s]`,
			err, call.blend.kind, call.sel, selectedEdgesContext(call.edges))
	}
	if body.volume.Value.Base() <= 0 {
		return nil, fmt.Errorf(`%w: the %s's rewritten brep record encloses no volume (brep-modify §5.3 step 7); selector %s matched [%s]`,
			ErrDegenerate, call.blend.kind, call.sel, selectedEdgesContext(call.edges))
	}
	return body, nil
}

// refuse is one Table SB refusal for one selected edge.
func (r *brepEdgeRoute) refuse(eb *brepEdgeBlend, row, reason string) error {
	return fmt.Errorf(`%w: this evaluator %s a brep edge through route E only where Table EB admits it; %s (brep-modify %s); selector %s, %s`,
		ErrUnsupported, r.call.op, reason, row, r.call.sel, selectedEdgeContext(eb.ordinal, eb.edge))
}

// render names a reference-coordinate point by its placed position.
func (r *brepEdgeRoute) render(c [3]float64) string {
	return renderVec(r.bp.refView().point(c[0], c[1], c[2]))
}

// partner is the other use of use ui's edge.
func (r *brepEdgeRoute) partner(ui int) int {
	pair := r.topo.edges[r.topo.edgeOf[ui]]
	if pair[0] == ui {
		return pair[1]
	}
	return pair[0]
}

// cornerLoops reads a planar face's region into its corner walks once.
func (r *brepEdgeRoute) cornerLoops(fi int) ([]cornerLoop, error) {
	if loops, ok := r.loops[fi]; ok {
		return loops, nil
	}
	loops, err := profileCornerLoopsBudget(r.budget, *r.bp.faces[fi].region)
	if err != nil {
		return nil, err
	}
	r.loops[fi] = loops
	return loops, nil
}

// admitEdge is EB1 (SB4): the selected edge is a Line3 that matches one
// edge of the receiver's face record, and its two vertices differ in exactly
// one reference coordinate, the edge's axis. The match lifts each recorded
// edge's ends through the reference frame and placement and compares them
// with the selected edge's vertices within 1e-6, as matchCornerBudget does for
// a prism; it identifies the edge and admits no geometry.
func (r *brepEdgeRoute) admitEdge(ordinal int, e *Edge) (*brepEdgeBlend, error) {
	eb := &brepEdgeBlend{ordinal: ordinal, edge: e}
	switch e.curve.(type) {
	case Line3:
	case Circle3, Arc3:
		return nil, r.refuse(eb, "SB4", `the edge is a circle or an arc (a hole rim or a boss root), and a cone or torus band is not a face this record holds`)
	default:
		return nil, r.refuse(eb, "SB4", fmt.Sprintf(`the edge is a %T, not a straight line`, e.curve))
	}
	if e.start == nil || e.end == nil {
		return nil, r.refuse(eb, "SB4", `the edge has no vertices`)
	}
	refView := r.bp.refView()
	const tol = 1e-6
	found := -1
	for ei, pair := range r.topo.edges {
		if err := survey2d.WallBudgetStep(r.budget); err != nil {
			return nil, err
		}
		owner := r.topo.uses[pair[0]]
		if owner.Key.Circular {
			continue
		}
		p := refView.point(owner.DirFrom[0], owner.DirFrom[1], owner.DirFrom[2])
		q := refView.point(owner.DirTo[0], owner.DirTo[1], owner.DirTo[2])
		if !matchEndpoints(e.start.position, e.end.position, p, q, tol) {
			continue
		}
		if found >= 0 {
			return nil, r.refuse(eb, "SB6", `the edge matches more than one edge of the receiver's face record`)
		}
		found = ei
	}
	if found < 0 {
		return nil, r.refuse(eb, "SB6", `the edge matches no whole edge of the receiver's face record`)
	}
	eb.pair = r.topo.edges[found]
	owner := r.topo.uses[eb.pair[0]]
	a, b := owner.DirFrom, owner.DirTo
	axis := -1
	for i := range 3 {
		if a[i] == b[i] {
			continue
		}
		if axis >= 0 {
			return nil, r.refuse(eb, "SB4", `the edge is an oblique line, not one along a reference axis`)
		}
		axis = i
	}
	if axis < 0 {
		return nil, r.refuse(eb, "SB4", `the edge has no length`)
	}
	if a[axis] > b[axis] {
		a, b = b, a
	}
	eb.axis, eb.v = axis, [2][3]float64{a, b}
	eb.adj = [2]int{owner.Face, r.topo.uses[eb.pair[1]].Face}
	return eb, nil
}

// incidence lists, per reference-coordinate vertex, the topology edges that
// start or end there. A closed edge counts once, at its seam.
func (r *brepEdgeRoute) incidence() map[[3]float64][]int {
	out := map[[3]float64][]int{}
	for ei, pair := range r.topo.edges {
		owner := r.topo.uses[pair[0]]
		out[owner.DirFrom] = append(out[owner.DirFrom], ei)
		if owner.DirTo != owner.DirFrom {
			out[owner.DirTo] = append(out[owner.DirTo], ei)
		}
	}
	return out
}

// requireThreeEdges is EB2 (SB6): each vertex of the edge meets exactly
// three edges, and the edge is a whole side line, not one piece of a split
// one.
func (r *brepEdgeRoute) requireThreeEdges(eb *brepEdgeBlend, incident map[[3]float64][]int) error {
	for _, v := range eb.v {
		if n := len(incident[v]); n != 3 {
			return r.refuse(eb, "SB6", fmt.Sprintf(`its vertex at (%s) meets %d edges, not three`, r.render(v), n))
		}
	}
	for _, ui := range eb.pair {
		u := r.topo.uses[ui]
		f := r.bp.faces[u.Face]
		if (u.Part == brepSide0 && len(f.side0) > 0) || (u.Part == brepSide1 && len(f.side1) > 0) {
			return r.refuse(eb, "SB6", fmt.Sprintf(`the edge is one piece of brep face %s's split side line`, f.role))
		}
	}
	return nil
}

// brepRestated is one swept straight wall restated as a planar face
// (brep-modify §5.2), with its new frame's embed.
type brepRestated struct {
	face  brepFace
	embed brepEmbed
}

// restate is brep-modify §5.2 on brep face fi, a swept face: the planar
// rectangle its straight wall sweeps, recorded with its frame normal
// outward and the wall's own sweep (general-boolean §4.2), or
// brepgeom.ErrRestate naming why the wall is no such rectangle.
// A face restated earlier in the call is returned as it was.
func (r *brepEdgeRoute) restate(fi int, restated map[int]brepRestated) (brepRestated, error) {
	if rs, ok := restated[fi]; ok {
		return rs, nil
	}
	f := r.bp.faces[fi]
	rec, embed, err := brepgeom.Restate(brepgeom.FaceRecord{
		Frame: f.frame, Wall: f.wall, Z0: f.z0, Z1: f.z1, Z0Delta: f.z0Delta, Z1Delta: f.z1Delta,
		Side0: f.side0, Side1: f.side1, Delta: f.delta, Role: f.role,
	}, r.bp.faces[0].frame)
	if err != nil {
		return brepRestated{}, err
	}
	region := ProfileRecord{Outer: rec.Region.Outer}
	rs := brepRestated{embed: embed, face: brepFace{frame: rec.Frame, region: &region, outward: true,
		sweep: f.frame.N(), z0: rec.Z0, z1: rec.Z1, z0Delta: rec.Z0Delta, z1Delta: rec.Z1Delta, delta: rec.Delta, role: rec.Role}}
	restated[fi] = rs
	return rs, nil
}

// restateRims is the restatement pass's SB8 half: a swept face beside the
// edge that sweeps across the edge's axis holds the edge as a rim, and is
// restated as a plane.
func (r *brepEdgeRoute) restateRims(eb *brepEdgeBlend, restated map[int]brepRestated) error {
	for _, fi := range eb.adj {
		f := r.bp.faces[fi]
		if f.planar() || r.topo.embeds[fi].Axis[2] == eb.axis {
			continue
		}
		if _, err := r.restate(fi, restated); err != nil {
			return r.refuse(eb, "SB8", fmt.Sprintf(`brep face %s beside the edge holds it as a rim and is not a straight wall that reads as a plane: %v`, f.role, err))
		}
	}
	return nil
}

// withRestated is the route over the record with every restated face in
// place, at its own index, and the topology read again. Each restated face's
// four segments carry the coordinates the wall's rims and side lines did, so
// every edge pairs as before; a record that does not is ErrUnsupported.
func (r *brepEdgeRoute) withRestated(ctx context.Context, restated map[int]brepRestated) (*brepEdgeRoute, error) {
	bp := r.bp
	bp.faces = slices.Clone(r.bp.faces)
	for fi, rs := range restated {
		bp.faces[fi] = rs.face
	}
	topo, err := brepTopologyContext(ctx, bp)
	if err != nil {
		return nil, fmt.Errorf(`%w; the restated brep record (brep-modify §5.2); selector %s matched [%s]`,
			err, r.call.sel, selectedEdgesContext(r.call.edges))
	}
	return newBrepEdgeRoute(bp, topo, r.call, r.budget), nil
}

// findEndFaces is EB3 (SB7) and the restatement pass's end-face half: at
// each vertex the third face — the face both other edges there bound, beside
// neither adjacent face — is a planar face across the edge's axis, or a swept
// straight wall that restates as one (recorded in restated). G0 is the end
// face at the lower axis coordinate.
func (r *brepEdgeRoute) findEndFaces(eb *brepEdgeBlend, incident map[[3]float64][]int, restated map[int]brepRestated) error {
	self := r.topo.edgeOf[eb.pair[0]]
	for k, v := range eb.v {
		third := -1
		for _, ei := range incident[v] {
			if ei == self {
				continue
			}
			pair := r.topo.edges[ei]
			fa, fb := r.topo.uses[pair[0]].Face, r.topo.uses[pair[1]].Face
			aAdj, bAdj := slices.Contains(eb.adj[:], fa), slices.Contains(eb.adj[:], fb)
			g := -1
			switch {
			case aAdj && !bAdj:
				g = fb
			case bAdj && !aAdj:
				g = fa
			}
			if g < 0 || (third >= 0 && g != third) {
				return r.refuse(eb, "SB7", fmt.Sprintf(`no single third face meets the edge's vertex at (%s)`, r.render(v)))
			}
			third = g
		}
		g := r.bp.faces[third]
		normal := r.topo.embeds[third].Axis[2]
		var what string
		switch {
		case g.blend != "":
			what = fmt.Sprintf(`the %s face of an earlier call`, g.blend)
		case !g.planar() && r.topo.walls[third].IsLine():
			rs, err := r.restate(third, restated)
			if err != nil {
				what = fmt.Sprintf(`a swept straight wall that does not read as a plane (%v)`, err)
				break
			}
			normal = rs.embed.Axis[2]
		case !g.planar():
			what = `a curved face`
		}
		if what == "" && normal != eb.axis {
			what = `a plane along the edge's axis`
		}
		if what != "" {
			return r.refuse(eb, "SB7", fmt.Sprintf(`the face across the edge's axis at its vertex (%s), brep face %s, is %s, not a plane across the axis`,
				r.render(v), g.role, what))
		}
		eb.end[k].face = third
	}
	return nil
}

// classifySides is EB4 (SB8) over the restated record: each face beside the
// edge is (sw), a swept face along the edge's axis whose side line is the
// edge, or (pl), a planar face that holds the edge as one loop segment whose
// two neighbouring segments are straight lines at constant axis coordinate.
// A restated wall that held the edge as a rim or a side line is (pl).
func (r *brepEdgeRoute) classifySides(eb *brepEdgeBlend) error {
	for j, ui := range eb.pair {
		u := r.topo.uses[ui]
		f := r.bp.faces[u.Face]
		along := r.topo.embeds[u.Face].Axis[2] == eb.axis
		switch {
		case !f.planar() && along && (u.Part == brepSide0 || u.Part == brepSide1):
			eb.side[j] = brepSideSwept
		case f.planar() && !along && u.Part == brepLoopSeg:
			n := len(r.topo.planar[u.Face][u.Loop])
			for _, s := range []int{(u.Seg + n - 1) % n, (u.Seg + 1) % n} {
				nu := r.topo.uses[r.loopUse[[3]int{u.Face, u.Loop, s}]]
				if !nu.Walk.IsLine() || nu.From[eb.axis] != nu.To[eb.axis] {
					return r.refuse(eb, "SB8", fmt.Sprintf(`brep face %s beside the edge continues past the edge's vertex on a segment that is not a straight line across the edge's axis`, f.role))
				}
			}
			eb.side[j] = brepSidePlanar
		default:
			return r.refuse(eb, "SB8", fmt.Sprintf(`brep face %s beside the edge neither sweeps along the edge nor holds it as a loop segment`, f.role))
		}
	}
	return nil
}

// naturalRange reports whether a recorded segment is a line, arc or circle
// over its whole natural range, 0→1 or 1→0.
func naturalRange(seg CurveSegment) bool {
	var t0, t1 float64
	switch s := seg.(type) {
	case LineSeg:
		t0, t1 = s.TStart, s.TEnd
	case ArcSeg:
		t0, t1 = s.TStart, s.TEnd
	case CircleSeg:
		t0, t1 = s.TStart, s.TEnd
	default:
		return false
	}
	return (t0 == 0 && t1 == 1) || (t0 == 1 && t1 == 0)
}

// requireNaturalFaces is EB5 (SB8): every segment of the end faces and the
// adjacent faces runs over its natural range, and no loop of a planar one
// holds two consecutive segments on one carrier — each corner walk is one
// segment, and no two consecutive arcs share centre, radius and sense.
func (r *brepEdgeRoute) requireNaturalFaces(eb *brepEdgeBlend) error {
	faces := []int{eb.end[0].face, eb.end[1].face, eb.adj[0], eb.adj[1]}
	for _, fi := range faces {
		f := r.bp.faces[fi]
		if !f.planar() {
			if !naturalRange(f.wall) {
				return r.refuse(eb, "SB8", fmt.Sprintf(`brep face %s's wall runs over a narrowed range`, f.role))
			}
			continue
		}
		loops, err := r.cornerLoops(fi)
		if err != nil {
			return err
		}
		records := append([]LoopRecord{f.region.Outer}, f.region.Holes...)
		for li, loop := range records {
			for _, seg := range loop.Segments {
				if !naturalRange(seg) {
					return r.refuse(eb, "SB8", fmt.Sprintf(`brep face %s holds a segment over a narrowed range`, f.role))
				}
			}
			walks := loops[li].walks
			if len(walks) != len(loop.Segments) {
				return r.refuse(eb, "SB8", fmt.Sprintf(`brep face %s holds two consecutive segments on one line`, f.role))
			}
			n := len(walks)
			for i, w := range walks {
				next := walks[(i+1)%n]
				if n > 1 && w.IsCircular() && next.IsCircular() && !w.Closed && !next.Closed &&
					w.CU == next.CU && w.CV == next.CV && w.Radius == next.Radius && (w.Th1 > w.Th0) == (next.Th1 > next.Th0) {
					return r.refuse(eb, "SB8", fmt.Sprintf(`brep face %s holds two consecutive arcs on one circle`, f.role))
				}
			}
		}
	}
	return nil
}

// brepCornerAt finds the corner of a planar face's corner walk whose
// junction lies at reference coordinates v: the walk start that maps there
// exactly (brep-modify §5.3 step 1). It reports false when none or several
// do.
func brepCornerAt(loops []cornerLoop, e brepEmbed, level float64, v [3]float64) (int, int, bool) {
	li, ci, n := 0, 0, 0
	for l, loop := range loops {
		for c, w := range loop.walks {
			if w.Closed || e.Canon(w.StartU, w.StartV, level) != v {
				continue
			}
			li, ci = l, c
			n++
		}
	}
	return li, ci, n == 1
}

// locateCorners is EB6 (SB8): in each end face the corner at the edge's
// vertex is found (brepCornerAt), and its two walks are the traces of the
// two adjacent faces — the arriving and the leaving walk each pair, in the
// receiver's topology, with one adjacent face's rim (sw) or with its
// neighbouring segment of the edge (pl), one walk per face.
func (r *brepEdgeRoute) locateCorners(eb *brepEdgeBlend) error {
	for k, v := range eb.v {
		end := &eb.end[k]
		g := r.bp.faces[end.face]
		loops, err := r.cornerLoops(end.face)
		if err != nil {
			return err
		}
		li, ci, ok := brepCornerAt(loops, r.topo.embeds[end.face], g.z0, v)
		if !ok {
			return r.refuse(eb, "SB8", fmt.Sprintf(`brep face %s has no single corner at the edge's vertex (%s)`, g.role, r.render(v)))
		}
		end.loop, end.corner = li, ci
		walks := loops[li].walks
		n := len(walks)
		var faceOf [2]int
		for w, wi := range [2]int{(ci + n - 1) % n, ci} {
			ui := r.loopUse[[3]int{end.face, li, walks[wi].Segs[0]}]
			mate := r.topo.uses[r.partner(ui)]
			faceOf[w] = -1
			for j := range 2 {
				if mate.Face == eb.adj[j] && r.traces(eb, j, mate) {
					faceOf[w] = j
				}
			}
		}
		if faceOf[0] < 0 || faceOf[1] < 0 || faceOf[0] == faceOf[1] {
			return r.refuse(eb, "SB8", fmt.Sprintf(`brep face %s's two segments at the edge's vertex (%s) are not the traces of the two faces beside the edge`,
				g.role, r.render(v)))
		}
		end.arriving = faceOf[0]
	}
	return nil
}

// traces reports whether use mate, on adjacent face j, is the part of that
// face an end-face segment at the edge's vertex must pair with: a rim of a
// (sw) face, or the (pl) face's segment next to the edge.
func (r *brepEdgeRoute) traces(eb *brepEdgeBlend, j int, mate brepUse) bool {
	if eb.side[j] == brepSideSwept {
		return brepIsRim(mate.Part)
	}
	self := r.topo.uses[eb.pair[j]]
	n := len(r.topo.planar[self.Face][self.Loop])
	return mate.Part == brepLoopSeg && mate.Loop == self.Loop &&
		(mate.Seg == (self.Seg+n-1)%n || mate.Seg == (self.Seg+1)%n)
}

// footOf is the blend's foot on adjacent face j's carrier in one end face:
// fA where the arriving walk pairs with that face, fB otherwise, with the
// cutback the foot claims from the corner.
func footOf(end brepEdgeEnd, cb *cornerBlend, j int) (Point2, float64) {
	if end.arriving == j {
		return cb.fA, cb.cutbackA
	}
	return cb.fB, cb.cutbackB
}

// computeBlends is brep-modify §5.3 steps 1 and 2: the corner blend in G0
// (S4, S5), then the same blend recomputed in G1 (S4, S5), whose centre and
// two feet, mapped into G0's frame, must equal G0's bit for bit (SB9).
func (r *brepEdgeRoute) computeBlends(eb *brepEdgeBlend) error {
	for k := range eb.end {
		if err := survey2d.WallBudgetStep(r.budget); err != nil {
			return err
		}
		end := eb.end[k]
		loops, err := r.cornerLoops(end.face)
		if err != nil {
			return err
		}
		cb, err := r.call.blend.corner(loops[end.loop], end.corner)
		if err != nil {
			return fmt.Errorf(`%w; selector %s, %s at brep face %s's corner (%s)`, err, r.call.sel,
				selectedEdgeContext(eb.ordinal, eb.edge), r.bp.faces[end.face].role, r.render(eb.v[k]))
		}
		eb.blend[k] = cb
	}
	g0, g1 := eb.end[0].face, eb.end[1].face
	m, ok := brepgeom.NewMap2(r.topo.embeds[g1], r.topo.embeds[g0])
	if !ok {
		return r.refuse(eb, "SB9", `the two end faces do not lie across one axis`)
	}
	for j := range 2 {
		f0, _ := footOf(eb.end[0], eb.blend[0], j)
		f1, _ := footOf(eb.end[1], eb.blend[1], j)
		u, v := m.Point(f1.U, f1.V)
		if u != f0.U || v != f0.V {
			return r.refuse(eb, "SB9", fmt.Sprintf(`the blend's foot on brep face %s computed in the two end faces disagrees: (%s, %s) against (%s, %s) in brep face %s's frame`,
				r.bp.faces[eb.adj[j]].role, renderCoord(f0.U), renderCoord(f0.V), renderCoord(u), renderCoord(v), r.bp.faces[g0].role))
		}
	}
	mapped, err := brepgeom.MapSegment(m, eb.blend[1].connector)
	if err != nil {
		return r.refuse(eb, "SB9", err.Error())
	}
	c0, arc0 := eb.blend[0].connector.(ArcSeg)
	c1, arc1 := mapped.(ArcSeg)
	if arc0 != arc1 || (arc0 && c0.Center != c1.Center) {
		return r.refuse(eb, "SB9", `the blend's centre computed in the two end faces disagrees`)
	}
	return nil
}

// brepFaceEdit collects one face's rewrite across every admitted edge: for
// a planar face, per loop, the blend or trim at each corner (rewriteLoop's
// input, whose cutbacks are the S6 claims); for a swept face, the trimmed
// wall ends and their claims.
type brepFaceEdit struct {
	blendAt              []map[int]*cornerBlend
	start, end           *Point2
	claimStart, claimEnd float64
}

// rewrite is brep-modify §5.3 steps 3 to 6 and the assembly of step 7: every
// end face takes its corner blends, every (pl) face its edge segment moved
// onto the feet, every (sw) face its wall end trimmed to its foot; one swept
// blend face is appended per edge; then every rewritten planar face runs the
// modify §5 audit and every trimmed wall the S6 claim test.
func (r *brepEdgeRoute) rewrite(blends []*brepEdgeBlend) (brepPayload, error) {
	edits := map[int]*brepFaceEdit{}
	editOf := func(fi int) *brepFaceEdit {
		if e, ok := edits[fi]; ok {
			return e
		}
		e := &brepFaceEdit{}
		if loops, ok := r.loops[fi]; ok {
			e.blendAt = make([]map[int]*cornerBlend, len(loops))
			for i := range e.blendAt {
				e.blendAt[i] = map[int]*cornerBlend{}
			}
		}
		edits[fi] = e
		return e
	}
	mark := func(eb *brepEdgeBlend, fi, li, ci int, cb *cornerBlend) error {
		at := editOf(fi).blendAt[li]
		if at[ci] != nil {
			return r.refuse(eb, "SB5", fmt.Sprintf(`brep face %s's corner there is claimed by another selected edge`, r.bp.faces[fi].role))
		}
		at[ci] = cb
		return nil
	}

	var blendFaces []brepFace
	for _, eb := range blends {
		g0 := eb.end[0].face
		e0 := r.topo.embeds[g0]
		level0 := r.bp.faces[g0].z0
		for k, end := range eb.end {
			if err := mark(eb, end.face, end.loop, end.corner, eb.blend[k]); err != nil {
				return brepPayload{}, err
			}
		}
		for j, fi := range eb.adj {
			foot, _ := footOf(eb.end[0], eb.blend[0], j)
			ref := e0.Canon(foot.U, foot.V, level0)
			embed := r.topo.embeds[fi]
			u := r.topo.uses[eb.pair[j]]
			if eb.side[j] == brepSideSwept {
				local := embed.Local(ref)
				p := Point2{U: local[0], V: local[1]}
				_, claim := footOf(eb.end[0], eb.blend[0], j)
				e := editOf(fi)
				if u.Part == brepSide1 {
					if e.end != nil {
						return brepPayload{}, r.refuse(eb, "SB5", `the wall's end is claimed by another selected edge`)
					}
					e.end, e.claimEnd = &p, claim
					continue
				}
				if e.start != nil {
					return brepPayload{}, r.refuse(eb, "SB5", `the wall's start is claimed by another selected edge`)
				}
				e.start, e.claimStart = &p, claim
				continue
			}
			// (pl): the edge's segment moves onto the foot at both levels, and
			// each neighbour ends there instead of at the vertex. The trim is a
			// connector-free blend at each end of the segment, whose cutback is
			// the neighbour's claim: the end face's own cutback on this carrier.
			loops, err := r.cornerLoops(fi)
			if err != nil {
				return brepPayload{}, err
			}
			walks := loops[u.Loop].walks
			n := len(walks)
			seg := walks[u.Seg]
			startRef := embed.Canon(seg.StartU, seg.StartV, r.bp.faces[fi].z0)
			for c, at := range [2][3]float64{startRef, embed.Canon(seg.EndU, seg.EndV, r.bp.faces[fi].z0)} {
				k := 0
				if at == eb.v[1] {
					k = 1
				}
				moved := ref
				moved[eb.axis] = eb.v[k][eb.axis]
				local := embed.Local(moved)
				p := Point2{U: local[0], V: local[1]}
				_, claim := footOf(eb.end[k], eb.blend[k], j)
				trim := &cornerBlend{fA: p, fB: p}
				ci := u.Seg
				if c == 0 {
					trim.cutbackA = claim
				} else {
					trim.cutbackB = claim
					ci = (u.Seg + 1) % n
				}
				if err := mark(eb, fi, u.Loop, ci, trim); err != nil {
					return brepPayload{}, err
				}
			}
		}
		blendFaces = append(blendFaces, r.blendFace(eb))
	}

	faces := slices.Clone(r.bp.faces)
	order := make([]int, 0, len(edits))
	for fi := range edits {
		order = append(order, fi)
	}
	// Stage 5 audits every rewritten planar face before any trimmed wall.
	slices.SortFunc(order, func(a, b int) int {
		pa, pb := r.bp.faces[a].planar(), r.bp.faces[b].planar()
		if pa != pb {
			if pa {
				return -1
			}
			return 1
		}
		return a - b
	})
	for _, fi := range order {
		edit := edits[fi]
		f := r.bp.faces[fi]
		if f.planar() {
			loops := r.loops[fi]
			profile, _, err := rewriteProfileBudget(r.budget, *f.region, loops, edit.blendAt)
			if err != nil {
				return brepPayload{}, err
			}
			// Modify §5's audit of the face's final record: S8, S6, S7, S9.
			if err := auditRewriteBudget(r.budget, *f.region, profile, loops, edit.blendAt); err != nil {
				return brepPayload{}, r.auditFailure(f, err)
			}
			faces[fi].region = &profile
			continue
		}
		w := r.topo.walls[fi]
		if err := auditTrimmedWall(w, edit.claimStart, edit.claimEnd); err != nil {
			return brepPayload{}, r.auditFailure(f, err)
		}
		sU, sV, eU, eV := w.StartU, w.StartV, w.EndU, w.EndV
		if edit.start != nil {
			sU, sV = edit.start.U, edit.start.V
		}
		if edit.end != nil {
			eU, eV = edit.end.U, edit.end.V
		}
		faces[fi].wall = walkSegment(survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}}, sU, sV, eU, eV)
	}
	out := brepPayload{faces: append(faces, blendFaces...), xform: r.bp.xform}
	out.assignRoles()
	return out, nil
}

// auditFailure wraps an audit refusal of one rewritten face with the face's
// role and the selection, passing a context error through unwrapped.
func (r *brepEdgeRoute) auditFailure(f brepFace, err error) error {
	if errors.Is(err, context.Canceled) {
		return context.Canceled
	}
	if errors.Is(err, context.DeadlineExceeded) {
		return context.DeadlineExceeded
	}
	return fmt.Errorf(`%w; in brep face %s; selector %s matched [%s]`,
		renderAuditCoordinates(err), f.role, r.call.sel, selectedEdgesContext(r.call.edges))
}

// blendFace is brep-modify §5.3 step 5: one swept face in G0's frame whose
// wall is G0's connector, swept between the two end faces' levels, with the
// solid's material on the wall's left.
//
// The walk sense reads G0's own record, not Edge.IsConvex: a line two planar
// faces of different sweeps share reads its convexity from a loop's role (an
// outer loop's edge reads convex), which a concave edge need not match
// (general-boolean §4.2). G0's connector already walks with G0's region on its left. Near the
// corner the solid between the two end faces is bounded by the two adjacent
// faces alone, and its section there is G0's region where G1 lies on G0's
// inner side (against G0's outward normal), and the rest of the plane where
// G1 lies on G0's outer side. So the connector keeps its sense in the first
// case and is reversed in the second.
func (r *brepEdgeRoute) blendFace(eb *brepEdgeBlend) brepFace {
	g0 := r.bp.faces[eb.end[0].face]
	embed := r.topo.embeds[eb.end[0].face]
	z0, z0Delta := g0.z0, g0.z0Delta
	z1, z1Delta := embed.Local(eb.v[1])[2], r.bp.faces[eb.end[1].face].z0Delta
	wall := eb.blend[0].connector
	if inner := (z1 > z0) != g0.outward; !inner {
		wall = reverseSegment(wall)
	}
	if z1 < z0 {
		z0, z1 = z1, z0
		z0Delta, z1Delta = z1Delta, z0Delta
	}
	return brepFace{frame: g0.frame, wall: wall, z0: z0, z1: z1,
		z0Delta: z0Delta, z1Delta: z1Delta, blend: r.call.blend.kind}
}

// reverseSegment walks a natural-range line or arc the other way: the same
// record over the reversed range.
func reverseSegment(seg CurveSegment) CurveSegment {
	switch s := seg.(type) {
	case LineSeg:
		s.TStart, s.TEnd = s.TEnd, s.TStart
		return s
	case ArcSeg:
		s.TStart, s.TEnd = s.TEnd, s.TStart
		return s
	default:
		return seg
	}
}
