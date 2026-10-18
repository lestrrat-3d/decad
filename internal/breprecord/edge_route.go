package breprecord

import (
	"context"
	"errors"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// EdgeSide identifies the swept or planar face beside a route E edge.
type EdgeSide int

const (
	SideSwept  EdgeSide = iota // (sw): a swept face along the edge's axis whose side line is the edge
	SidePlanar                 // (pl): a planar face that holds the edge as one loop segment
)

// EdgeEnd is one end face's corner at an admitted edge's vertex: the
// loop and corner of the face's corner walk whose junction is the vertex
// (brepgeom.BrepCornerAt), and which of the edge's two adjacent faces the arriving
// walk there pairs with.
type EdgeEnd struct {
	Face     int
	Loop     int
	Corner   int
	Arriving int
}

// EdgeBlend is one selected edge through route E: its use pair in the
// receiver's topology, its axis, its two vertices in reference coordinates
// (V0 at the lower axis coordinate), its two adjacent faces with their
// classes, its two end faces, and the blend computed in each end face.
type EdgeBlend struct {
	Ordinal  int
	Context  string
	Pair     [2]int
	Axis     int
	V        [2][3]float64
	Adj      [2]int
	Side     [2]EdgeSide
	End      [2]EdgeEnd
	Blend    [2]*offset2d.Blend
	Terminal [2]bool
}

// EdgeRoute holds one route E call's readings of the receiver: its
// record, topology, and each planar face's corner walk, read once.
type EdgeRoute struct {
	BP        Payload
	Topo      *Topology
	Op        string
	Selector  string
	Selected  string
	BlendKind string
	Budget    *proofbound.WorkBudget
	Loops     map[int][]survey2d.CornerLoop
	// loopUse maps a planar face's (face, loop, segment) to its use index.
	LoopUse   map[[3]int]int
	Terminals map[[3]float64]bool
}

// MaterialWedgeMismatchError marks the route E corner-model refusal that a
// selected partial planar loop may be able to build through route L instead.
type MaterialWedgeMismatchError struct{ error }

func (e *MaterialWedgeMismatchError) Unwrap() error { return e.error }

// NewEdgeRoute reads one record for route E: its topology's loop-segment
// uses are indexed once, and corner walks are read on demand.
func NewEdgeRoute(bp Payload, btopo *Topology, op, selector, selected, kind string, budget *proofbound.WorkBudget) *EdgeRoute {
	r := &EdgeRoute{BP: bp, Topo: btopo, Op: op, Selector: selector, Selected: selected, BlendKind: kind, Budget: budget,
		Loops: map[int][]survey2d.CornerLoop{}, LoopUse: map[[3]int]int{}}
	for ui, u := range btopo.Uses {
		if u.Part == brepgeom.LoopSeg {
			r.LoopUse[[3]int{u.Face, u.Loop, u.Seg}] = ui
		}
	}
	return r
}

// RequireBlendMaterialWedge checks that the endpoint corners agree with the
// interior material wedge before rewriting a selected edge.
func (r *EdgeRoute) RequireBlendMaterialWedge(eb *EdgeBlend) error {
	materialConvex, decided := brepgeom.MaterialWedgeConvex(eb.Pair, r.Topo.Uses,
		r.Topo.Embeds, r.Topo.Walls)
	if !decided {
		return r.Refuse(eb, "SB8", `the interior material wedge has no decided turn`)
	}
	for k, end := range eb.End {
		loops, err := r.CornerLoops(end.Face)
		if err != nil {
			return err
		}
		walks := loops[end.Loop].Walks
		previous := walks[(end.Corner+len(walks)-1)%len(walks)]
		current := walks[end.Corner]
		cornerConvex := previous.TanOutU*current.TanInV-previous.TanOutV*current.TanInU > 0
		face := r.BP.Faces[end.Face]
		other := r.Topo.Embeds[end.Face].Local(eb.V[1-k])
		inner := (other[2] > face.Z0) != face.Outward
		if materialConvex != (cornerConvex == inner) {
			return &MaterialWedgeMismatchError{r.Refuse(eb, "SB8",
				`the endpoint face corner differs from the interior material wedge`)}
		}
	}
	return nil
}

// Refuse names the selected edge and Table SB gate that rejected it.
func (r *EdgeRoute) Refuse(eb *EdgeBlend, row, reason string) error {
	return fmt.Errorf(`%w: this evaluator %s a brep edge through route E only where Table EB admits it; %s (brep-modify %s); selector %s, %s`,
		decaderr.ErrUnsupported, r.Op, reason, row, r.Selector, eb.Context)
}

// Render names a reference-coordinate point by its placed position.
func (r *EdgeRoute) Render(c [3]float64) string {
	return selectorquery.RenderVec(r.BP.RefView().Point(c[0], c[1], c[2]))
}

// partner is the other use of use ui's edge, -1 when ui is a route L band's
// boundary, which pairs with no face of the record.
func (r *EdgeRoute) partner(ui int) int {
	if r.Topo.EdgeOf[ui] < 0 {
		return -1
	}
	pair := r.Topo.Edges[r.Topo.EdgeOf[ui]]
	if pair[0] == ui {
		return pair[1]
	}
	return pair[0]
}

// CornerLoops reads a planar face's region into its corner walks once.
func (r *EdgeRoute) CornerLoops(fi int) ([]survey2d.CornerLoop, error) {
	if loops, ok := r.Loops[fi]; ok {
		return loops, nil
	}
	loops, err := offset2d.ProfileCornerLoopsBudget(r.Budget, *r.BP.Faces[fi].Region)
	if err != nil {
		return nil, err
	}
	r.Loops[fi] = loops
	return loops, nil
}

// Incidence lists topology edges at each reference-coordinate vertex.
func (r *EdgeRoute) Incidence() map[[3]float64][]int {
	out := map[[3]float64][]int{}
	for ei, pair := range r.Topo.Edges {
		owner := r.Topo.Uses[pair[0]]
		out[owner.DirFrom] = append(out[owner.DirFrom], ei)
		if owner.DirTo != owner.DirFrom {
			out[owner.DirTo] = append(out[owner.DirTo], ei)
		}
	}
	return out
}

// RequireThreeEdges is EB2 (SB6): each vertex of the edge meets exactly
// three edges, and the edge is a whole side line, not one piece of a split
// one.
func (r *EdgeRoute) RequireThreeEdges(eb *EdgeBlend, incident map[[3]float64][]int) error {
	for _, v := range eb.V {
		if n := len(incident[v]); n != 3 {
			return r.Refuse(eb, "SB6", fmt.Sprintf(`its vertex at (%s) meets %d edges, not three`, r.Render(v), n))
		}
	}
	for _, ui := range eb.Pair {
		u := r.Topo.Uses[ui]
		f := r.BP.Faces[u.Face]
		if (u.Part == brepgeom.Side0 && len(f.Side0) > 0) || (u.Part == brepgeom.Side1 && len(f.Side1) > 0) {
			return r.Refuse(eb, "SB6", fmt.Sprintf(`the edge is one piece of brep face %s's split side line`, f.Role))
		}
	}
	return nil
}

// EdgeRestated is one swept straight wall restated as a planar face
// (brep-modify §5.2), with its new frame's embed.
type EdgeRestated struct {
	Face  Face
	Embed brepgeom.Embed
}

// restate is brep-modify §5.2 on brep face fi, a swept face: the planar
// rectangle its straight wall sweeps, recorded with its frame normal
// outward and the wall's own sweep (general-boolean §4.2), or
// brepgeom.ErrRestate naming why the wall is no such rectangle.
// A face restated earlier in the call is returned as it was.
func (r *EdgeRoute) restate(fi int, restated map[int]EdgeRestated) (EdgeRestated, error) {
	if rs, ok := restated[fi]; ok {
		return rs, nil
	}
	f := r.BP.Faces[fi]
	rec, embed, err := brepgeom.Restate(brepgeom.FaceRecord{
		Frame: f.Frame, Wall: f.Wall, Z0: f.Z0, Z1: f.Z1, Z0Delta: f.Z0Delta, Z1Delta: f.Z1Delta,
		Side0: f.Side0, Side1: f.Side1, Delta: f.Delta, Role: f.Role,
	}, r.BP.Faces[0].Frame)
	if err != nil {
		return EdgeRestated{}, err
	}
	region := momentinput.Profile{Outer: rec.Region.Outer}
	rs := EdgeRestated{Embed: embed, Face: Face{Frame: rec.Frame, Region: &region, Outward: true,
		Sweep: f.Frame.N(), Z0: rec.Z0, Z1: rec.Z1, Z0Delta: rec.Z0Delta, Z1Delta: rec.Z1Delta, Delta: rec.Delta, Role: rec.Role}}
	restated[fi] = rs
	return rs, nil
}

// RestateRims is the restatement pass's SB8 half: a swept face beside the
// edge that sweeps across the edge's axis holds the edge as a rim, and is
// restated as a plane.
func (r *EdgeRoute) RestateRims(eb *EdgeBlend, restated map[int]EdgeRestated) error {
	for _, fi := range eb.Adj {
		f := r.BP.Faces[fi]
		if f.Planar() || r.Topo.Embeds[fi].Axis[2] == eb.Axis {
			continue
		}
		if _, err := r.restate(fi, restated); err != nil {
			return r.Refuse(eb, "SB8", fmt.Sprintf(`brep face %s beside the edge holds it as a rim and is not a straight wall that reads as a plane: %v`, f.Role, err))
		}
	}
	return nil
}

// WithRestated is the route over the record with every restated face in
// place, at its own index, and the topology read again. Each restated face's
// four segments carry the coordinates the wall's rims and side lines did, so
// every edge pairs as before; a record that does not is decaderr.ErrUnsupported.
func (r *EdgeRoute) WithRestated(ctx context.Context, restated map[int]EdgeRestated) (*EdgeRoute, error) {
	bp := r.BP
	bp.Faces = slices.Clone(r.BP.Faces)
	for fi, rs := range restated {
		bp.Faces[fi] = rs.Face
	}
	btopo, err := TopologyContext(ctx, bp)
	if err != nil {
		return nil, fmt.Errorf(`%w; the restated brep record (brep-modify §5.2); selector %s matched [%s]`,
			err, r.Selector, r.Selected)
	}
	next := NewEdgeRoute(bp, btopo, r.Op, r.Selector, r.Selected, r.BlendKind, r.Budget)
	if r.Terminals != nil {
		next.Terminals = map[[3]float64]bool{}
		oldView, newView := r.BP.RefView(), bp.RefView()
		for old, on := range r.Terminals {
			if !on {
				continue
			}
			point := oldView.Point(old[0], old[1], old[2])
			for _, use := range btopo.Uses {
				for _, candidate := range [2][3]float64{use.DirFrom, use.DirTo} {
					if point.Sub(newView.Point(candidate[0], candidate[1], candidate[2])).Len() <= 1e-6 {
						next.Terminals[candidate] = true
					}
				}
			}
		}
	}
	return next, nil
}

// FindEndFaces is EB3 (SB7) and the restatement pass's end-face half: at
// each vertex the third face — the face both other edges there bound, beside
// neither adjacent face — is a planar face across the edge's axis, or a swept
// straight wall that restates as one (recorded in restated). G0 is the end
// face at the lower axis coordinate.
func (r *EdgeRoute) FindEndFaces(eb *EdgeBlend, incident map[[3]float64][]int, restated map[int]EdgeRestated) error {
	self := r.Topo.EdgeOf[eb.Pair[0]]
	for k, v := range eb.V {
		if !eb.Terminal[k] {
			continue
		}
		third := -1
		for _, ei := range incident[v] {
			if ei == self {
				continue
			}
			pair := r.Topo.Edges[ei]
			fa, fb := r.Topo.Uses[pair[0]].Face, r.Topo.Uses[pair[1]].Face
			aAdj, bAdj := slices.Contains(eb.Adj[:], fa), slices.Contains(eb.Adj[:], fb)
			g := -1
			switch {
			case aAdj && !bAdj:
				g = fb
			case bAdj && !aAdj:
				g = fa
			}
			if g < 0 || (third >= 0 && g != third) {
				return r.Refuse(eb, "SB7", fmt.Sprintf(`no single third face meets the edge's vertex at (%s)`, r.Render(v)))
			}
			third = g
		}
		g := r.BP.Faces[third]
		normal := r.Topo.Embeds[third].Axis[2]
		var what string
		switch {
		case g.Blend != "":
			what = fmt.Sprintf(`the %s face of an earlier call`, g.Blend)
		case !g.Planar() && r.Topo.Walls[third].IsLine():
			rs, err := r.restate(third, restated)
			if err != nil {
				what = fmt.Sprintf(`a swept straight wall that does not read as a plane (%v)`, err)
				break
			}
			normal = rs.Embed.Axis[2]
		case !g.Planar():
			what = `a curved face`
		}
		if what == "" && normal != eb.Axis {
			what = `a plane along the edge's axis`
		}
		if what != "" {
			return r.Refuse(eb, "SB7", fmt.Sprintf(`the face across the edge's axis at its vertex (%s), brep face %s, is %s, not a plane across the axis`,
				r.Render(v), g.Role, what))
		}
		eb.End[k].Face = third
	}
	return nil
}

// ClassifySides is EB4 (SB8) over the restated record: each face beside the
// edge is (sw), a swept face along the edge's axis whose side line is the
// edge, or (pl), a planar face that holds the edge as one loop segment whose
// two neighbouring segments are straight lines at constant axis coordinate.
// A restated wall that held the edge as a rim or a side line is (pl).
func (r *EdgeRoute) ClassifySides(eb *EdgeBlend) error {
	for j, ui := range eb.Pair {
		u := r.Topo.Uses[ui]
		f := r.BP.Faces[u.Face]
		along := r.Topo.Embeds[u.Face].Axis[2] == eb.Axis
		switch {
		case !f.Planar() && along && (u.Part == brepgeom.Side0 || u.Part == brepgeom.Side1):
			eb.Side[j] = SideSwept
		case f.Planar() && !along && u.Part == brepgeom.LoopSeg:
			n := len(r.Topo.Planar[u.Face][u.Loop])
			for endIndex, s := range []int{(u.Seg + n - 1) % n, (u.Seg + 1) % n} {
				vertex := u.From
				if endIndex == 1 {
					vertex = u.To
				}
				if r.Terminals != nil && !r.Terminals[vertex] {
					continue
				}
				nu := r.Topo.Uses[r.LoopUse[[3]int{u.Face, u.Loop, s}]]
				if !nu.Walk.IsLine() || nu.From[eb.Axis] != nu.To[eb.Axis] {
					return r.Refuse(eb, "SB8", fmt.Sprintf(`brep face %s beside the edge continues past the edge's vertex on a segment that is not a straight line across the edge's axis`, f.Role))
				}
			}
			eb.Side[j] = SidePlanar
		default:
			return r.Refuse(eb, "SB8", fmt.Sprintf(`brep face %s beside the edge neither sweeps along the edge nor holds it as a loop segment`, f.Role))
		}
	}
	return nil
}

// RequireNaturalFaces is EB5 (SB8): every segment of the end faces and the
// adjacent faces runs over its natural range, and no loop of a planar one
// holds two consecutive segments on one carrier — each corner walk is one
// segment, and no two consecutive arcs share centre, radius and sense.
func (r *EdgeRoute) RequireNaturalFaces(eb *EdgeBlend) error {
	faces := []int{eb.Adj[0], eb.Adj[1]}
	for k, active := range eb.Terminal {
		if active {
			faces = append(faces, eb.End[k].Face)
		}
	}
	for _, fi := range faces {
		f := r.BP.Faces[fi]
		if !f.Planar() {
			if !brepgeom.NaturalRange(f.Wall) {
				return r.Refuse(eb, "SB8", fmt.Sprintf(`brep face %s's wall runs over a narrowed range`, f.Role))
			}
			continue
		}
		loops, err := r.CornerLoops(fi)
		if err != nil {
			return err
		}
		records := append([]sectionrecord.LoopRecord{f.Region.Outer}, f.Region.Holes...)
		for li, loop := range records {
			for _, seg := range loop.Segments {
				if !brepgeom.NaturalRange(seg) {
					return r.Refuse(eb, "SB8", fmt.Sprintf(`brep face %s holds a segment over a narrowed range`, f.Role))
				}
			}
			walks := loops[li].Walks
			if len(walks) != len(loop.Segments) {
				return r.Refuse(eb, "SB8", fmt.Sprintf(`brep face %s holds two consecutive segments on one line`, f.Role))
			}
			n := len(walks)
			for i, w := range walks {
				next := walks[(i+1)%n]
				if n > 1 && w.IsCircular() && next.IsCircular() && !w.Closed && !next.Closed &&
					w.CU == next.CU && w.CV == next.CV && w.Radius == next.Radius && (w.Th1 > w.Th0) == (next.Th1 > next.Th0) {
					return r.Refuse(eb, "SB8", fmt.Sprintf(`brep face %s holds two consecutive arcs on one circle`, f.Role))
				}
			}
		}
	}
	return nil
}

// LocateCorners is EB6 (SB8): in each end face the corner at the edge's
// vertex is found (brepgeom.BrepCornerAt), and its two walks are the traces of the
// two adjacent faces — the arriving and the leaving walk each pair, in the
// receiver's topology, with one adjacent face's rim (sw) or with its
// neighbouring segment of the edge (pl), one walk per face.
func (r *EdgeRoute) LocateCorners(eb *EdgeBlend) error {
	for k, v := range eb.V {
		if !eb.Terminal[k] {
			continue
		}
		end := &eb.End[k]
		g := r.BP.Faces[end.Face]
		loops, err := r.CornerLoops(end.Face)
		if err != nil {
			return err
		}
		li, ci, ok := brepgeom.BrepCornerAt(loops, r.Topo.Embeds[end.Face], g.Z0, v)
		if !ok {
			return r.Refuse(eb, "SB8", fmt.Sprintf(`brep face %s has no single corner at the edge's vertex (%s)`, g.Role, r.Render(v)))
		}
		end.Loop, end.Corner = li, ci
		walks := loops[li].Walks
		n := len(walks)
		var faceOf [2]int
		for w, wi := range [2]int{(ci + n - 1) % n, ci} {
			ui := r.LoopUse[[3]int{end.Face, li, walks[wi].Segs[0]}]
			faceOf[w] = -1
			mateUse := r.partner(ui)
			if mateUse < 0 {
				continue
			}
			mate := r.Topo.Uses[mateUse]
			for j := range 2 {
				if mate.Face == eb.Adj[j] && r.traces(eb, j, mate) {
					faceOf[w] = j
				}
			}
		}
		if faceOf[0] < 0 || faceOf[1] < 0 || faceOf[0] == faceOf[1] {
			return r.Refuse(eb, "SB8", fmt.Sprintf(`brep face %s's two segments at the edge's vertex (%s) are not the traces of the two faces beside the edge`,
				g.Role, r.Render(v)))
		}
		end.Arriving = faceOf[0]
	}
	return nil
}

// traces reports whether use mate, on adjacent face j, is the part of that
// face an end-face segment at the edge's vertex must pair with: a rim of a
// (sw) face, or the (pl) face's segment next to the edge.
func (r *EdgeRoute) traces(eb *EdgeBlend, j int, mate brepgeom.Use) bool {
	if eb.Side[j] == SideSwept {
		return brepgeom.IsRim(mate.Part)
	}
	self := r.Topo.Uses[eb.Pair[j]]
	n := len(r.Topo.Planar[self.Face][self.Loop])
	return mate.Part == brepgeom.LoopSeg && mate.Loop == self.Loop &&
		(mate.Seg == (self.Seg+n-1)%n || mate.Seg == (self.Seg+1)%n)
}

// FootOf is the blend's foot on adjacent face j's carrier in one end face:
// fA where the arriving walk pairs with that face, fB otherwise, with the
// cutback the foot claims from the corner.
func FootOf(end EdgeEnd, cb *offset2d.Blend, j int) (sectionrecord.Point2, float64) {
	if end.Arriving == j {
		return cb.FA, cb.CutbackA
	}
	return cb.FB, cb.CutbackB
}

type edgeFaceEdit struct {
	blendAt              []map[int]*offset2d.Blend
	start, End           *sectionrecord.Point2
	claimStart, claimEnd float64
}

// Rewrite is brep-modify §5.3 steps 3 to 6 and the assembly of step 7: every
// end face takes its corner blends, every (pl) face its edge segment moved
// onto the feet, every (sw) face its wall end trimmed to its foot; one swept
// blend face is appended per edge; then every rewritten planar face runs the
// modify §5 audit and every trimmed wall the S6 claim test.
func (r *EdgeRoute) Rewrite(blends []*EdgeBlend) (Payload, error) {
	edits := map[int]*edgeFaceEdit{}
	editOf := func(fi int) *edgeFaceEdit {
		if e, ok := edits[fi]; ok {
			return e
		}
		e := &edgeFaceEdit{}
		if loops, ok := r.Loops[fi]; ok {
			e.blendAt = make([]map[int]*offset2d.Blend, len(loops))
			for i := range e.blendAt {
				e.blendAt[i] = map[int]*offset2d.Blend{}
			}
		}
		edits[fi] = e
		return e
	}
	mark := func(eb *EdgeBlend, fi, li, ci int, cb *offset2d.Blend) error {
		at := editOf(fi).blendAt[li]
		if at[ci] != nil {
			return r.Refuse(eb, "SB5", fmt.Sprintf(`brep face %s's corner there is claimed by another selected edge`, r.BP.Faces[fi].Role))
		}
		at[ci] = cb
		return nil
	}

	var blendFaces []Face
	for _, eb := range blends {
		g0 := eb.End[0].Face
		e0 := r.Topo.Embeds[g0]
		level0 := r.BP.Faces[g0].Z0
		for k, end := range eb.End {
			if err := mark(eb, end.Face, end.Loop, end.Corner, eb.Blend[k]); err != nil {
				return Payload{}, err
			}
		}
		for j, fi := range eb.Adj {
			foot, _ := FootOf(eb.End[0], eb.Blend[0], j)
			ref := e0.Canon(foot.U, foot.V, level0)
			embed := r.Topo.Embeds[fi]
			u := r.Topo.Uses[eb.Pair[j]]
			if eb.Side[j] == SideSwept {
				local := embed.Local(ref)
				p := sectionrecord.Point2{U: local[0], V: local[1]}
				_, claim := FootOf(eb.End[0], eb.Blend[0], j)
				e := editOf(fi)
				if u.Part == brepgeom.Side1 {
					if e.End != nil {
						return Payload{}, r.Refuse(eb, "SB5", `the wall's end is claimed by another selected edge`)
					}
					e.End, e.claimEnd = &p, claim
					continue
				}
				if e.start != nil {
					return Payload{}, r.Refuse(eb, "SB5", `the wall's start is claimed by another selected edge`)
				}
				e.start, e.claimStart = &p, claim
				continue
			}
			// (pl): the edge's segment moves onto the foot at both levels, and
			// each neighbour ends there instead of at the vertex. The trim is a
			// connector-free blend at each end of the segment, whose cutback is
			// the neighbour's claim: the end face's own cutback on this carrier.
			loops, err := r.CornerLoops(fi)
			if err != nil {
				return Payload{}, err
			}
			walks := loops[u.Loop].Walks
			n := len(walks)
			seg := walks[u.Seg]
			startRef := embed.Canon(seg.StartU, seg.StartV, r.BP.Faces[fi].Z0)
			for c, at := range [2][3]float64{startRef, embed.Canon(seg.EndU, seg.EndV, r.BP.Faces[fi].Z0)} {
				k := 0
				if at == eb.V[1] {
					k = 1
				}
				moved := ref
				moved[eb.Axis] = eb.V[k][eb.Axis]
				local := embed.Local(moved)
				p := sectionrecord.Point2{U: local[0], V: local[1]}
				_, claim := FootOf(eb.End[k], eb.Blend[k], j)
				trim := &offset2d.Blend{FA: p, FB: p}
				ci := u.Seg
				if c == 0 {
					trim.CutbackA = claim
				} else {
					trim.CutbackB = claim
					ci = (u.Seg + 1) % n
				}
				if err := mark(eb, fi, u.Loop, ci, trim); err != nil {
					return Payload{}, err
				}
			}
		}
		blendFaces = append(blendFaces, r.blendFace(eb))
	}

	faces := slices.Clone(r.BP.Faces)
	order := make([]int, 0, len(edits))
	for fi := range edits {
		order = append(order, fi)
	}
	// Stage 5 audits every rewritten planar face before any trimmed wall.
	slices.SortFunc(order, func(a, b int) int {
		pa, pb := r.BP.Faces[a].Planar(), r.BP.Faces[b].Planar()
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
		f := r.BP.Faces[fi]
		if f.Planar() {
			loops := r.Loops[fi]
			profile, _, err := offset2d.RewriteProfileBudget(r.Budget, *f.Region, loops, edit.blendAt)
			if err != nil {
				return Payload{}, err
			}
			// Modify §5's audit of the face's final record: S8, S6, S7, S9.
			if err := offset2d.AuditRewriteBudget(r.Budget, *f.Region, profile, loops, edit.blendAt); err != nil {
				return Payload{}, r.auditFailure(f, err)
			}
			faces[fi].Region = &profile
			continue
		}
		w := r.Topo.Walls[fi]
		if err := sectionaudit.TrimmedWall(w, edit.claimStart, edit.claimEnd); err != nil {
			return Payload{}, r.auditFailure(f, err)
		}
		sU, sV, eU, eV := w.StartU, w.StartV, w.EndU, w.EndV
		if edit.start != nil {
			sU, sV = edit.start.U, edit.start.V
		}
		if edit.End != nil {
			eU, eV = edit.End.U, edit.End.V
		}
		faces[fi].Wall = offset2d.OriginalSegment(survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}}, sU, sV, eU, eV)
	}
	// A receiver's route L bands keep their faces and loops: route E rewrites
	// faces in place and appends its blend faces after them, and a selected
	// edge ending on a band's boundary meets too few record edges (SB6).
	out := Payload{Faces: append(faces, blendFaces...), Xform: r.BP.Xform, LoopBands: r.BP.LoopBands}
	out.AssignRoles()
	return out, nil
}

// auditFailure wraps an audit refusal of one rewritten face with the face's
// role and the selection, passing a context error through unwrapped.
func (r *EdgeRoute) auditFailure(f Face, err error) error {
	if errors.Is(err, context.Canceled) {
		return context.Canceled
	}
	if errors.Is(err, context.DeadlineExceeded) {
		return context.DeadlineExceeded
	}
	return fmt.Errorf(`%w; in brep face %s; selector %s matched [%s]`,
		renderRouteEAuditCoordinates(err), f.Role, r.Selector, r.Selected)
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
func (r *EdgeRoute) blendFace(eb *EdgeBlend) Face {
	g0 := r.BP.Faces[eb.End[0].Face]
	embed := r.Topo.Embeds[eb.End[0].Face]
	z0, z0Delta := g0.Z0, g0.Z0Delta
	z1, z1Delta := embed.Local(eb.V[1])[2], r.BP.Faces[eb.End[1].Face].Z0Delta
	wall := eb.Blend[0].Connector
	if inner := (z1 > z0) != g0.Outward; !inner {
		wall = brepgeom.ReverseSegment(wall)
	}
	if z1 < z0 {
		z0, z1 = z1, z0
		z0Delta, z1Delta = z1Delta, z0Delta
	}
	return Face{Frame: g0.Frame, Wall: wall, Z0: z0, Z1: z1,
		Z0Delta: z0Delta, Z1Delta: z1Delta, Blend: r.BlendKind}
}

type routeEAuditDiagnosticError struct {
	cause   error
	message string
}

func (e *routeEAuditDiagnosticError) Error() string { return e.message }
func (e *routeEAuditDiagnosticError) Unwrap() error { return e.cause }

func renderRouteEAuditCoordinates(err error) error {
	diagnostic, ok := err.(*sectionaudit.DiagnosticError)
	if !ok {
		return err
	}
	return &routeEAuditDiagnosticError{cause: err, message: diagnostic.Detailed()}
}
