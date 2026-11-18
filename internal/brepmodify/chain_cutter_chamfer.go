package brepmodify

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/retainedblend"
	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// ChainCutterChamfer chamfers the tangent chains of a mesh Boolean result whose edges share no one base plane
// (agent-docs/modify-design.md Part M §15b): a bar rounded about one axis at one end and about another at the
// other, whose rim runs over both caps and both sides, or a slot whose rim runs down its sides and round its end on the
// top and bottom faces. The retained chain route (§5) builds a chain on one base plane only.
//
// Every selected edge is a retained line or arc between two faces with retained plane or cylinder tags, and convex.
// The edges group into chains whose junctions §5.2 proves tangent from the tags. Each junction proves both its edges'
// walls normal to its base, so every edge's two faces meet at a right angle along its whole length. A chain is closed,
// or open with each end at a straight edge that meets one end face at a convex corner of exactly three faces.
//
// Each chain is cut from the body by one cutter: the chamfer's cross-section, the triangle between the edge and the
// bevel that leaves each face d from the edge, grown past both faces by the overshoot and swept along the chain. A line
// sweeps it exactly; an arc steps it through chords whose sagitta is at most cutterSagitta. Proven tangent junctions
// share one cross-section, so the cutter closes on a closed chain. An open end carries the cross-section straight on
// past the end face (chainEnd). A held facet of any face other than the chain's faces, its end faces and their
// neighbours that meets the cutter refuses the chain (checkCutterClearOf). The result is a Boolean Cut: faceted, with
// the Cut's own readings.
//
// It reports false, leaving the earlier refusal in charge, for any receiver or selection it does not recognise. A
// recognised selection that cannot be cut is refused as ErrUnsupported.
func ChainCutterChamfer(ctx context.Context, env topo.Env, hooks Hooks, body *topo.Body, edges []*topo.Edge,
	d, dDelta float64, asym *Asymmetric) (Result, bool, error) {
	if _, ok := body.Payload.(facetedrecord.Payload); !ok || !body.Solid || asym != nil || dDelta != 0 || !(d > 0) ||
		len(edges) == 0 || hooks.MeshSolid == nil || hooks.Boolean == nil {
		return Result{}, false, nil
	}
	chains, ok, err := readCrossChains(body, edges)
	if !ok {
		return Result{}, false, nil
	}
	if err != nil {
		return Result{}, true, err
	}
	for _, c := range chains {
		if err := c.readEnds(body, d); err != nil {
			return Result{}, true, err
		}
	}
	var last error
	for _, over := range partialOvershoots {
		out, used, err := cutCrossChains(ctx, env, hooks, body, chains, d, over*d)
		if err == nil {
			return Result{Body: out, Span: used}, true, nil
		}
		last = err
		if ctx.Err() != nil || !isUnsupported(err) {
			break
		}
	}
	return Result{}, true, last
}

// chainStep is one edge of a chain in walk order, from one vertex to the other.
type chainStep struct {
	edge     *topo.Edge
	from, to *topo.Vertex
}

// crossChain is one tangent chain the cutter route cuts.
type crossChain struct {
	steps []chainStep
	open  bool
	// ends are an open chain's two ends: at steps[0].from and at the last step's to.
	ends [2]chainEnd
	// faces are the faces the cutter may meet: both faces of every edge, each end face and the end faces' neighbours.
	faces map[*topo.Face]struct{}
}

// chainEnd is an open chain's end: the plane end face it meets and how far past the end vertex the cross-section must
// reach to clear it.
type chainEnd struct {
	face  *topo.Face
	reach float64
}

// readCrossChains groups edges into tangent chains. recognised is false unless every edge is a convex retained line
// or arc between faces with retained tags, every edge has a tangent neighbour, every proven tangent continuation is
// selected, and no chain lies on one base face. A recognised selection with chains that meet is refused.
func readCrossChains(body *topo.Body, edges []*topo.Edge) (chains []*crossChain, recognised bool, err error) {
	selected := make(map[*topo.Edge]struct{}, len(edges))
	for _, e := range edges {
		if !retainedblend.Chainable(e) || !e.Convex {
			return nil, false, nil
		}
		selected[e] = struct{}{}
	}
	incident := map[*topo.Vertex][]*topo.Edge{}
	for _, e := range topobuild.Edges(body) {
		if e.Start == nil || e.End == nil || e.Start == e.End {
			continue
		}
		incident[e.Start] = append(incident[e.Start], e)
		incident[e.End] = append(incident[e.End], e)
	}
	// links[e][v] is the selected edge that continues e through v.
	links := map[*topo.Edge]map[*topo.Vertex]*topo.Edge{}
	for _, e := range edges {
		links[e] = map[*topo.Vertex]*topo.Edge{}
		for _, v := range [2]*topo.Vertex{e.Start, e.End} {
			var found *topo.Edge
			for _, c := range incident[v] {
				if c == e {
					continue
				}
				_, sel := selected[c]
				switch retainedblend.Tangent(e, c, v) {
				case clearance.DegYes:
					if !sel || found != nil {
						return nil, false, nil
					}
					found = c
				case clearance.DegUnknown:
					if sel {
						return nil, false, nil
					}
				}
			}
			if found != nil {
				links[e][v] = found
			}
		}
		if len(links[e]) == 0 {
			return nil, false, nil
		}
	}
	other := func(e *topo.Edge, v *topo.Vertex) *topo.Vertex {
		if e.Start == v {
			return e.End
		}
		return e.Start
	}
	seen := map[*topo.Edge]struct{}{}
	walk := func(first *topo.Edge, from *topo.Vertex, open bool) *crossChain {
		c := &crossChain{open: open, faces: map[*topo.Face]struct{}{}}
		cur := first
		for {
			seen[cur] = struct{}{}
			to := other(cur, from)
			c.steps = append(c.steps, chainStep{edge: cur, from: from, to: to})
			for _, f := range cur.Faces {
				c.faces[f] = struct{}{}
			}
			next := links[cur][to]
			if next == nil {
				return c
			}
			if _, done := seen[next]; done {
				return c
			}
			cur, from = next, to
		}
	}
	// An open chain starts at an edge with an end that nothing continues.
	for _, e := range edges {
		if _, done := seen[e]; done || len(links[e]) == 2 {
			continue
		}
		from := e.Start
		if links[e][e.Start] != nil {
			from = e.End
		}
		chains = append(chains, walk(e, from, true))
	}
	for _, e := range edges {
		if _, done := seen[e]; done {
			continue
		}
		c := walk(e, e.Start, false)
		// A closed chain returns to its first edge through the first edge's start.
		last := c.steps[len(c.steps)-1]
		if last.to != e.Start || links[last.edge][last.to] != e {
			return nil, false, nil
		}
		chains = append(chains, c)
	}
	for _, c := range chains {
		if len(c.steps) < 2 || c.oneBase() {
			return nil, false, nil
		}
	}
	// Chains that share a vertex meet at a corner, where two cutters would overlap.
	owner := map[*topo.Vertex]int{}
	for i, c := range chains {
		for _, s := range c.steps {
			for _, v := range [2]*topo.Vertex{s.from, s.to} {
				if j, ok := owner[v]; ok && j != i {
					return nil, true, fmt.Errorf("%w: two chamfered tangent chains meet at the corner %s",
						decaderr.ErrUnsupported, selectorquery.RenderVec(v.Position))
				}
				owner[v] = i
			}
		}
	}
	return chains, true, nil
}

// oneBase reports whether one face borders every edge of the chain, which makes it a chain of the retained chain
// route (§5).
func (c *crossChain) oneBase() bool {
	for _, f := range c.steps[0].edge.Faces {
		all := true
		for _, s := range c.steps[1:] {
			all = all && (s.edge.Faces[0] == f || s.edge.Faces[1] == f)
		}
		if all {
			return true
		}
	}
	return false
}

// endMinSine is the least sine at which an open chain's end face may meet the edge it ends: a face nearly along the
// edge would need the cross-section carried far past the end.
const endMinSine = 1e-3

// readEnds reads both ends of an open chain (chainEnd). Each end is a straight edge whose end vertex is a corner of
// exactly three faces: the edge's two faces and one end face G, joined by two convex edges, one from each of the
// edge's faces to G. G's outward normal at the vertex is read from its plane, or from its held facets there. The
// cross-section carried past the vertex by reach clears G's plane: reach = d·max(0, n_G·n_A, n_G·n_B)/(n_G·t), with t
// the edge's direction out of the chain. A convex corner keeps the body inside all three planes, so what the cutter
// takes past the vertex is only the wedge between the two faces and the bevel.
func (c *crossChain) readEnds(body *topo.Body, d float64) error {
	if !c.open {
		return nil
	}
	n := len(c.steps)
	for k, at := range [2]struct {
		step chainStep
		v    *topo.Vertex
	}{{c.steps[0], c.steps[0].from}, {c.steps[n-1], c.steps[n-1].to}} {
		e := at.step.edge
		refuse := func(why string) error {
			return fmt.Errorf("%w: an open tangent chain's end at %s %s", decaderr.ErrUnsupported,
				selectorquery.RenderVec(at.v.Position), why)
		}
		if _, line := e.Curve.(surfacegeom.Line3); !line {
			return refuse("is the end of an arc")
		}
		otherV := e.Start
		if otherV == at.v {
			otherV = e.End
		}
		t, ok := at.v.Position.Sub(otherV.Position).Normalize()
		if !ok {
			return refuse("is the end of an edge of no length")
		}
		var g *topo.Face
		count := 0
		for _, x := range topobuild.Edges(body) {
			if x == e || (x.Start != at.v && x.End != at.v) {
				continue
			}
			count++
			if len(x.Faces) != 2 || !x.Convex {
				return refuse("is not a convex corner")
			}
			var mine, third *topo.Face
			for _, f := range x.Faces {
				if f == e.Faces[0] || f == e.Faces[1] {
					mine = f
				} else {
					third = f
				}
			}
			if mine == nil || third == nil || (g != nil && g != third) {
				return refuse("is not a corner of three faces")
			}
			g = third
		}
		if count != 2 || g == nil {
			return refuse("is not a corner of three faces")
		}
		nG, ok := endNormal(body, g, at.v)
		if !ok {
			return refuse("meets an end face that is not a plane there")
		}
		nA, okA := outwardNormal(e.Faces[0], at.v.Position)
		nB, okB := outwardNormal(e.Faces[1], at.v.Position)
		if !okA || !okB {
			return refuse("has a face with no normal there")
		}
		gt := nG.Dot(t)
		if gt < endMinSine {
			return refuse("meets an end face that does not face away from the chain")
		}
		c.ends[k] = chainEnd{face: g, reach: d * max(0, nG.Dot(nA), nG.Dot(nB)) / gt}
		c.faces[g] = struct{}{}
		// The bevel may reach the far edge of G, where G's neighbours begin.
		for _, x := range topobuild.Edges(body) {
			if len(x.Faces) != 2 || (x.Faces[0] != g && x.Faces[1] != g) {
				continue
			}
			c.faces[x.Faces[0]] = struct{}{}
			c.faces[x.Faces[1]] = struct{}{}
		}
	}
	return nil
}

// endNormal is the unit outward normal of the end face g at v: its plane's, or the common normal of its held facets
// that hold v when g is faceted. It reports false when g is curved or its facets there are not coplanar.
func endNormal(body *topo.Body, g *topo.Face, v *topo.Vertex) (r3.Vec, bool) {
	if pl, ok := g.Surface.(surfacegeom.Plane); ok {
		n := pl.Frame.N()
		if g.Reversed {
			n = n.Scale(-1)
		}
		return n.Normalize()
	}
	if _, ok := g.Surface.(surfacegeom.Faceted); !ok {
		return r3.Vec{}, false
	}
	fp, ok := body.Payload.(facetedrecord.Payload)
	if !ok || len(fp.FaceOf) != len(fp.Tris) {
		return r3.Vec{}, false
	}
	gi := -1
	for i, f := range topobuild.Faces(body) {
		if f == g {
			gi = i
			break
		}
	}
	held := -1
	for i, p := range fp.Verts {
		if p == v.Position {
			held = i
			break
		}
	}
	if gi < 0 || held < 0 {
		return r3.Vec{}, false
	}
	var n r3.Vec
	found := false
	for ti, tri := range fp.Tris {
		if fp.FaceOf[ti] != gi || (tri[0] != held && tri[1] != held && tri[2] != held) {
			continue
		}
		a, b, cc := fp.Verts[tri[0]], fp.Verts[tri[1]], fp.Verts[tri[2]]
		m, ok := b.Sub(a).Cross(cc.Sub(a)).Normalize()
		if !ok {
			return r3.Vec{}, false
		}
		if !found {
			n, found = m, true
			continue
		}
		if m.Dot(n) < 1-1e-12 {
			return r3.Vec{}, false
		}
	}
	return n, found
}

// outwardNormal is the unit outward normal at p of a face with a plane or cylinder tag.
func outwardNormal(f *topo.Face, p r3.Vec) (r3.Vec, bool) {
	var n r3.Vec
	switch s := f.Surface.(type) {
	case surfacegeom.Plane:
		n = s.Frame.N()
	case surfacegeom.Cylinder:
		rel := p.Sub(s.Origin)
		n = rel.Sub(s.Axis.Scale(rel.Dot(s.Axis) / s.Axis.Dot(s.Axis)))
	default:
		return r3.Vec{}, false
	}
	if f.Reversed {
		n = n.Scale(-1)
	}
	return n.Normalize()
}

// chainSection is the cutter's cross-section at p, where the chain runs along t between faces with outward normals nA and
// nB (ordered so that nA × nB points along t). In the frame (a, b) along (nA, nB) the body is a, b ≤ 0 near p and the
// chamfer removes a + b ≥ −d. The cutter is the triangle a ≤ o, b ≤ o, a + b ≥ −d with o = over/√2, so it holds the
// removed wedge and reaches o past each face. Its corners are returned in an order that turns clockwise about t.
func chainSection(p, t, nA, nB r3.Vec, d, over float64) [3]r3.Vec {
	if nA.Cross(nB).Dot(t) < 0 {
		nA, nB = nB, nA
	}
	o := over / math.Sqrt2
	return [3]r3.Vec{
		p.Add(nA.Scale(o)).Sub(nB.Scale(d + o)),
		p.Sub(nA.Scale(d + o)).Add(nB.Scale(o)),
		p.Add(nA.Scale(o)).Add(nB.Scale(o)),
	}
}

// cutterMesh sweeps the chain's cross-section into a closed polyhedron. The sections sit at every vertex of the chain
// and at the chord points of every arc; an open chain adds one past each end, reach + over beyond the end vertex.
func (c *crossChain) cutterMesh(d, over float64) (cutterMesh, error) {
	var rings [][3]r3.Vec
	sectionAt := func(e *topo.Edge, p, t r3.Vec) error {
		nA, okA := outwardNormal(e.Faces[0], p)
		nB, okB := outwardNormal(e.Faces[1], p)
		if !okA || !okB {
			return fmt.Errorf("%w: a chain edge's face has no normal at %s", decaderr.ErrUnsupported,
				selectorquery.RenderVec(p))
		}
		rings = append(rings, chainSection(p, t, nA, nB, d, over))
		return nil
	}
	total := 0
	for _, s := range c.steps {
		p0, p1 := s.from.Position, s.to.Position
		switch arc := s.edge.Curve.(type) {
		case surfacegeom.Line3:
			t, ok := p1.Sub(p0).Normalize()
			if !ok {
				return cutterMesh{}, fmt.Errorf("%w: a chain edge has no length", decaderr.ErrUnsupported)
			}
			if err := sectionAt(s.edge, p0, t); err != nil {
				return cutterMesh{}, err
			}
		case surfacegeom.Arc3:
			axis, ok := arc.Axis.Normalize()
			if !ok {
				return cutterMesh{}, fmt.Errorf("%w: a chain arc has no axis", decaderr.ErrUnsupported)
			}
			// An Arc3 turns counter-clockwise about its axis from its start to its end; walking it the other way turns
			// about the reversed axis.
			if s.from != s.edge.Start {
				axis = axis.Scale(-1)
			}
			a, b := p0.Sub(arc.Center), p1.Sub(arc.Center)
			a = a.Sub(axis.Scale(a.Dot(axis)))
			b = b.Sub(axis.Scale(b.Dot(axis)))
			sweep := math.Atan2(axis.Dot(a.Cross(b)), a.Dot(b))
			if sweep <= 0 {
				sweep += 2 * math.Pi
			}
			r := a.Len()
			rmax := r + d + over
			step := 2 * math.Acos(1-cutterSagitta/rmax)
			cells := max(2, int(math.Ceil(sweep/step)))
			total += cells
			if total > loftmesh.MaxLoftAuditTriangles {
				return cutterMesh{}, fmt.Errorf(`%w: the chain chamfer's cutter needs more than %d sections`,
					decaderr.ErrUnsupported, loftmesh.MaxLoftAuditTriangles)
			}
			u := a.Scale(1 / r)
			w := axis.Cross(u)
			for k := range cells {
				phi := sweep * float64(k) / float64(cells)
				p := p0
				if k > 0 {
					p = arc.Center.Add(axis.Scale(p0.Sub(arc.Center).Dot(axis))).
						Add(u.Scale(r * math.Cos(phi))).Add(w.Scale(r * math.Sin(phi)))
				}
				t := w.Scale(math.Cos(phi)).Sub(u.Scale(math.Sin(phi)))
				if err := sectionAt(s.edge, p, t); err != nil {
					return cutterMesh{}, err
				}
			}
		default:
			return cutterMesh{}, fmt.Errorf("%w: a chain edge is not a line or an arc", decaderr.ErrUnsupported)
		}
	}
	if c.open {
		first, last := c.steps[0], c.steps[len(c.steps)-1]
		lt, _ := last.to.Position.Sub(last.from.Position).Normalize()
		ft, _ := first.to.Position.Sub(first.from.Position).Normalize()
		if err := sectionAt(last.edge, last.to.Position, lt); err != nil {
			return cutterMesh{}, err
		}
		if err := sectionAt(last.edge, last.to.Position.Add(lt.Scale(c.ends[1].reach+over)), lt); err != nil {
			return cutterMesh{}, err
		}
		head := rings
		rings = nil
		if err := sectionAt(first.edge, first.from.Position.Sub(ft.Scale(c.ends[0].reach+over)), ft); err != nil {
			return cutterMesh{}, err
		}
		rings = append(rings, head...)
	}
	m := cutterMesh{bound: cutterSagitta}
	for _, r := range rings {
		m.verts = append(m.verts, r[0], r[1], r[2])
	}
	n := len(rings)
	quads := n
	if c.open {
		quads = n - 1
		// The corners turn clockwise about t, so the first cap faces back along t and the last forward.
		m.tris = append(m.tris, [3]int{0, 1, 2}, [3]int{3 * (n - 1), 3*(n-1) + 2, 3*(n-1) + 1})
	}
	for k := range quads {
		k2 := (k + 1) % n
		for j := range 3 {
			j2 := (j + 1) % 3
			a, b := 3*k+j, 3*k+j2
			cc, e := 3*k2+j2, 3*k2+j
			m.tris = append(m.tris, [3]int{a, cc, b}, [3]int{a, e, cc})
		}
	}
	return m, nil
}

// cutCrossChains cuts every chain from body with a cutter that reaches over past the faces, under producer identities
// from env.Ref on. Every cutter is built and checked against body before any Cut, and no two cutters' boxes meet.
func cutCrossChains(ctx context.Context, env topo.Env, hooks Hooks, body *topo.Body, chains []*crossChain,
	d, over float64) (*topo.Body, int, error) {
	used := 0
	next := func() topo.Env {
		step := topo.Env{Doc: env.Doc, Ref: env.Ref + provenance.Producer(used), Alloc: env.Alloc}
		used++
		return step
	}
	faces := topobuild.Faces(body)
	index := make(map[*topo.Face]int, len(faces))
	for i, f := range faces {
		index[f] = i
	}
	cutters := make([]*topo.Body, len(chains))
	boxes := make([]facetBox, len(chains))
	for i, c := range chains {
		mesh, err := c.cutterMesh(d, over)
		if err != nil {
			return nil, used, err
		}
		box := boxOfFacet(mesh.verts, mesh.tris[0])
		for _, t := range mesh.tris[1:] {
			box = box.join(boxOfFacet(mesh.verts, t))
		}
		for j := range i {
			if box.meets(boxes[j]) {
				return nil, used, fmt.Errorf("%w: two chamfered tangent chains' cutters meet", decaderr.ErrUnsupported)
			}
		}
		boxes[i] = box
		cutter, err := hooks.MeshSolid(ctx, next(), mesh.verts, mesh.tris, mesh.bound)
		if err != nil {
			return nil, used, fmt.Errorf("%w: the chain chamfer's cutter: %v", decaderr.ErrUnsupported, err)
		}
		allowed := make(map[int]struct{}, len(c.faces))
		for f := range c.faces {
			if i, ok := index[f]; ok {
				allowed[i] = struct{}{}
			}
		}
		if err := checkCutterClearOf(ctx, body, cutter, mesh.bound, allowed, "chain chamfer"); err != nil {
			return nil, used, err
		}

		cutters[i] = cutter
	}
	out := body
	var warnings opdiagnostic.CompositeSummaries
	for _, cutter := range cutters {
		var err error
		if out, err = hooks.Boolean(allowCoarse(ctx, out), meshbool.OpCut, next(), out, cutter); err != nil {
			return nil, used, err
		}
		warnings.Add(out.OperationWarnings, 0)
	}
	if out.Volume.Value.Base() >= body.Volume.Value.Base() {
		return nil, used, fmt.Errorf(`%w: the chain chamfer removed no material`, decaderr.ErrUnsupported)
	}
	out.OperationWarnings = warnings.Records()
	return out, used, ctx.Err()
}
