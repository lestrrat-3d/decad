package decad

import (
	"context"
	"fmt"
)

// This file is docs/tessellation-design.md §1.2's manifold-with-boundary
// audit for a BodySheet mesh, and it leans on docs/surface-design.md §2.3's
// own consistent-orientation claim for a sheet shell rather than re-proving
// it. requireSheetMesh and requireSheetVertexLinks run where a BodySolid mesh
// runs the closed-mesh audit and its own vertex-link safety net
// (requireClosedMesh, tessellate.go; requireVertexLinks,
// tessellate_revolve_proof.go): a safety net AFTER the walls are built,
// never a substitute for building them correctly. Every check here is purely
// COMBINATORIAL, over the mesh's own directed-edge structure and the body's
// own recorded free-edge topology — no coordinate is compared against
// another, so nothing here can admit a claim it did not already hold by
// construction (CLAUDE.md's reject-only rule).

// requireSheetMesh runs docs/tessellation-design.md §1.2's audit in place of
// the closed-mesh audit a BodySolid mesh takes:
//
//   - no directed edge occurs more than once;
//   - an edge whose reverse is present occurs exactly once (an interior
//     edge shared by two facets, wound in opposite directions);
//   - every free directed edge of the mesh — grouped by the face
//     [Mesh.SourceFaces] attributes it to — matches the body's own recorded
//     free [Edge]s: the same set of faces, and, for each of them, the same
//     count of boundary chains.
//
// Orientation across every interior edge is proven by CONSTRUCTION
// (docs/surface-design.md §2.3): the mesh is built from the walls with the
// payload's own winding, so a facet and its neighbor already traverse their
// shared edge in opposite senses, and the tests assert this directly rather
// than this audit re-deriving it from a per-triangle sign test against a
// bounded [Face.NormalAt] — a geometric admission gate CLAUDE.md's
// reject-only rule forbids, where construction already proves the answer.
func requireSheetMesh(ctx context.Context, b *Body, m *Mesh) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	meshFree, err := freeSheetEdgesByFace(m)
	if err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	bodyFree := freeChainCountsByFace(b)
	if err := ctx.Err(); err != nil {
		return err
	}
	return requireMatchingFreeAttribution(meshFree, bodyFree)
}

// freeSheetEdgesByFace walks every triangle's three directed edges, refusing
// a directed edge that occurs more than once, then groups every remaining
// FREE directed edge — one whose reverse is absent — by the face
// [Mesh.SourceFaces] names for the triangle it belongs to. An edge whose
// reverse IS present therefore occurs exactly once in each direction,
// because the count of every directed edge this function admits is already
// at most one.
func freeSheetEdgesByFace(m *Mesh) (map[*Face][][2]int, error) {
	directed := make(map[[2]int]int, 3*len(m.triangles))
	for _, tri := range m.triangles {
		for k := range 3 {
			directed[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	for e, n := range directed {
		if n > 1 {
			return nil, fmt.Errorf(`%w: the sheet mesh's directed edge %v occurs %d times, so it is not a manifold-with-boundary skin`, ErrDegenerate, e, n)
		}
	}
	out := map[*Face][][2]int{}
	for i, tri := range m.triangles {
		face := m.source[i]
		for k := range 3 {
			e := [2]int{tri[k], tri[(k+1)%3]}
			rev := [2]int{e[1], e[0]}
			if directed[rev] != 0 {
				continue // an interior edge, shared with the facet across it
			}
			out[face] = append(out[face], e)
		}
	}
	return out, nil
}

// freeChainCountsByFace groups the body's own recorded free Edges by their
// single adjacent face and counts, per face, the CONNECTED COMPONENTS its
// free Edges form over their own Vertex pointers — one boundary CHAIN per
// component, not one per Edge object. A coalesced wall's rim is usually one
// recorded curve, a single segment or several merged collinear ones, so it
// is usually one Edge object AND one chain. But two of a face's free Edges
// that share an interned Vertex — a revolve wall's two rims meeting at a pole
// the payload interns between them — are still only ONE chain: the two
// Edges are two chords of the same connected boundary walk, exactly as
// countChains reads the mesh side of the same face by connected component
// rather than by directed-edge count.
func freeChainCountsByFace(b *Body) map[*Face]int {
	byFace := map[*Face][][2]*Vertex{}
	for _, e := range b.Edges() {
		if !e.IsFree() {
			continue
		}
		faces := e.Faces()
		if len(faces) != 1 {
			continue // IsFree already guarantees exactly one; defensive only
		}
		byFace[faces[0]] = append(byFace[faces[0]], [2]*Vertex{e.Start(), e.End()})
	}
	counts := make(map[*Face]int, len(byFace))
	for face, edges := range byFace {
		counts[face] = countVertexChains(edges)
	}
	return counts
}

// countVertexChains counts the connected components of the undirected graph
// one face's free Edges form over their own Vertex pointers, mirroring
// surface.go's splitConnectedFaces/unionFindRoot shape — the same
// union-find over pointer identity, keyed on *Vertex here instead of *Face.
func countVertexChains(edges [][2]*Vertex) int {
	parent := map[*Vertex]*Vertex{}
	for _, e := range edges {
		if _, ok := parent[e[0]]; !ok {
			parent[e[0]] = e[0]
		}
		if _, ok := parent[e[1]]; !ok {
			parent[e[1]] = e[1]
		}
	}
	for _, e := range edges {
		ra, rb := vertexChainRoot(parent, e[0]), vertexChainRoot(parent, e[1])
		if ra != rb {
			parent[ra] = rb
		}
	}
	roots := map[*Vertex]struct{}{}
	for v := range parent {
		roots[vertexChainRoot(parent, v)] = struct{}{}
	}
	return len(roots)
}

// vertexChainRoot is countVertexChains's own path-compressing find, a
// top-level function rather than a closure over parent for the same reason
// surface.go's unionFindRoot is: a self-referential closure cannot be
// declared and assigned in one statement (staticcheck S1021 does not account
// for the recursion).
func vertexChainRoot(parent map[*Vertex]*Vertex, v *Vertex) *Vertex {
	for parent[v] != v {
		parent[v] = parent[parent[v]]
		v = parent[v]
	}
	return v
}

// requireMatchingFreeAttribution is the combinatorial admission decision
// itself: the mesh's free-edge face set and the body's own must agree
// exactly, and every face they share must carry the same number of boundary
// chains on both sides — countChains' connected-component reading of the
// mesh's own free vertex graph against freeChainCountsByFace's own
// connected-component reading of the body's recorded free Edges.
func requireMatchingFreeAttribution(mesh map[*Face][][2]int, body map[*Face]int) error {
	if len(mesh) != len(body) {
		return fmt.Errorf(`%w: the sheet mesh's free boundary names %d face(s) but the body's own recorded free edges name %d`, ErrDegenerate, len(mesh), len(body))
	}
	for face, edges := range mesh {
		want, ok := body[face]
		if !ok {
			return fmt.Errorf(`%w: the sheet mesh's free boundary names a face the body carries no recorded free edge for`, ErrDegenerate)
		}
		got := countChains(edges)
		if got != want {
			return fmt.Errorf(`%w: one face carries %d free boundary chain(s) in the mesh but %d recorded free edge(s) on the body`, ErrDegenerate, got, want)
		}
	}
	return nil
}

// countChains counts the connected components of the undirected graph one
// face's free directed edges form over the mesh's vertex indices — the
// number of boundary CHAINS (open polylines, or closed cycles for a whole
// coalesced loop built as one face) that face's free boundary decomposes
// into.
func countChains(edges [][2]int) int {
	parent := map[int]int{}
	for _, e := range edges {
		ra, rb := chainRoot(parent, e[0]), chainRoot(parent, e[1])
		if ra != rb {
			parent[ra] = rb
		}
	}
	roots := map[int]struct{}{}
	for v := range parent {
		roots[chainRoot(parent, v)] = struct{}{}
	}
	return len(roots)
}

// chainRoot is countChains's own path-compressing find, a top-level function
// rather than a closure over parent for the same reason surface.go's
// unionFindRoot is: a self-referential closure cannot be declared and
// assigned in one statement (staticcheck S1021 does not account for the
// recursion). It lazily seeds an unseen vertex as its own root.
func chainRoot(parent map[int]int, v int) int {
	if _, ok := parent[v]; !ok {
		parent[v] = v
	}
	for parent[v] != v {
		parent[v] = parent[parent[v]]
		v = parent[v]
	}
	return v
}

// requireSheetVertexLinks is docs/tessellation-design.md §1.2's own vertex-
// link safety net, run where a BodySolid mesh runs requireVertexLinks
// (tessellate_revolve_proof.go, §9). requireVertexLinks demands one
// connected CYCLE at every degree-two vertex, which is right for a closed
// mesh but wrong for a boundary vertex: a free edge's two endpoints each
// have a link that is an open PATH terminated by the two free edges meeting
// there, not a cycle, so requireVertexLinks's cycle-only rule would refuse
// every sound open sheet. This audit keeps requireVertexLinks unweakened and
// instead admits either shape: at every stored vertex, the combinatorial
// link — the edge each incident triangle contributes between its other two
// corners — must be ONE connected component that is either a cycle, every
// link vertex at degree two, or a path, exactly two link vertices at degree
// one and the rest at degree two. More than one component, a fork (a link
// vertex at degree three or more), or any other shape is a pinched or
// forked vertex and is refused with [ErrUnsupported], the same sentinel
// requireVertexLinks uses. Like requireVertexLinks, it visits vertices in
// index order and returns on the first failure, so two runs over the same
// mesh report the same one.
func requireSheetVertexLinks(ctx context.Context, m *Mesh) error {
	budget := newWorkBudget(ctx)
	links := make(map[int]map[int][]int, len(m.vertices))
	add := func(center, from, to int) {
		l, ok := links[center]
		if !ok {
			l = map[int][]int{}
			links[center] = l
		}
		l[from] = append(l[from], to)
		l[to] = append(l[to], from)
	}
	for _, tri := range m.triangles {
		if err := budget.step(); err != nil {
			return err
		}
		add(tri[0], tri[1], tri[2])
		add(tri[1], tri[2], tri[0])
		add(tri[2], tri[0], tri[1])
	}
	// Vertex index order, never map order: a refusal names the FIRST vertex
	// that fails, so two runs over the same mesh report the same one.
	for center := range m.vertices {
		link, ok := links[center]
		if !ok {
			continue
		}
		if err := budget.step(); err != nil {
			return err
		}
		if err := requireCycleOrPathLink(center, link); err != nil {
			return err
		}
	}
	return budget.err()
}

// requireCycleOrPathLink checks one vertex's own link graph — center's own
// degree-tagged neighbour lists, keyed by link vertex — against
// requireSheetVertexLinks' cycle-or-path rule.
func requireCycleOrPathLink(center int, link map[int][]int) error {
	ends, cycleStart, pathStart := 0, -1, -1
	for v, nbrs := range link {
		switch len(nbrs) {
		case 1:
			ends++
			if pathStart < 0 || v < pathStart {
				pathStart = v
			}
		case 2:
			// an interior link vertex either shape admits
		default:
			return fmt.Errorf(`%w: the mesh vertex at index %d has a forked link: its neighbour %d meets %d link edges rather than one or two`, ErrUnsupported, center, v, len(nbrs))
		}
		if cycleStart < 0 || v < cycleStart {
			cycleStart = v
		}
	}
	if cycleStart < 0 {
		return nil
	}
	if ends != 0 && ends != 2 {
		return fmt.Errorf(`%w: the mesh vertex at index %d has a link with %d degree-one end(s) rather than zero (a cycle) or two (a path)`, ErrUnsupported, center, ends)
	}
	// A cycle (ends == 0) can start anywhere, so it starts at its
	// lowest-indexed vertex; a path (ends == 2) MUST start at one of its two
	// degree-one ends — starting mid-path would only walk one branch and
	// miss the other — so it starts at the lower-indexed end.
	start := cycleStart
	if ends == 2 {
		start = pathStart
	}
	seen := map[int]struct{}{start: {}}
	prev, cur := -1, start
	for {
		next := -1
		for _, n := range link[cur] {
			if n != prev {
				next = n
				break
			}
		}
		if next < 0 {
			break // a path's far end: its only neighbour was where we came from
		}
		if next == start {
			break // a cycle closing back on its own start
		}
		if _, done := seen[next]; done {
			return fmt.Errorf(`%w: the mesh vertex at index %d has a pinched link`, ErrUnsupported, center)
		}
		seen[next] = struct{}{}
		prev, cur = cur, next
	}
	if len(seen) != len(link) {
		return fmt.Errorf(`%w: the mesh vertex at index %d has a link of more than one connected component`, ErrUnsupported, center)
	}
	return nil
}
