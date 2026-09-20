package decad

import (
	"context"
	"fmt"
)

// This file is docs/tessellation-design.md §1.2's manifold-with-boundary
// audit for a BodySheet mesh, and it leans on docs/surface-design.md §2.3's
// own consistent-orientation claim for a sheet shell rather than re-proving
// it. requireSheetMesh runs where a BodySolid mesh runs the closed-mesh
// audit (requireClosedMesh, tessellate.go): a safety net AFTER the walls are
// built, never a substitute for building them correctly. Every check here is
// purely COMBINATORIAL, over the mesh's own directed-edge structure and the
// body's own recorded free-edge topology — no coordinate is compared against
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
// single adjacent face, one Edge (docs/api-design.md §6.1's [Edge.IsFree])
// per boundary CHAIN: a coalesced wall's rim is one recorded curve — a
// single segment or several merged collinear ones — so each free Edge object
// already names one whole chain rather than one chord, whatever number of
// mesh samples it was chorded into.
func freeChainCountsByFace(b *Body) map[*Face]int {
	counts := map[*Face]int{}
	for _, e := range b.Edges() {
		if !e.IsFree() {
			continue
		}
		faces := e.Faces()
		if len(faces) != 1 {
			continue // IsFree already guarantees exactly one; defensive only
		}
		counts[faces[0]]++
	}
	return counts
}

// requireMatchingFreeAttribution is the combinatorial admission decision
// itself: the mesh's free-edge face set and the body's own must agree
// exactly, and every face they share must carry the same number of boundary
// chains on both sides — countChains' connected-component reading of the
// mesh's own free vertex graph against freeChainCountsByFace's per-Edge
// count.
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
