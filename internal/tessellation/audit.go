// Package tessellation contains mesh audits and exact restatements over
// neutral triangle data. The root package owns Mesh, the cache, verification
// levels and the mapping from face numbers back to live faces.
package tessellation

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// This file is docs/tessellation-design.md §1.2's manifold-with-boundary
// audit for a sheet mesh, and the closed-mesh audit a solid mesh takes in its
// place. The sheet audit leans on docs/surface-design.md §2.3's own
// consistent-orientation claim for a sheet shell rather than re-proving it.
// Every check here is purely COMBINATORIAL, over the mesh's own directed-edge
// structure and the body's own recorded free-edge topology, supplied by the
// caller as face numbers — no coordinate is compared against another, so
// nothing here can admit a claim it did not already hold by construction
// (CLAUDE.md's reject-only rule). OrientationSign is the one coordinate
// reader, and it computes an exact sign over dyadics.

// pollInterval is how many counted steps may pass between context polls.
const pollInterval = 256

// AuditBudget shares one bounded cancellation counter across a counted loop. It
// holds closures rather than a stored context, so no geometry state holds a
// context.
type AuditBudget struct {
	stepFn func() error
	errFn  func() error
}

func NewAuditBudget(ctx context.Context) *AuditBudget {
	work := 0
	return &AuditBudget{
		stepFn: func() error {
			work++
			if work%pollInterval == 0 {
				return ctx.Err()
			}
			return nil
		},
		errFn: ctx.Err,
	}
}

// Step counts one candidate operation and returns ctx.Err() on the polling
// interval.
func (b *AuditBudget) Step() error { return b.stepFn() }

// Err polls the context unconditionally, the phase-boundary check.
func (b *AuditBudget) Err() error { return b.errFn() }

// SharedVertexIndices returns the vertex indices two triangles hold in common
// and their count. A shared vertex is a shared table index, so this needs no
// coordinate comparison or allocation.
func SharedVertexIndices(a, b [3]int) ([3]int, int) {
	var shared [3]int
	count := 0
	for _, va := range a {
		for _, vb := range b {
			if va == vb {
				shared[count] = va
				count++
				break
			}
		}
	}
	return shared, count
}

// TriangleApexIndex returns the vertex outside the named edge.
func TriangleApexIndex(tri [3]int, edgeA, edgeB int) int {
	for _, vertex := range tri {
		if vertex != edgeA && vertex != edgeB {
			return vertex
		}
	}
	return -1
}

// RequireClosedMesh proves the mesh is a closed 2-manifold — every directed
// edge is matched by its reverse — refusing a mesh the cap triangulator could
// not close.
func RequireClosedMesh(triangles [][3]int) error {
	directed := make(map[[2]int]int, 3*len(triangles))
	for _, tri := range triangles {
		for k := range 3 {
			directed[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	for e := range directed {
		if directed[e] != 1 || directed[[2]int{e[1], e[0]}] != 1 {
			return degenerate(`the chorded boundary could not be triangulated into a watertight mesh`)
		}
	}
	return nil
}

// SheetBoundary is the free-boundary attribution audit's complete input.
type SheetBoundary struct {
	Triangles [][3]int
	// SourceFaces names, per triangle, the face it belongs to, as the number
	// the caller assigned that face. Two triangles of one face share a number.
	SourceFaces []int
	// FreeChainCounts is the body's own recorded free boundary: per face
	// number, how many connected chains its free edges form. A face with no
	// free edge is absent.
	FreeChainCounts map[int]int
}

// RequireSheetBoundary runs docs/tessellation-design.md §1.2's audit in place
// of the closed-mesh audit a solid mesh takes:
//
//   - no directed edge occurs more than once;
//   - an edge whose reverse is present occurs exactly once (an interior
//     edge shared by two facets, wound in opposite directions);
//   - every free directed edge of the mesh — grouped by the face
//     SourceFaces attributes it to — matches the body's own recorded free
//     edges: the same set of faces, and, for each of them, the same count of
//     boundary chains.
//
// Orientation across every interior edge is proven by CONSTRUCTION
// (docs/surface-design.md §2.3): the mesh is built from the walls with the
// payload's own winding, so a facet and its neighbor already traverse their
// shared edge in opposite senses, and the tests assert this directly rather
// than this audit re-deriving it from a per-triangle sign test against a
// bounded face normal — a geometric admission gate CLAUDE.md's reject-only
// rule forbids, where construction already proves the answer.
func RequireSheetBoundary(ctx context.Context, in SheetBoundary) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	meshFree, err := freeEdgesByFace(in.Triangles, in.SourceFaces)
	if err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	return requireMatchingFreeAttribution(meshFree, in.FreeChainCounts)
}

// freeEdgesByFace walks every triangle's three directed edges, refusing a
// directed edge that occurs more than once, then groups every remaining FREE
// directed edge — one whose reverse is absent — by the face number the
// triangle it belongs to carries. An edge whose reverse IS present therefore
// occurs exactly once in each direction, because the count of every directed
// edge this function admits is already at most one.
func freeEdgesByFace(triangles [][3]int, source []int) (map[int][][2]int, error) {
	directed := make(map[[2]int]int, 3*len(triangles))
	for _, tri := range triangles {
		for k := range 3 {
			directed[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	for e, n := range directed {
		if n > 1 {
			return nil, degenerate(`the sheet mesh's directed edge %v occurs %d times, so it is not a manifold-with-boundary skin`, e, n)
		}
	}
	out := map[int][][2]int{}
	for i, tri := range triangles {
		face := source[i]
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

// requireMatchingFreeAttribution is the combinatorial admission decision
// itself: the mesh's free-edge face set and the body's own must agree
// exactly, and every face they share must carry the same number of boundary
// chains on both sides — countChains' connected-component reading of the
// mesh's own free vertex graph against the caller's own connected-component
// reading of the body's recorded free edges.
func requireMatchingFreeAttribution(mesh map[int][][2]int, body map[int]int) error {
	if len(mesh) != len(body) {
		return degenerate(`the sheet mesh's free boundary names %d face(s) but the body's own recorded free edges name %d`, len(mesh), len(body))
	}
	for face, edges := range mesh {
		want, ok := body[face]
		if !ok {
			return degenerate(`the sheet mesh's free boundary names a face the body carries no recorded free edge for`)
		}
		got := countChains(edges)
		if got != want {
			return degenerate(`one face carries %d free boundary chain(s) in the mesh but %d recorded free edge(s) on the body`, got, want)
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
// rather than a closure over parent: a self-referential closure cannot be
// declared and assigned in one statement (staticcheck S1021 does not account
// for the recursion). It lazily seeds an unseen vertex as its own root.
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

// RequireSheetVertexLinks is docs/tessellation-design.md §1.2's own vertex-
// link safety net for a sheet mesh. The root's closed-mesh vertex-link audit
// demands one connected CYCLE at every degree-two vertex, which is right for
// a closed mesh but wrong for a boundary vertex: a free edge's two endpoints
// each have a link that is an open PATH terminated by the two free edges
// meeting there, not a cycle, so a cycle-only rule would refuse every sound
// open sheet. This audit keeps that rule unweakened and instead admits either
// shape: at every stored vertex, the combinatorial link — the edge each
// incident triangle contributes between its other two corners — must be ONE
// connected component that is either a cycle, every link vertex at degree
// two, or a path, exactly two link vertices at degree one and the rest at
// degree two. More than one component, a fork (a link vertex at degree three
// or more), or any other shape is a pinched or forked vertex and is refused
// as Unsupported, the same sentinel the closed-mesh audit uses. Like it, this
// audit visits vertices in index order and returns on the first failure, so
// two runs over the same mesh report the same one.
//
// vertexCount is the length of the mesh's vertex table; only indices are read.
func RequireSheetVertexLinks(ctx context.Context, vertexCount int, triangles [][3]int) error {
	budget := NewAuditBudget(ctx)
	links := make(map[int]map[int][]int, vertexCount)
	add := func(center, from, to int) {
		l, ok := links[center]
		if !ok {
			l = map[int][]int{}
			links[center] = l
		}
		l[from] = append(l[from], to)
		l[to] = append(l[to], from)
	}
	for _, tri := range triangles {
		if err := budget.Step(); err != nil {
			return err
		}
		add(tri[0], tri[1], tri[2])
		add(tri[1], tri[2], tri[0])
		add(tri[2], tri[0], tri[1])
	}
	// Vertex index order, never map order: a refusal names the FIRST vertex
	// that fails, so two runs over the same mesh report the same one.
	for center := range vertexCount {
		link, ok := links[center]
		if !ok {
			continue
		}
		if err := budget.Step(); err != nil {
			return err
		}
		if err := requireCycleOrPathLink(center, link); err != nil {
			return err
		}
	}
	return budget.Err()
}

// requireCycleOrPathLink checks one vertex's own link graph — center's own
// degree-tagged neighbour lists, keyed by link vertex — against
// RequireSheetVertexLinks' cycle-or-path rule.
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
			return unsupported(`the mesh vertex at index %d has a forked link: its neighbour %d meets %d link edges rather than one or two`, center, v, len(nbrs))
		}
		if cycleStart < 0 || v < cycleStart {
			cycleStart = v
		}
	}
	if cycleStart < 0 {
		return nil
	}
	if ends != 0 && ends != 2 {
		return unsupported(`the mesh vertex at index %d has a link with %d degree-one end(s) rather than zero (a cycle) or two (a path)`, center, ends)
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
			return unsupported(`the mesh vertex at index %d has a pinched link`, center)
		}
		seen[next] = struct{}{}
		prev, cur = cur, next
	}
	if len(seen) != len(link) {
		return unsupported(`the mesh vertex at index %d has a link of more than one connected component`, center)
	}
	return nil
}

// OrientationSign is the sign of the signed tetrahedron sum
// docs/loft-design.md §8 defines, over the complete triangle set anchored at
// anchor — the same identity §5's whole-shell orientation rule reads, computed
// once directly over exact dyadics rather than through the full
// loftMassAccumulator (which also folds in the area/bounds bookkeeping this
// sign check does not need). It reads nothing loft-specific, so
// docs/tessellation-design.md §4's signed-volume audit runs on it too, for
// every payload class that assembles its own triangle set.
func OrientationSign(vertices []r3.Vec, triangles [][3]int, anchor r3.Vec) int {
	xa := proof.DyVec(anchor)
	sum := proof.DyZero()
	for _, t := range triangles {
		a := proof.DvSub(proof.DyVec(vertices[t[0]]), xa)
		b := proof.DvSub(proof.DyVec(vertices[t[1]]), xa)
		c := proof.DvSub(proof.DyVec(vertices[t[2]]), xa)
		sum = proof.DyAdd(sum, proof.DvDot(a, proof.DvCross(b, c)))
	}
	return sum.Sign()
}
