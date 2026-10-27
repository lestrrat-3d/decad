package facetproof

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// closedFacetEdges checks the audit's directed-edge counts and records each
// reverse owner in its local edge slot. One undirected key replaces separate
// count and adjacency maps. A signed edge ordinal records the first direction;
// zero marks an already paired edge, so a third occurrence still fails.
//
// A mesh whose corners all lie in [0, 3·len(tris)) takes the grouped form,
// closedFacetEdgesGrouped, which makes the same polls and returns the same
// result; any other mesh keeps the map.
func closedFacetEdges(ctx context.Context, tris [][3]int) ([]int, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if vertices, ok := groupedVertexRange(tris); ok {
		return closedFacetEdgesGrouped(ctx, tris, vertices)
	}
	owner := make(map[[2]int]int, 3*len(tris)/2)
	neighbors := make([]int, 3*len(tris))
	closed := true
	for i, tri := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		for edge := range 3 {
			a, b := tri[edge], tri[(edge+1)%3]
			ordinal := 3*i + edge
			code := ordinal + 1
			if a > b {
				a, b = b, a
				code = -code
			}
			key := [2]int{a, b}
			first, seen := owner[key]
			if a == b {
				// The count rule accepts one self-edge: its reverse is itself.
				// Adjacency excludes the facet itself, as the general builder does.
				closed = closed && !seen
				owner[key] = 0
				continue
			}
			if !seen {
				owner[key] = code
				continue
			}
			if first == 0 || (first < 0) == (code < 0) {
				closed = false
				continue
			}
			if first < 0 {
				first = -first
			}
			first--
			neighbors[first] = i
			neighbors[ordinal] = first / 3
			owner[key] = 0
		}
	}
	for _, first := range owner {
		if first != 0 {
			closed = false
			break
		}
	}
	if !closed {
		return nil, fmt.Errorf(`%w: the held boundary does not close`, decaderr.ErrBooleanFailed)
	}
	return neighbors, nil
}

// groupedVertexRange reports one past the largest corner of tris, and whether
// every corner lies in [0, 3·len(tris)): the range closedFacetEdgesGrouped's
// per-vertex storage may span. A closed mesh needs no more vertices than
// corners, so only unused indices push a mesh past it.
func groupedVertexRange(tris [][3]int) (int, bool) {
	limit := 3 * len(tris)
	vertices := 0
	for _, t := range tris {
		for _, c := range t {
			if c < 0 || c >= limit {
				return 0, false
			}
			vertices = max(vertices, c+1)
		}
	}
	return vertices, true
}

// closedFacetEdgesGrouped is closedFacetEdges over corners in [0, vertices),
// with the edge map replaced by grouping. Every occurrence of the undirected
// edge {a, b}, a ≤ b, lands in group a of a stable counting sort, so the
// group visits that edge's occurrences in ordinal order — the order the map
// loop visits them — and owner[b] runs the map entry's state machine for it:
// first direction, paired (0), or failed. Distinct edges never share state,
// so the order the groups visit different edges changes neither the
// neighbors written nor the closure verdict. The map's final scan for an
// unpaired entry is the second pass over each group. The polls stay in the
// facet-order counting pass at the map loop's positions; the passes after it
// are linear in the facets and vertices, as the map's final scan was.
//
// Every access reads or writes one int: under the race detector a whole
// array or struct copy costs a range check, several times a word's.
func closedFacetEdgesGrouped(ctx context.Context, tris [][3]int, vertices int) ([]int, error) {
	// The counting sort: bound[a+2] counts group a, the prefix sums turn
	// bound[a+1] into group a's start, and placing the facets in order moves
	// it to the group's end. Group a is then order[bound[a]:bound[a+1]], in
	// increasing ordinal order.
	bound := make([]int, vertices+2)
	for i := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		a, b, c := tris[i][0], tris[i][1], tris[i][2]
		bound[min(a, b)+2]++
		bound[min(b, c)+2]++
		bound[min(c, a)+2]++
	}
	for v := range vertices {
		bound[v+1] += bound[v]
	}
	n := 3 * len(tris)
	order := make([]int, n)
	for i := range tris {
		a, b, c := tris[i][0], tris[i][1], tris[i][2]
		lo := min(a, b) + 1
		order[bound[lo]] = 3 * i
		bound[lo]++
		lo = min(b, c) + 1
		order[bound[lo]] = 3*i + 1
		bound[lo]++
		lo = min(c, a) + 1
		order[bound[lo]] = 3*i + 2
		bound[lo]++
	}
	// owner[b] is the map entry for {a, b} while group a runs; seen[b] ==
	// a+1 marks it as written in group a, so no group clears another's.
	seen := make([]int, vertices)
	owner := make([]int, vertices)
	neighbors := make([]int, n)
	closed := true
	for a := range vertices {
		group := order[bound[a]:bound[a+1]]
		for _, ordinal := range group {
			i, edge := ordinal/3, ordinal%3
			u, v := tris[i][edge], tris[i][(edge+1)%3]
			b := u + v - a
			code := ordinal + 1
			if u > v {
				code = -code
			}
			had := seen[b] == a+1
			seen[b] = a + 1
			first := owner[b]
			if u == v {
				// The count rule accepts one self-edge: its reverse is itself.
				closed = closed && !had
				owner[b] = 0
				continue
			}
			if !had {
				owner[b] = code
				continue
			}
			if first == 0 || (first < 0) == (code < 0) {
				closed = false
				continue
			}
			if first < 0 {
				first = -first
			}
			first--
			neighbors[first] = i
			neighbors[ordinal] = first / 3
			owner[b] = 0
		}
		for _, ordinal := range group {
			i, edge := ordinal/3, ordinal%3
			if owner[tris[i][edge]+tris[i][(edge+1)%3]-a] != 0 {
				closed = false
			}
		}
	}
	if !closed {
		return nil, fmt.Errorf(`%w: the held boundary does not close`, decaderr.ErrBooleanFailed)
	}
	return neighbors, nil
}

// closedFacetAdjacency assembles rows after the audit lifts its coordinates,
// preserving that phase's precedence over adjacency cancellation checks.
func closedFacetAdjacency(ctx context.Context, tris [][3]int, neighbors []int) ([][]int, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	adj := make([][]int, len(tris))
	for i, tri := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		// A closed triangle with a repeated vertex has one self-edge and an
		// opposite pair owned by that same facet. All three neighbors are self,
		// which FacetAdjacencyContext excludes. A third owner would fail above.
		if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
			continue
		}
		adj[i] = neighbors[3*i : 3*i+3 : 3*i+3]
	}
	return adj, nil
}
