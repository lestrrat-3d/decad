package facetproof

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// referenceClosedFacetEdges is closedFacetEdges as it stood with one edge map
// for every mesh, kept verbatim for the differential tests.
func referenceClosedFacetEdges(ctx context.Context, tris [][3]int) ([]int, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
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
