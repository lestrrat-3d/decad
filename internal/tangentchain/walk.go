// Package tangentchain walks edges whose endpoint continuations are proven.
package tangentchain

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/clearance"
)

// Graph supplies the ordered edges and the continuation decision at each vertex.
// A zero vertex denotes an absent endpoint.
type Graph[E comparable, V comparable] interface {
	Edges() []E
	Endpoints(E) (V, V)
	Continue(E, E, V) clearance.DegState
	Ambiguous(V, int, int) error
}

// Step returns the sole proven candidate, if any, and counts decisions that
// prevent a unique continuation. Candidate order never breaks a tie.
func Step(states []clearance.DegState) (next, proven, undecided int) {
	next = -1
	for i, state := range states {
		switch state {
		case clearance.DegYes:
			proven++
			next = i
		case clearance.DegUnknown:
			undecided++
		}
	}
	return next, proven, undecided
}

// Expand visits each proven continuation once, then returns selected edges in
// the graph's original order. An undecided candidate stops with an error.
func Expand[E comparable, V comparable](ctx context.Context, graph Graph[E, V], seeds []E) ([]E, error) {
	edges := graph.Edges()
	incident := make(map[V][]E)
	var zero V
	for _, edge := range edges {
		start, end := graph.Endpoints(edge)
		if start != zero {
			incident[start] = append(incident[start], edge)
		}
		if end != zero && end != start {
			incident[end] = append(incident[end], edge)
		}
	}
	in := make(map[E]struct{}, len(seeds))
	queue := make([]E, 0, len(seeds))
	for _, edge := range seeds {
		if _, ok := in[edge]; ok {
			continue
		}
		in[edge] = struct{}{}
		queue = append(queue, edge)
	}
	for len(queue) > 0 {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		edge := queue[0]
		queue = queue[1:]
		start, end := graph.Endpoints(edge)
		// A closed curve has no endpoint corner to continue through.
		if start == zero || end == zero || start == end {
			continue
		}
		for _, vertex := range [2]V{start, end} {
			candidates := incident[vertex]
			states := make([]clearance.DegState, len(candidates))
			for i, candidate := range candidates {
				if candidate == edge {
					states[i] = clearance.DegNo
					continue
				}
				states[i] = graph.Continue(edge, candidate, vertex)
			}
			next, proven, undecided := Step(states)
			if proven > 1 || undecided > 0 {
				return nil, graph.Ambiguous(vertex, proven, undecided)
			}
			if next < 0 {
				continue
			}
			candidate := candidates[next]
			if _, ok := in[candidate]; ok {
				continue
			}
			in[candidate] = struct{}{}
			queue = append(queue, candidate)
		}
	}
	out := make([]E, 0, len(in))
	for _, edge := range edges {
		if _, ok := in[edge]; ok {
			out = append(out, edge)
		}
	}
	return out, nil
}
