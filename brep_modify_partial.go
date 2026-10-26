package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/retainedblend"
	"github.com/lestrrat-3d/decad/internal/tangentchain"
)

// This file adapts WithTangentChain's expansion (agent-docs/modify-reach-design.md §5)
// to public handles and selectors. The analytic continuation decision lives in
// internal/tangentchain. An undecided continuation is SX2, never a chain stop:
// stopping would under-blend the caller's rule.

// expandTangentChain expands seeds (stage 2 of the reach gate order, §4) and
// returns the expanded set in body edge order. A faceted receiver is SX9
// before any expansion runs: its curves and surfaces never enter the oracle.
func expandTangentChain(ctx context.Context, b *Body, sel EdgeSelector, seeds []*Edge) ([]*Edge, error) {
	_, faceted := b.n.Payload.(facetedPayload)
	if faceted {
		// A retained body expands when every seed is a retained line or arc between
		// faces that carry exact tags (modify-reach-retained §5.3); the walk then
		// decides each junction from the tags.
		retained := len(seeds) > 0
		for _, e := range seeds {
			retained = retained && retainedblend.Chainable(edgeNode(e))
		}
		if retained {
			return tangentchain.Expand(ctx, tangentChainGraph{edges: b.Edges(), selector: sel, retained: true}, seeds)
		}
		return nil, fmt.Errorf(`%w: a faceted boolean result carries no analytic carrier to prove a tangent continuation over; this evaluator modifies no faceted body (modify-reach SX9)`, ErrUnsupported)
	}
	return tangentchain.Expand(ctx, tangentChainGraph{edges: b.Edges(), selector: sel}, seeds)
}

type tangentChainGraph struct {
	edges    []*Edge
	selector EdgeSelector
	// retained decides junctions from retained tags (retainedblend.Tangent).
	retained bool
}

func (g tangentChainGraph) Edges() []*Edge { return g.edges }

func (g tangentChainGraph) Endpoints(e *Edge) (*Vertex, *Vertex) {
	return vertexHandle(e.n.Start), vertexHandle(e.n.End)
}

func (g tangentChainGraph) Continue(e, c *Edge, v *Vertex) clearance.DegState {
	if g.retained {
		return retainedblend.Tangent(edgeNode(e), edgeNode(c), vertexNode(v))
	}
	return tangentchain.Continues(edgeNode(e), edgeNode(c), vertexNode(v))
}

func (g tangentChainGraph) Ambiguous(v *Vertex, proven, undecided int) error {
	return fmt.Errorf(`%w: the tangent chain branches or cannot be decided here: %d edges continue it and %d cannot be decided; this evaluator does not choose a branch or stop early (modify-reach SX2); selector %s, the chain through (%s)`,
		ErrUnsupported, proven, undecided, g.selector, renderVec(v.n.Position))
}
