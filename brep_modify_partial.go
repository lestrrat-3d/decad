package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/retainedblend"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tangentchain"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// This file adapts TangentChain's expansion (agent-docs/modify-design.md Part R §5)
// to public handles and selectors. The analytic continuation decision lives in
// internal/tangentchain. An undecided continuation is SX2, never a chain stop:
// stopping would under-blend the caller's rule.

// expandTangentChain expands seeds (stage 2 of the reach gate order, §4) and
// returns the expanded set in body edge order. A faceted receiver is SX9
// before any expansion runs: its curves and surfaces never enter the oracle.
func expandTangentChain(ctx context.Context, b *Body, sel EdgeSelector, seeds []*Edge) ([]*Edge, error) {
	fp, faceted := b.n.Payload.(facetedPayload)
	if faceted {
		// A retained body expands when every seed is a retained line or arc between
		// faces that carry exact tags (modify-reach-retained §5.3); the walk then
		// decides each junction from the tags.
		retained := len(seeds) > 0
		for _, e := range seeds {
			retained = retained && retainedblend.Chainable(edgeNode(e))
		}
		if retained {
			g := tangentChainGraph{edges: b.Edges(), selector: sel, retained: true,
				held:   tangentchain.HeldMesh{Verts: fp.Verts, Bounds: fp.VertexBound, Tris: fp.Tris, FaceOf: fp.FaceOf},
				faceIx: map[*topo.Face]int{}}
			for i, f := range topobuild.Faces(&b.n) {
				g.faceIx[f] = i
			}
			return tangentchain.Expand(ctx, g, seeds)
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
	// held and faceIx are a retained body's held mesh and the index of each face in its FaceOf order. This adapter
	// reads them from the body's payload because a Boolean's chord-chain edge has no direction of its own.
	held   tangentchain.HeldMesh
	faceIx map[*topo.Face]int
}

func (g tangentChainGraph) Edges() []*Edge { return g.edges }

func (g tangentChainGraph) Endpoints(e *Edge) (*Vertex, *Vertex) {
	return vertexHandle(e.n.Start), vertexHandle(e.n.End)
}

func (g tangentChainGraph) Continue(e, c *Edge, v *Vertex) clearance.DegState {
	if g.retained {
		// A candidate whose faces the tags cannot read, such as an earlier bevel's edge, is no continuation when its
		// direction at v is proven apart from e's: the oracle's "no" needs no tags.
		en, cn, vn := edgeNode(e), edgeNode(c), vertexNode(v)
		state := retainedblend.Tangent(en, cn, vn)
		if state != clearance.DegUnknown {
			return state
		}
		if tangentchain.Apart(en, cn, vn) {
			return clearance.DegNo
		}
		// A chord chain's direction at v is read from the held chord there.
		if chain, ok := cn.Curve.(surfacegeom.FacetedCurve); ok && len(cn.Faces) == 2 {
			fa, okA := g.faceIx[cn.Faces[0]]
			fb, okB := g.faceIx[cn.Faces[1]]
			if okA && okB && tangentchain.HeldApart(g.held, fa, fb, chain.Bound.Base(), en, vn) {
				return clearance.DegNo
			}
		}
		return state
	}
	return tangentchain.Continues(edgeNode(e), edgeNode(c), vertexNode(v))
}

func (g tangentChainGraph) Ambiguous(v *Vertex, proven, undecided int) error {
	return fmt.Errorf(`%w: the tangent chain branches or cannot be decided here: %d edges continue it and %d cannot be decided; this evaluator does not choose a branch or stop early (modify-reach SX2); selector %s, the chain through (%s)`,
		ErrUnsupported, proven, undecided, g.selector, renderVec(v.n.Position))
}
