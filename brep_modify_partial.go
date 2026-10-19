package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/breploop"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/modifyselect"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tangentchain"
)

// selectedStraightEdgesShareVertex gates the restatement retry to straight
// edge selections whose topology has a common corner.
func selectedStraightEdgesShareVertex(edges []*Edge) bool {
	for i, edge := range edges {
		if _, ok := edge.n.Curve.(surfacegeom.Line3); !ok {
			return false
		}
		for _, earlier := range edges[:i] {
			if edge.n.Start == earlier.n.Start || edge.n.Start == earlier.n.End ||
				edge.n.End == earlier.n.Start || edge.n.End == earlier.n.End {
				return true
			}
		}
	}
	return false
}

// brepFilletPartialLoop reads each selected edge through route E, then moves
// only the selected walls and offsets only their walks of the common cap
// loop (breploop.State.RewritePartialFillet). The band's patches close the
// record's open cap, side and terminal arcs during the body build. It stays
// in the root because it subsets the call's public edges for route E.
func brepFilletPartialLoop(ctx context.Context, d *Document, bp brepPayload, call brepModifyRequest,
	sel brepLoopSel, selected []bool) (*Body, error) {
	initial, err := newBrepLoopRead(ctx, bp, call)
	if err != nil {
		return nil, err
	}
	if err := initial.matchEdges(); err != nil {
		return nil, err
	}
	ordinals, terminals, err := initial.state.PartialFilletLines(sel, selected)
	if err != nil {
		return nil, err
	}
	lineCall := call
	lineCall.edges = make([]*Edge, len(ordinals))
	for i, ordinal := range ordinals {
		lineCall.edges[i] = call.edges[ordinal]
	}
	partialTerminals := terminals
	if len(lineCall.edges) == len(call.edges) {
		partialTerminals = nil
	}
	er, blends, err := prepareBrepEdgeBlends(ctx, bp, lineCall, true, partialTerminals)
	if err != nil {
		return nil, err
	}
	if partialTerminals == nil {
		breploop.MarkChainTerminals(blends)
	}
	for i, eb := range blends {
		eb.Ordinal = ordinals[i]
	}
	r, err := newBrepLoopRead(ctx, er.BP, call)
	if err != nil {
		return nil, err
	}
	if err := r.matchEdges(); err != nil {
		return nil, err
	}
	if err := r.state.RequirePartialRestated(sel, selected,
		`restating the straight walls changed the selected planar loop`); err != nil {
		return nil, err
	}
	return r.buildPartialFillet(ctx, d, sel, selected, blends)
}

func (r *brepLoopRead) buildPartialFillet(ctx context.Context, d *Document, sel brepLoopSel,
	selected []bool, blends []*brepEdgeBlend) (*Body, error) {
	out, err := r.state.RewritePartialFillet(sel, selected, blends)
	if err != nil {
		return nil, err
	}
	body, err := bodyResult(breprecord.Build(ctx, envOf(documentNode(d), d.nextProducerID()), out))
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, ctxErr
		}
		return nil, fmt.Errorf(`%w; the partial-loop fillet's rewritten brep record; selector %s matched [%s]`,
			err, r.call.sel, modifyselect.EdgesContext(edgeNodes(r.call.edges)))
	}
	if body.n.Volume.Value.Base() <= 0 {
		return nil, fmt.Errorf(`%w: the partial-loop fillet encloses no volume`, ErrDegenerate)
	}
	return body, nil
}

// This file adapts WithTangentChain's expansion (docs/modify-reach-design.md §5)
// to public handles and selectors. The analytic continuation decision lives in
// internal/tangentchain. An undecided continuation is SX2, never a chain stop:
// stopping would under-blend the caller's rule.

// expandTangentChain expands seeds (stage 2 of the reach gate order, §4) and
// returns the expanded set in body edge order. A faceted receiver is SX9
// before any expansion runs: its curves and surfaces never enter the oracle.
func expandTangentChain(ctx context.Context, b *Body, sel EdgeSelector, seeds []*Edge) ([]*Edge, error) {
	if _, ok := b.n.Payload.(facetedPayload); ok {
		return nil, fmt.Errorf(`%w: a faceted boolean result carries no analytic carrier to prove a tangent continuation over; this evaluator modifies no faceted body (modify-reach SX9)`, ErrUnsupported)
	}
	return tangentchain.Expand(ctx, tangentChainGraph{edges: b.Edges(), selector: sel}, seeds)
}

type tangentChainGraph struct {
	edges    []*Edge
	selector EdgeSelector
}

func (g tangentChainGraph) Edges() []*Edge { return g.edges }

func (g tangentChainGraph) Endpoints(e *Edge) (*Vertex, *Vertex) {
	return vertexHandle(e.n.Start), vertexHandle(e.n.End)
}

func (g tangentChainGraph) Continue(e, c *Edge, v *Vertex) clearance.DegState {
	return tangentchain.Continues(edgeNode(e), edgeNode(c), vertexNode(v))
}

func (g tangentChainGraph) Ambiguous(v *Vertex, proven, undecided int) error {
	return fmt.Errorf(`%w: the tangent chain branches or cannot be decided here: %d edges continue it and %d cannot be decided; this evaluator does not choose a branch or stop early (modify-reach SX2); selector %s, the chain through (%s)`,
		ErrUnsupported, proven, undecided, g.selector, renderVec(v.n.Position))
}

// brepChamferPartialLoop admits one or two adjacent selected straight walks
// on a planar face (breploop.State.AdmitPartialChamfer). The chain's two
// vertices end on unselected straight walks. Route E supplies the two
// end-face corner rewrites and its topology gates.
func brepChamferPartialLoop(ctx context.Context, d *Document, bp brepPayload,
	call brepModifyRequest, sel brepLoopSel, selected []bool) (*Body, error) {
	initial, err := newBrepLoopRead(ctx, bp, call)
	if err != nil {
		return nil, err
	}
	if err := initial.matchEdges(); err != nil {
		return nil, err
	}
	if err := initial.state.AdmitPartialChamfer(sel, selected, len(call.edges)); err != nil {
		return nil, err
	}
	er, blends, err := prepareBrepEdgeBlends(ctx, bp, call, true, nil)
	if err != nil {
		return nil, err
	}
	breploop.MarkChainTerminals(blends)
	r, err := newBrepLoopRead(ctx, er.BP, call)
	if err != nil {
		return nil, err
	}
	if err := r.matchEdges(); err != nil {
		return nil, err
	}
	if err := r.state.RequirePartialRestated(sel, selected,
		`restating the straight walls changed the selected partial loop`); err != nil {
		return nil, err
	}
	out, err := r.state.RewritePartialChamfer(sel, selected, blends)
	if err != nil {
		return nil, err
	}
	body, err := bodyResult(breprecord.Build(ctx, envOf(documentNode(d), d.nextProducerID()), out))
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, ctxErr
		}
		return nil, fmt.Errorf(`%w; the partial-loop chamfer's rewritten brep record`, err)
	}
	return body, nil
}
