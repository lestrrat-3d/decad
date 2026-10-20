package brepmodify

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/breploop"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/modifyselect"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// selectedStraightEdgesShareVertex gates the restatement retry to straight
// edge selections whose topology has a common corner.
func selectedStraightEdgesShareVertex(edges []*topo.Edge) bool {
	for i, edge := range edges {
		if _, ok := edge.Curve.(surfacegeom.Line3); !ok {
			return false
		}
		for _, earlier := range edges[:i] {
			if edge.Start == earlier.Start || edge.Start == earlier.End ||
				edge.End == earlier.Start || edge.End == earlier.End {
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
func brepFilletPartialLoop(ctx context.Context, env topo.Env, bp breprecord.Payload, call Request,
	sel breploop.Sel, selected []bool) (*topo.Body, error) {
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
	lineCall.Edges = make([]*topo.Edge, len(ordinals))
	for i, ordinal := range ordinals {
		lineCall.Edges[i] = call.Edges[ordinal]
	}
	partialTerminals := terminals
	if len(lineCall.Edges) == len(call.Edges) {
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
	return r.buildPartialFillet(ctx, env, sel, selected, blends)
}

func (r *brepLoopRead) buildPartialFillet(ctx context.Context, env topo.Env, sel breploop.Sel,
	selected []bool, blends []*breprecord.EdgeBlend) (*topo.Body, error) {
	out, err := r.state.RewritePartialFillet(sel, selected, blends)
	if err != nil {
		return nil, err
	}
	body, err := breprecord.Build(ctx, env, out)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, ctxErr
		}
		return nil, fmt.Errorf(`%w; the partial-loop fillet's rewritten brep record; selector %s matched [%s]`,
			err, r.call.Sel, modifyselect.EdgesContext(r.call.Edges))
	}
	if body.Volume.Value.Base() <= 0 {
		return nil, fmt.Errorf(`%w: the partial-loop fillet encloses no volume`, decaderr.ErrDegenerate)
	}
	return body, nil
}

// brepChamferPartialLoop admits one or two adjacent selected straight walks
// on a planar face (breploop.State.AdmitPartialChamfer). The chain's two
// vertices end on unselected straight walks. Route E supplies the two
// end-face corner rewrites and its topology gates.
func brepChamferPartialLoop(ctx context.Context, env topo.Env, bp breprecord.Payload,
	call Request, sel breploop.Sel, selected []bool) (*topo.Body, error) {
	initial, err := newBrepLoopRead(ctx, bp, call)
	if err != nil {
		return nil, err
	}
	if err := initial.matchEdges(); err != nil {
		return nil, err
	}
	if err := initial.state.AdmitPartialChamfer(sel, selected, len(call.Edges)); err != nil {
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
	body, err := breprecord.Build(ctx, env, out)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, ctxErr
		}
		return nil, fmt.Errorf(`%w; the partial-loop chamfer's rewritten brep record`, err)
	}
	return body, nil
}
