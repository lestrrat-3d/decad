package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"

	"github.com/lestrrat-3d/decad/internal/capedge"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// tryRoundedBrepCapEdgeFillet handles the documented P8 source whose front
// top edge ends on two radius-3 cylindrical corner walls. Route E cannot
// restate those walls as planes (SB7). The complete source record and the
// selected edge must match the exact construction below; no coordinate
// tolerance admits a nearby boolean or a displaced face. The bore reaches
// z=13, while each admitted cutter starts at z>=19, and its y reach is at
// most 1 against a body whose back is y=20. Thus it can touch the front
// rounded wall and cap only. The mesh boolean proves its actual contacts,
// occupied volume, and closure before the caller's body is retired.
func tryRoundedBrepCapEdgeFillet(ctx context.Context, body *Body, bp brepPayload,
	edges []*Edge, radius, radiusDelta float64) (*Body, bool, error) {
	if !breprecord.P8QuarterRadius(radius) || radiusDelta != 0 || len(edges) != 1 || !breprecord.IsP8Rounded(bp) {
		return nil, false, nil
	}
	edge := edges[0]
	if _, ok := edge.n.Curve.(surfacegeom.Line3); !ok || edge.n.Start == nil || edge.n.End == nil ||
		!capedge.P8TopFrontEdge(edge.n.Start.Position, edge.n.End.Position) {
		return nil, false, nil
	}
	// The source integers and admitted quarter-millimeter radii are binary exact.
	if !(10+3 < 20-radius && radius < 20) {
		return nil, false, nil
	}
	frame, err := r3.NewFrame(r3.NewVec(20, 0, 20), r3.NewVec(0, -1, 0), r3.NewVec(0, 0, 1))
	if err != nil {
		return nil, true, fmt.Errorf(`%w: the rounded cap-edge cutter has no frame: %v`, ErrUnsupported, err)
	}
	d := documentHandle(body.n.Doc)
	ref := d.nextProducerID()
	tool, err := bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), prismrecord.Payload{
		Profile: capedge.QuarterCutter(radius), Frame: frame, Z0: -42, Z1: 42, Xform: r3.Identity(),
	}, freeform.NewFreeformWork()))
	if err != nil {
		return nil, true, err
	}
	// The ordinary pair tolerance proves the cut but leaves faceted Area
	// coarser than Verify's default limit. The exact P8 source has no
	// restating operand, so 1/64 chords both analytic operands more finely.
	// The same contact, stitch, and occupied-volume proofs decide the result.
	eval, err := evaluateBooleanMeshes(ctx, meshbool.OpCut, body, tool, pairMeshes{}, 1.0/64)
	if err != nil {
		return nil, true, asBooleanError(meshbool.OpCut, err)
	}
	out, err := bodyResult(facetedrecord.BuildWithProof(ctx, envOf(documentNode(d), ref+1), eval.payload, eval.audit, measurementToInternal(eval.volume), eval.volumeRat))
	if err != nil {
		return nil, true, asBooleanError(meshbool.OpCut, err)
	}
	if err := ctx.Err(); err != nil {
		return nil, true, err
	}
	d.commitSpan(out, 2, body)
	return out, true, nil
}
