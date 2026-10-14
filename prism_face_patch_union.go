package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// tryPrismFacePatchUnion admits two exact rectangular prisms whose interiors
// are disjoint and whose boundaries share a positive-area patch on one face.
// It re-expresses the second box as a prism in the first box's frame, then
// lets the existing prism or stacked Union scenes split the shared patch.
func tryPrismFacePatchUnion(ctx context.Context, a, b *Body) (topo.Payload, bool, error) {
	pa, okA := a.n.Payload.(prismPayload)
	pb, okB := b.n.Payload.(prismPayload)
	if !okA || !okB || pa.Xform != pb.Xform || pa.SurfaceResult || pb.SurfaceResult {
		return nil, false, nil
	}
	// A1's unequal-level BRep keeps planar box volumes exact. Try that
	// reference first, then the equal-level prism scene if neither frame
	// admits A1.
	for pass := range 2 {
		for _, pair := range [][2]prismPayload{{pa, pb}, {pb, pa}} {
			x, y := pair[0], pair[1]
			if x.Frame.N() == y.Frame.N() || x.Frame.N() == y.Frame.N().Scale(-1) {
				continue // class A already reads co-directional boxes.
			}
			xBox, ok := x.ExactBoxInFrame(x.Frame)
			if !ok {
				continue
			}
			yBox, ok := y.ExactBoxInFrame(x.Frame)
			if !ok || !prismplacement.BoxesShareFacePatch(xBox, yBox, 2) {
				continue
			}
			equalLevels := xBox.Lo[2] == yBox.Lo[2] && xBox.Hi[2] == yBox.Hi[2]
			if equalLevels != (pass == 1) {
				continue
			}
			yReframed := prismrecord.Payload{
				Profile: prismplacement.BoxProfile(yBox), Frame: x.Frame, Xform: x.Xform,
				Z0: yBox.Lo[2], Z1: yBox.Hi[2],
			}
			var payload topo.Payload
			var built bool
			var err error
			if equalLevels {
				payload, built, err = tryPrismBoolean(ctx, meshbool.OpUnion,
					bornBody(&Body{n: topo.Body{Payload: x}}), bornBody(&Body{n: topo.Body{Payload: yReframed}}))
			} else {
				payload, built, err = tryStackedUnion(ctx, bornBody(&Body{n: topo.Body{Payload: x}}), bornBody(&Body{n: topo.Body{Payload: yReframed}}))
			}
			if err != nil || built {
				return payload, built, err
			}
			// A silent miss in this frame may be admitted with the operands
			// reversed: that changes which in-plane axis A1 splits into slabs.
		}
	}
	return nil, false, nil
}
