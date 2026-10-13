package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/r3"
)

// tryPrismFacePatchUnion admits two exact rectangular prisms whose interiors
// are disjoint and whose boundaries share a positive-area patch on one face.
// It re-expresses the second box as a prism in the first box's frame, then
// lets the existing prism or stacked Union scenes split the shared patch.
func tryPrismFacePatchUnion(ctx context.Context, a, b *Body) (featurePayload, bool, error) {
	pa, okA := a.payload.(prismPayload)
	pb, okB := b.payload.(prismPayload)
	if !okA || !okB || pa.xform != pb.xform || pa.surfaceResult || pb.surfaceResult {
		return nil, false, nil
	}
	// A1's unequal-level BRep keeps planar box volumes exact. Try that
	// reference first, then the equal-level prism scene if neither frame
	// admits A1.
	for pass := range 2 {
		for _, pair := range [][2]prismPayload{{pa, pb}, {pb, pa}} {
			x, y := pair[0], pair[1]
			if x.frame.N() == y.frame.N() || x.frame.N() == y.frame.N().Scale(-1) {
				continue // class A already reads co-directional boxes.
			}
			xBox, ok := exactPrismBoxInFrame(x, x.frame)
			if !ok {
				continue
			}
			yBox, ok := exactPrismBoxInFrame(y, x.frame)
			if !ok || !prismplacement.BoxesShareFacePatch(xBox, yBox, 2) {
				continue
			}
			equalLevels := xBox.Lo[2] == yBox.Lo[2] && xBox.Hi[2] == yBox.Hi[2]
			if equalLevels != (pass == 1) {
				continue
			}
			yReframed := prismPayload{
				profile: prismplacement.BoxProfile(yBox), frame: x.frame, xform: x.xform,
				z0: yBox.Lo[2], z1: yBox.Hi[2],
			}
			var payload featurePayload
			var built bool
			var err error
			if equalLevels {
				payload, built, err = tryPrismBoolean(ctx, meshbool.OpUnion,
					&Body{payload: x}, &Body{payload: yReframed})
			} else {
				payload, built, err = tryStackedUnion(ctx, &Body{payload: x}, &Body{payload: yReframed})
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

// exactPrismBoxInFrame admits only an authenticated, undisplaced rectangular
// source prism and asks prismplacement.ExactBoxInFrame for its exact box in ref.
func exactPrismBoxInFrame(source prismPayload, ref r3.Frame) (prismplacement.Box, bool) {
	r, ok := axisRectangle(source.profile)
	if !ok || source.sectionDelta != 0 || source.z0Delta != 0 || source.z1Delta != 0 ||
		source.blendKind != "" || len(source.blendSegs) != 0 || source.z0 >= source.z1 {
		return prismplacement.Box{}, false
	}
	return prismplacement.ExactBoxInFrame(r.u0, r.u1, r.v0, r.v1, source.z0, source.z1, source.frame, ref)
}
