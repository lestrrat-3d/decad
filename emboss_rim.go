package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// embossRimTol is how far, in millimetres, a face's plane may lie from where the
// relief's rim is when it is found in the result.
const embossRimTol = 1e-6

// filletEmbossRim rounds the rim of the relief that Emboss left in body, with a
// fillet of radius millimetres, retiring body. plane and outward are the
// embossed face's plane and outward normal, and height the relief's height.
func filletEmbossRim(ctx context.Context, body *Body, plane Plane, outward r3.Vec, height float64,
	mode EmbossMode, radius float64) (*Body, error) {
	edges := embossRimEdges(body, plane, outward, height, mode)
	if len(edges) == 0 {
		return nil, fmt.Errorf(`%w: the relief's rim is not an edge of the result, so it has nothing to fillet`, ErrUnsupported)
	}
	return Fillet(ctx, body, edgesQuery(edges), units.Millimeters(radius))
}

// embossRimEdges lists the edges of the relief's rim in body: for a raised
// relief every edge of the planar faces facing outward in the plane height out of
// the embossed face, and for an engraving the edges of the inner loops of the
// planar faces facing outward in the embossed face's own plane.
func embossRimEdges(body *Body, plane Plane, outward r3.Vec, height float64, mode EmbossMode) []*Edge {
	level := 0.0
	if mode == EmbossRaised {
		level = height
	}
	var edges []*Edge
	for _, f := range body.Faces() {
		p, ok := f.Surface().(Plane)
		if !ok {
			continue
		}
		offset := p.Frame.Origin().Sub(plane.Frame.Origin()).Dot(outward)
		if math.Abs(offset-level) > embossRimTol {
			continue
		}
		n, err := f.NormalAt(p.Frame.Origin())
		if err != nil || n.Value.Dot(outward) < 1-embossRimTol {
			continue
		}
		for _, loop := range f.Loops() {
			if mode == EmbossEngraved && loop.IsOuter() {
				continue
			}
			edges = append(edges, loop.Edges()...)
		}
	}
	return edges
}
