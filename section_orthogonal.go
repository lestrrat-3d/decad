package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/orthoshell"
	"github.com/lestrrat-3d/decad/internal/sectionclip"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// orthogonalSection sections a faceted solid every face of which squares to a world axis by a plane square to
// an axis (agent-docs/orthogonal-section-design.md): the cut is read from the grid of the solid's coordinates, so
// its corners are coordinates of the solid. ok is false when the route does not apply or declines, and the caller
// takes its other routes: the receiver is placed or holds a vertex bound, the plane's frame is not made of
// world axes, the plane lies on a grid line or outside the solid, the cut is not one region, or a corner does
// not map back into the plane's frame exactly.
func orthogonalSection(ctx context.Context, fp facetedPayload, plane r3.Frame) (*sketch.Sketch, []*sketch.Profile, bool) {
	if len(fp.Tris) == 0 || fp.Xform != r3.Identity() {
		return nil, nil, false
	}
	for _, beta := range fp.VertexBound {
		if beta != 0 {
			return nil, nil, false
		}
	}
	ia, su, okU := unitAxis(plane.U())
	iv, sv, okV := unitAxis(plane.V())
	axis, _, okN := unitAxis(plane.N())
	if !okU || !okV || !okN || ia == iv || ia == axis || iv == axis {
		return nil, nil, false
	}
	origin := [3]float64{plane.Origin().X, plane.Origin().Y, plane.Origin().Z}
	loops, err := orthoshell.SectionLoops(fp.Verts, fp.Tris, axis, origin[axis], ia, iv)
	if err != nil {
		return nil, nil, false
	}
	var profile profileRecord
	outers := 0
	for _, loop := range loops {
		// The grid loops run counter-clockwise round material in (ia, iv): an outer loop has positive area.
		area := 0.0
		for i, p := range loop {
			q := loop[(i+1)%len(loop)]
			area += p[0]*q[1] - q[0]*p[1]
		}
		local := make([]planePoint, len(loop))
		for i, p := range loop {
			u, v := su*(p[0]-origin[ia]), sv*(p[1]-origin[iv])
			// The point must come back to the world coordinate from the frame it is stated in.
			if origin[ia]+su*u != p[0] || origin[iv]+sv*v != p[1] {
				return nil, nil, false
			}
			local[i] = planePoint{U: u, V: v}
		}
		if su*sv < 0 {
			for i, j := 0, len(local)-1; i < j; i, j = i+1, j-1 {
				local[i], local[j] = local[j], local[i]
			}
		}
		segments := make([]curveSegment, len(local))
		for i, p := range local {
			segments[i] = lineSeg{Start: p, End: local[(i+1)%len(local)], TStart: 0, TEnd: 1}
		}
		if area > 0 {
			outers++
			profile.Outer = loopRecord{Segments: segments}
		} else {
			profile.Holes = append(profile.Holes, loopRecord{Segments: segments})
		}
	}
	if outers != 1 {
		return nil, nil, false
	}
	s, ps, err := sectionclip.ReconstructExactSection(ctx, profile, plane)
	if err != nil {
		return nil, nil, false
	}
	return s, ps, true
}

// unitAxis reports the world axis a vector is, exactly, and its sign.
func unitAxis(v r3.Vec) (int, float64, bool) {
	c := [3]float64{v.X, v.Y, v.Z}
	axis, sign := -1, 0.0
	for i, x := range c {
		switch x {
		case 0:
		case 1, -1:
			if axis >= 0 {
				return 0, 0, false
			}
			axis, sign = i, x
		default:
			return 0, 0, false
		}
	}
	return axis, sign, axis >= 0
}
