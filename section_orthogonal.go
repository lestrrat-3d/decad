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
	if len(fp.Tris) == 0 {
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
	// Each loop in the plane's frame, with its sense in the world axes (ia, iv): positive area is an outer loop.
	type ring struct {
		points []planePoint
		world  [][2]float64
		area   float64
	}
	var outers, holes []ring
	for _, loop := range loops {
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
		r := ring{points: local, world: loop, area: area}
		if area > 0 {
			outers = append(outers, r)
		} else {
			holes = append(holes, r)
		}
	}
	if len(outers) == 0 {
		return nil, nil, false
	}
	segments := func(points []planePoint) loopRecord {
		out := make([]curveSegment, len(points))
		for i, p := range points {
			out[i] = lineSeg{Start: p, End: points[(i+1)%len(points)], TStart: 0, TEnd: 1}
		}
		return loopRecord{Segments: out}
	}
	records := make([]profileRecord, len(outers))
	for i, o := range outers {
		records[i].Outer = segments(o.points)
	}
	// A hole belongs to the smallest outer loop that contains it: its first corner lies inside that loop.
	for _, h := range holes {
		best := -1
		for i, o := range outers {
			if pointInLoop(h.world[0], o.world) && (best < 0 || o.area < outers[best].area) {
				best = i
			}
		}
		if best < 0 {
			return nil, nil, false
		}
		records[best].Holes = append(records[best].Holes, segments(h.points))
	}
	s, ps, err := sectionclip.ReconstructExactSections(ctx, records, plane)
	if err != nil {
		return nil, nil, false
	}
	return s, ps, true
}

// pointInLoop reports whether p lies strictly inside the polygon loop, by counting the crossings of a ray to +x. A
// corner of the loop being tested lies on no other loop's boundary, since the cells of the cut meet only along sides.
func pointInLoop(p [2]float64, loop [][2]float64) bool {
	in := false
	for i, a := range loop {
		b := loop[(i+1)%len(loop)]
		if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < a[0]+(p[1]-a[1])*(b[0]-a[0])/(b[1]-a[1]) {
			in = !in
		}
	}
	return in
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
