package stepbrep

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/stepread"
	"github.com/lestrrat-3d/r3"
)

// sphereFace triangulates an ADVANCED_FACE on a SPHERICAL_SURFACE that is a cap:
// the part of the sphere on one side of a circle, bounded by one loop whose
// points all lie on a circle of the sphere. The cap is cut into rings of latitude
// between its boundary and its pole, each carrying the boundary's own azimuths, so
// the boundary's points are the outermost ring's and the cap meets its neighbours
// along the same chords. A sphere face with any other bound, a zone between two
// circles or a whole sphere with its seam, is refused.
func (r *reader) sphereFace(f, surface stepread.Entity, flipped bool) error {
	if len(surface.Params) < 3 {
		return fmt.Errorf("%w: #%d is not a %s", ErrMalformed, surface.ID, surface.Name)
	}
	frame, err := r.placement(surface.Params[1])
	if err != nil {
		return err
	}
	rad, ok := surface.Params[2].(stepread.Number)
	radius := float64(rad)
	if !ok || !(radius > 0) || math.IsInf(radius, 0) {
		return fmt.Errorf("%w: #%d has no positive radius", ErrMalformed, surface.ID)
	}
	bounds, _ := f.Params[1].(stepread.List)
	if len(bounds) != 1 {
		return fmt.Errorf("%w: face #%d is a sphere face with %d bounds, and only a cap with one circular bound is read",
			decaderr.ErrUnsupported, f.ID, len(bounds))
	}
	ring, _, err := r.ring(bounds[0])
	if err != nil {
		return err
	}
	if len(ring) < 3 {
		return fmt.Errorf("%w: a bound of face #%d has %d points", ErrMalformed, f.ID, len(ring))
	}
	eps := 1e-7 * math.Max(1, radius)
	for _, p := range ring {
		if math.Abs(p.Sub(frame.o).Len()-radius) > eps {
			return fmt.Errorf("%w: a boundary point of face #%d lies off its sphere", ErrMalformed, f.ID)
		}
	}
	// The ring's own centre and the normal of its plane, by Newell's method, which is
	// the right-hand axis of the walk. A ring walked counter-clockwise about its axis
	// seen from outside the sphere has the cap round that axis on its left.
	var centre r3.Vec
	for _, p := range ring {
		centre = centre.Add(p)
	}
	centre = centre.Scale(1 / float64(len(ring)))
	var sum r3.Vec
	for i := range ring {
		sum = sum.Add(ring[i].Sub(centre).Cross(ring[(i+1)%len(ring)].Sub(centre)))
	}
	axis, ok := sum.Normalize()
	if !ok {
		return fmt.Errorf("%w: the boundary of face #%d encloses no area", ErrMalformed, f.ID)
	}
	outward := 1.0
	if sense, _ := f.Params[3].(stepread.Enum); sense == "F" {
		// The face's normal points into the sphere, and the walk is counter-clockwise about
		// that normal, so the cap lies round the opposite end of the axis.
		axis = axis.Scale(-1)
		outward = -1
	}
	height := 0.0
	for _, p := range ring {
		height += p.Sub(frame.o).Dot(axis)
	}
	height /= float64(len(ring))
	for _, p := range ring {
		if math.Abs(p.Sub(frame.o).Dot(axis)-height) > eps {
			return fmt.Errorf("%w: the boundary of face #%d is not a circle of latitude of its sphere, and only a cap is read",
				decaderr.ErrUnsupported, f.ID)
		}
	}
	theta := math.Acos(math.Max(-1, math.Min(1, height/radius)))
	if !(theta > 1e-9 && theta < math.Pi-1e-9) {
		return fmt.Errorf("%w: the boundary of face #%d is a point of its sphere", ErrMalformed, f.ID)
	}
	e1, ok := perpendicular(axis)
	if !ok {
		return fmt.Errorf("%w: a sphere face has a zero axis", ErrMalformed)
	}
	e2 := axis.Cross(e1)
	azimuth := make([]float64, len(ring))
	for i, p := range ring {
		d := p.Sub(frame.o)
		azimuth[i] = math.Atan2(d.Dot(e2), d.Dot(e1))
	}
	// The meridian step matches the boundary's own angular step round the axis.
	levels := max(1, int(math.Ceil(theta*float64(len(ring))/(2*math.Pi))))
	// rings[k] is the ring at polar angle theta·(levels−k)/levels, rings[0] the boundary.
	rings := make([][]r3.Vec, levels)
	rings[0] = ring
	for k := 1; k < levels; k++ {
		psi := theta * float64(levels-k) / float64(levels)
		rings[k] = make([]r3.Vec, len(ring))
		for i := range ring {
			dir := e1.Scale(math.Cos(azimuth[i])).Add(e2.Scale(math.Sin(azimuth[i])))
			rings[k][i] = frame.o.Add(axis.Scale(radius * math.Cos(psi))).Add(dir.Scale(radius * math.Sin(psi)))
		}
	}
	pole := frame.o.Add(axis.Scale(radius))
	var triangles [][3]r3.Vec
	for k := range levels {
		for i := range ring {
			j := (i + 1) % len(ring)
			if k+1 < levels {
				a, b, c, d := rings[k][i], rings[k][j], rings[k+1][j], rings[k+1][i]
				triangles = append(triangles, [3]r3.Vec{a, b, c}, [3]r3.Vec{a, c, d})
			} else {
				triangles = append(triangles, [3]r3.Vec{rings[k][i], rings[k][j], pole})
			}
		}
	}
	for _, t := range triangles {
		if len(r.tris) >= r.limit {
			return fmt.Errorf("%w: more than %d triangles", ErrTooLarge, r.limit)
		}
		// The triangle faces the way the surface does: along the radius, or against it
		// when the face's sense is reversed.
		centroid := t[0].Add(t[1]).Add(t[2]).Scale(1.0 / 3)
		normal := t[1].Sub(t[0]).Cross(t[2].Sub(t[0]))
		a, b, c := r.weld(t[0]), r.weld(t[1]), r.weld(t[2])
		if normal.Dot(centroid.Sub(frame.o))*outward < 0 {
			b, c = c, b
		}
		if flipped {
			b, c = c, b
		}
		r.tris = append(r.tris, [3]int{a, b, c})
	}
	return nil
}
