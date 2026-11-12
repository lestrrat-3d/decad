package explodeblock

import "math"

// p2 is a point in the plane perpendicular to the projection direction.
type p2 struct{ x, y float64 }

// tri is one projected triangle: counter-clockwise corners, the height of
// each corner along the direction, and the 2D bounding box.
type tri struct {
	v                      [3]p2
	z                      [3]float64
	minX, maxX, minY, maxY float64
	minZ, maxZ             float64
}

// shifted returns t translated by (du, dv, dw).
func (t *tri) shifted(du, dv, dw float64) tri {
	r := *t
	for i := range 3 {
		r.v[i].x += du
		r.v[i].y += dv
		r.z[i] += dw
	}
	r.minX += du
	r.maxX += du
	r.minY += dv
	r.maxY += dv
	r.minZ += dw
	r.maxZ += dw
	return r
}

// affine is z = ax*x + ay*y + c.
type affine struct{ ax, ay, c float64 }

func (a affine) at(q p2) float64 { return a.ax*q.x + a.ay*q.y + a.c }

// plane returns the height function of a non-degenerate projected triangle.
func (t *tri) plane() affine {
	x1, y1 := t.v[1].x-t.v[0].x, t.v[1].y-t.v[0].y
	x2, y2 := t.v[2].x-t.v[0].x, t.v[2].y-t.v[0].y
	z1, z2 := t.z[1]-t.z[0], t.z[2]-t.z[0]
	det := x1*y2 - x2*y1
	ax := (z1*y2 - z2*y1) / det
	ay := (x1*z2 - x2*z1) / det
	return affine{ax, ay, t.z[0] - ax*t.v[0].x - ay*t.v[0].y}
}

// clipper holds the buffers the clipping writes into, so that testing one pair
// of meshes allocates nothing once the buffers have grown. The polygon a clip
// returns lives in one of the buffers and stays valid until the clip after next.
type clipper struct {
	buf   [2][]p2
	cur   int
	edges []p2
}

// clipHalf keeps the part of the convex polygon where f(q) > 0, f affine.
func (c *clipper) clipHalf(poly []p2, f func(p2) float64) []p2 {
	if len(poly) < 3 {
		return nil
	}
	c.cur ^= 1
	out := c.buf[c.cur][:0]
	for i := range poly {
		a, b := poly[i], poly[(i+1)%len(poly)]
		fa, fb := f(a), f(b)
		if fa > 0 {
			out = append(out, a)
		}
		if (fa > 0) != (fb > 0) {
			s := fa / (fa - fb)
			out = append(out, p2{a.x + s*(b.x-a.x), a.y + s*(b.y-a.y)})
		}
	}
	c.buf[c.cur] = out
	if len(out) < 3 {
		return nil
	}
	return out
}

// clipTriangles intersects two counter-clockwise triangles.
func (c *clipper) clipTriangles(a, b *tri) []p2 {
	c.cur ^= 1
	poly := append(c.buf[c.cur][:0], a.v[0], a.v[1], a.v[2])
	c.buf[c.cur] = poly
	for i := range 3 {
		p, q := b.v[i], b.v[(i+1)%3]
		poly = c.clipHalf(poly, func(r p2) float64 { return (q.x-p.x)*(r.y-p.y) - (q.y-p.y)*(r.x-p.x) })
		if poly == nil {
			return nil
		}
	}
	return poly
}

// erode shrinks a counter-clockwise convex polygon by r on every side. A nil
// result means no point of the polygon lies at least r inside every edge.
func (c *clipper) erode(poly []p2, r float64) []p2 {
	if area(poly) <= minArea {
		return nil
	}
	c.edges = append(c.edges[:0], poly...)
	edges := c.edges
	for i := range edges {
		p, q := edges[i], edges[(i+1)%len(edges)]
		l := math.Hypot(q.x-p.x, q.y-p.y)
		if l == 0 {
			continue
		}
		poly = c.clipHalf(poly, func(s p2) float64 { return ((q.x-p.x)*(s.y-p.y)-(q.y-p.y)*(s.x-p.x))/l - r })
		if poly == nil {
			return nil
		}
	}
	return poly
}

// minArea is the area in square millimetres at or below which a clipped piece
// is a rounding sliver and not a region.
const minArea = 1e-12

func area(poly []p2) float64 {
	a := 0.0
	for i := range poly {
		p, q := poly[i], poly[(i+1)%len(poly)]
		a += p.x*q.y - q.x*p.y
	}
	return a / 2
}

func centroid(poly []p2) p2 {
	var c p2
	for _, p := range poly {
		c.x += p.x
		c.y += p.y
	}
	n := float64(len(poly))
	return p2{c.x / n, c.y / n}
}

func cross2(a, b, c p2) float64 { return (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x) }

func inTri(t *tri, q p2) bool {
	return cross2(t.v[0], t.v[1], q) >= 0 && cross2(t.v[1], t.v[2], q) >= 0 && cross2(t.v[2], t.v[0], q) >= 0
}

func segPointDist(a, b, q p2) float64 {
	dx, dy := b.x-a.x, b.y-a.y
	l2 := dx*dx + dy*dy
	s := 0.0
	if l2 > 0 {
		s = math.Max(0, math.Min(1, ((q.x-a.x)*dx+(q.y-a.y)*dy)/l2))
	}
	return math.Hypot(a.x+s*dx-q.x, a.y+s*dy-q.y)
}

// segsCross reports a proper crossing of segment ab and segment cd. A
// collinear or touching pair is not a crossing: the distance test covers it,
// and rounding noise must not turn a collinear gap into a crossing.
func segsCross(a, b, c, d p2) bool {
	const off = 1e-9
	side := func(p, q, r p2) float64 {
		l := math.Hypot(q.x-p.x, q.y-p.y)
		if l == 0 {
			return 0
		}
		return cross2(p, q, r) / l
	}
	d1, d2 := side(a, b, c), side(a, b, d)
	d3, d4 := side(c, d, a), side(c, d, b)
	return (d1 > off && d2 < -off || d1 < -off && d2 > off) && (d3 > off && d4 < -off || d3 < -off && d4 > off)
}

// within reports whether the 2D distance between two triangles is at most s.
func within(a, b *tri, s float64) bool {
	if inTri(a, b.v[0]) || inTri(b, a.v[0]) {
		return true
	}
	for i := range 3 {
		for j := range 3 {
			if segsCross(a.v[i], a.v[(i+1)%3], b.v[j], b.v[(j+1)%3]) {
				return true
			}
			if segPointDist(a.v[i], a.v[(i+1)%3], b.v[j]) <= s || segPointDist(b.v[i], b.v[(i+1)%3], a.v[j]) <= s {
				return true
			}
		}
	}
	return false
}
