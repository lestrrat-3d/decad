package loftcontact

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// CertifyCapTiling reports whether tris tile polygon exactly: every triangle
// lies inside it, no two overlap, their areas sum to its area, and their
// unshared edges match its boundary.
func CertifyCapTiling(budget *proofbound.WorkBudget, polygon []Point,
	tris [][3]int, arithmetic ...*Arithmetic) (bool, error) {
	c := ReplayArithmetic(arithmetic)
	area := PolygonArea(polygon, c)
	if !c.OK || area.Sign() <= 0 {
		return false, nil
	}
	sum := new(big.Rat)
	edges := make(map[string]int)
	for i, tri := range tris {
		if err := budget.Step(); err != nil {
			return false, err
		}
		for _, vertex := range tri {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if vertex < 0 || vertex >= len(polygon) {
				return false, nil
			}
		}
		if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
			return false, nil
		}
		p := [3]Point{polygon[tri[0]], polygon[tri[1]], polygon[tri[2]]}
		double := c.PointCross(p[0], p[1], p[2])
		if !c.OK || double.Sign() <= 0 {
			return false, nil
		}
		sum = c.Add(sum, double)
		if !c.OK {
			return false, nil
		}
		inside, err := TriangleInside(budget, polygon, p, c)
		if err != nil || !inside || !c.OK {
			return false, err
		}
		for j := range 3 {
			key := fragmentGeometryKey(p[j], p[(j+1)%3], c)
			if !c.OK {
				return false, nil
			}
			dir := 1
			if p2After(p[j], p[(j+1)%3], c) {
				dir = -1
			}
			if !c.OK {
				return false, nil
			}
			edges[key] += dir
		}
		for j := range i {
			if err := budget.Step(); err != nil {
				return false, err
			}
			q := [3]Point{polygon[tris[j][0]], polygon[tris[j][1]], polygon[tris[j][2]]}
			overlap, err := TrianglesOverlap(budget, p, q, c)
			if err != nil || !c.OK {
				return false, err
			}
			if overlap {
				return false, nil
			}
		}
	}
	if c.Cmp(sum, c.Mul(area, big.NewRat(2, 1))) != 0 || !c.OK {
		return false, nil
	}
	for i, a := range polygon {
		if err := budget.Step(); err != nil {
			return false, err
		}
		b := polygon[(i+1)%len(polygon)]
		key := fragmentGeometryKey(a, b, c)
		if !c.OK {
			return false, nil
		}
		dir := 1
		if p2After(a, b, c) {
			dir = -1
		}
		if !c.OK {
			return false, nil
		}
		edges[key] -= dir
	}
	for _, count := range edges {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if count != 0 {
			return false, nil
		}
	}
	return true, nil
}

func p2After(a, b Point, arithmetic ...*Arithmetic) bool {
	c := ReplayArithmetic(arithmetic)
	if cmp := c.Cmp(a.U, b.U); cmp != 0 {
		return cmp > 0
	}
	return c.Cmp(a.V, b.V) > 0 && c.OK
}

// PolygonArea returns the signed area of p, positive when counter-clockwise.
func PolygonArea(p []Point, arithmetic ...*Arithmetic) *big.Rat {
	c := ReplayArithmetic(arithmetic)
	area := new(big.Rat)
	for i, a := range p {
		b := p[(i+1)%len(p)]
		area = c.Add(area, c.Sub(c.Mul(a.U, b.V), c.Mul(a.V, b.U)))
		if !c.OK {
			return area
		}
	}
	return c.Quo(area, big.NewRat(2, 1))
}

func onSegment(p, a, b Point, arithmetic ...*Arithmetic) bool {
	c := ReplayArithmetic(arithmetic)
	if c.PointCross(a, b, p).Sign() != 0 || !c.OK {
		return false
	}
	inside := c.Cmp(p.U, MinRat(a.U, b.U, c)) >= 0 &&
		c.Cmp(p.U, MaxRat(a.U, b.U, c)) <= 0 &&
		c.Cmp(p.V, MinRat(a.V, b.V, c)) >= 0 &&
		c.Cmp(p.V, MaxRat(a.V, b.V, c)) <= 0
	return c.OK && inside
}

// MinRat returns the smaller of a and b.
func MinRat(a, b *big.Rat, arithmetic ...*Arithmetic) *big.Rat {
	if ReplayArithmetic(arithmetic).Cmp(a, b) < 0 {
		return a
	}
	return b
}

// MaxRat returns the larger of a and b.
func MaxRat(a, b *big.Rat, arithmetic ...*Arithmetic) *big.Rat {
	if ReplayArithmetic(arithmetic).Cmp(a, b) > 0 {
		return a
	}
	return b
}

// 1 is strict interior, 0 boundary, -1 exterior.
func pointInPolygon(budget *proofbound.WorkBudget, p Point,
	polygon []Point, arithmetic ...*Arithmetic) (int, error) {
	c := ReplayArithmetic(arithmetic)
	inside := false
	for i, a := range polygon {
		if err := budget.Step(); err != nil {
			return -1, err
		}
		b := polygon[(i+1)%len(polygon)]
		if onSegment(p, a, b, c) {
			return 0, nil
		}
		if !c.OK {
			return -1, nil
		}
		if (c.Cmp(a.V, p.V) > 0) == (c.Cmp(b.V, p.V) > 0) {
			if !c.OK {
				return -1, nil
			}
			continue
		}
		x := c.Add(c.Mul(c.Sub(b.U, a.U), c.Quo(c.Sub(p.V, a.V), c.Sub(b.V, a.V))), a.U)
		if c.Cmp(x, p.U) > 0 {
			inside = !inside
		}
		if !c.OK {
			return -1, nil
		}
	}
	if inside {
		return 1, nil
	}
	return -1, nil
}

func segmentParam(a, b, p Point,
	arithmetic ...*Arithmetic) (*big.Rat, bool) {
	c := ReplayArithmetic(arithmetic)
	if !onSegment(p, a, b, c) || !c.OK {
		return nil, false
	}
	if c.Cmp(a.U, b.U) != 0 {
		t := c.Quo(c.Sub(p.U, a.U), c.Sub(b.U, a.U))
		return t, c.OK
	}
	if c.Cmp(a.V, b.V) != 0 {
		t := c.Quo(c.Sub(p.V, a.V), c.Sub(b.V, a.V))
		return t, c.OK
	}
	return nil, false
}

// EdgeInside reports whether the segment from a to b lies in the closed
// polygon.
func EdgeInside(budget *proofbound.WorkBudget, a, b Point,
	polygon []Point, arithmetic ...*Arithmetic) (bool, error) {
	c := ReplayArithmetic(arithmetic)
	values := []*big.Rat{big.NewRat(0, 1), big.NewRat(1, 1)}
	for i := range polygon {
		if err := budget.Step(); err != nil {
			return false, err
		}
		d := polygon[(i+1)%len(polygon)]
		r := Point{U: c.Sub(b.U, a.U), V: c.Sub(b.V, a.V)}
		s := Point{U: c.Sub(d.U, polygon[i].U), V: c.Sub(d.V, polygon[i].V)}
		den := c.Sub(c.Mul(r.U, s.V), c.Mul(r.V, s.U))
		if !c.OK {
			return false, nil
		}
		if den.Sign() == 0 {
			for _, p := range []Point{polygon[i], d} {
				if t, ok := segmentParam(a, b, p, c); ok {
					values = append(values, t)
				}
				if !c.OK {
					return false, nil
				}
			}
			continue
		}
		ca := Point{U: c.Sub(polygon[i].U, a.U), V: c.Sub(polygon[i].V, a.V)}
		t := c.Quo(c.Sub(c.Mul(ca.U, s.V), c.Mul(ca.V, s.U)), den)
		u := c.Quo(c.Sub(c.Mul(ca.U, r.V), c.Mul(ca.V, r.U)), den)
		if !c.OK {
			return false, nil
		}
		if t.Sign() >= 0 && c.Cmp(t, big.NewRat(1, 1)) <= 0 &&
			u.Sign() >= 0 && c.Cmp(u, big.NewRat(1, 1)) <= 0 {
			values = append(values, t)
		}
		if !c.OK {
			return false, nil
		}
	}
	for i := 1; i < len(values); i++ {
		for j := i; j > 0 && c.Cmp(values[j], values[j-1]) < 0; j-- {
			if err := budget.Step(); err != nil {
				return false, err
			}
			values[j], values[j-1] = values[j-1], values[j]
		}
		if !c.OK {
			return false, nil
		}
	}
	for i := 1; i < len(values); i++ {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if c.Cmp(values[i], values[i-1]) == 0 {
			if !c.OK {
				return false, nil
			}
			continue
		}
		mid := c.Quo(c.Add(values[i-1], values[i]), big.NewRat(2, 1))
		point := c.Interpolate(a, b, mid)
		if !c.OK {
			return false, nil
		}
		status, err := pointInPolygon(budget, point, polygon, c)
		if err != nil || status < 0 || !c.OK {
			return false, err
		}
	}
	return true, nil
}

// TriangleInside reports whether tri lies in the closed polygon with its
// centroid strictly inside.
func TriangleInside(budget *proofbound.WorkBudget, polygon []Point,
	tri [3]Point, arithmetic ...*Arithmetic) (bool, error) {
	c := ReplayArithmetic(arithmetic)
	for _, p := range tri {
		status, err := pointInPolygon(budget, p, polygon, c)
		if err != nil || status < 0 || !c.OK {
			return false, err
		}
	}
	for i := range 3 {
		inside, err := EdgeInside(budget, tri[i], tri[(i+1)%3], polygon, c)
		if err != nil || !inside || !c.OK {
			return false, err
		}
	}
	centroid := Point{U: new(big.Rat), V: new(big.Rat)}
	for _, p := range tri {
		centroid.U = c.Add(centroid.U, p.U)
		centroid.V = c.Add(centroid.V, p.V)
	}
	centroid.U = c.Quo(centroid.U, big.NewRat(3, 1))
	centroid.V = c.Quo(centroid.V, big.NewRat(3, 1))
	if !c.OK {
		return false, nil
	}
	status, err := pointInPolygon(budget, centroid, polygon, c)
	return c.OK && status == 1, err
}

// TrianglesOverlap reports whether the counter-clockwise triangles a and b
// share positive area.
func TrianglesOverlap(budget *proofbound.WorkBudget,
	a, b [3]Point, arithmetic ...*Arithmetic) (bool, error) {
	c := ReplayArithmetic(arithmetic)
	poly := []Point{a[0], a[1], a[2]}
	for i := range b {
		d := b[(i+1)%3]
		input := poly
		poly = nil
		for j, p := range input {
			if err := budget.Step(); err != nil {
				return false, err
			}
			q := input[(j+1)%len(input)]
			sp, sq := c.PointCross(b[i], d, p), c.PointCross(b[i], d, q)
			if !c.OK {
				return false, nil
			}
			if sp.Sign() >= 0 {
				poly = append(poly, p)
			}
			if sp.Sign() == 0 || sq.Sign() == 0 || sp.Sign() == sq.Sign() {
				continue
			}
			t := c.Quo(sp, c.Sub(sp, sq))
			poly = append(poly, c.Interpolate(p, q, t))
			if !c.OK {
				return false, nil
			}
		}
		if len(poly) < 3 {
			return false, nil
		}
	}
	area := PolygonArea(poly, c)
	return c.OK && area.Sign() > 0, nil
}
