package meshbool

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type slabPoint [3]ival

func heldPoint(a r3.Vec) slabPoint { return slabPoint{{a.X, a.X}, {a.Y, a.Y}, {a.Z, a.Z}} }
func absoluteUpper(a ival) float64 { return math.Max(math.Abs(a.lo), math.Abs(a.hi)) }
func squareInterval(a ival) ival {
	lo, hi := a.absRange()
	return ival{down(lo.lo * lo.lo), up(hi.hi * hi.hi)}
}
func midpointInterval(a, b slabPoint) (p slabPoint) {
	for j := range 3 {
		p[j] = a[j].add(b[j]).mul(ival{.5, .5})
	}
	return
}

// TightenTorusSlabs reads only the exact retained source assigned to each
// triangle of this unshifted operand. Its finite patch already lies within
// FacetBound of this triangle. No other face, trim or vertex bound is used
// as an ownership claim. A failed enclosure leaves the existing slab intact.
func (m *BoolMesh) TightenTorusSlabs(mesh *topo.Mesh, budget *proofbound.WorkBudget) error {
	if m.Gate <= 0 || mesh == nil || len(mesh.Source) != len(m.Tris) {
		return nil
	}
	if err := budget.Err(); err != nil {
		return err
	}
	for i, tri := range m.Tris {
		if err := budget.Step(); err != nil {
			return err
		}
		if m.SlabBound[i] <= m.Gate {
			continue
		}
		face := mesh.Source[i]
		if face == nil || !face.RetainedTag || face.Denoted != nil || face.TagDeparture != 0 || face.AxialDelta != 0 || face.ExtentDelta != 0 {
			continue
		}
		torus, ok := face.Surface.(surfacegeom.Torus)
		if !ok || face.FilletCarrier == nil || face.FilletCarrier.Surface != torus || !(face.FilletCarrier.Distance >= 0) || proofbound.IsNonFinite(face.FilletCarrier.Distance) {
			continue
		}
		corners := [3]r3.Vec{m.Verts[tri[0]], m.Verts[tri[1]], m.Verts[tri[2]]}
		slab, proven, err := torusBound(corners, m.FacetBound[i], face.FilletCarrier.Distance, torus, m.Gate, budget)
		if err != nil {
			return err
		}
		if proven {
			m.SlabBound[i] = slab
		}
	}
	return nil
}

// torusBound encloses the signed height of every torus point q within beta
// of the triangle. Each subtriangle has an exact centroid p in the original
// plane and a box containing p, q and their connecting segment. For signed
// distance F to the stored torus, |F(q)| is at most the authenticated
// carrier displacement. The mean value theorem gives
//
//	(+/- n)·(q-p)  = F(q)-F(p) + ((+/- n)-grad F(x))·(q-p)
//
// for some x on this segment, where n is the triangle's unit normal.
// Both signed expressions bound the same absolute height. The proof covers
// all four midpoint subtriangles or declines. It never projects onto an
// unverified trim or spends beta as an additional Boolean allowance.
func torusBound(tri [3]r3.Vec, beta, departure float64, s surfacegeom.Torus, gate float64, budget *proofbound.WorkBudget) (float64, bool, error) {
	axis := -1
	axisComponents := [3]float64{s.Axis.X, s.Axis.Y, s.Axis.Z}
	for i, k := range axisComponents {
		if math.Abs(k) == 1 && axis < 0 {
			axis = i
		} else if k != 0 {
			return 0, false, nil
		}
	}
	if axis < 0 || !proofbound.FiniteVec(s.Center) || proofbound.IsNonFinite(beta) || proofbound.IsNonFinite(gate) {
		return 0, false, nil
	}
	major, majorErr := s.Major.In(units.Millimeter)
	minor, minorErr := s.Minor.In(units.Millimeter)
	if majorErr != nil || minorErr != nil {
		return 0, false, nil
	}
	if !(major > minor && minor > 0 && beta >= 0 && gate > 0) {
		return 0, false, nil
	}
	u, v := (axis+1)%3, (axis+2)%3
	var c [3]slabPoint
	origin := heldPoint(s.Center)
	for k := range tri {
		p := heldPoint(tri[k])
		for j := range 3 {
			c[k][j] = p[j].sub(origin[j])
		}
	}
	var n slabPoint
	for j := range 3 {
		a, b := (j+1)%3, (j+2)%3
		left := c[1][a].sub(c[0][a]).mul(c[2][b].sub(c[0][b]))
		right := c[1][b].sub(c[0][b]).mul(c[2][a].sub(c[0][a]))
		n[j] = left.sub(right)
	}
	nl := sqrtIval(squareInterval(n[0]).add(squareInterval(n[1])).add(squareInterval(n[2])))
	if !(nl.lo > 0) {
		return 0, false, nil
	}
	for j := range 3 {
		n[j] = n[j].div(nl)
	}
	scalar := func(x float64) ival { return ival{x, x} }
	lengths := func(p slabPoint) (ival, ival, ival) {
		rho := sqrtIval(squareInterval(p[u]).add(squareInterval(p[v])))
		side := rho.sub(scalar(major))
		length := sqrtIval(squareInterval(side).add(squareInterval(p[axis])))
		return rho, side, length
	}
	var visit func([3]slabPoint, int) (float64, bool, error)
	visit = func(t [3]slabPoint, depth int) (float64, bool, error) {
		if e := budget.SpendWork(64); e != nil {
			return 0, false, e
		}
		var p, box slabPoint
		for j := range 3 {
			p[j] = t[0][j].add(t[1][j]).add(t[2][j]).div(scalar(3))
			box[j] = t[0][j].hull(t[1][j]).hull(t[2][j]).widen(beta)
		}
		_, _, lp := lengths(p)
		rho, side, l := lengths(box)
		if !(l.lo > 0 && rho.lo > 0) {
			return 0, false, nil
		}
		var g slabPoint
		g[axis] = box[axis].div(l)
		scale := side.div(l).div(rho)
		g[u] = box[u].mul(scale)
		g[v] = box[v].mul(scale)
		best := math.Inf(1)
		for _, sign := range []float64{-1, 1} {
			height := scalar(0).sub(lp.sub(scalar(minor)))
			for j := range 3 {
				height = height.add(n[j].mul(scalar(sign)).sub(g[j]).mul(box[j].sub(p[j])))
			}
			best = math.Min(best, absoluteUpper(height))
		}
		best = proofbound.AbsSumUpper(best, departure)
		if best <= gate {
			return best, true, nil
		}
		if depth >= 5 {
			return 0, false, nil
		}
		a, b, c := midpointInterval(t[0], t[1]), midpointInterval(t[1], t[2]), midpointInterval(t[2], t[0])
		worst := 0.0
		for _, sub := range [4][3]slabPoint{{t[0], a, c}, {a, t[1], b}, {c, b, t[2]}, {a, b, c}} {
			v, ok, e := visit(sub, depth+1)
			if e != nil || !ok {
				return 0, false, e
			}
			worst = math.Max(worst, v)
		}
		return worst, true, nil
	}
	return visit(c, 0)
}
