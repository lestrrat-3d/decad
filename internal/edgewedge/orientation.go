package edgewedge

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// materialWalk returns the sense in which planar face f walks e with its
// material on the left seen from outside: +1 along e's own sense (Start to
// End, or counter-clockwise about a circle's axis), −1 against it. It reads
// the walk from the loop that holds e: the sign of that loop's area about
// f's outward normal says whether the loop runs counter-clockwise seen from
// outside, and the loop's role says which side of it the material lies on,
// inside an outer loop and outside a hole. A loop whose area the proven
// bounds do not separate from zero, an edge f holds other than once, and a
// loop holding a curve other than a line, circle or arc give no walk.
func materialWalk(f *topo.Face, e *topo.Edge) (float64, bool) {
	var loop *topo.Loop
	forward := false
	uses := 0
	for _, l := range f.Loops {
		for _, ce := range l.CoEdges {
			if ce.Edge == e {
				loop, forward = l, ce.Forward
				uses++
			}
		}
	}
	if uses != 1 {
		return 0, false
	}
	normal, err := topobuild.NormalAt(f, e.Start.Position)
	if err != nil {
		return 0, false
	}
	area, bound, ok := loopArea(loop)
	if !ok {
		return 0, false
	}
	nb := normal.Bound.Base()
	signed := area.Dot(normal.Value)
	// The held normal is within nb of the true unit one.
	bound += area.Len() * nb
	bound = math.Nextafter(bound*(1+0x1p-50), math.Inf(1))
	if math.IsNaN(signed) || math.IsNaN(bound) || math.Abs(signed) <= bound {
		return 0, false
	}
	walk := 1.0
	if (signed > 0) != loop.Outer {
		walk = -1
	}
	if !forward {
		walk = -walk
	}
	return walk, true
}

// loopArea returns the vector area of a loop, half the integral of p × dp
// around it in walk order, and a bound on its departure from the true loop's.
// The bound charges the float rounding of every term, and a band of the
// largest vertex and curve displacement around the loop's length: moving a
// loop's boundary by d changes its area by at most d times its length plus
// a disc of radius d, taken twice for margin.
func loopArea(l *topo.Loop) (r3.Vec, float64, bool) {
	if len(l.CoEdges) == 0 {
		return r3.Vec{}, 0, false
	}
	origin := walkStart(l.CoEdges[0])
	var sum r3.Vec
	magnitude, length, displacement := 0.0, 0.0, 0.0
	for _, ce := range l.CoEdges {
		e := ce.Edge
		ds, ok := vertexBound(e.Start)
		if !ok {
			return r3.Vec{}, 0, false
		}
		de, ok := vertexBound(e.End)
		if !ok {
			return r3.Vec{}, 0, false
		}
		displacement = math.Max(displacement, math.Max(ds, de))
		a, b := e.Start.Position.Sub(origin), e.End.Position.Sub(origin)
		var term r3.Vec
		switch c := e.Curve.(type) {
		case surfacegeom.Line3:
			term = a.Cross(b).Scale(0.5)
			magnitude += a.Len() * b.Len()
			length += b.Sub(a).Len()
		case surfacegeom.Circle3:
			t, m, arc, ok := arcArea(e, c.Center.Sub(origin), c.Axis, c.Radius, a, b)
			if !ok {
				return r3.Vec{}, 0, false
			}
			term, magnitude, length = t, magnitude+m, length+arc
			displacement = math.Max(displacement, e.CurveBound)
		case surfacegeom.Arc3:
			t, m, arc, ok := arcArea(e, c.Center.Sub(origin), c.Axis, c.Radius, a, b)
			if !ok {
				return r3.Vec{}, 0, false
			}
			term, magnitude, length = t, magnitude+m, length+arc
			displacement = math.Max(displacement, e.CurveBound)
		default:
			return r3.Vec{}, 0, false
		}
		if !ce.Forward {
			term = term.Scale(-1)
		}
		sum = sum.Add(term)
	}
	bound := arithSlack*magnitude + 2*length*displacement + 4*math.Pi*displacement*displacement
	if math.IsNaN(bound) || math.IsInf(bound, 0) {
		return r3.Vec{}, 0, false
	}
	return sum, bound, true
}

// arcArea is half the integral of p × dp along a circular edge swept
// counter-clockwise about axis from a to b, both relative to the loop's
// origin as center is: (center × (b − a) + r²θ·axis)/2 for the sweep θ,
// a whole turn where the edge starts and ends at one vertex. It also returns
// the magnitude its rounding scales with and the arc's length. The caller
// charges the curve's own displacement from its held circle with the vertex
// displacement, so an edge without that proof gives no area.
func arcArea(e *topo.Edge, center, axis r3.Vec, radius units.Value, a, b r3.Vec) (r3.Vec, float64, float64, bool) {
	if !e.CurveBounded {
		return r3.Vec{}, 0, 0, false
	}
	r, err := radius.In(units.Millimeter)
	if err != nil || !(r > e.CurveBound) {
		return r3.Vec{}, 0, 0, false
	}
	theta := 2 * math.Pi
	if e.Start != e.End {
		u, v := a.Sub(center), b.Sub(center)
		theta = math.Atan2(u.Cross(v).Dot(axis), u.Dot(v))
		if theta <= 0 {
			theta += 2 * math.Pi
		}
	}
	term := center.Cross(b.Sub(a)).Add(axis.Scale(r * r * theta)).Scale(0.5)
	magnitude := center.Len()*b.Sub(a).Len() + r*r*2*math.Pi
	return term, magnitude, r * theta, true
}

// walkStart is the point a loop use starts at.
func walkStart(ce topo.CoEdge) r3.Vec {
	if ce.Forward {
		return ce.Edge.Start.Position
	}
	return ce.Edge.End.Position
}
