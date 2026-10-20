package facetedrecord

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

const (
	edgeEps = 0x1p-52
	// edgeParallel is the largest |cross product| of two unit directions that
	// the edge-curve rules read as parallel. The tags are stored floats, so a
	// direction that is exactly axis-aligned in the model is exact here.
	edgeParallel = 1e-12
)

// RetainedEdgeCurve returns the analytic curve of the rim between two faces
// whose retained tags meet along it (docs/faceted-source-retention-design.md
// §10), and whether one is admitted. chain lists the rim's held vertices in
// order with their bounds; closed is true when the rim starts and ends at one
// vertex. A rim that no rule admits keeps its chord chain: false is a decline,
// never a refusal. a and b must not be nil.
func RetainedEdgeCurve(a, b *topo.Face, chain []r3.Vec, bounds []float64, closed bool) (surfacegeom.Curve, bool) {
	if a == b || !a.RetainedTag || !b.RetainedTag || len(chain) < 2 || len(chain) != len(bounds) {
		return nil, false
	}
	ta, tb := a.Surface, b.Surface
	if _, ok := tb.(surfacegeom.Plane); ok {
		ta, tb = tb, ta
	}
	plane, ok := ta.(surfacegeom.Plane)
	if !ok {
		return nil, false
	}
	scale := 1.0
	for _, v := range chain {
		scale = max(scale, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	slack := 64 * edgeEps * scale
	n := plane.Frame.N()
	o := plane.Frame.Origin()
	switch s := tb.(type) {
	case surfacegeom.Plane:
		return planePlaneRim(n, o, s, chain, bounds, slack)
	case surfacegeom.Cylinder:
		return planeCylinderRim(n, o, s, chain, bounds, closed, slack)
	case surfacegeom.Sphere:
		radius, ok := millimeters(s.Radius)
		if !ok {
			return nil, false
		}
		d := n.Dot(s.Center.Sub(o))
		rr := radius*radius - d*d
		if !(rr > 0) {
			return nil, false
		}
		return circleRim(s.Center.Sub(n.Scale(d)), n, math.Sqrt(rr), chain, bounds, closed, slack)
	}
	return nil, false
}

type rimLine struct{ point, dir r3.Vec }

func (l rimLine) distance(v r3.Vec) float64 {
	rel := v.Sub(l.point)
	return rel.Sub(l.dir.Scale(rel.Dot(l.dir))).Len()
}

// acceptLine reports whether some candidate line passes the falsifier of §10.1
// E6: every chain vertex lies within its own bound, plus the arithmetic slack,
// of the line. The test can only reject; a line near the vertices is trusted
// because both tags are recorded surfaces.
func acceptLine(lines []rimLine, chain []r3.Vec, bounds []float64, slack float64) (surfacegeom.Curve, bool) {
	for _, l := range lines {
		near := true
		for i, v := range chain {
			if l.distance(v) > bounds[i]+slack {
				near = false
				break
			}
		}
		if near {
			return surfacegeom.Line3{}, true
		}
	}
	return nil, false
}

func planePlaneRim(n1, o1 r3.Vec, p2 surfacegeom.Plane, chain []r3.Vec, bounds []float64, slack float64) (surfacegeom.Curve, bool) {
	n2, o2 := p2.Frame.N(), p2.Frame.Origin()
	cross := n1.Cross(n2)
	if !(cross.Len() > 1e-9) {
		return nil, false
	}
	dir, ok := cross.Normalize()
	if !ok {
		return nil, false
	}
	c := n1.Dot(n2)
	h1, h2 := n1.Dot(o1), n2.Dot(o2)
	den := 1 - c*c
	point := n1.Scale((h1 - h2*c) / den).Add(n2.Scale((h2 - h1*c) / den))
	return acceptLine([]rimLine{{point, dir}}, chain, bounds, slack)
}

func planeCylinderRim(n, o r3.Vec, cyl surfacegeom.Cylinder, chain []r3.Vec, bounds []float64,
	closed bool, slack float64) (surfacegeom.Curve, bool) {
	radius, ok := millimeters(cyl.Radius)
	if !ok {
		return nil, false
	}
	axis := cyl.Axis
	switch {
	case axis.Cross(n).Len() <= edgeParallel:
		t := n.Dot(o.Sub(cyl.Origin)) / n.Dot(axis)
		return circleRim(cyl.Origin.Add(axis.Scale(t)), n, radius, chain, bounds, closed, slack)
	case math.Abs(axis.Dot(n)) <= edgeParallel:
		s := n.Dot(cyl.Origin.Sub(o))
		foot := cyl.Origin.Sub(n.Scale(s))
		half := radius*radius - s*s
		if half < -slack*radius {
			return nil, false
		}
		w := math.Sqrt(max(half, 0))
		side, ok := n.Cross(axis).Normalize()
		if !ok {
			return nil, false
		}
		return acceptLine([]rimLine{
			{foot.Add(side.Scale(w)), axis}, {foot.Sub(side.Scale(w)), axis},
		}, chain, bounds, slack)
	}
	return nil, false
}

// circleRim returns the circle of radius r about centre in the plane with
// unit normal n, as a Circle3 for a closed rim and an Arc3 swept
// counter-clockwise about its Axis for an open one (§10.1 E3, E5, E6, E7).
func circleRim(centre, n r3.Vec, r float64, chain []r3.Vec, bounds []float64, closed bool,
	slack float64) (surfacegeom.Curve, bool) {
	u, ok := perpendicular(n)
	if !ok {
		return nil, false
	}
	w := n.Cross(u)
	maxBound := 0.0
	theta := make([]float64, len(chain))
	for i, v := range chain {
		rel := v.Sub(centre)
		h := rel.Dot(n)
		rho := rel.Sub(n.Scale(h)).Len()
		if math.Hypot(rho-r, h) > bounds[i]+slack {
			return nil, false
		}
		maxBound = max(maxBound, bounds[i])
		theta[i] = math.Atan2(rel.Dot(w), rel.Dot(u))
	}
	radius := units.Millimeters(r)
	if closed {
		return surfacegeom.Circle3{Center: centre, Axis: n, Radius: radius}, true
	}
	// An open rim needs an interior vertex to fix which way round it runs, and
	// every step must clear what the vertex bounds can move it.
	if len(chain) < 3 {
		return nil, false
	}
	floor := 4*(maxBound+slack)/r + 1e-9
	sum, sign := 0.0, 0.0
	for i := 1; i < len(theta); i++ {
		step := theta[i] - theta[i-1]
		for step > math.Pi {
			step -= 2 * math.Pi
		}
		for step <= -math.Pi {
			step += 2 * math.Pi
		}
		if math.Abs(step) < floor || math.Abs(step) > math.Pi-floor {
			return nil, false
		}
		if sign == 0 {
			sign = math.Copysign(1, step)
		} else if math.Copysign(1, step) != sign {
			return nil, false
		}
		sum += math.Abs(step)
	}
	if sum > 2*math.Pi-floor {
		return nil, false
	}
	axis := n
	if sign < 0 {
		axis = n.Scale(-1)
	}
	return surfacegeom.Arc3{Center: centre, Axis: axis, Radius: radius}, true
}

func perpendicular(n r3.Vec) (r3.Vec, bool) {
	pick := r3.NewVec(1, 0, 0)
	if math.Abs(n.X) > 0.9 {
		pick = r3.NewVec(0, 1, 0)
	}
	return n.Cross(pick).Normalize()
}
