package facetedrecord

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
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
	// edgeOblique is the smallest |axis · normal| for which a plane and a
	// cylinder meet in an ellipse the rules publish (§10.5). The ellipse's
	// centre amplifies the rounding of its inputs by its reciprocal.
	edgeOblique = 1e-3
)

// planeRimAmp scales a plane tag's displacement to the displacement of the rim
// it meets: planes whose normals meet at a sine of at least 1e-3 move their
// common line by at most 2·delta/sine per plane.
const planeRimAmp = 4000.0

// RetainedEdgeCurve returns the analytic curve of the rim between two faces
// whose retained tags meet along it (agent-docs/faceted-source-retention-design.md
// §10), and whether one is admitted. chain lists the rim's held vertices in
// order with their bounds; closed is true when the rim starts and ends at one
// vertex. A rim that no rule admits keeps its chord chain: false is a decline,
// never a refusal. a and b must not be nil.
func RetainedEdgeCurve(a, b *topo.Face, chain []r3.Vec, bounds []float64, closed bool) (surfacegeom.Curve, bool) {
	if a == b || !a.RetainedTag || !b.RetainedTag || len(chain) < 2 || len(chain) != len(bounds) {
		return nil, false
	}
	ta, tb := a.Surface, b.Surface
	if circle, ok := cylinderTorusRim(ta, tb, chain, bounds, closed); ok {
		return circle, true
	}
	if line, ok := parallelCylindersRim(ta, tb, chain, bounds); ok {
		return line, true
	}
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
	// A plane tag may lie off the true plane by its stated AxialDelta. A Line3
	// rim moves by at most planeRimAmp times that, so a line's falsifier slack
	// widens by it (lineSlack); the widening never admits by itself. A circle or
	// ellipse rim is published with a zero curve bound, so it has no room for the
	// displacement: it declines when any delta exceeds the arithmetic allowance.
	lineSlack := slack
	displaced := false
	for _, f := range []*topo.Face{a, b} {
		if _, isPlane := f.Surface.(surfacegeom.Plane); !isPlane || !f.HasAxialDelta {
			continue
		}
		if !(f.AxialDelta >= 0) || math.IsInf(f.AxialDelta, 0) {
			return nil, false
		}
		if f.AxialDelta == 0 {
			continue
		}
		displaced = displaced || f.AxialDelta > slack
		lineSlack = proofbound.AbsSumUpper(lineSlack, proofbound.ProductUpper(planeRimAmp, f.AxialDelta))
	}
	n := plane.Frame.N()
	o := plane.Frame.Origin()
	switch s := tb.(type) {
	case surfacegeom.Plane:
		return planePlaneRim(n, o, s, chain, bounds, lineSlack)
	case surfacegeom.Cylinder:
		return planeCylinderRim(n, o, s, chain, bounds, closed, slack, lineSlack, displaced)
	case surfacegeom.Sphere:
		radius, ok := millimeters(s.Radius)
		if !ok || displaced {
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

// cylinderTorusRim returns the junction circle of a blend cylinder and a blend
// torus of one tube radius (agent-docs/retained-modify-reach-design.md §5.4,
// Claim J, and retention §10.1 E11). The cylinder's axis d is normal to the
// torus axis a, and the plane through the torus axis normal to d cuts the
// cylinder axis at the tube centre c_J: c_J lies in the torus's equatorial
// plane at distance Major from its centre. Both surfaces then hold the circle
// of radius r about c_J in that plane, which is published as an Arc3 (an open
// rim) or a Circle3. The chain falsifier of E6 and the sweep rule of E7 apply;
// a pair that fails any identity declines and keeps its chord chain.
func cylinderTorusRim(sa, sb surfacegeom.Surface, chain []r3.Vec, bounds []float64,
	closed bool) (surfacegeom.Curve, bool) {
	cyl, ok := sa.(surfacegeom.Cylinder)
	tor, tok := sb.(surfacegeom.Torus)
	if !ok || !tok {
		cyl, ok = sb.(surfacegeom.Cylinder)
		tor, tok = sa.(surfacegeom.Torus)
		if !ok || !tok {
			return nil, false
		}
	}
	scale := 1.0
	for _, v := range chain {
		scale = max(scale, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	slack := 64 * edgeEps * scale
	radius, rok := millimeters(cyl.Radius)
	minor, mok := millimeters(tor.Minor)
	major, jok := millimeters(tor.Major)
	if !rok || !mok || !jok || math.Abs(radius-minor) > slack {
		return nil, false
	}
	if math.Abs(cyl.Axis.Dot(tor.Axis)) > edgeParallel {
		return nil, false
	}
	centre := cyl.Origin.Add(cyl.Axis.Scale(cyl.Axis.Dot(tor.Center.Sub(cyl.Origin))))
	q := centre.Sub(tor.Center)
	if math.Abs(q.Dot(tor.Axis)) > slack || math.Abs(q.Len()-major) > slack {
		return nil, false
	}
	return circleRim(centre, cyl.Axis, radius, chain, bounds, closed, slack)
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
	closed bool, slack, lineSlack float64, displaced bool) (surfacegeom.Curve, bool) {
	radius, ok := millimeters(cyl.Radius)
	if !ok {
		return nil, false
	}
	axis := cyl.Axis
	switch {
	case axis.Cross(n).Len() <= edgeParallel:
		if displaced {
			return nil, false
		}
		t := n.Dot(o.Sub(cyl.Origin)) / n.Dot(axis)
		return circleRim(cyl.Origin.Add(axis.Scale(t)), n, radius, chain, bounds, closed, slack)
	case math.Abs(axis.Dot(n)) <= edgeParallel:
		s := n.Dot(cyl.Origin.Sub(o))
		foot := cyl.Origin.Sub(n.Scale(s))
		half := radius*radius - s*s
		if half < -lineSlack*radius {
			return nil, false
		}
		w := math.Sqrt(max(half, 0))
		side, ok := n.Cross(axis).Normalize()
		if !ok {
			return nil, false
		}
		return acceptLine([]rimLine{
			{foot.Add(side.Scale(w)), axis}, {foot.Sub(side.Scale(w)), axis},
		}, chain, bounds, lineSlack)
	case math.Abs(axis.Dot(n)) >= edgeOblique:
		if displaced {
			return nil, false
		}
		return ellipseRim(n, o, cyl.Origin, axis, radius, chain, bounds, closed, slack)
	}
	return nil, false
}

// ellipseRim returns the ellipse in which the plane (unit normal n through o)
// meets the cylinder (axis a, origin c, radius R) when the axis is oblique to
// the plane (§10.5, E8, E9). A vertex farther than its bound plus slack from
// the plane or from the cylinder disproves the pairing; a vertex that passes
// both proves nothing.
func ellipseRim(n, o, c, a r3.Vec, radius float64, chain []r3.Vec, bounds []float64, closed bool,
	slack float64) (surfacegeom.Curve, bool) {
	k := math.Abs(a.Dot(n))
	centre := c.Add(a.Scale(n.Dot(o.Sub(c)) / n.Dot(a)))
	major, ok := a.Sub(n.Scale(a.Dot(n))).Normalize()
	if !ok {
		return nil, false
	}
	w := n.Cross(major)
	theta := make([]float64, len(chain))
	maxBound := 0.0
	for i, v := range chain {
		rel := v.Sub(c)
		radial := rel.Sub(a.Scale(rel.Dot(a))).Len()
		if math.Abs(n.Dot(v.Sub(o))) > bounds[i]+slack || math.Abs(radial-radius) > bounds[i]+slack {
			return nil, false
		}
		maxBound = max(maxBound, bounds[i])
		q := v.Sub(centre)
		theta[i] = math.Atan2(w.Dot(q), k*major.Dot(q))
	}
	ellipse := func(axis r3.Vec) surfacegeom.Curve {
		return surfacegeom.Ellipse3{Center: centre, Axis: axis, Major: major,
			SemiMajor: units.Millimeters(radius / k), SemiMinor: units.Millimeters(radius)}
	}
	if closed {
		return ellipse(n), true
	}
	sign, ok := sweepSign(theta, maxBound, radius, slack)
	if !ok {
		return nil, false
	}
	return ellipse(n.Scale(sign)), true
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
	sign, ok := sweepSign(theta, maxBound, r, slack)
	if !ok {
		return nil, false
	}
	return surfacegeom.Arc3{Center: centre, Axis: n.Scale(sign), Radius: radius}, true
}

// sweepSign reads which way round an open rim runs from the angles of its chain
// vertices about the rim's axis (E7, E9). It returns +1 for a counter-clockwise
// run and −1 for a clockwise one. The rim needs an interior vertex, every
// step must clear what the bounds can move an angle (4·(max bound + slack)/r,
// with r the smallest radius of curvature the angle is measured on), stay below
// π, share one sign, and the steps must sum below 2π.
func sweepSign(theta []float64, maxBound, r, slack float64) (float64, bool) {
	if len(theta) < 3 {
		return 0, false
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
			return 0, false
		}
		if sign == 0 {
			sign = math.Copysign(1, step)
		} else if math.Copysign(1, step) != sign {
			return 0, false
		}
		sum += math.Abs(step)
	}
	if sum > 2*math.Pi-floor {
		return 0, false
	}
	return sign, true
}

func perpendicular(n r3.Vec) (r3.Vec, bool) {
	pick := r3.NewVec(1, 0, 0)
	if math.Abs(n.X) > 0.9 {
		pick = r3.NewVec(0, 1, 0)
	}
	return n.Cross(pick).Normalize()
}
