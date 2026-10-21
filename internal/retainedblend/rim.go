// Package retainedblend builds Fillet and Chamfer of an edge between two
// retained faces of a mesh Boolean result (agent-docs/retained-edge-blend-design.md).
// It rewrites the held mesh near the edge so the new faces lie on exact
// surfaces computed from the two retained tags.
package retainedblend

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

const (
	ulp = 0x1p-52
	// parallelTol is the largest |axis × normal| for which the cylinder axis
	// and the plane normal count as one direction.
	parallelTol = 1e-9
)

// blendSpec is the size of one blend: a fillet's radius, or a chamfer's two
// setbacks, along the plane and along the wall.
type blendSpec struct {
	fillet bool
	r      float64
	dA, dB float64
}

func (s blendSpec) planeSetback() float64 {
	if s.fillet {
		return s.r
	}
	return s.dA
}

func (s blendSpec) wallSetback() float64 {
	if s.fillet {
		return s.r
	}
	return s.dB
}

// rim is a closed circular edge between a retained plane and a coaxial
// retained cylinder, read from the two tags (§4.2).
type rim struct {
	center, n, u, w r3.Vec
	radius          float64
	// kappa is +1 when the plane face lies outside the rim (ρ > radius) and
	// -1 when it lies inside.
	kappa float64
	// side is +1 when the wall extends toward the plane's outward normal and
	// -1 when it extends into the material.
	side       float64
	convex     bool
	matOutside bool
	planeFace  *topo.Face
	wallFace   *topo.Face
	scale      float64
}

// slack is the arithmetic allowance for a value computed from the tags.
func (r rim) slack() float64 { return 64 * ulp * r.scale }

// looksLikeRim reports whether edge is a candidate for this route: a closed
// retained Circle3 between exactly one retained Plane and one retained
// Cylinder, neither with a stated tag departure. It reads no mesh and refuses
// nothing.
func looksLikeRim(edge *topo.Edge) bool {
	if edge == nil || edge.Start == nil || edge.Start != edge.End {
		return false
	}
	_, ok := edge.Curve.(surfacegeom.Circle3)
	return ok && planeAndWall(edge)
}

// looksLikeArc reports whether edge is a candidate for the open-arc route: a
// retained Arc3 between exactly one retained Plane and one retained Cylinder,
// with two distinct ends.
func looksLikeArc(edge *topo.Edge) bool {
	if edge == nil || edge.Start == nil || edge.End == nil || edge.Start == edge.End {
		return false
	}
	_, ok := edge.Curve.(surfacegeom.Arc3)
	return ok && planeAndWall(edge)
}

// planeAndWall reports whether edge is a retained curve between exactly one
// retained Plane and one retained Cylinder, neither with a stated tag
// departure.
func planeAndWall(edge *topo.Edge) bool {
	if !edge.RetainedCurve || len(edge.Faces) != 2 {
		return false
	}
	planes, cylinders := 0, 0
	for _, f := range edge.Faces {
		if f == nil || !f.RetainedTag || f.TagDeparture != 0 || f.Denoted != nil {
			return false
		}
		switch f.Surface.(type) {
		case surfacegeom.Plane:
			planes++
		case surfacegeom.Cylinder:
			cylinders++
		}
	}
	return planes == 1 && cylinders == 1
}

// circleOf is the centre and radius of a circle or an arc curve.
func circleOf(curve surfacegeom.Curve) (r3.Vec, units.Value, bool) {
	switch c := curve.(type) {
	case surfacegeom.Circle3:
		return c.Center, c.Radius, true
	case surfacegeom.Arc3:
		return c.Center, c.Radius, true
	}
	return r3.Vec{}, units.Value{}, false
}

// readRim reads the rim of edge from its two tags and the held wall. It
// refuses (SR1, SR6) an edge whose tags and held mesh do not describe one rim.
func readRim(edge *topo.Edge, fp facetedrecord.Payload, faceIndex map[*topo.Face]int) (rim, error) {
	var r rim
	for _, f := range edge.Faces {
		if _, ok := f.Surface.(surfacegeom.Plane); ok {
			r.planeFace = f
		} else {
			r.wallFace = f
		}
	}
	plane, planeOK := r.planeFace.Surface.(surfacegeom.Plane)
	cyl, cylOK := r.wallFace.Surface.(surfacegeom.Cylinder)
	circleCentre, circleRadiusValue, circleOK := circleOf(edge.Curve)
	if !planeOK || !cylOK || !circleOK {
		return rim{}, fmt.Errorf("%w: the rim's faces or curve changed kind", decaderr.ErrUnsupported)
	}
	radius, err := cyl.Radius.In(units.Millimeter)
	if err != nil || !(radius > 0) || math.IsInf(radius, 0) {
		return rim{}, fmt.Errorf("%w: the rim's cylinder has no finite radius", decaderr.ErrUnsupported)
	}
	n, ok := plane.Frame.N().Normalize()
	if !ok {
		return rim{}, fmt.Errorf("%w: the rim's plane has no normal", decaderr.ErrUnsupported)
	}
	if r.planeFace.Reversed {
		n = n.Scale(-1)
	}
	axis, ok := cyl.Axis.Normalize()
	if !ok || axis.Cross(n).Len() > parallelTol {
		return rim{}, fmt.Errorf("%w: the rim's cylinder axis is not normal to its plane", decaderr.ErrUnsupported)
	}
	r.scale = 1
	for _, v := range fp.Verts {
		r.scale = max(r.scale, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	o := plane.Frame.Origin()
	center := cyl.Origin.Add(axis.Scale(n.Dot(o.Sub(cyl.Origin)) / n.Dot(axis)))
	circleRadius, err := circleRadiusValue.In(units.Millimeter)
	if err != nil {
		return rim{}, fmt.Errorf("%w: the rim's circle has no radius", decaderr.ErrUnsupported)
	}
	r.center, r.n, r.radius = center, n, radius
	slack := r.slack()
	if center.Sub(circleCentre).Len() > slack || math.Abs(circleRadius-radius) > slack {
		return rim{}, fmt.Errorf("%w: the rim's circle is not the meeting of its two tags", decaderr.ErrUnsupported)
	}
	r.u, ok = perpendicular(n)
	if !ok {
		return rim{}, fmt.Errorf("%w: the rim's plane has no frame", decaderr.ErrUnsupported)
	}
	r.w = n.Cross(r.u)
	r.matOutside = r.wallFace.Reversed
	// The wall extends away from the plane's outward normal into the material
	// (a hole, a shaft end) or toward it (a boss, a pocket floor).
	wall := faceIndex[r.wallFace]
	below, above := 0, 0
	for i, tri := range fp.Tris {
		if fp.FaceOf[i] != wall {
			continue
		}
		for _, vi := range tri {
			z := r.height(fp.Verts[vi])
			switch {
			case z < -slack-fp.VertexBound[vi]:
				below++
			case z > slack+fp.VertexBound[vi]:
				above++
			}
		}
	}
	switch {
	case below > 0 && above == 0:
		r.side = -1
	case above > 0 && below == 0:
		r.side = 1
	default:
		return rim{}, fmt.Errorf("%w: the rim's wall does not lie on one side of its plane", decaderr.ErrUnsupported)
	}
	r.convex = r.side < 0
	if edge.Convex != r.convex {
		return rim{}, fmt.Errorf("%w: the rim's convexity disagrees with its two tags", decaderr.ErrUnsupported)
	}
	r.kappa = -1
	if r.convex == r.matOutside {
		r.kappa = 1
	}
	return r, nil
}

func perpendicular(n r3.Vec) (r3.Vec, bool) {
	pick := r3.NewVec(1, 0, 0)
	if math.Abs(n.X) > 0.9 {
		pick = r3.NewVec(0, 1, 0)
	}
	return n.Cross(pick).Normalize()
}

// height is the signed distance of p from the rim's plane, along the plane's
// outward normal.
func (r rim) height(p r3.Vec) float64 { return r.n.Dot(p.Sub(r.center)) }

// radial is the distance of p from the rim's axis.
func (r rim) radial(p r3.Vec) float64 {
	q := p.Sub(r.center)
	return q.Sub(r.n.Scale(q.Dot(r.n))).Len()
}

// angle is the counter-clockwise angle of p about the axis, from r.u.
func (r rim) angle(p r3.Vec) float64 {
	q := p.Sub(r.center)
	return math.Atan2(q.Dot(r.w), q.Dot(r.u))
}

// at is the point at radial distance rho, angle phi and height z.
func (r rim) at(rho, phi, z float64) r3.Vec {
	return r.center.Add(r.u.Scale(rho * math.Cos(phi))).Add(r.w.Scale(rho * math.Sin(phi))).Add(r.n.Scale(z))
}

// onRim reports whether the held vertex v lies within its bound of the rim
// circle (the falsifier of retention §10.1 E6).
func (r rim) onRim(v r3.Vec, bound float64) bool {
	return math.Hypot(r.radial(v)-r.radius, r.height(v)) <= bound+r.slack()
}

// checkSize refuses a blend that has no regular surface (SR3).
func (r rim) checkSize(s blendSpec) error {
	ta, tb := s.planeSetback(), s.wallSetback()
	if !(ta > 0) || !(tb > 0) || math.IsInf(ta, 0) || math.IsInf(tb, 0) {
		return fmt.Errorf("%w: a blend needs a positive finite size", decaderr.ErrDegenerate)
	}
	if r.kappa < 0 && !(r.radius-ta > r.slack()) {
		return fmt.Errorf("%w: the blend reaches the rim's axis: setback %g mm against radius %g mm",
			decaderr.ErrDegenerate, ta, r.radius)
	}
	return nil
}

// rimKey identifies a rim across rewrites of the mesh: its centre, axis and
// radius.
type rimKey struct {
	center, n r3.Vec
	radius    float64
}

func (r rim) key() rimKey { return rimKey{r.center, r.n, r.radius} }

// findRim returns the retained-circle edge of body whose circle is key's, and
// false when there is none or more than one.
func findRim(body *topo.Body, key rimKey, slack float64) (*topo.Edge, bool) {
	var found *topo.Edge
	for _, e := range topobuild.Edges(body) {
		c, ok := e.Curve.(surfacegeom.Circle3)
		if !ok || !looksLikeRim(e) {
			continue
		}
		radius, err := c.Radius.In(units.Millimeter)
		if err != nil || math.Abs(radius-key.radius) > slack || c.Center.Sub(key.center).Len() > slack {
			continue
		}
		if found != nil {
			return nil, false
		}
		found = e
	}
	return found, found != nil
}

// faceIndexOf maps each face of body to its index in topobuild.Faces order,
// which facetedrecord.Payload.FaceOf uses.
func faceIndexOf(body *topo.Body) map[*topo.Face]int {
	out := map[*topo.Face]int{}
	for i, f := range topobuild.Faces(body) {
		out[f] = i
	}
	return out
}

// checkFinite refuses non-finite parameters.
func checkFinite(vals ...float64) error {
	if slices.ContainsFunc(vals, proofbound.IsNonFinite) {
		return fmt.Errorf("%w: a blend parameter is not finite", decaderr.ErrDegenerate)
	}
	return nil
}

// findLine returns the retained straight edge of body whose corners are key's,
// and false when there is none or more than one.
func findLine(body *topo.Body, key lineKey, slack float64) (*topo.Edge, bool) {
	return findEdge(body, key, slack, looksLikeLine)
}

// findEdge returns the edge of body that satisfies ok and whose corners are
// key's, and false when there is none or more than one.
func findEdge(body *topo.Body, key lineKey, slack float64, ok func(*topo.Edge) bool) (*topo.Edge, bool) {
	var found *topo.Edge
	for _, e := range topobuild.Edges(body) {
		if !ok(e) {
			continue
		}
		p, q := e.Start.Position, e.End.Position
		near := func(a, b r3.Vec) bool { return a.Sub(b).Len() <= slack+e.Start.Bound.Base() }
		forward := near(p, key.p0) && near(q, key.p1)
		backward := near(p, key.p1) && near(q, key.p0)
		if !forward && !backward {
			continue
		}
		if found != nil {
			return nil, false
		}
		found = e
	}
	return found, found != nil
}
