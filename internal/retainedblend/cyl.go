package retainedblend

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// cylLine is a straight edge between a retained plane and a retained cylinder
// whose axis lies in the plane, ending on retained planes normal to the axis
// (§4.3).
type cylLine struct {
	faceA, faceB *topo.Face
	a            plane
	axis, origin r3.Vec // the axis direction, end 0 to end 1, and a point on it
	radius       float64
	matOutside   bool
	ends         [2]lineEnd // the exact corner of each end
	edgeConvex   bool
	scale        float64
}

func (c cylLine) slack() float64 { return 64 * ulp * c.scale }

func (c cylLine) key() lineKey { return lineKey{c.ends[0].exact, c.ends[1].exact} }

// axisPoint is the point of the axis in the plane through p normal to it.
func (c cylLine) axisPoint(p r3.Vec) r3.Vec {
	return c.origin.Add(c.axis.Scale(p.Sub(c.origin).Dot(c.axis)))
}

// looksLikeCylLine reports whether edge is a candidate for the plane-cylinder
// line route: a retained Line3 between one retained Plane and one retained
// Cylinder, with two end corners.
func looksLikeCylLine(edge *topo.Edge) bool {
	if edge == nil || edge.Start == nil || edge.End == nil || edge.Start == edge.End ||
		!edge.RetainedCurve || len(edge.Faces) != 2 {
		return false
	}
	if _, ok := edge.Curve.(surfacegeom.Line3); !ok {
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

// readCylLine reads the edge from its tags and the held mesh. It refuses an
// edge whose tags and held mesh do not describe one corner at each end.
func readCylLine(edge *topo.Edge, fp facetedrecord.Payload, faceIndex map[*topo.Face]int) (cylLine, error) {
	var c cylLine
	for _, f := range edge.Faces {
		if _, ok := f.Surface.(surfacegeom.Plane); ok {
			c.faceA = f
		} else {
			c.faceB = f
		}
	}
	c.a, _ = retainedPlane(c.faceA)
	cyl, ok := c.faceB.Surface.(surfacegeom.Cylinder)
	if !ok {
		return cylLine{}, fmt.Errorf("%w: the edge's wall changed kind", decaderr.ErrUnsupported)
	}
	radius, err := cyl.Radius.In(units.Millimeter)
	if err != nil || !(radius > 0) || math.IsInf(radius, 0) {
		return cylLine{}, fmt.Errorf("%w: the edge's cylinder has no finite radius", decaderr.ErrUnsupported)
	}
	axis, ok := cyl.Axis.Normalize()
	if !ok || math.Abs(axis.Dot(c.a.n)) > parallelTol {
		return cylLine{}, fmt.Errorf("%w: the edge's cylinder axis does not lie in its plane", decaderr.ErrUnsupported)
	}
	c.axis, c.origin, c.radius = axis, cyl.Origin, radius
	c.matOutside = c.faceB.Reversed
	c.edgeConvex = edge.Convex
	c.scale = 1
	for _, v := range fp.Verts {
		c.scale = max(c.scale, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	fa, fb := faceIndex[c.faceA], faceIndex[c.faceB]
	for i, vertex := range []*topo.Vertex{edge.Start, edge.End} {
		end, err := c.readEnd(vertex, fp, faceIndex, fa, fb)
		if err != nil {
			return cylLine{}, err
		}
		c.ends[i] = end
	}
	if c.axis.Dot(c.ends[1].exact.Sub(c.ends[0].exact)) < 0 {
		c.axis = c.axis.Scale(-1)
	}
	if c.ends[0].face == c.ends[1].face {
		return cylLine{}, fmt.Errorf("%w: both ends of the edge meet one face", decaderr.ErrUnsupported)
	}
	return c, nil
}

func (c *cylLine) readEnd(vertex *topo.Vertex, fp facetedrecord.Payload, faceIndex map[*topo.Face]int,
	fa, fb int) (lineEnd, error) {
	var end lineEnd
	var err error
	if end.v, end.face, err = cornerOf(vertex, fp, faceIndex, fa, fb); err != nil {
		return lineEnd{}, err
	}
	g, ok := retainedPlane(end.face)
	if !ok {
		return lineEnd{}, fmt.Errorf("%w: an edge end meets a face that is not a retained plane", decaderr.ErrUnsupported)
	}
	if g.n.Cross(c.axis).Len() > parallelTol {
		return lineEnd{}, fmt.Errorf("%w: an edge end meets a face that is not normal to the axis", decaderr.ErrUnsupported)
	}
	end.g = g
	// The corner lies in the end plane, on the plane's meeting with the circle
	// the cylinder cuts there.
	o := c.axisPoint(g.o)
	s := c.a.offset(o)
	half := c.radius*c.radius - s*s
	if half < (minSine*c.radius)*(minSine*c.radius) {
		return lineEnd{}, fmt.Errorf("%w: the plane meets the cylinder at a grazing angle", decaderr.ErrUnsupported)
	}
	side := c.a.n.Cross(c.axis)
	base := o.Sub(c.a.n.Scale(s))
	held := fp.Verts[end.v]
	best := math.Inf(1)
	for _, sign := range []float64{1, -1} {
		cand := base.Add(side.Scale(sign * math.Sqrt(half)))
		if d := cand.Sub(held).Len(); d < best {
			best, end.exact = d, cand
		}
	}
	if best > fp.VertexBound[end.v]+c.slack() {
		return lineEnd{}, fmt.Errorf("%w: an edge end is not where its three tags meet", decaderr.ErrUnsupported)
	}
	return end, nil
}

// findCylLine returns the retained plane-cylinder line of body whose corners
// are key's, and false when there is none or more than one.
func findCylLine(body *topo.Body, key lineKey, slack float64) (*topo.Edge, bool) {
	return findEdge(body, key, slack, looksLikeCylLine)
}
