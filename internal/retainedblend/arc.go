package retainedblend

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// arcRim is an open circular arc between a retained plane and a coaxial
// retained cylinder, ending on retained planes parallel to the axis (§4.4).
type arcRim struct {
	rim
	ends [2]lineEnd // the arc's end on each end face; exact is the ring point there
	// sign is +1 when the angle about the plane's normal grows from end 0 to
	// end 1 along the arc, and -1 when it shrinks.
	sign float64
	// theta0 is the signed angle theta = sign·angle of end 0, and span how far
	// theta grows to end 1, in (0, 2π).
	theta0, span float64
}

// theta is the signed angle of p about the axis, unwrapped to lie within π of
// ref.
func (a arcRim) theta(p r3.Vec, ref float64) float64 {
	t := a.sign * a.angle(p)
	return t + 2*math.Pi*math.Round((ref-t)/(2*math.Pi))
}

// ringAzimuths are the two angles at which the circle of radius rho in the
// plane at height z meets end plane g, as theta values within π of ref. They
// are the roots of ρ(a·cos φ + b·sin φ) = h for the end plane's normal
// (a, b) in the rim's frame and offset h.
func (a arcRim) ringAzimuths(rho float64, g plane, ref float64) ([2]float64, bool) {
	ca, cb := g.n.Dot(a.u), g.n.Dot(a.w)
	h := g.n.Dot(g.o.Sub(a.center))
	norm := math.Hypot(ca, cb)
	if norm == 0 || math.Abs(h) >= rho*norm {
		return [2]float64{}, false
	}
	beta := math.Atan2(cb, ca)
	off := math.Acos(h / (rho * norm))
	var out [2]float64
	for i, phi := range []float64{beta + off, beta - off} {
		t := a.sign * phi
		out[i] = t + 2*math.Pi*math.Round((ref-t)/(2*math.Pi))
	}
	return out, true
}

// at is the point at radius rho, signed angle theta and height z.
func (a arcRim) atTheta(rho, theta, z float64) r3.Vec { return a.at(rho, a.sign*theta, z) }

// looksLikeArcEdge is looksLikeArc.
func readArc(edge *topo.Edge, fp facetedrecord.Payload, faceIndex map[*topo.Face]int) (arcRim, error) {
	var a arcRim
	r, err := readRim(edge, fp, faceIndex)
	if err != nil {
		return arcRim{}, err
	}
	a.rim = r
	arc, ok := edge.Curve.(surfacegeom.Arc3)
	if !ok {
		return arcRim{}, fmt.Errorf("%w: the edge's curve changed kind", decaderr.ErrUnsupported)
	}
	a.sign = 1
	if arc.Axis.Dot(r.n) < 0 {
		a.sign = -1
	}
	fa, fb := faceIndex[r.planeFace], faceIndex[r.wallFace]
	for i, vertex := range []*topo.Vertex{edge.Start, edge.End} {
		end, err := a.readEnd(vertex, fp, faceIndex, fa, fb)
		if err != nil {
			return arcRim{}, err
		}
		a.ends[i] = end
	}
	if a.ends[0].face == a.ends[1].face {
		return arcRim{}, fmt.Errorf("%w: both ends of the arc meet one face", decaderr.ErrUnsupported)
	}
	a.theta0 = a.sign * a.angle(a.ends[0].exact)
	span := a.sign*a.angle(a.ends[1].exact) - a.theta0
	span = math.Mod(span, 2*math.Pi)
	if span < 0 {
		span += 2 * math.Pi
	}
	if !(span > minSine) || !(span < 2*math.Pi-minSine) {
		return arcRim{}, fmt.Errorf("%w: the arc's ends coincide", decaderr.ErrUnsupported)
	}
	a.span = span
	return a, nil
}

func (a *arcRim) readEnd(vertex *topo.Vertex, fp facetedrecord.Payload, faceIndex map[*topo.Face]int,
	fa, fb int) (lineEnd, error) {
	var end lineEnd
	var err error
	if end.v, end.face, err = cornerOf(vertex, fp, faceIndex, fa, fb); err != nil {
		return lineEnd{}, err
	}
	g, ok := retainedPlane(end.face)
	if !ok {
		return lineEnd{}, fmt.Errorf("%w: an arc end meets a face that is not a retained plane", decaderr.ErrUnsupported)
	}
	if math.Abs(g.n.Dot(a.n)) > parallelTol {
		return lineEnd{}, fmt.Errorf("%w: an arc end meets a face that is not parallel to the axis", decaderr.ErrUnsupported)
	}
	end.g = g
	a.charge = addCharge(a.charge, g)
	held := fp.Verts[end.v]
	roots, ok := a.ringAzimuths(a.radius, g, a.sign*a.angle(held))
	if !ok {
		return lineEnd{}, fmt.Errorf("%w: an arc end face does not cut the rim", decaderr.ErrUnsupported)
	}
	// The end face must cross the rim at a real angle: its offset from the axis
	// is short of the radius by more than minSine allows.
	if h := math.Abs(g.n.Dot(g.o.Sub(a.center))); h > a.radius*math.Sqrt(1-minSine*minSine) {
		return lineEnd{}, fmt.Errorf("%w: an arc end face meets the cylinder at a grazing angle", decaderr.ErrUnsupported)
	}
	best := math.Inf(1)
	for _, t := range roots {
		cand := a.atTheta(a.radius, t, 0)
		if d := cand.Sub(held).Len(); d < best {
			best, end.exact = d, cand
		}
	}
	if best > fp.VertexBound[end.v]+a.slack() {
		return lineEnd{}, fmt.Errorf("%w: an arc end is not where its three tags meet", decaderr.ErrUnsupported)
	}
	return end, nil
}

// key identifies the arc across rewrites by its two ends.
func (a arcRim) arcKey() lineKey { return lineKey{a.ends[0].exact, a.ends[1].exact} }

// findArc returns the retained arc of body whose ends are key's.
func findArc(body *topo.Body, key lineKey, slack float64) (*topo.Edge, bool) {
	return findEdge(body, key, slack, looksLikeArc)
}
