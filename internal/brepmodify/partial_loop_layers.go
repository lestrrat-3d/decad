package brepmodify

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// maxPartialLayers is the most layers of rings a chain cutter steps through.
const maxPartialLayers = 32

// sharpArcCorner reports whether the run's chain holds a sharp corner between two segments of which at least one is an
// arc. The offset of such a corner moves along a conic as the setback grows, so a straight rule between two rings
// leaves the bevel there.
func (r chamferRun) sharpArcCorner() bool {
	for j := 0; j+1 < r.count; j++ {
		if a, b := r.seg(j), r.seg(j+1); (a.arc || b.arc) && !continues(a, b) {
			return true
		}
	}
	return false
}

// layers is how many layers of rings the chain's cutter steps through between the heights top and bottom, and how far
// its facets then leave the bevel (layerDeparture). A chain without a sharp corner beside an arc keeps one layer, two
// rings, which holds the bevel exactly there. Otherwise the layers step up from 2 by one until the departure is within
// partialSagitta, the fewest that hold it, and the cutter's bound carries it. Only the cells beside a curved column
// carry the rings between (cutterMesh).
func (r chamferRun) layers(d, top, bottom float64) (int, float64, error) {
	if !r.sharpArcCorner() {
		return 1, 0, nil
	}
	for layers := 2; layers <= maxPartialLayers; layers++ {
		heights := make([]float64, layers+1)
		for k := range heights {
			heights[k] = top + (bottom-top)*float64(k)/float64(layers)
		}
		dep, err := r.layerDeparture(d, heights)
		if err != nil {
			return 0, 0, err
		}
		if dep <= partialSagitta {
			return layers, dep, nil
		}
	}
	return 0, 0, fmt.Errorf(`%w: a chamfer corner beside an arc needs more than %d layers to hold the bevel`,
		decaderr.ErrUnsupported, maxPartialLayers)
}

// layerDeparture bounds how far a layered cutter's facets leave the bevel at the chain's sharp corners beside an arc,
// for rings at the given heights. It is the largest, over the corners and the layers, of the sum of two terms.
//
// The first is the corner's own path. Between two rings the corner is joined by a straight edge, and the true corner
// moves along a conic. Over a layer of height dt the straight edge lies within dt^2/8 of the path's largest second
// derivative. At a corner between a line and an arc, let v be the distance along the line from the foot of the arc's
// centre. With rho = r + sigma t and u = h + lambda t (sigma^2 = lambda^2 = 1), v^2 = rho^2 - u^2 is linear in the
// offset t, so d2v/dt2 = -B^2/(4 v^3) with B = d(v^2)/dt. At a corner between two arcs, x along the line of centres is
// linear in t (the squares of the radii differ by a linear function), and y^2 = a + b t + c t^2, so
// |d2y/dt2| = |4ac - b^2|/(4 y^3). The offset of the other coordinate is linear in t. Both are largest where v or y is
// smallest over the layer.
//
// The second is the twist. An arc's cell between two rings joins samples at azimuths about its centre that differ by at
// most the angle dphi that the corner turns about it over the layer, and a chord of that turn at radius rho lies within
// rho dphi^2/8 of the cone.
//
// A corner that reaches the arc's foot, where v or y is not positive over the layer, is refused: the bound has no
// finite value there.
func (r chamferRun) layerDeparture(d float64, heights []float64) (float64, error) {
	return r.layerDepartureWith(d, heights, partialSagitta)
}

// layerDepartureWith is layerDeparture for a cutter whose arc chords lie within
// sagitta of their circles: the twist's cross term reads that sagitta.
func (r chamferRun) layerDepartureWith(d float64, heights []float64, sagitta float64) (float64, error) {
	l := r.loop
	worst := 0.0
	for j := 0; j+1 < r.count; j++ {
		a, b := r.seg(j), r.seg(j+1)
		if (!a.arc && !b.arc) || continues(a, b) {
			continue
		}
		corners := make([]pt2, len(heights))
		for k, h := range heights {
			c, err := l.corner(a, b, d+h)
			if err != nil {
				return 0, err
			}
			corners[k] = c
		}
		for k := 0; k+1 < len(heights); k++ {
			dt := math.Abs(heights[k+1] - heights[k])
			p, q := corners[k], corners[k+1]
			dep := 0.0
			for _, s := range []loopSeg{a, b} {
				if s.arc {
					dep = max(dep, twistWith(s, p, q, sagitta))
				}
			}
			path, err := l.cornerPathDeparture(a, b, d+heights[k], d+heights[k+1], p, q)
			if err != nil {
				return 0, err
			}
			worst = max(worst, dep+path*dt*dt/8)
		}
	}
	return worst, nil
}

// twistWith is how far a chord of the arc segment s, whose end at the corner moves from p to q, lies inside the cone: the
// larger radius times the square of the turn about the arc's centre, over 8.
// The cutter's chords lie within sagitta of their circles.
func twistWith(s loopSeg, p, q pt2, sagitta float64) float64 {
	rho := max(p.dist(s.c), q.dist(s.c))
	dphi := math.Abs(math.Remainder(math.Atan2(q.y-s.c.y, q.x-s.c.x)-math.Atan2(p.y-s.c.y, p.x-s.c.x), 2*math.Pi))
	// Each quad of the cutter is split along a diagonal that spans the chord step of the arc, whose sagitta is at most
	// partialSagitta, and the turn dphi together: its sag is rho (step + dphi)^2 / 8 = (sqrt(sagitta) + sqrt(twist))^2.
	// The sagitta itself is charged apart, so the twist carries the rest, with the cross term.
	own := rho * dphi * dphi / 8
	return own + 2*math.Sqrt(sagitta*own)
}

// cornerPathDeparture is the largest magnitude of the second derivative, with respect to the offset t, of the path of
// the sharp corner of a and b, over t0 to t1, where the corner is at p at t0 and at q at t1.
func (l *chamferLoop) cornerPathDeparture(a, b loopSeg, t0, t1 float64, p, q pt2) (float64, error) {
	foot := fmt.Errorf(`%w: a chamfer corner beside an arc meets the arc's foot`, decaderr.ErrUnsupported)
	if a.arc != b.arc {
		arc, line := a, b
		if !a.arc {
			arc, line = b, a
		}
		dir := line.p1.sub(line.p0).unit()
		vp, vq := p.sub(arc.c).dot(dir), q.sub(arc.c).dot(dir)
		// v^2 is linear in t, so v does not change sign unless it reaches 0.
		if !(vp*vq > 0) {
			return 0, foot
		}
		bb := (vq*vq - vp*vp) / (t1 - t0)
		vmin := min(math.Abs(vp), math.Abs(vq))
		return bb * bb / (4 * vmin * vmin * vmin), nil
	}
	// Two arcs. Work in the frame of the line of centres, with the first arc's centre at the origin.
	dist := a.c.dist(b.c)
	if dist < 1e-9 {
		return 0, foot
	}
	sigma := func(s loopSeg) float64 {
		if l.toward(s) {
			return -1
		}
		return 1
	}
	sa, sb := sigma(a), sigma(b)
	x0 := (dist*dist + a.r*a.r - b.r*b.r) / (2 * dist)
	x1 := (sa*a.r - sb*b.r) / dist
	// y^2 = (a.r + sa t)^2 - (x0 + x1 t)^2.
	ca, cb, cc := a.r*a.r-x0*x0, 2*sa*a.r-2*x0*x1, 1-x1*x1
	g := func(t float64) float64 { return ca + cb*t + cc*t*t }
	gmin := min(g(t0), g(t1))
	if cc > 0 {
		if tv := -cb / (2 * cc); tv > min(t0, t1) && tv < max(t0, t1) {
			gmin = min(gmin, g(tv))
		}
	}
	// The corner stays on one side of the line of centres.
	axis := b.c.sub(a.c)
	if !(gmin > 0) || !(axis.cross(p.sub(a.c))*axis.cross(q.sub(a.c)) > 0) {
		return 0, foot
	}
	return math.Abs(4*ca*cc-cb*cb) / (4 * gmin * math.Sqrt(gmin)), nil
}
