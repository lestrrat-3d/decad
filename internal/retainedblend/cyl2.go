package retainedblend

import (
	"fmt"
	"math"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// cylWall is one cylinder of a cylinder-cylinder edge.
type cylWall struct {
	face       *topo.Face
	origin     r3.Vec
	radius     float64
	matOutside bool
}

// axisPoint is the point of the wall's axis in the plane through p normal to
// the shared direction axis.
func (c cylWall) axisPoint(axis, p r3.Vec) r3.Vec {
	return c.origin.Add(axis.Scale(p.Sub(c.origin).Dot(axis)))
}

// cylCyl is a straight edge between two retained cylinders with parallel
// axes, ending on retained planes normal to the axes (R5,
// agent-docs/retained-modify-reach-design.md §8).
type cylCyl struct {
	wall       [2]cylWall
	axis       r3.Vec // the shared direction, end 0 to end 1
	ends       [2]lineEnd
	edgeConvex bool
	scale      float64
	// charge is the plane displacement the end planes add to slack.
	charge float64
}

func (c cylCyl) slack() float64 { return chargedSlack(c.scale, c.charge) }

func (c cylCyl) key() lineKey { return lineKey{c.ends[0].exact, c.ends[1].exact} }

// looksLikeCylCyl reports whether edge is a candidate for the parallel
// cylinder route: a retained Line3 between two distinct retained Cylinders
// whose axes are parallel, with two end corners.
func looksLikeCylCyl(edge *topo.Edge) bool {
	if edge == nil || edge.Start == nil || edge.End == nil || edge.Start == edge.End ||
		!edge.RetainedCurve || len(edge.Faces) != 2 || edge.Faces[0] == edge.Faces[1] {
		return false
	}
	if _, ok := edge.Curve.(surfacegeom.Line3); !ok {
		return false
	}
	var axes [2]r3.Vec
	for i, f := range edge.Faces {
		if f == nil || !f.RetainedTag || f.TagDeparture != 0 || !tagIsRecord(f) {
			return false
		}
		cyl, ok := f.Surface.(surfacegeom.Cylinder)
		if !ok {
			return false
		}
		axes[i] = cyl.Axis
	}
	return axes[0].Cross(axes[1]).Len() <= parallelTol
}

// readCylCyl reads the edge from its tags and the held mesh.
func readCylCyl(edge *topo.Edge, fp facetedrecord.Payload, faceIndex map[*topo.Face]int) (cylCyl, error) {
	var c cylCyl
	for i, f := range edge.Faces {
		cyl, ok := f.Surface.(surfacegeom.Cylinder)
		if !ok {
			return cylCyl{}, fmt.Errorf("%w: the edge's wall changed kind", decaderr.ErrUnsupported)
		}
		radius, err := cyl.Radius.In(units.Millimeter)
		if err != nil || !(radius > 0) || math.IsInf(radius, 0) {
			return cylCyl{}, fmt.Errorf("%w: the edge's cylinder has no finite radius", decaderr.ErrUnsupported)
		}
		axis, ok := cyl.Axis.Normalize()
		if !ok {
			return cylCyl{}, fmt.Errorf("%w: the edge's cylinder has no axis", decaderr.ErrUnsupported)
		}
		if i == 0 {
			c.axis = axis
		}
		c.wall[i] = cylWall{face: f, origin: cyl.Origin, radius: radius, matOutside: f.Reversed}
	}
	c.edgeConvex = edge.Convex
	c.scale = 1
	for _, v := range fp.Verts {
		c.scale = max(c.scale, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	fa, fb := faceIndex[c.wall[0].face], faceIndex[c.wall[1].face]
	for i, vertex := range []*topo.Vertex{edge.Start, edge.End} {
		end, err := c.readEnd(vertex, fp, faceIndex, fa, fb)
		if err != nil {
			return cylCyl{}, err
		}
		c.ends[i] = end
	}
	if c.axis.Dot(c.ends[1].exact.Sub(c.ends[0].exact)) < 0 {
		c.axis = c.axis.Scale(-1)
	}
	if c.ends[0].face == c.ends[1].face {
		return cylCyl{}, fmt.Errorf("%w: both ends of the edge meet one face", decaderr.ErrUnsupported)
	}
	return c, nil
}

func (c *cylCyl) readEnd(vertex *topo.Vertex, fp facetedrecord.Payload, faceIndex map[*topo.Face]int,
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
		return lineEnd{}, fmt.Errorf("%w: an edge end meets a face that is not normal to the axes", decaderr.ErrUnsupported)
	}
	end.g = g
	c.charge = addCharge(c.charge, g)
	// The corner lies in the end plane, where the two circles the cylinders cut
	// there meet.
	o1, o2 := c.wall[0].axisPoint(c.axis, g.o), c.wall[1].axisPoint(c.axis, g.o)
	r1, r2 := c.wall[0].radius, c.wall[1].radius
	rel := o2.Sub(o1)
	s := rel.Len()
	if !(s > math.Abs(r1-r2)+minSine*r1) || !(s < r1+r2-minSine*r1) {
		return lineEnd{}, fmt.Errorf("%w: the two cylinders do not cross at a clear angle", decaderr.ErrUnsupported)
	}
	toward := rel.Scale(1 / s)
	side := c.axis.Cross(toward)
	x := (s*s + r1*r1 - r2*r2) / (2 * s)
	h := math.Sqrt(max(r1*r1-x*x, 0))
	held := fp.Verts[end.v]
	best := math.Inf(1)
	for _, sign := range []float64{1, -1} {
		cand := o1.Add(toward.Scale(x)).Add(side.Scale(sign * h))
		if d := cand.Sub(held).Len(); d < best {
			best, end.exact = d, cand
		}
	}
	if best > fp.VertexBound[end.v]+c.slack() {
		return lineEnd{}, fmt.Errorf("%w: an edge end is not where its three tags meet", decaderr.ErrUnsupported)
	}
	return end, nil
}

// findCylCyl returns the retained cylinder-cylinder line of body whose corners
// are key's, and false when there is none or more than one.
func findCylCyl(body *topo.Body, key lineKey, slack float64) (*topo.Edge, bool) {
	return findEdge(body, key, slack, looksLikeCylCyl)
}

// cylSection is the cross-section of a cylinder-cylinder blend in the plane
// normal to the axes through the first end.
type cylSection struct {
	t      [2]r3.Vec // tangent points on wall 0 and wall 1
	centre r3.Vec
	rows   []r3.Vec // the blend's rows, from t[0] to t[1]
	phi    float64
	theta  [2]float64 // angle of each tangent point about its axis, from the corner into the face
}

// solveSection finds the tangent points of the blend of the corner k0
// (agent-docs/retained-modify-reach-design.md §8, R5). o are the walls' axis
// points in the plane, u the radial unit vectors at k0 and w the unit tangents
// into each wall's face.
func (c cylCyl) solveSection(spec blendSpec, k0 r3.Vec, o, u, w [2]r3.Vec, convex bool, tau,
	margin float64) (cylSection, error) {
	var s cylSection
	if !spec.fillet {
		d := [2]float64{spec.dA, spec.dB}
		for k := range 2 {
			s.theta[k] = d[k] / c.wall[k].radius
			if !(s.theta[k] > minSine) || !(s.theta[k] < math.Pi-minSine) {
				return s, fmt.Errorf("%w: the chamfer's setback does not fit on the wall", decaderr.ErrDegenerate)
			}
			R := c.wall[k].radius
			s.t[k] = o[k].Add(u[k].Scale(math.Cos(s.theta[k]) * R)).Add(w[k].Scale(math.Sin(s.theta[k]) * R))
		}
		s.rows = []r3.Vec{s.t[0], s.t[1]}
		return s, nil
	}
	r := spec.r
	var rho [2]float64
	for k := range 2 {
		kappa := -1.0
		if convex == c.wall[k].matOutside {
			kappa = 1
		}
		rho[k] = c.wall[k].radius + kappa*r
		if !(rho[k] > margin) {
			return s, fmt.Errorf("%w: the blend reaches a cylinder's axis: radius %g mm against %g mm",
				decaderr.ErrDegenerate, r, c.wall[k].radius)
		}
	}
	rel := o[1].Sub(o[0])
	d := rel.Len()
	if !(d > margin) {
		return s, fmt.Errorf("%w: the two cylinders share an axis", decaderr.ErrUnsupported)
	}
	if !(d > math.Abs(rho[0]-rho[1])) || !(d < rho[0]+rho[1]) {
		return s, fmt.Errorf("%w: no ball of that radius touches both faces", decaderr.ErrDegenerate)
	}
	toward := rel.Scale(1 / d)
	side := c.axis.Cross(toward)
	x := (d*d + rho[0]*rho[0] - rho[1]*rho[1]) / (2 * d)
	h := math.Sqrt(max(rho[0]*rho[0]-x*x, 0))
	best := math.Inf(1)
	for _, sign := range []float64{1, -1} {
		cand := o[0].Add(toward.Scale(x)).Add(side.Scale(sign * h))
		if dist := cand.Sub(k0).Len(); dist < best {
			best, s.centre = dist, cand
		}
	}
	for k := range 2 {
		cd := s.centre.Sub(o[k])
		s.t[k] = o[k].Add(cd.Scale(c.wall[k].radius / cd.Len()))
		q := s.t[k].Sub(o[k])
		s.theta[k] = math.Atan2(q.Dot(w[k]), q.Dot(u[k]))
		if !(s.theta[k] > minSine) || !(s.theta[k] < math.Pi-minSine) {
			return s, fmt.Errorf("%w: the blend's tangent point does not lie on a wall's face", decaderr.ErrUnsupported)
		}
	}
	a0 := s.t[0].Sub(s.centre).Scale(1 / r)
	a1 := s.t[1].Sub(s.centre).Scale(1 / r)
	s.phi = math.Acos(math.Max(-1, math.Min(1, a0.Dot(a1))))
	if !(s.phi > minSine) || !(s.phi < math.Pi-minSine) {
		return s, fmt.Errorf("%w: the blend's arc is degenerate", decaderr.ErrDegenerate)
	}
	// The arc between the tangent points faces the corner.
	if k0.Sub(s.centre).Dot(a0.Add(a1)) <= 0 {
		return s, fmt.Errorf("%w: the blend's ball does not sit against the corner", decaderr.ErrUnsupported)
	}
	step := 2 * math.Acos(1-math.Min(tau/r, 1))
	m := max(2, int(math.Ceil(s.phi/step)))
	sin := math.Sin(s.phi)
	for j := 0; j <= m; j++ {
		alpha := s.phi * float64(j) / float64(m)
		s.rows = append(s.rows, s.centre.Add(a0.Scale(r*math.Sin(s.phi-alpha)/sin)).Add(a1.Scale(r*math.Sin(alpha)/sin)))
	}
	s.rows[0], s.rows[m] = s.t[0], s.t[1]
	return s, nil
}

// cornerArea is the area of the cross-section's region between the walls'
// circles from the corner k0 to the tangent points and the blend.
func (c cylCyl) cornerArea(spec blendSpec, s cylSection, k0 r3.Vec, o, w [2]r3.Vec) float64 {
	e2 := c.axis.Cross(w[0])
	to2 := func(p r3.Vec) (float64, float64) {
		d := p.Sub(k0)
		return d.Dot(w[0]), d.Dot(e2)
	}
	segment := func(p, q r3.Vec) float64 {
		x0, y0 := to2(p)
		x1, y1 := to2(q)
		return (x0*y1 - x1*y0) / 2
	}
	arc := func(centre r3.Vec, rho float64, p, q r3.Vec) float64 {
		ax, ay := to2(centre)
		px, py := to2(p)
		qx, qy := to2(q)
		a0 := math.Atan2(py-ay, px-ax)
		sweep := math.Atan2(qy-ay, qx-ax) - a0
		for sweep > math.Pi {
			sweep -= 2 * math.Pi
		}
		for sweep <= -math.Pi {
			sweep += 2 * math.Pi
		}
		a1 := a0 + sweep
		return (rho*rho*sweep + ax*rho*(math.Sin(a1)-math.Sin(a0)) - ay*rho*(math.Cos(a1)-math.Cos(a0))) / 2
	}
	total := arc(o[0], c.wall[0].radius, k0, s.t[0])
	if spec.fillet {
		total += arc(s.centre, spec.r, s.t[0], s.t[1])
	} else {
		total += segment(s.t[0], s.t[1])
	}
	total += arc(o[1], c.wall[1].radius, s.t[1], k0)
	return math.Abs(total)
}

// wallCut is one wall's cut at the generator through its tangent point.
type wallCut struct {
	cols []int    // the cut vertices, in order along the axis
	rem  [2][]int // the removed vertices of the wall's chain at each end
}

// cutWall cuts the wall at the generator through angle theta about its axis
// (measured from the corner along u1 toward u2) and keeps the part beyond it,
// as the wall of R3 is cut (agent-docs/retained-edge-blend-design.md §4.3).
// o0 is the wall's axis point in the plane through the corner k0.
func (w *work) cutWall(fp facetedrecord.Payload, f faceLoops, run edgeRun, c cylCyl, wall cylWall, o0, u1, u2 r3.Vec,
	theta float64, k0 r3.Vec) (wallCut, error) {
	slack := c.slack()
	axis := c.axis
	w.cuts = map[[2]int]int{}
	thetaOf := func(p r3.Vec) float64 {
		q := p.Sub(o0)
		q = q.Sub(axis.Scale(q.Dot(axis)))
		return math.Atan2(q.Dot(u2), q.Dot(u1))
	}
	removed := map[int]bool{}
	for _, fi := range f.facets {
		for _, v := range fp.Tris[fi] {
			if _, seen := removed[v]; seen {
				continue
			}
			th := thetaOf(w.verts[v])
			if th < 0 {
				th += 2 * math.Pi
			}
			tol := (w.beta[v] + slack + w.tau) / wall.radius
			if th > 2*math.Pi-tol {
				th -= 2 * math.Pi
			}
			switch {
			case th < theta-tol:
				removed[v] = true
			case th > theta+tol:
				removed[v] = false
			default:
				return wallCut{}, fmt.Errorf("%w: a vertex of the wall lies on the blend's setback line",
					decaderr.ErrUnsupported)
			}
		}
	}
	nu := u1.Scale(-math.Sin(theta)).Add(u2.Scale(math.Cos(theta)))
	gT := func(v int) float64 { return nu.Dot(w.verts[v].Sub(o0)) }
	cut := func(a, b int) (int, error) { // a removed, b kept
		key := [2]int{min(a, b), max(a, b)}
		if id, ok := w.cuts[key]; ok {
			return id, nil
		}
		ga, gb := gT(a), gT(b)
		if !(ga < 0 && gb > 0) {
			return 0, fmt.Errorf("%w: a wall facet reaches across half the cylinder", decaderr.ErrUnsupported)
		}
		p := w.verts[a].Add(w.verts[b].Sub(w.verts[a]).Scale(ga / (ga - gb)))
		id := w.vertex(p, max(w.beta[a], w.beta[b])+slack+w.tau)
		w.cuts[key] = id
		return id, nil
	}
	for _, fi := range f.facets {
		t := fp.Tris[fi]
		var gone []int
		for k, v := range t {
			if removed[v] {
				gone = append(gone, k)
			}
		}
		switch len(gone) {
		case 0:
			w.emit(t, f.group)
		case 3:
		case 1:
			z := gone[0]
			x, y, below := t[(z+1)%3], t[(z+2)%3], t[z]
			cyz, err := cut(below, y)
			if err != nil {
				return wallCut{}, err
			}
			czx, err := cut(below, x)
			if err != nil {
				return wallCut{}, err
			}
			w.emit([3]int{x, y, cyz}, f.group)
			w.emit([3]int{x, cyz, czx}, f.group)
		case 2:
			x := 0
			for k := range 3 {
				if !removed[t[k]] {
					x = k
				}
			}
			a, b, d := t[x], t[(x+1)%3], t[(x+2)%3]
			cab, err := cut(b, a)
			if err != nil {
				return wallCut{}, err
			}
			cda, err := cut(d, a)
			if err != nil {
				return wallCut{}, err
			}
			w.emit([3]int{a, cab, cda}, f.group)
		}
	}
	w.removed = append(w.removed, f.facets...)

	var out wallCut
	var pEnd [2]int
	for i := range 2 {
		seq := append([]int{c.ends[i].v}, run.chain[i]...)
		if !removed[seq[0]] {
			return wallCut{}, fmt.Errorf("%w: an edge end is not on the removed side of the wall's setback line",
				decaderr.ErrUnsupported)
		}
		crossed := false
		for k := 1; k < len(seq); k++ {
			if removed[seq[k-1]] && !removed[seq[k]] {
				id, ok := w.cuts[[2]int{min(seq[k-1], seq[k]), max(seq[k-1], seq[k])}]
				if !ok {
					return wallCut{}, fmt.Errorf("%w: the wall's chain edge at the setback line was not cut",
						decaderr.ErrUnsupported)
				}
				pEnd[i], crossed = id, true
				break
			}
			if !removed[seq[k]] {
				break
			}
			out.rem[i] = append(out.rem[i], seq[k])
		}
		if !crossed {
			return wallCut{}, fmt.Errorf("%w: the wall's chain ends before the blend's setback line", decaderr.ErrDegenerate)
		}
	}
	exempt := map[int]bool{}
	for i := range 2 {
		exempt[c.ends[i].v] = true
		for _, v := range run.chain[i] {
			exempt[v] = true
		}
	}
	for _, v := range run.inner {
		exempt[v] = true
	}
	for v, gone := range removed {
		if gone && !exempt[v] {
			return wallCut{}, fmt.Errorf("%w: another part of the wall lies within the blend's reach of the edge",
				decaderr.ErrUnsupported)
		}
	}
	for _, id := range w.cuts {
		out.cols = append(out.cols, id)
	}
	sort.Slice(out.cols, func(a, b int) bool {
		return w.verts[out.cols[a]].Sub(k0).Dot(axis) < w.verts[out.cols[b]].Sub(k0).Dot(axis)
	})
	if len(out.cols) < 2 || out.cols[0] != pEnd[0] || out.cols[len(out.cols)-1] != pEnd[1] {
		return wallCut{}, fmt.Errorf("%w: the wall's cut does not run from one end to the other", decaderr.ErrUnsupported)
	}
	for k := 1; k < len(out.cols); k++ {
		d := w.verts[out.cols[k]].Sub(w.verts[out.cols[k-1]]).Dot(axis)
		if !(d > 0) {
			return wallCut{}, fmt.Errorf("%w: two cut vertices of the wall share a place on the axis", decaderr.ErrUnsupported)
		}
	}
	return out, nil
}

// zipOpen joins two rows of vertices, each ordered along the axis, into a band
// of triangles by advancing whichever row's next vertex comes first.
func (w *work) zipOpen(a, b []int, k0, axis r3.Vec) [][3]int {
	at := func(v int) float64 { return w.verts[v].Sub(k0).Dot(axis) }
	var out [][3]int
	i, j := 0, 0
	for i < len(a)-1 || j < len(b)-1 {
		if i < len(a)-1 && (j >= len(b)-1 || at(a[i+1]) <= at(b[j+1])) {
			out = append(out, [3]int{a[i], a[i+1], b[j]})
			i++
			continue
		}
		out = append(out, [3]int{a[i], b[j+1], b[j]})
		j++
	}
	return out
}
