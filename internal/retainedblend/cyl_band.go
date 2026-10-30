package retainedblend

import (
	"fmt"
	"math"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// The removed band of a plane–cylinder edge's wall (retained-edge-blend
// §4.3, §5.0): the wall's points whose angle about the axis, measured from the
// edge into the wall, is below the setback generator's, and which lie inside
// the end plane of each end where the wall extends past the edge's corner.
// Where the wall ends with the edge it holds no point past the end plane, and
// the angle alone bounds the band. The band is the meeting of three
// half-spaces on the wall, so it meets each flat held facet in a convex
// polygon; cutting the wall keeps the rest of each facet, triangulated again
// with every point where the band's boundary crosses the facet's edges and
// every corner of the band inside the facet.

// The planes that bound the band.
const (
	bandGen  = iota // the half-plane through the axis and the setback generator
	bandEnd0        // the end plane at end 0, where the wall extends past it
	bandEnd1        // likewise at end 1
)

// wallBand is the band and the frame its planes are read in.
type wallBand struct {
	w *work
	// o0 is the axis point at end 0; u1 runs from the axis to the edge and u2
	// into the wall, normal to the axis; nu is the setback generator's
	// half-plane normal, toward the kept wall.
	o0, u1, u2, nu r3.Vec
	radius, thetaT float64
	slack          float64
	ends           [2]*bandEnd
	// side caches each held vertex's side of each plane.
	side [3]map[int]int
}

// bandEnd is the end plane of an end where the wall extends past the edge's
// corner.
type bandEnd struct {
	g plane
	// sign is +1 when the band lies on g's positive side.
	sign float64
	// on holds the held vertices that lie on the plane by construction: the
	// edge's corner and the wall's chain along the end face.
	on map[int]bool
}

// bandPt is a point of a facet being cut: a held vertex, a cut point on a held
// facet edge, or a corner of the band inside the facet.
type bandPt struct {
	p    r3.Vec
	id   int // the vertex, or -1 until a cut point is made
	held bool
	// edge is the held facet edge a cut point lies on; a corner has none.
	edge [2]int
	// planes are the band planes a cut point lies on; -1 marks none.
	planes [2]int
	bound  float64
}

// bandCuts is what cutting the wall by the band records.
type bandCuts struct {
	// removed holds the held vertices strictly inside the band.
	removed map[int]bool
	// corner is the vertex where the setback generator meets each extending
	// end plane, or -1.
	corner [2]int
	// runs joins the vertices of the cut along each extending end plane: a
	// path from the corner to the edge's end vertex.
	runs [2]map[int][]int
}

type bandEdgeKey struct {
	plane int
	edge  [2]int
}

// value is the signed distance of p from plane q, positive inside the band.
func (b *wallBand) value(q int, p r3.Vec) float64 {
	if q == bandGen {
		return -b.nu.Dot(p.Sub(b.o0))
	}
	e := b.ends[q-bandEnd0]
	return e.sign * e.g.offset(p)
}

// tolerance is how far a point with bound beta can lie from its true place.
func (b *wallBand) tolerance(beta float64) float64 { return beta + b.slack + b.w.tau }

// angleSide reads the side of the setback generator from p's angle about the
// axis, in [0, 2π) from the edge: +1 below the generator's angle, inside the
// band, and -1 beyond it. A vertex within its bound of the generator refuses.
func (b *wallBand) angleSide(p r3.Vec, beta float64) (int, error) {
	q := p.Sub(b.o0)
	th := math.Atan2(q.Dot(b.u2), q.Dot(b.u1))
	if th < 0 {
		th += 2 * math.Pi
	}
	tol := b.tolerance(beta) / b.radius
	if th > 2*math.Pi-tol {
		th -= 2 * math.Pi
	}
	switch {
	case th < b.thetaT-tol:
		return 1, nil
	case th > b.thetaT+tol:
		return -1, nil
	}
	return 0, fmt.Errorf("%w: a vertex of the wall lies on the blend's setback line", decaderr.ErrUnsupported)
}

// sideOf is pt's side of plane q: +1 inside the band, -1 outside, 0 on the
// plane. Only a point that lies on the plane by construction reads 0; any
// other point within its bound of the plane refuses.
func (b *wallBand) sideOf(pt bandPt, q int) (int, error) {
	if !pt.held && (pt.planes[0] == q || pt.planes[1] == q) {
		return 0, nil
	}
	if pt.held {
		if s, ok := b.side[q][pt.id]; ok {
			return s, nil
		}
	}
	var s int
	var err error
	if q == bandGen {
		s, err = b.angleSide(pt.p, pt.bound)
	} else {
		switch v := b.value(q, pt.p); {
		case pt.held && b.ends[q-bandEnd0].on[pt.id]:
			s = 0
		case v > b.tolerance(pt.bound):
			s = 1
		case v < -b.tolerance(pt.bound):
			s = -1
		default:
			err = fmt.Errorf("%w: a vertex of the wall lies on an end face's plane", decaderr.ErrUnsupported)
		}
	}
	if err != nil {
		return 0, err
	}
	if pt.held {
		b.side[q][pt.id] = s
	}
	return s, nil
}

// sharedEdge is the held facet edge both points lie on.
func sharedEdge(u, v bandPt) ([2]int, bool) {
	has := func(e [2]int, id int) bool { return e[0] == id || e[1] == id }
	switch {
	case u.held && v.held:
		return [2]int{min(u.id, v.id), max(u.id, v.id)}, true
	case u.held && v.edge[0] >= 0:
		return v.edge, has(v.edge, u.id)
	case v.held && u.edge[0] >= 0:
		return u.edge, has(u.edge, v.id)
	case u.edge[0] >= 0 && u.edge == v.edge:
		return u.edge, true
	}
	return [2]int{}, false
}

// planesOf are the band planes pt lies on.
func (b *wallBand) planesOf(pt bandPt) []int {
	if !pt.held {
		var out []int
		for _, q := range pt.planes {
			if q >= 0 {
				out = append(out, q)
			}
		}
		return out
	}
	var out []int
	for i, e := range b.ends {
		if e != nil && e.on[pt.id] {
			out = append(out, bandEnd0+i)
		}
	}
	return out
}

// chordPlane is the band plane the chord uv runs along, or -1.
func (b *wallBand) chordPlane(u, v bandPt) int {
	of := b.planesOf(v)
	for _, p := range b.planesOf(u) {
		if slices.Contains(of, p) {
			return p
		}
	}
	return -1
}

// crossing is the point where plane q crosses the polygon edge uv, whose ends
// lie on opposite sides of it. On a held facet edge the point is read from
// the edge's two held vertices, so the facets on either side of the edge make
// the same point; elsewhere uv runs along another band plane inside the facet,
// and the point is the band's corner there.
func (b *wallBand) crossing(u, v bandPt, q int, facetBound float64) (bandPt, error) {
	if e, ok := sharedEdge(u, v); ok {
		a, c := bandPt{p: b.w.verts[e[0]], id: e[0], held: true, bound: b.w.beta[e[0]]},
			bandPt{p: b.w.verts[e[1]], id: e[1], held: true, bound: b.w.beta[e[1]]}
		sa, err := b.sideOf(a, q)
		if err != nil {
			return bandPt{}, err
		}
		sc, err := b.sideOf(c, q)
		if err != nil {
			return bandPt{}, err
		}
		fa, fc := b.value(q, a.p), b.value(q, c.p)
		if sa*sc >= 0 || (fa > 0) != (sa > 0) || (fc > 0) != (sc > 0) || fa == 0 || fc == 0 {
			if q == bandGen {
				return bandPt{}, fmt.Errorf("%w: a wall facet reaches across half the cylinder", decaderr.ErrUnsupported)
			}
			return bandPt{}, fmt.Errorf("%w: a wall facet edge crosses an end face's plane where its ends do not",
				decaderr.ErrUnsupported)
		}
		p := a.p.Add(c.p.Sub(a.p).Scale(fa / (fa - fc)))
		return bandPt{p: p, id: -1, edge: e, planes: [2]int{q, -1},
			bound: max(a.bound, c.bound) + b.slack + b.w.tau}, nil
	}
	along := b.chordPlane(u, v)
	if along < 0 {
		return bandPt{}, fmt.Errorf("%w: the wall's cut leaves a facet's boundary", decaderr.ErrUnsupported)
	}
	fu, fv := b.value(q, u.p), b.value(q, v.p)
	if fu*fv >= 0 {
		return bandPt{}, fmt.Errorf("%w: two cuts of the wall meet on a facet's boundary", decaderr.ErrUnsupported)
	}
	p := u.p.Add(v.p.Sub(u.p).Scale(fu / (fu - fv)))
	return bandPt{p: p, id: -1, edge: [2]int{-1, -1}, planes: [2]int{min(q, along), max(q, along)},
		bound: facetBound + b.slack + b.w.tau}, nil
}

// clip keeps the part of the convex polygon poly inside plane q.
func (b *wallBand) clip(poly []bandPt, q int, facetBound float64) ([]bandPt, error) {
	out := make([]bandPt, 0, len(poly)+1)
	for k, u := range poly {
		v := poly[(k+1)%len(poly)]
		su, err := b.sideOf(u, q)
		if err != nil {
			return nil, err
		}
		sv, err := b.sideOf(v, q)
		if err != nil {
			return nil, err
		}
		if su >= 0 {
			out = append(out, u)
		}
		if su*sv < 0 {
			x, err := b.crossing(u, v, q, facetBound)
			if err != nil {
				return nil, err
			}
			out = append(out, x)
		}
	}
	return out, nil
}

// cutBand cuts the wall's facets by the band: it emits the part of each facet
// outside the band and records the cut. A cut point on a held facet edge is
// made once, for both facets beside the edge, and takes the larger bound of
// the edge's two vertices plus the rounding; a corner of the band inside a
// facet takes the facet's bound.
func (w *work) cutBand(f faceLoops, b *wallBand) (bandCuts, error) {
	tris := w.fp.Tris
	b.w = w
	for q := range b.side {
		b.side[q] = map[int]int{}
	}
	out := bandCuts{removed: map[int]bool{}, corner: [2]int{-1, -1}}
	var planes []int
	for i, e := range b.ends {
		if e != nil {
			planes = append(planes, bandEnd0+i)
			out.runs[i] = map[int][]int{}
		}
	}
	planes = append(planes, bandGen)
	beyond := b.beyondEnds(f.facets)
	for _, fi := range f.facets {
		for _, v := range tris[fi] {
			if beyond[v] {
				continue
			}
			if _, err := b.sideOf(w.heldPt(v), bandGen); err != nil {
				return bandCuts{}, err
			}
		}
	}
	edgePts := map[bandEdgeKey]int{}
	corners := map[[2]int]int{}
	make1 := func(pt *bandPt) error {
		if pt.held {
			return nil
		}
		if pt.edge[0] >= 0 {
			k := bandEdgeKey{pt.planes[0], pt.edge}
			if id, ok := edgePts[k]; ok {
				pt.id = id
				return nil
			}
			pt.id = w.vertex(pt.p, pt.bound)
			edgePts[k] = pt.id
			if pt.planes[0] == bandGen {
				w.cuts[pt.edge] = pt.id
			}
			return nil
		}
		if _, ok := corners[pt.planes]; ok {
			return fmt.Errorf("%w: a corner of the blend's band lies on a wall facet's edge", decaderr.ErrUnsupported)
		}
		pt.id = w.vertex(pt.p, pt.bound)
		corners[pt.planes] = pt.id
		if pt.planes[0] == bandGen {
			out.corner[pt.planes[1]-bandEnd0] = pt.id
		}
		return nil
	}
	for _, fi := range f.facets {
		t := tris[fi]
		inside := false
		for _, v := range t {
			if beyond[v] {
				// Its side of the generator may be undecided: count it inside,
				// so the facet is clipped, and the end plane drops it.
				s, err := b.angleSide(w.verts[v], w.beta[v])
				inside = inside || err != nil || s > 0
				continue
			}
			s, _ := b.sideOf(w.heldPt(v), bandGen)
			inside = inside || s > 0
		}
		if !inside {
			w.emit(t, f.group)
			continue
		}
		facetBound := max(w.beta[t[0]], w.beta[t[1]], w.beta[t[2]])
		poly := []bandPt{w.heldPt(t[0]), w.heldPt(t[1]), w.heldPt(t[2])}
		var err error
		for _, q := range planes {
			if poly, err = b.clip(poly, q, facetBound); err != nil {
				return bandCuts{}, err
			}
			if len(poly) < 3 {
				break
			}
		}
		if len(poly) < 3 {
			w.emit(t, f.group)
			continue
		}
		for k := range poly {
			if err := make1(&poly[k]); err != nil {
				return bandCuts{}, err
			}
		}
		if err := w.keepOutside(t, f.group, poly, b, &out); err != nil {
			return bandCuts{}, err
		}
	}
	for _, fi := range f.facets {
		for _, v := range tris[fi] {
			if beyond[v] {
				continue
			}
			in := true
			for k := len(planes) - 1; k >= 0 && in; k-- {
				s, err := b.sideOf(w.heldPt(v), planes[k])
				if err != nil {
					return bandCuts{}, err
				}
				in = s > 0
			}
			if in {
				out.removed[v] = true
			}
		}
	}
	return out, nil
}

// beyondEnds marks the wall's vertices that lie outside an extending end plane
// by more than their bound, and records that side. The band lies inside every
// such plane, so a marked vertex is never in it, and a facet clipped by the
// band loses the vertex to that plane's clip before the generator's is
// applied: its side of the generator is never read there, except where a
// crossing on a held edge reads both of the edge's vertices, which still
// refuses an undecided one. cutBand therefore tests no marked vertex against
// the generator and removes none, and a feature on the wall past an end
// plane, such as a cross hole, does not refuse. A vertex within its bound of
// the plane is not marked, and cutBand tests it against the generator.
func (b *wallBand) beyondEnds(facets []int) map[int]bool {
	beyond := map[int]bool{}
	for _, fi := range facets {
		for _, v := range b.w.fp.Tris[fi] {
			for i, e := range b.ends {
				if e == nil || e.on[v] {
					continue
				}
				if b.value(bandEnd0+i, b.w.verts[v]) < -b.tolerance(b.w.beta[v]) {
					beyond[v] = true
					b.side[bandEnd0+i][v] = -1
				}
			}
		}
	}
	return beyond
}

func (w *work) heldPt(v int) bandPt {
	return bandPt{p: w.verts[v], id: v, held: true, edge: [2]int{-1, -1}, planes: [2]int{-1, -1}, bound: w.beta[v]}
}

// keepOutside emits the part of the held facet t outside the band, whose part
// inside it is the convex polygon in, wound as t is. Each run of in's edges
// that crosses the facet's inside, from one point X of the facet's boundary to
// another Y, cuts off one piece: the facet's boundary from X on to Y, then the
// run back to X.
func (w *work) keepOutside(t [3]int, group int, in []bandPt, b *wallBand, out *bandCuts) error {
	n := len(in)
	chord := make([]bool, n)
	crosses := false
	for k := range n {
		_, onEdge := sharedEdge(in[k], in[(k+1)%n])
		chord[k] = !onEdge
		crosses = crosses || chord[k]
	}
	if !crosses {
		for _, pt := range in {
			if !pt.held {
				return fmt.Errorf("%w: the wall's cut leaves a facet's boundary", decaderr.ErrUnsupported)
			}
		}
		return nil
	}
	nrm := w.verts[t[1]].Sub(w.verts[t[0]]).Cross(w.verts[t[2]].Sub(w.verts[t[0]]))
	u, ok := perpendicular(nrm)
	if !ok {
		return fmt.Errorf("%w: a wall facet has no plane", decaderr.ErrUnsupported)
	}
	nu, _ := nrm.Normalize()
	v := nu.Cross(u)
	to2 := func(id int) pt2 {
		q := w.verts[id].Sub(w.verts[t[0]])
		return pt2{q.Dot(u), q.Dot(v)}
	}
	ids := make([]int, n)
	for k, pt := range in {
		ids[k] = pt.id
	}
	{
		ps := make([]pt2, n)
		loop := make([]int, n)
		for k := range n {
			ps[k], loop[k] = to2(ids[k]), k
		}
		if !(signedArea2(ps, loop) > 0) {
			return fmt.Errorf("%w: the wall's cut leaves a degenerate piece of a facet", decaderr.ErrUnsupported)
		}
	}
	// The facet's boundary, with the cut points on each edge in order along it.
	var bnd []int
	for k := range 3 {
		a, c := t[k], t[(k+1)%3]
		e := [2]int{min(a, c), max(a, c)}
		dir := w.verts[c].Sub(w.verts[a])
		bnd = append(bnd, a)
		var on []bandPt
		for _, pt := range in {
			if !pt.held && pt.edge == e {
				on = append(on, pt)
			}
		}
		sort.Slice(on, func(i, j int) bool {
			return on[i].p.Sub(w.verts[a]).Dot(dir) < on[j].p.Sub(w.verts[a]).Dot(dir)
		})
		for _, pt := range on {
			bnd = append(bnd, pt.id)
		}
	}
	at := func(id int) int {
		for k, x := range bnd {
			if x == id {
				return k
			}
		}
		return -1
	}
	for s := range n {
		if !chord[s] || chord[(s+n-1)%n] {
			continue
		}
		// A run of chords from in[s] to in[e].
		var inner []int
		e := s
		for chord[e%n] {
			if p := b.chordPlane(in[e%n], in[(e+1)%n]); p > bandGen {
				i := p - bandEnd0
				x, y := in[e%n].id, in[(e+1)%n].id
				out.runs[i][x] = append(out.runs[i][x], y)
				out.runs[i][y] = append(out.runs[i][y], x)
			}
			e++
			if chord[e%n] {
				inner = append(inner, in[e%n].id)
			}
		}
		x, y := in[s].id, in[e%n].id
		from, to := at(x), at(y)
		if from < 0 || to < 0 {
			return fmt.Errorf("%w: the wall's cut leaves a facet's boundary", decaderr.ErrUnsupported)
		}
		piece := []int{x}
		for k := (from + 1) % len(bnd); ; k = (k + 1) % len(bnd) {
			piece = append(piece, bnd[k])
			if k == to {
				break
			}
		}
		for _, id := range slices.Backward(inner) {
			piece = append(piece, id)
		}
		ps := make([]pt2, len(piece))
		loop := make([]int, len(piece))
		for k, id := range piece {
			ps[k], loop[k] = to2(id), k
		}
		if !(signedArea2(ps, loop) > 0) {
			return fmt.Errorf("%w: the wall's cut leaves a degenerate piece of a facet", decaderr.ErrUnsupported)
		}
		tri, err := triangulateRegion(ps, loop, nil)
		if err != nil {
			return err
		}
		for _, tr := range tri {
			w.emit([3]int{piece[tr[0]], piece[tr[1]], piece[tr[2]]}, group)
		}
	}
	return nil
}

// endRun is the cut along extending end i's plane, from the corner where the
// setback generator meets it to the edge's end vertex v, both left out: the
// vertices an end face's loop takes between the blend's last row and v.
func (c bandCuts) endRun(i, v int) ([]int, error) {
	g := c.runs[i]
	start := c.corner[i]
	bad := fmt.Errorf("%w: the wall's cut along an end face does not run from the setback line to the corner",
		decaderr.ErrUnsupported)
	if start < 0 || len(g[start]) != 1 || len(g[v]) != 1 {
		return nil, bad
	}
	var run []int
	prev, at := -1, start
	for at != v {
		nb := g[at]
		if at != start && len(nb) != 2 {
			return nil, bad
		}
		next := nb[0]
		if next == prev {
			if len(nb) < 2 {
				return nil, bad
			}
			next = nb[1]
		}
		if at != start {
			run = append(run, at)
		}
		prev, at = at, next
		if len(run) > len(g) {
			return nil, bad
		}
	}
	if len(run)+2 != len(g) {
		return nil, bad
	}
	return run, nil
}
