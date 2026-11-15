package retainedblend

import (
	"errors"
	"fmt"
	"math"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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
	bandGen   = iota // the main plane: the half-plane through the axis and the setback generator, or an exact plane
	bandEnd0         // the end plane at end 0, where the wall extends past it
	bandEnd1         // likewise at end 1
	bandOther        // the first of wallBand.others, further exact planes
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
	// main, when set, is the main plane in place of the setback generator's
	// half-plane: an exact plane, whose cut row is the strip's last row (the
	// arc route, arc_band.go). The generator's fields are then unused.
	main *bandEnd
	ends [2]*bandEnd
	// others are further exact planes, bandOther on, that bound the band.
	others []*bandEnd
	// noteCuts records each vertex a cut on the main plane makes with its facet
	// bound (noteCut), for the strip's band charge.
	noteCuts bool
	// side caches each held vertex's side of each plane.
	side []map[int]int
	// active are the planes cutBand applies, in the order it applies them.
	active []int
	// sign lets sideOf read a point within its bound of an end plane by the sign of
	// its exact offset, charging the offset (signRead). The cut turns it on only
	// where reading without it refuses.
	sign bool
}

// bandEnd is an exact plane of the band: the end plane of an end where the wall
// extends past the edge's corner, or a plane the arc route adds.
type bandEnd struct {
	g plane
	// sign is +1 when the band lies on g's positive side.
	sign float64
	// on holds the held vertices that lie on the plane by construction: the
	// edge's corner and the wall's chain along the end face.
	on map[int]bool
	// what names the plane in a refusal; empty reads "an end face's plane".
	what string
	// sine is a proven lower bound on the sine of the angle between the true wall
	// and this plane at every point of the wall within reach of the plane's
	// tag, or 0 when there is none (wallSine). A sign reading (signRead) needs
	// it; an end plane without one refuses as before.
	sine func(reach float64) float64
	// planarWall says the wall is a plane, where the distance bound of a sign
	// reading is c/s from the held point; on a curved wall it is β + c/s.
	planarWall bool
}

// exact is the exact plane q, or nil for the setback generator's half-plane.
func (b *wallBand) exact(q int) *bandEnd {
	switch {
	case q == bandGen:
		return b.main
	case q < bandOther:
		return b.ends[q-bandEnd0]
	}
	return b.others[q-bandOther]
}

// isEnd reports whether q is an end plane.
func isEnd(q int) bool { return q == bandEnd0 || q == bandEnd1 }

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
	// signed is, for each end plane, the side signRead put this cut point on,
	// plus 2 (1 outside, 2 on, 3 inside), or 0 when it was never read by sign.
	// The point's bound already holds the charge.
	signed [2]int8
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
	e := b.exact(q)
	if e == nil {
		return -b.nu.Dot(p.Sub(b.o0))
	}
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
// plane. A point that lies on the plane by construction reads 0. Any other point
// within its bound of the plane refuses, unless sign reading is on and the plane
// is an end plane of a band with an exact main plane (signRead).
func (b *wallBand) sideOf(pt bandPt, q int) (int, error) { return b.sideAt(&pt, q) }

// sideAt is sideOf for a point whose bound a sign reading may charge in place.
func (b *wallBand) sideAt(pt *bandPt, q int) (int, error) {
	if !pt.held && (pt.planes[0] == q || pt.planes[1] == q) {
		return 0, nil
	}
	if pt.held {
		if s, ok := b.side[q][pt.id]; ok {
			return s, nil
		}
	}
	if isEnd(q) && pt.signed[q-bandEnd0] != 0 {
		return int(pt.signed[q-bandEnd0]) - 2, nil
	}
	var s int
	var err error
	if e := b.exact(q); e == nil {
		s, err = b.angleSide(pt.p, pt.bound)
	} else {
		what := e.what
		if what == "" {
			what = "an end face's plane"
		}
		switch v := b.value(q, pt.p); {
		case pt.held && e.on[pt.id]:
			s = 0
		case v > b.tolerance(pt.bound):
			s = 1
		case v < -b.tolerance(pt.bound):
			s = -1
		default:
			var ok bool
			if b.sign && isEnd(q) && b.main != nil {
				s, ok = b.signRead(pt, q, v)
			}
			if !ok {
				err = fmt.Errorf("%w: a vertex of the wall lies on %s", decaderr.ErrUnsupported, what)
			}
		}
	}
	if err != nil {
		return 0, &undecidedError{err: err, plane: q, id: pt.id, held: pt.held}
	}
	if pt.held {
		b.side[q][pt.id] = s
	}
	return s, nil
}

// signRead puts pt, which lies within its bound of end plane q at offset v, on
// the side of v's sign, and raises its bound to cover the distance to the band's
// boundary on the wall: a held vertex's bound in the work (and w.raised), or the
// cut point's own. It reports false, and changes nothing, when the end plane
// carries no proven lower bound on the sine of the wall's slant to it (bandEnd.sine)
// or that bound is below minSine, so the reading refuses as before.
//
// The side is a choice, not a claim. No residual admits an input claim here. The
// true point of pt lies within its input bound β of pt, as before. pt lies within
// |v| plus the plane's slack of the plane the exact band is cut at, so the true
// point lies within c = β + |v| + slack of that plane, on whichever side: a
// distance to the plane. The band's boundary on the wall is where the wall
// crosses the plane. Measured along the wall the true point is at most c/s from it,
// s_lo the sine of the wall's slant to the plane over the path from the true point
// to the plane; every point of that path lies within c of the plane, because the
// offset falls along it, so s_lo(c) holds there (slantBound).
//
// The published bound is the distance from the held point to the boundary. On a
// plane wall, split the held point's error x − v into a part normal to the wall,
// at most β, and a part across the boundary line within the wall, at most
// (a + β·cos θ)/s for a = |v| + slack; their root-sum-square is at most
// √(β² + ((a + β cos θ)/s)²) ≤ (β + a)/s = c/s, since 2aβ cos θ ≤ 2aβ. So a plane
// wall publishes c/s_lo. On a curved wall the path bends, and the triangle
// inequality gives β (to the true point) plus c/s_lo (from it to the boundary), so
// a curved wall publishes β + c/s_lo. A kept vertex carries the bound; a cut point
// carries it, and a cut of a held edge reads its two ends' raised bounds. That
// read can raise an end that another plane already puts outside the band: it only
// widens a bound. When s_lo is below minSine, or the end plane has none, the
// reading refuses.
//
// The input β of a cut point is its crossing bound, which has no slant term of its
// own (see the follow-up in the design doc); a held vertex's β does not depend on it.
func (b *wallBand) signRead(pt *bandPt, q int, v float64) (int, bool) {
	e := b.exact(q)
	if e == nil || e.sine == nil {
		return 0, false
	}
	beta := pt.bound
	if pt.held {
		// The input claim: the true point lies within the held vertex's own bound of
		// it. An earlier reading's raise (on this or another plane) states another
		// distance and is kept by the max below, but it is no part of this claim, so
		// charges do not compound.
		beta = b.w.beta[pt.id]
		if pt.id < len(b.w.fp.VertexBound) {
			beta = b.w.fp.VertexBound[pt.id]
		}
	}
	c := proofbound.AbsSumUpper(beta, v, b.slack)
	bound, ok := slantBound(c, e.sine)
	if !ok {
		return 0, false
	}
	if !e.planarWall {
		bound = proofbound.AbsSumUpper(beta, bound)
	}
	s := 0
	switch {
	case v > 0:
		s = 1
	case v < 0:
		s = -1
	}
	if pt.held {
		b.w.beta[pt.id] = max(b.w.beta[pt.id], bound)
		b.w.raised = max(b.w.raised, b.w.beta[pt.id])
		return s, true
	}
	pt.bound = bound
	pt.signed[q-bandEnd0] = int8(s + 2)
	return s, true
}

// slantBound is the distance c/s along a wall to the boundary of a band whose end
// plane the wall meets at sine s, for a point within c of the plane. sine(reach)
// is a proven lower bound on the slant's sine at wall points within reach of the
// plane. The path along the wall from the point's foot to the plane stays within c
// of the plane, because its offset falls along it while the sine stays above zero,
// so sine(c) bounds the sine on the whole path and the distance is at most
// c/sine(c). It is false when sine(c) is below minSine.
func slantBound(c float64, sine func(reach float64) float64) (float64, bool) {
	s := sine(c)
	if !(s >= minSine) {
		return 0, false
	}
	return max(c, proofbound.DivUpper(c, s)), true
}

// undecidedError is a refusal because a point lies within its bound of plane
// q, which therefore does not decide its side.
type undecidedError struct {
	err   error
	plane int
	id    int
	held  bool
}

func (e *undecidedError) Error() string { return e.err.Error() }
func (e *undecidedError) Unwrap() error { return e.err }

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
	for q := bandGen; q < bandOther+len(b.others); q++ {
		if e := b.exact(q); e != nil && e.on[pt.id] {
			out = append(out, q)
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

// alongBoth is a band plane other than q that both held points lie on, or -1.
// A cut of a held edge whose two ends lie on a plane lies on it too: by the
// facet's claim the point lies within its facet bound of the affine segment
// between the ends' true points, which lies on the plane, and the bound the cut
// carries covers it. Only a band with an exact main plane (the arc and the
// chain routes) reads it; the angle-main band of R3 keeps its rule.
func (b *wallBand) alongBoth(u, v bandPt, q int) int {
	if b.main == nil {
		return -1
	}
	of := b.planesOf(v)
	for _, p := range b.planesOf(u) {
		if p != q && slices.Contains(of, p) {
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
		// A sign reading above may have charged an end into the work's bounds: the cut
		// takes the bounds as they now stand.
		a.bound, c.bound = b.w.beta[e[0]], b.w.beta[e[1]]
		fa, fc := b.value(q, a.p), b.value(q, c.p)
		if sa*sc >= 0 || (fa > 0) != (sa > 0) || (fc > 0) != (sc > 0) || fa == 0 || fc == 0 {
			if b.exact(q) == nil {
				return bandPt{}, fmt.Errorf("%w: a wall facet reaches across half the cylinder", decaderr.ErrUnsupported)
			}
			return bandPt{}, fmt.Errorf("%w: a wall facet edge crosses an end face's plane where its ends do not",
				decaderr.ErrUnsupported)
		}
		p := a.p.Add(c.p.Sub(a.p).Scale(fa / (fa - fc)))
		// The point lies on the held facet, which claims its true piece of the
		// curved wall within its facet bound, the largest of its three
		// corners'; the edge's two ends alone do not carry that claim.
		return bandPt{p: p, id: -1, edge: e, planes: [2]int{q, b.alongBoth(a, c, q)},
			bound: max(a.bound, c.bound, facetBound) + b.slack + b.w.tau}, nil
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
	for k := range poly {
		u, v := &poly[k], &poly[(k+1)%len(poly)]
		su, err := b.sideAt(u, q)
		if err != nil {
			return nil, err
		}
		sv, err := b.sideAt(v, q)
		if err != nil {
			return nil, err
		}
		if su >= 0 {
			out = append(out, *u)
		}
		if su*sv < 0 {
			x, err := b.crossing(*u, *v, q, facetBound)
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
//
// A held vertex's membership is decided by every band plane (readSides): a
// plane that puts it outside by more than its bound proves it outside, and the
// vertex's readings on the band's other planes are then never used, so a vertex
// within its bound of one plane, such as a run of cut vertices an earlier blend
// left along an end face, does not refuse where another plane puts it well
// outside the band. A facet whose three corners lie outside one plane is
// emitted unchanged, and a facet that is cut meets the planes that decide its
// corners first (clipAll).
func (w *work) cutBand(f faceLoops, b *wallBand) (bandCuts, error) {
	tris := w.fp.Tris
	b.w = w
	b.side = make([]map[int]int, bandOther+len(b.others))
	for q := range b.side {
		b.side[q] = map[int]int{}
	}
	out := bandCuts{removed: map[int]bool{}, corner: [2]int{-1, -1}}
	b.active = b.active[:0]
	for i, e := range b.ends {
		if e != nil {
			b.active = append(b.active, bandEnd0+i)
			out.runs[i] = map[int][]int{}
		}
	}
	for i := range b.others {
		b.active = append(b.active, bandOther+i)
	}
	b.active = append(b.active, bandGen)
	edgePts := map[bandEdgeKey]int{}
	corners := map[[2]int]int{}
	make1 := func(pt *bandPt, facetBound float64) error {
		if pt.held {
			return nil
		}
		if pt.edge[0] >= 0 {
			k := bandEdgeKey{pt.planes[0], pt.edge}
			if id, ok := edgePts[k]; ok {
				pt.id = id
				w.chargeShared(id, pt)
				return nil
			}
			// A cut of the main plane on a held edge along an end plane is the band's
			// corner there: the cuts of the wall's row do not hold it.
			corner := pt.planes[0] == bandGen && isEnd(pt.planes[1])
			if id, ok := w.cuts[pt.edge]; ok && pt.planes[0] == bandGen && !corner {
				// The wall beside this one already cut the edge they share: the
				// two walls' strips meet at that one vertex.
				pt.id = id
				edgePts[k] = id
				w.chargeShared(id, pt)
				return nil
			}
			pt.id = w.vertex(pt.p, pt.bound)
			edgePts[k] = pt.id
			if pt.planes[0] == bandGen && !corner {
				w.cuts[pt.edge] = pt.id
			}
			if pt.planes[1] >= 0 {
				if w.seam == nil {
					w.seam = map[[2]int][]int{}
				}
				w.seam[pt.edge] = append(w.seam[pt.edge], pt.id)
			}
			if corner {
				i := pt.planes[1] - bandEnd0
				if out.corner[i] >= 0 {
					return fmt.Errorf("%w: a corner of the blend's band lies on a wall facet's edge", decaderr.ErrUnsupported)
				}
				out.corner[i] = pt.id
			}
			if pt.planes[0] == bandGen && b.noteCuts {
				w.noteCut(pt.id, facetBound)
			}
			return nil
		}
		if _, ok := corners[pt.planes]; ok {
			return fmt.Errorf("%w: a corner of the blend's band lies on a wall facet's edge", decaderr.ErrUnsupported)
		}
		if pt.planes[0] == bandGen && isEnd(pt.planes[1]) && out.corner[pt.planes[1]-bandEnd0] >= 0 {
			return fmt.Errorf("%w: a corner of the blend's band lies on a wall facet's edge", decaderr.ErrUnsupported)
		}
		pt.id = w.vertex(pt.p, pt.bound)
		corners[pt.planes] = pt.id
		if pt.planes[0] == bandGen && isEnd(pt.planes[1]) {
			out.corner[pt.planes[1]-bandEnd0] = pt.id
		}
		if pt.planes[0] == bandGen && b.noteCuts {
			w.noteCut(pt.id, facetBound)
		}
		return nil
	}
	for _, fi := range f.facets {
		t := tris[fi]
		if b.facetOutside(t) {
			w.emit(t, f.group)
			continue
		}
		facetBound := max(w.beta[t[0]], w.beta[t[1]], w.beta[t[2]])
		poly, err := b.clipAll([]bandPt{w.heldPt(t[0]), w.heldPt(t[1]), w.heldPt(t[2])}, facetBound)
		if err != nil {
			return bandCuts{}, b.reword(err, t[:]...)
		}
		if len(poly) < 3 {
			w.emit(t, f.group)
			continue
		}
		for k := range poly {
			if err := make1(&poly[k], facetBound); err != nil {
				return bandCuts{}, err
			}
		}
		if err := w.keepOutside(t, f.group, poly, b, &out); err != nil {
			return bandCuts{}, err
		}
	}
	seen := map[int]bool{}
	for _, fi := range f.facets {
		for _, v := range tris[fi] {
			if seen[v] {
				continue
			}
			seen[v] = true
			in, err := b.inside(v)
			if err != nil {
				return bandCuts{}, b.reword(err, v)
			}
			if in {
				out.removed[v] = true
			}
		}
	}
	return out, nil
}

// read is held vertex v's side of plane q, and false when the vertex lies
// within its bound of q and q does not decide it.
func (b *wallBand) read(v, q int) (int, bool) {
	s, err := b.sideOf(b.w.heldPt(v), q)
	return s, err == nil
}

// facetOutside reports whether the band misses the facet: its three corners
// lie outside one band plane, or on or outside the main plane, the band's
// first bound. Each reading is decided; an undecided corner proves nothing.
// The band is the meeting of the planes' half-spaces, so a facet inside the
// closed outer side of one plane meets none of it.
func (b *wallBand) facetOutside(t [3]int) bool {
	for _, q := range b.active {
		out := true
		for _, v := range t {
			s, ok := b.read(v, q)
			out = out && ok && (s < 0 || (s == 0 && q == bandGen))
		}
		if out {
			return true
		}
	}
	return false
}

// clipAll keeps the part of the convex polygon poly inside every band plane.
// The planes are applied in their fixed order, except that a plane whose clip
// refuses, because a corner within its bound of the plane is undecided there,
// is deferred: another plane may put that corner outside the band and drop it
// from the polygon, after which the plane never reads it. A plane that still
// refuses once every other plane is applied has no order that avoids it, and
// the call refuses with its text.
func (b *wallBand) clipAll(poly []bandPt, facetBound float64) ([]bandPt, error) {
	rest := slices.Clone(b.active)
	for len(rest) > 0 && len(poly) >= 3 {
		var first error
		applied := -1
		for i, q := range rest {
			cut, err := b.clip(poly, q, facetBound)
			if err != nil {
				if first == nil {
					first = err
				}
				continue
			}
			poly, applied = cut, i
			break
		}
		if applied < 0 {
			poly, applied = b.clipBySign(poly, rest, facetBound)
			if applied < 0 {
				return nil, first
			}
		}
		rest = slices.Delete(rest, applied, applied+1)
	}
	return poly, nil
}

// clipBySign is the last resort of clipAll, reached when every remaining plane
// refuses: it applies the first remaining end plane of an exact-main band that
// reads by sign (signRead), once, and returns the clipped polygon and the plane's
// place in rest, or -1 when no plane could be applied. Sign reading is on for
// that one clip only, so a point that another plane decides, or that no refusal
// reaches, is never charged: every cut that builds without it is unchanged.
func (b *wallBand) clipBySign(poly []bandPt, rest []int, facetBound float64) ([]bandPt, int) {
	if b.main == nil {
		return nil, -1
	}
	b.sign = true
	defer func() { b.sign = false }()
	for i, q := range rest {
		if !isEnd(q) {
			continue
		}
		if cut, err := b.clip(poly, q, facetBound); err == nil {
			return cut, i
		}
	}
	return nil, -1
}

// inside reports whether held vertex v lies strictly inside every band plane.
// A vertex that any plane decides on or outside is not inside, whatever the
// other planes read; one that none puts outside and some leaves undecided
// refuses.
func (b *wallBand) inside(v int) (bool, error) {
	var first error
	for _, q := range slices.Backward(b.active) {
		s, err := b.sideOf(b.w.heldPt(v), q)
		switch {
		case err != nil:
			if first == nil {
				first = err
			}
		case s <= 0:
			return false, nil
		}
	}
	if first == nil || b.main == nil {
		return first == nil, first
	}
	// No plane puts v outside and an end plane leaves it undecided: read it by
	// sign (signRead).
	b.sign = true
	defer func() { b.sign = false }()
	for _, q := range slices.Backward(b.active) {
		if s, err := b.sideOf(b.w.heldPt(v), q); err == nil && s <= 0 {
			return false, nil
		}
	}
	for _, q := range b.active {
		if _, err := b.sideOf(b.w.heldPt(v), q); err != nil {
			return false, first
		}
	}
	return true, nil
}

// reword names the overlap of two blends where err is a refusal on an end
// plane and one of the held vertices (a facet's corners, or the vertex whose
// side refused) lies within its bound of that plane and on the inside of, or
// on, every other band plane: the vertex belongs to an earlier cut along the
// plane, and the band reaches it. Any other refusal keeps its text.
func (b *wallBand) reword(err error, vs ...int) error {
	var u *undecidedError
	if !errors.As(err, &u) || !isEnd(u.plane) {
		return err
	}
	if e := b.exact(u.plane); e == nil || e.what != "" {
		return err
	}
	for _, v := range vs {
		if _, ok := b.read(v, u.plane); ok || !b.insideOthers(v, u.plane) {
			continue
		}
		return fmt.Errorf("%w: an earlier cut on an end face's plane lies within the blend's reach",
			decaderr.ErrUnsupported)
	}
	return err
}

// insideOthers reports whether every band plane but q decides held vertex v
// inside or on it.
func (b *wallBand) insideOthers(v, q int) bool {
	for _, o := range b.active {
		if s, ok := b.read(v, o); o != q && (!ok || s < 0) {
			return false
		}
	}
	return true
}

// chargeShared makes the vertex id, which an earlier cut of the same held edge
// made, carry the charge of a sign reading (signRead) that this facet's copy of
// the point took: the facets on both sides of the edge share the vertex, so its
// bound is the larger of the two.
func (w *work) chargeShared(id int, pt *bandPt) {
	if pt.signed != [2]int8{} {
		w.beta[id] = max(w.beta[id], pt.bound)
	}
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
	b.noteBoundaryRuns(in, chord, out)
	if !crosses {
		for _, pt := range in {
			if !pt.held {
				return fmt.Errorf("%w: the wall's cut leaves a facet's boundary", decaderr.ErrUnsupported)
			}
		}
		return nil
	}
	nrm := w.verts[t[1]].Sub(w.verts[t[0]]).Cross(w.verts[t[2]].Sub(w.verts[t[0]]))
	nu, unit := nrm.Normalize()
	u, ok := perpendicular(nrm)
	if !ok && unit {
		// perpendicular needs a unit normal. A small facet normal to x has a raw
		// normal shorter than 0.9, so it picks X and crosses to zero. Only that
		// refusal retries on the unit normal, so every facet that framed from the
		// raw normal keeps its frame.
		u, ok = perpendicular(nu)
	}
	if !ok {
		return fmt.Errorf("%w: a wall facet has no plane", decaderr.ErrUnsupported)
	}
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
			if p := b.chordPlane(in[e%n], in[(e+1)%n]); isEnd(p) {
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

// noteBoundaryRuns records the polygon edges that lie on the facet's own
// boundary and along an end plane, whose two ends both lie on it: a held edge
// along the plane inside the band is part of the cut along the plane, as a chord
// inside the facet is. Only a band with an exact main plane reads it.
func (b *wallBand) noteBoundaryRuns(in []bandPt, chord []bool, out *bandCuts) {
	if b.main == nil {
		return
	}
	n := len(in)
	for k := range n {
		if chord[k] {
			continue
		}
		p := b.chordPlane(in[k], in[(k+1)%n])
		if !isEnd(p) {
			continue
		}
		x, y := in[k].id, in[(k+1)%n].id
		g := out.runs[p-bandEnd0]
		if x == y || slices.Contains(g[x], y) {
			continue
		}
		g[x] = append(g[x], y)
		g[y] = append(g[y], x)
	}
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
