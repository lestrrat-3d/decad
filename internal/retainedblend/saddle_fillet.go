package retainedblend

import (
	"context"
	"fmt"
	"math"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/saddlecanal"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// The saddle fillet (agent-docs/retained-edge-blend-design.md §4.10): the
// rolling-ball fillet of an R7 saddle, built by rewriting the held mesh near
// the mouth. In the fixed local pose (shaft axis z, bore axis x, mouth at
// +x) it
//
//   - cuts the shaft wall's held facets out of a window W, a convex polygon in
//     the (y, z) projection around T1, and the bore wall's held facets beyond
//     the plane x = X_w, both exact linear clips of flat facets;
//   - fills W less T1 with a band of triangles between the held cut ring and
//     the exact ring of T1 points, and the bore from its cut ring to the exact
//     ring of T2 points the same way;
//   - joins T1 to T2 by the exact pipe of internal/saddlecanal at the
//     saddlecanal.Stations stations and the canal's rows, proven cell by cell.
//
// A window that is a polygon of planes parallel to the bore axis, rather than
// a level curve of T1's family, cuts every held facet it crosses along an
// edge crossing: a level curve can pass through a long facet of the shaft
// wall between its vertices, which a cut read from vertex levels misses.

const (
	// windowSides is the number of sides of the shaft wall's window.
	windowSides = 64
	// windowMargin is the window's clearance outside T1, and the bore cut's
	// inside T2, as a fraction of r' = r + ρ.
	windowMargin = 1.0 / 32
	// clipTol is the least distance, in millimetres, a held vertex keeps from
	// a cut line or plane and a window corner from a held edge: a closer one
	// would make a facet piece of nearly zero size.
	clipTol = 1e-6
	// foldMargin is the least cosine a new band facet's normal keeps with
	// the direction its embedding argument reads.
	foldMargin = 1e-9
)

// looksLikeSaddleFillet reports whether e is an R7 saddle a fillet can
// build: the ball inside the shaft and outside the bore.
func looksLikeSaddleFillet(e *topo.Edge) bool { return looksLikeSaddle(e) }

// SaddleFillet fillets the selected saddle edges of b (Table RR R7 fillet,
// agent-docs/retained-edge-blend-design.md §4.10), one mouth at a time on the
// evolving mesh. recognized is false when some selected edge is no saddle;
// the caller then tries the other routes.
func SaddleFillet(ctx context.Context, env topo.Env, b *topo.Body, fp facetedrecord.Payload,
	edges []*topo.Edge, rho, rhoDelta float64) (*topo.Body, bool, error) {
	if len(edges) == 0 || !b.Solid {
		return nil, false, nil
	}
	for _, e := range edges {
		if !looksLikeSaddleFillet(e) {
			return nil, false, nil
		}
	}
	switch {
	case rhoDelta != 0:
		return nil, true, fmt.Errorf("%w: a retained edge blend needs a size that converts exactly to millimetres",
			decaderr.ErrUnsupported)
	case proofbound.IsNonFinite(rho) || !(rho > 0):
		return nil, true, fmt.Errorf("%w: a blend needs a positive size", decaderr.ErrDegenerate)
	case fp.Xform != r3.Identity():
		return nil, true, fmt.Errorf("%w: a saddle fillet reads an unplaced mesh only", decaderr.ErrUnsupported)
	}
	faceIndex := faceIndexOf(b)
	keys := make([]saddleKey, len(edges))
	for i, e := range edges {
		s, err := readSaddle(e, fp, faceIndex)
		if err != nil {
			return nil, true, err
		}
		if !(saddlecanal.Canal{R: s.key.R, Bore: s.key.r, Rho: rho}).Exists() {
			return nil, true, fmt.Errorf("%w: no ball of that radius touches both walls of the saddle "+
				"(retained-edge-blend SR3)", decaderr.ErrDegenerate)
		}
		if slices.Contains(keys[:i], s.key) {
			return nil, true, fmt.Errorf("%w: the selection names one rim twice", decaderr.ErrDegenerate)
		}
		keys[i] = s.key
	}
	cur, curFP := b, fp
	for i, key := range keys {
		s, err := findSaddle(cur, curFP, key)
		if err != nil {
			return nil, true, err
		}
		next, err := filletSaddle(ctx, env, curFP, s, rho, i, faceIndexOf(cur))
		if err != nil {
			return nil, true, err
		}
		cur = next
		var ok bool
		if curFP, ok = next.Payload.(facetedrecord.Payload); !ok {
			return nil, true, fmt.Errorf("%w: the blended body lost its payload", decaderr.ErrBooleanFailed)
		}
	}
	return cur, true, nil
}

// findSaddle finds the saddle with key on body, which an earlier step may
// have rewritten elsewhere.
func findSaddle(body *topo.Body, fp facetedrecord.Payload, key saddleKey) (saddle, error) {
	faceIndex := faceIndexOf(body)
	for _, e := range topobuild.Edges(body) {
		if !looksLikeSaddleFillet(e) {
			continue
		}
		if _, _, k, ok := saddleTags(e); !ok || k.R != key.R || k.r != key.r || k.sigma != key.sigma {
			continue
		}
		s, err := readSaddle(e, fp, faceIndex)
		if err == nil && s.key == key {
			return s, nil
		}
	}
	return saddle{}, fmt.Errorf("%w: a selected edge is no longer an edge of the partly blended body",
		decaderr.ErrUnsupported)
}

// saddleFillet is one mouth's rewrite in progress.
type saddleFillet struct {
	*work
	s     saddle
	c     saddlecanal.Canal
	rows  []float64
	slack float64
	// loc holds the local coordinates of every vertex the rewrite reads or
	// makes, by vertex index.
	loc map[int]r3.Vec
}

// localOf returns v's local coordinates.
func (f *saddleFillet) localOf(v int) r3.Vec {
	if p, ok := f.loc[v]; ok {
		return p
	}
	p := local(f.s.key.pose, f.verts[v])
	f.loc[v] = p
	return p
}

// worldVec maps a local direction to the world: the pose without its shift.
func worldVec(pose crossbore.Pose, d r3.Vec) r3.Vec {
	l := [3]float64{d.X, d.Y, d.Z}
	var w [3]float64
	for i := range 3 {
		w[pose.Axis[i]] = pose.Sign[i] * l[i]
	}
	return r3.Vec{X: w[0], Y: w[1], Z: w[2]}
}

// filletSaddle fillets one mouth and builds the body over the rewritten mesh.
func filletSaddle(ctx context.Context, env topo.Env, fp facetedrecord.Payload, s saddle, rho float64,
	ord int, faceIndex map[*topo.Face]int) (*topo.Body, error) {
	c := saddlecanal.Canal{R: s.key.R, Bore: s.key.r, Rho: rho}
	rows, err := c.Rows()
	if err != nil {
		return nil, err
	}
	f := &saddleFillet{
		work: &work{fp: fp, cuts: map[[2]int]int{}},
		s:    s, c: c, rows: rows, slack: 64 * ulp * s.scale, loc: map[int]r3.Vec{},
	}
	f.verts = append([]r3.Vec(nil), fp.Verts...)
	f.beta = append([]float64(nil), fp.VertexBound...)

	// The strip's vertices, station by station, and their proof.
	const n = saddlecanal.Stations
	m := len(rows) - 1
	grid := make([]int, n*(m+1))
	for k := range n {
		for j := 0; j <= m; j++ {
			p := c.Point(k, rows[j])
			grid[k*(m+1)+j] = f.vertex(s.key.pose.World(p), 0)
			f.loc[grid[k*(m+1)+j]] = p
		}
	}
	proof, err := saddlecanal.Prove(ctx, c, rows, s.key.pose, func(k, j int) r3.Vec {
		return f.verts[grid[k*(m+1)+j]]
	})
	if err != nil {
		return nil, err
	}

	win, xw, err := f.chooseCuts(faceIndex)
	if err != nil {
		return nil, err
	}
	if err := f.checkReach(faceIndex, win, xw); err != nil {
		return nil, err
	}
	shaftGroup, err := f.groupOf(facetsOfFace(fp.FaceOf, faceIndex[s.big]))
	if err != nil {
		return nil, err
	}
	boreGroup, err := f.groupOf(facetsOfFace(fp.FaceOf, faceIndex[s.bore]))
	if err != nil {
		return nil, err
	}
	shaftRing, err := f.clipShaft(faceIndex[s.big], win)
	if err != nil {
		return nil, err
	}
	boreRing, err := f.clipBore(faceIndex[s.bore], xw)
	if err != nil {
		return nil, err
	}
	kept := f.keptVertices()
	if slices.ContainsFunc(s.chain, func(v int) bool { _, ok := kept[v]; return ok }) {
		return nil, fmt.Errorf("%w: a facet the saddle fillet keeps holds a seam vertex", decaderr.ErrUnsupported)
	}

	t1, t2 := make([]ringPt, n), make([]ringPt, n)
	for k := range n {
		t1[k] = ringPt{idx: grid[k*(m+1)], ang: saddlecanal.Theta(k)}
		t2[k] = ringPt{idx: grid[k*(m+1)+m], ang: saddlecanal.Theta(k)}
	}
	band1From := len(f.out)
	eps1, area1, err := f.band(shaftRing, t1, shaftGroup, proof.Sample, true)
	if err != nil {
		return nil, err
	}
	band2From := len(f.out)
	eps2, area2, err := f.band(boreRing, t2, boreGroup, proof.Sample, false)
	if err != nil {
		return nil, err
	}
	stripFrom := len(f.out)
	groups := append([]facetedrecord.Group(nil), fp.Groups...)
	stripGroup := len(groups)
	groups = append(groups, facetedrecord.Group{
		Origins: []provenance.Ref{{Producer: env.Ref, Role: fmt.Sprintf("fillet(%d)", ord)}},
		Source:  facetedrecord.BlendSource(env.Ref, ord),
	})
	for _, g := range []int{shaftGroup, boreGroup} {
		groups[g].HasCentroid = false
		groups[g].Centroid = measurement.VecMeasurement{}
	}
	flip := poseHand(s.key.pose) < 0
	for k := range n {
		next := (k + 1) % n
		for j := range m {
			a, b := grid[k*(m+1)+j], grid[next*(m+1)+j]
			c2, d := grid[next*(m+1)+j+1], grid[k*(m+1)+j+1]
			for _, t := range [][3]int{{a, b, c2}, {a, c2, d}} {
				if flip {
					t[1], t[2] = t[2], t[1]
				}
				f.emit(t, stripGroup)
			}
		}
	}

	// Every new vertex takes the largest bound of the three pieces it may
	// border: a band or strip facet lies within that of its exact surface.
	eps := proofbound.AbsSumUpper(max(proof.Chord, eps1, eps2), f.slack)
	for v := len(fp.Verts); v < len(f.verts); v++ {
		f.beta[v] = max(f.beta[v], eps)
	}
	if err := f.checkStripClear(ctx, stripFrom, [][2]int{{band1From, band2From}, {band2From, stripFrom}}); err != nil {
		return nil, err
	}
	if err := f.clearOf(ctx, f.out[stripFrom:], f.out[stripFrom:], "the fillet", "the fillet's", nil); err != nil {
		return nil, err
	}
	trueBands := proofbound.AbsSumUpper(area1.trueHi, area2.trueHi)
	heldBands := proofbound.AbsSumUpper(area1.held, area2.held)
	return f.buildSaddle(ctx, env, groups, stripFrom, proof, eps, heldBands, trueBands, faceIndex)
}

// poseHand is the determinant of the pose's signed permutation: the product
// of its signs, negated for an odd permutation of the axes.
func poseHand(pose crossbore.Pose) float64 {
	h := pose.Sign[0] * pose.Sign[1] * pose.Sign[2]
	for i := range 3 {
		for j := i + 1; j < 3; j++ {
			if pose.Axis[i] > pose.Axis[j] {
				h = -h
			}
		}
	}
	return h
}

// keptVertices is the set of vertices a facet the rewrite keeps or emits
// holds.
func (f *saddleFillet) keptVertices() map[int]struct{} {
	gone := make(map[int]struct{}, len(f.removed))
	for _, i := range f.removed {
		gone[i] = struct{}{}
	}
	kept := map[int]struct{}{}
	for i, t := range f.fp.Tris {
		if _, ok := gone[i]; !ok {
			kept[t[0]], kept[t[1]], kept[t[2]] = struct{}{}, struct{}{}, struct{}{}
		}
	}
	for _, t := range f.out {
		kept[t[0]], kept[t[1]], kept[t[2]] = struct{}{}, struct{}{}, struct{}{}
	}
	return kept
}

// shaftWindow is the window the shaft wall is cut out of: a convex polygon,
// counter-clockwise in the local (y, z) projection.
type shaftWindow struct {
	corners []pt2
}

func (w shaftWindow) side(i int) (pt2, pt2) {
	return w.corners[i], w.corners[(i+1)%len(w.corners)]
}

// cross2 is (b − a) × (p − a).
func cross2(a, b, p pt2) float64 { return (b.u-a.u)*(p.w-a.w) - (b.w-a.w)*(p.u-a.u) }

// pointSegment2 is the distance from p to the segment ab.
func pointSegment2(p, a, b pt2) float64 {
	du, dw := b.u-a.u, b.w-a.w
	t := 0.0
	if l := du*du + dw*dw; l > 0 {
		t = math.Min(1, math.Max(0, ((p.u-a.u)*du+(p.w-a.w)*dw)/l))
	}
	return math.Hypot(p.u-a.u-t*du, p.w-a.w-t*dw)
}

// chooseCuts places the shaft wall's window and the bore's cut plane. The
// window's corners lie on the ellipse (R'/R)²y² + z² = s², which the map
// (y, z) ↦ ((R'/R)·y, z) takes to a circle, and s·cos(π/K) = r' + margin, so
// the window holds T1 ((R'/R)²y² + z² = r'²) with that margin. The bore is cut
// at x = X_w = √(R'² − r'²) − margin, inside T2's least x. Each is turned or
// moved until no held vertex comes within clipTol of a cut and no window
// corner within clipTol of a held edge.
func (f *saddleFillet) chooseCuts(faceIndex map[*topo.Face]int) (shaftWindow, float64, error) {
	R := f.s.key.R
	rp, Rp := f.c.Bore+f.c.Rho, f.c.R-f.c.Rho
	margin := rp * windowMargin
	s := (rp + margin) / math.Cos(math.Pi/windowSides)
	shaft := f.mouthFacets(faceIndex[f.s.big])
	var win shaftWindow
	found := false
	for _, off := range []float64{0.5, 0.3, 0.7, 0.1, 0.9, 0.2, 0.8, 0.4, 0.6} {
		w := shaftWindow{corners: make([]pt2, windowSides)}
		for i := range windowSides {
			g := 2 * math.Pi * (float64(i) + off) / windowSides
			w.corners[i] = pt2{s * math.Cos(g) * R / Rp, s * math.Sin(g)}
		}
		if f.windowClear(w, shaft) {
			win, found = w, true
			break
		}
	}
	if !found {
		return shaftWindow{}, 0, fmt.Errorf("%w: the saddle fillet's window runs through a held vertex",
			decaderr.ErrUnsupported)
	}
	bore := facetsOfFace(f.fp.FaceOf, faceIndex[f.s.bore])
	for _, shift := range []float64{0, 0.25, 0.5, 0.75} {
		xw := math.Sqrt(Rp*Rp-rp*rp) - margin*(1+shift)
		apart := true
		for _, t := range bore {
			for _, v := range f.fp.Tris[t] {
				apart = apart && math.Abs(f.localOf(v).X-xw) > clipTol
			}
		}
		if apart && xw > 0 {
			return win, xw, nil
		}
	}
	return shaftWindow{}, 0, fmt.Errorf("%w: the saddle fillet's bore cut runs through a held vertex",
		decaderr.ErrUnsupported)
}

// mouthFacets lists the facets of face fi whose (y, z) projection may meet
// the mouth's region: every facet with a vertex on the mouth's side, x > 0.
func (f *saddleFillet) mouthFacets(fi int) []int {
	var out []int
	for _, t := range facetsOfFace(f.fp.FaceOf, fi) {
		for _, v := range f.fp.Tris[t] {
			if f.localOf(v).X > 0 {
				out = append(out, t)
				break
			}
		}
	}
	return out
}

func (f *saddleFillet) yz(v int) pt2 {
	p := f.localOf(v)
	return pt2{p.Y, p.Z}
}

// windowClear reports whether no vertex of the facets lies within clipTol of
// a side of w and no corner of w within clipTol of a facet edge.
func (f *saddleFillet) windowClear(w shaftWindow, facets []int) bool {
	lo, hi := w.corners[0], w.corners[0]
	for _, c := range w.corners {
		lo = pt2{math.Min(lo.u, c.u), math.Min(lo.w, c.w)}
		hi = pt2{math.Max(hi.u, c.u), math.Max(hi.w, c.w)}
	}
	for _, t := range facets {
		ps := [3]pt2{f.yz(f.fp.Tris[t][0]), f.yz(f.fp.Tris[t][1]), f.yz(f.fp.Tris[t][2])}
		if max(ps[0].u, ps[1].u, ps[2].u) < lo.u-1 || min(ps[0].u, ps[1].u, ps[2].u) > hi.u+1 ||
			max(ps[0].w, ps[1].w, ps[2].w) < lo.w-1 || min(ps[0].w, ps[1].w, ps[2].w) > hi.w+1 {
			continue
		}
		for i := range w.corners {
			a, b := w.side(i)
			for k := range 3 {
				if pointSegment2(ps[k], a, b) <= clipTol || pointSegment2(a, ps[k], ps[(k+1)%3]) <= clipTol {
					return false
				}
			}
		}
	}
	return true
}

// checkReach refuses when anything but the two walls near this mouth comes
// within the rewrite's reach: a facet of another face, or a segment of another
// boundary loop of either wall, whose box grown by its bounds plus slack meets
// the box of the window, the bore's cut-off end and the fillet
// (agent-docs/retained-edge-blend-design.md §4.10, SR5). Inside that box the
// solid is then the region inside the shaft and outside the bore.
func (f *saddleFillet) checkReach(faceIndex map[*topo.Face]int, w shaftWindow, xw float64) error {
	yMax, zMax := 0.0, 0.0
	for _, c := range w.corners {
		yMax, zMax = math.Max(yMax, math.Abs(c.u)), math.Max(zMax, math.Abs(c.w))
	}
	R := f.s.key.R
	grow := proofbound.AbsSumUpper(f.s.betaBig, f.s.betaBore, f.slack)
	xLo := math.Min(xw, math.Sqrt(math.Max(0, R*R-yMax*yMax))) - grow
	lo := r3.Vec{X: xLo, Y: -yMax - grow, Z: -zMax - grow}
	hi := r3.Vec{X: R + grow, Y: yMax + grow, Z: zMax + grow}
	a, b := f.s.key.pose.World(lo), f.s.key.pose.World(hi)
	box := facetBox{
		lo: r3.NewVec(math.Min(a.X, b.X), math.Min(a.Y, b.Y), math.Min(a.Z, b.Z)),
		hi: r3.NewVec(math.Max(a.X, b.X), math.Max(a.Y, b.Y), math.Max(a.Z, b.Z)),
	}
	return checkMouthClear(f.fp, faceIndex, f.s, box, "fillet")
}

// pieceSet maps the vertices of one facet's pieces to their (y, z) points.
type pieceSet map[int]pt2

// windowPoint is a cut point on a held edge or a window corner, placed on the
// window's boundary at pos: side index plus the fraction along that side.
type windowPoint struct {
	idx int
	pos float64
}

// edgeClip is a held edge's crossings with the window's boundary, read along
// the edge from its lower vertex index to its higher.
type edgeClip struct {
	in, out *windowPoint
}

// clipShaft cuts the shaft wall's mouth facets out of the window w and
// emits the pieces outside it. It returns the cut ring around the window in
// counter-clockwise order, with each point's angle about the bore axis in
// the (R'/R·y, z) map.
func (f *saddleFillet) clipShaft(fi int, w shaftWindow) ([]ringPt, error) {
	facets := f.mouthFacets(fi)
	edges := map[undirected]edgeClip{}
	corner := map[int]int{}
	cornerFacet := map[int]int{}
	pts := pieceSet{}
	bySide := map[int][]*windowPoint{}
	arcs := map[int][]int{}
	var holeRing []int
	inside := func(p pt2) bool {
		for i := range w.corners {
			a, b := w.side(i)
			if cross2(a, b, p) <= 0 {
				return false
			}
		}
		return true
	}
	clipEdge := func(a, b int) edgeClip {
		key := edgeKey(a, b)
		if c, ok := edges[key]; ok {
			return c
		}
		pa, pb := f.yz(key.a), f.yz(key.b)
		tIn, tOut, sideIn, sideOut := 0.0, 1.0, -1, -1
		miss := false
		for i := range w.corners {
			c0, c1 := w.side(i)
			num := cross2(c0, c1, pa)
			den := (c1.u-c0.u)*(pb.w-pa.w) - (c1.w-c0.w)*(pb.u-pa.u)
			if den == 0 {
				miss = miss || num < 0
				continue
			}
			t := -num / den
			if den > 0 && t > tIn {
				tIn, sideIn = t, i
			}
			if den < 0 && t < tOut {
				tOut, sideOut = t, i
			}
		}
		var out edgeClip
		if miss || tIn >= tOut {
			edges[key] = out
			return out
		}
		place := func(t float64, side int) *windowPoint {
			va, vb := f.verts[key.a], f.verts[key.b]
			p := va.Add(vb.Sub(va).Scale(t))
			bound := proofbound.AbsSumUpper(max(f.beta[key.a], f.beta[key.b]), f.slack)
			q := pt2{pa.u + t*(pb.u-pa.u), pa.w + t*(pb.w-pa.w)}
			c0, c1 := w.side(side)
			du, dw := c1.u-c0.u, c1.w-c0.w
			frac := math.Min(math.Max(((q.u-c0.u)*du+(q.w-c0.w)*dw)/(du*du+dw*dw), 0), math.Nextafter(1, 0))
			// A held sliver facet puts two crossings within rounding of each
			// other; they become one vertex, and a piece they collapse is no
			// piece.
			for _, x := range bySide[side] {
				if d := f.verts[x.idx].Sub(p).Len(); d <= f.slack {
					f.beta[x.idx] = max(f.beta[x.idx], proofbound.AbsSumUpper(bound, d))
					return x
				}
			}
			v := f.vertex(p, bound)
			pts[v] = q
			x := &windowPoint{idx: v, pos: float64(side) + frac}
			bySide[side] = append(bySide[side], x)
			return x
		}
		if sideIn >= 0 {
			out.in = place(tIn, sideIn)
		}
		if sideOut >= 0 {
			out.out = place(tOut, sideOut)
		}
		edges[key] = out
		return out
	}
	cornerVertex := func(i, t int) (int, error) {
		if v, ok := corner[i]; ok {
			if cornerFacet[i] != t {
				return 0, fmt.Errorf("%w: a corner of the saddle fillet's window lies on two facets",
					decaderr.ErrUnsupported)
			}
			return v, nil
		}
		tri := f.fp.Tris[t]
		p0, p1, p2 := f.yz(tri[0]), f.yz(tri[1]), f.yz(tri[2])
		q := w.corners[i]
		d := cross2(p0, p1, p2)
		l1, l2 := cross2(p0, q, p2)/d, cross2(p0, p1, q)/d
		l0 := 1 - l1 - l2
		x := l0*f.localOf(tri[0]).X + l1*f.localOf(tri[1]).X + l2*f.localOf(tri[2]).X
		bound := proofbound.AbsSumUpper(max(f.beta[tri[0]], f.beta[tri[1]], f.beta[tri[2]]), f.slack)
		local := r3.Vec{X: x, Y: q.u, Z: q.w}
		v := f.vertex(f.s.key.pose.World(local), bound)
		f.loc[v] = local
		pts[v] = q
		corner[i], cornerFacet[i] = v, t
		return v, nil
	}

	for _, t := range facets {
		tri := f.fp.Tris[t]
		order := [3]int{tri[0], tri[1], tri[2]}
		reversed := orient(f.yz(tri[0]), f.yz(tri[1]), f.yz(tri[2])) < 0
		if reversed {
			order[1], order[2] = order[2], order[1]
		}
		var in [3]bool
		nIn := 0
		for k, v := range order {
			pts[v] = f.yz(v)
			if in[k] = inside(pts[v]); in[k] {
				nIn++
			}
		}
		// The boundary of the facet with its crossings, in order. Read from
		// its lower vertex index to its higher, an edge enters the window at
		// its clip's in point and leaves at its out point; read the other way
		// the two swap roles.
		var walk []walkItem
		crossings := 0
		for k := range 3 {
			a, b := order[k], order[(k+1)%3]
			if !in[k] {
				walk = append(walk, walkItem{idx: a})
			}
			c := clipEdge(a, b)
			first, second := c.in, c.out
			if a > b {
				first, second = c.out, c.in
			}
			if first != nil {
				walk = append(walk, walkItem{idx: first.idx, pos: first.pos, cross: true, entry: true})
				crossings++
			}
			if second != nil {
				walk = append(walk, walkItem{idx: second.idx, pos: second.pos, cross: true})
				crossings++
			}
		}
		if crossings == 0 {
			switch {
			case nIn == 3:
				f.removed = append(f.removed, t)
			case nIn == 0 && pointInTriangle(w.corners[0], pts[order[0]], pts[order[1]], pts[order[2]]):
				hole, err := f.holeFacet(t, order, reversed, cornerVertex, pts)
				if err != nil {
					return nil, err
				}
				holeRing = hole
			case nIn != 0:
				return nil, fmt.Errorf("%w: a facet of the shaft wall crosses the saddle fillet's window "+
					"without a cut", decaderr.ErrUnsupported)
			}
			continue
		}
		if err := f.traceFacet(t, walk, reversed, cornerVertex, pts, arcs); err != nil {
			return nil, err
		}
	}

	if len(corner) != windowSides {
		return nil, fmt.Errorf("%w: a corner of the saddle fillet's window lies on no facet of the shaft wall",
			decaderr.ErrUnsupported)
	}
	ring, err := chainArcs(arcs, holeRing)
	if err != nil {
		return nil, err
	}
	scale := (f.c.R - f.c.Rho) / f.s.key.R
	out := make([]ringPt, len(ring))
	for i, x := range ring {
		q := pts[x]
		a := math.Atan2(q.w, scale*q.u)
		if a < 0 {
			a += 2 * math.Pi
		}
		out[i] = ringPt{idx: x, ang: a}
	}
	return monotoneRing(out, "the shaft wall's cut")
}

// chainArcs joins the window's boundary arcs, each running clockwise inside
// one facet from the crossing where the facet's boundary enters the window
// to the one where it leaves, into the counter-clockwise ring of cut points.
// The exit of one arc is the entry of the next facet's. A window inside one
// facet is that facet's ring of corners, hole, already clockwise.
func chainArcs(arcs map[int][]int, hole []int) ([]int, error) {
	if hole != nil {
		if len(arcs) != 0 {
			return nil, fmt.Errorf("%w: the saddle fillet's window lies inside a facet and crosses others",
				decaderr.ErrUnsupported)
		}
		return reverseRing(hole), nil
	}
	start := -1
	for e := range arcs {
		if start < 0 || e < start {
			start = e
		}
	}
	if start < 0 {
		return nil, fmt.Errorf("%w: the saddle fillet's window meets no facet of the shaft wall", decaderr.ErrUnsupported)
	}
	var cw []int
	used := 0
	for at := start; ; {
		arc, ok := arcs[at]
		if !ok {
			return nil, fmt.Errorf("%w: the shaft wall's cut does not close around the window", decaderr.ErrUnsupported)
		}
		used++
		cw = append(cw, arc[:len(arc)-1]...)
		at = arc[len(arc)-1]
		if at == start {
			break
		}
		if used > len(arcs) {
			return nil, fmt.Errorf("%w: the shaft wall's cut runs around the window more than once",
				decaderr.ErrUnsupported)
		}
	}
	if used != len(arcs) {
		return nil, fmt.Errorf("%w: the shaft wall's cut is more than one loop", decaderr.ErrUnsupported)
	}
	return reverseRing(cw), nil
}

func reverseRing(cw []int) []int {
	out := make([]int, len(cw))
	for i, v := range cw {
		out[len(cw)-1-i] = v
	}
	return out
}

// walkItem is one point of a facet's boundary walk: a held vertex outside
// the window, or a crossing with entry set where the walk enters it.
type walkItem struct {
	idx   int
	pos   float64
	cross bool
	entry bool
}

// pointInTriangle reports whether p lies inside the counter-clockwise
// triangle abc.
func pointInTriangle(p, a, b, c pt2) bool {
	return orient(a, b, p) > 0 && orient(b, c, p) > 0 && orient(c, a, p) > 0
}

// monotoneRing rotates ring to start at its least angle and refuses unless
// the angles then increase strictly and leave no gap of half a turn.
func monotoneRing(ring []ringPt, what string) ([]ringPt, error) {
	if len(ring) < 3 {
		return nil, fmt.Errorf("%w: %s leaves fewer than three vertices", decaderr.ErrUnsupported, what)
	}
	start := 0
	for i, p := range ring {
		if p.ang < ring[start].ang {
			start = i
		}
	}
	out := append(append([]ringPt(nil), ring[start:]...), ring[:start]...)
	for i, p := range out {
		next := 2*math.Pi + out[0].ang
		if i+1 < len(out) {
			next = out[i+1].ang
		}
		if !(next > p.ang) || next-p.ang >= math.Pi {
			fmt.Println("DEBUG ring", what, len(out), i, p.ang, next)
			for _, q := range out {
				fmt.Printf("%.4f ", q.ang)
			}
			fmt.Println()
			return nil, fmt.Errorf("%w: %s does not run once around the bore", decaderr.ErrUnsupported, what)
		}
	}
	return out, nil
}

// traceFacet emits the part of facet t outside the window: each loop runs
// along the facet's boundary outside the window and, from each entry, along
// the window's boundary clockwise to the next crossing, which must be an
// exit, through the window corners between them. Each such arc is recorded
// in arcs by its entry, for chainArcs.
func (f *saddleFillet) traceFacet(t int, walk []walkItem, reversed bool,
	cornerVertex func(i, t int) (int, error), pts pieceSet, arcs map[int][]int) error {
	// exitOf returns the walk index of the crossing reached first going
	// clockwise along the window's boundary from the entry at walk index e;
	// of two at one place, the exit.
	exitOf := func(e int) int {
		best, bestD := -1, math.Inf(1)
		for i, it := range walk {
			if !it.cross || i == e {
				continue
			}
			d := math.Mod(walk[e].pos-it.pos+windowSides, windowSides)
			if d < bestD || (d == bestD && !it.entry) {
				best, bestD = i, d
			}
		}
		return best
	}
	used := map[int]bool{}
	var loops [][]int
	for s := range walk {
		if used[s] || walk[s].entry {
			continue
		}
		var loop []int
		add := func(v int) {
			if len(loop) == 0 || loop[len(loop)-1] != v {
				loop = append(loop, v)
			}
		}
		for i := s; ; {
			if used[i] {
				if i != s {
					return fmt.Errorf("%w: a facet of the shaft wall splits into crossing pieces", decaderr.ErrUnsupported)
				}
				break
			}
			used[i] = true
			it := walk[i]
			add(it.idx)
			if !it.entry {
				i = (i + 1) % len(walk)
				continue
			}
			x := exitOf(i)
			if x < 0 || walk[x].entry {
				return fmt.Errorf("%w: the saddle fillet's window enters a facet twice", decaderr.ErrUnsupported)
			}
			// Corners from the entry's side down to the exit's, clockwise.
			arc := []int{it.idx}
			lo, hi := walk[x].pos, it.pos
			if lo > hi {
				hi += windowSides
			}
			for c := math.Floor(hi); c > lo; c-- {
				v, err := cornerVertex(int(c)%windowSides, t)
				if err != nil {
					return err
				}
				add(v)
				arc = append(arc, v)
			}
			arc = append(arc, walk[x].idx)
			if len(arc) > 2 || arc[0] != arc[1] {
				if _, dup := arcs[it.idx]; dup {
					return fmt.Errorf("%w: two facets enter the saddle fillet's window at one point",
						decaderr.ErrUnsupported)
				}
				arcs[it.idx] = arc
			}
			i = x
		}
		if len(loop) > 1 && loop[0] == loop[len(loop)-1] {
			loop = loop[:len(loop)-1]
		}
		if len(loop) >= 3 {
			loops = append(loops, loop)
		}
	}
	for i, it := range walk {
		if !used[i] {
			return fmt.Errorf("%w: a piece of a shaft wall facet is not traced (vertex %d)", decaderr.ErrUnsupported, it.idx)
		}
	}
	return f.emitPieces(t, loops, reversed, pts)
}

// holeFacet emits facet t less the window inside it.
func (f *saddleFillet) holeFacet(t int, order [3]int, reversed bool,
	cornerVertex func(i, t int) (int, error), pts pieceSet) ([]int, error) {
	hole := make([]int, windowSides)
	for i := range windowSides {
		v, err := cornerVertex(windowSides-1-i, t)
		if err != nil {
			return nil, err
		}
		hole[i] = v
	}
	ids, ps := f.planar(append(order[:], hole...), pts)
	tris, err := triangulateRegion(ps, ids[:3], [][]int{ids[3:]})
	if err != nil {
		return nil, err
	}
	f.emitLocal(t, tris, order[:], hole, reversed)
	return hole, nil
}

// planar numbers the vertices and returns their (y, z) points in that order.
func (f *saddleFillet) planar(vs []int, pts pieceSet) ([]int, []pt2) {
	ids := make([]int, len(vs))
	ps := make([]pt2, len(vs))
	for i, v := range vs {
		ids[i], ps[i] = i, pts[v]
	}
	return ids, ps
}

func (f *saddleFillet) emitLocal(t int, tris [][3]int, outer, hole []int, reversed bool) {
	all := append(append([]int(nil), outer...), hole...)
	for _, tr := range tris {
		out := [3]int{all[tr[0]], all[tr[1]], all[tr[2]]}
		if reversed {
			out[1], out[2] = out[2], out[1]
		}
		f.emit(out, f.fp.Src[t])
	}
	f.removed = append(f.removed, t)
}

// emitPieces triangulates each traced loop of facet t in the (y, z)
// projection and emits it with the facet's winding.
func (f *saddleFillet) emitPieces(t int, loops [][]int, reversed bool, pts pieceSet) error {
	for _, loop := range loops {
		ids, ps := f.planar(loop, pts)
		tris, err := earClip(ps, ids)
		if err != nil {
			return fmt.Errorf("%w: a piece of a shaft wall facet outside the saddle fillet's window: %w",
				decaderr.ErrUnsupported, err)
		}
		for _, tr := range flipSlivers(ps, tris) {
			out := [3]int{loop[tr[0]], loop[tr[1]], loop[tr[2]]}
			if reversed {
				out[1], out[2] = out[2], out[1]
			}
			f.emit(out, f.fp.Src[t])
		}
	}
	f.removed = append(f.removed, t)
	return nil
}

// clipBore cuts the bore wall's facets by the plane x = xw and keeps the
// part toward the bore's far end. It returns the cut ring sorted by angle
// about the bore axis.
func (f *saddleFillet) clipBore(fi int, xw float64) ([]ringPt, error) {
	g := func(v int) float64 { return xw - f.localOf(v).X }
	cut := func(a, b int) int {
		k := [2]int{min(a, b), max(a, b)}
		if c, ok := f.cuts[k]; ok {
			return c
		}
		ga, gb := g(k[0]), g(k[1])
		t := ga / (ga - gb)
		va, vb := f.verts[k[0]], f.verts[k[1]]
		p := va.Add(vb.Sub(va).Scale(t))
		bound := proofbound.AbsSumUpper(max(f.beta[a], f.beta[b]), f.slack)
		// Two cuts within rounding of each other, across a held sliver,
		// become one vertex.
		for _, c := range f.cuts {
			if d := f.verts[c].Sub(p).Len(); d <= f.slack {
				f.beta[c] = max(f.beta[c], proofbound.AbsSumUpper(bound, d))
				f.cuts[k] = c
				return c
			}
		}
		c := f.vertex(p, bound)
		f.cuts[k] = c
		return c
	}
	for _, t := range facetsOfFace(f.fp.FaceOf, fi) {
		tri := f.fp.Tris[t]
		var above []int
		for k, v := range tri {
			if g(v) > 0 {
				above = append(above, k)
			}
		}
		switch len(above) {
		case 3:
			continue
		case 0:
		case 1:
			x := above[0]
			a, b, c := tri[x], tri[(x+1)%3], tri[(x+2)%3]
			f.emitSome([3]int{a, cut(a, b), cut(c, a)}, f.fp.Src[t])
		case 2:
			z := 0
			for k := range 3 {
				if g(tri[k]) <= 0 {
					z = k
				}
			}
			x, y, below := tri[(z+1)%3], tri[(z+2)%3], tri[z]
			cyz, czx := cut(y, below), cut(below, x)
			f.emitSome([3]int{x, y, cyz}, f.fp.Src[t])
			f.emitSome([3]int{x, cyz, czx}, f.fp.Src[t])
		}
		f.removed = append(f.removed, t)
	}
	ring := make([]ringPt, 0, len(f.cuts))
	seen := map[int]struct{}{}
	for _, c := range f.cuts {
		if _, dup := seen[c]; dup {
			continue
		}
		seen[c] = struct{}{}
		p := f.localOf(c)
		a := math.Atan2(p.Z, p.Y)
		if a < 0 {
			a += 2 * math.Pi
		}
		ring = append(ring, ringPt{idx: c, ang: a})
	}
	sort.Slice(ring, func(i, j int) bool { return ring[i].ang < ring[j].ang })
	return monotoneRing(ring, "the bore's cut")
}

// emitSome emits t unless merged cuts collapse it.
func (f *saddleFillet) emitSome(t [3]int, src int) {
	if t[0] != t[1] && t[1] != t[2] && t[2] != t[0] {
		f.emit(t, src)
	}
}

// bandArea is a band's held area and an upper bound on the exact area of
// the wall it stands for.
type bandArea struct{ held, trueHi float64 }

// band joins the held cut ring to the exact ring of tangency points by a
// zipper of triangles on the wall, emitted under group. On the shaft wall
// (shaft set) each facet faces away from the shaft's axis and its (y, z)
// projection keeps its orientation, so the band is embedded: both rings run
// once around in the (R'/R·y, z) angle and every facet keeps the sense of
// that projection. On the bore each facet faces the bore's axis from every
// vertex's direction, so the band's radial projection onto the bore keeps
// its sense and the band is embedded. A facet that fails refuses. It returns
// the band's bound: the largest over its facets of the largest distance of
// a vertex from the wall, which is the vertex's bound (sample for an exact
// tangency point), plus the sagitta R·Δ²/8 of the facet's angular spread Δ
// about the wall's axis.
func (f *saddleFillet) band(cut, exact []ringPt, group int, sample float64, shaft bool) (float64, bandArea, error) {
	pose := f.s.key.pose
	radius := f.s.key.r
	if shaft {
		radius = f.s.key.R
	}
	isExact := map[int]struct{}{}
	for _, p := range exact {
		isExact[p.idx] = struct{}{}
	}
	from := len(f.out)
	eps := 0.0
	for _, t := range zipper(cut, exact) {
		var dirs [3]r3.Vec
		var angs [3]float64
		dist := 0.0
		for k, v := range t {
			p := f.localOf(v)
			if shaft {
				dirs[k] = worldVec(pose, r3.Vec{X: p.X, Y: p.Y})
				angs[k] = math.Atan2(p.Y, p.X)
			} else {
				dirs[k] = worldVec(pose, r3.Vec{Y: -p.Y, Z: -p.Z})
				angs[k] = math.Atan2(p.Z, p.Y)
			}
			d := f.beta[v]
			if _, ok := isExact[v]; ok {
				d = sample
			}
			dist = max(dist, d)
		}
		a, b, c := f.verts[t[0]], f.verts[t[1]], f.verts[t[2]]
		n := b.Sub(a).Cross(c.Sub(a))
		if n.Dot(dirs[0].Add(dirs[1]).Add(dirs[2])) < 0 {
			t[1], t[2] = t[2], t[1]
			n = n.Scale(-1)
		}
		checks := dirs[:]
		if shaft {
			checks = append(checks, worldVec(pose, r3.Vec{X: 1}))
		}
		for _, d := range checks {
			if !(n.Dot(d) > foldMargin*n.Len()*d.Len()) {
				return 0, bandArea{}, fmt.Errorf("%w: the saddle fillet's band on the %s folds", decaderr.ErrUnsupported,
					map[bool]string{true: "shaft wall", false: "bore"}[shaft])
			}
		}
		// The spread of the vertices' angles, widened by the angle their
		// distance from the wall can hide, |Δ| ≤ 2·dist/(radius − dist).
		lo, hi := angs[0], angs[0]
		for _, x := range angs[1:] {
			if x-angs[0] > math.Pi {
				x -= 2 * math.Pi
			} else if angs[0]-x > math.Pi {
				x += 2 * math.Pi
			}
			lo, hi = math.Min(lo, x), math.Max(hi, x)
		}
		spread := proofbound.AbsSumUpper(hi-lo, proofbound.DivUpper(2*dist, radius-dist), 1e-12)
		sag := proofbound.DivUpper(proofbound.ProductUpper(radius, proofbound.ProductUpper(spread, spread)), 8)
		eps = max(eps, proofbound.AbsSumUpper(dist, sag))
		f.emit(t, group)
	}
	held, _ := heldArea(f.verts, f.out[from:])
	trueHi, err := f.bandTrueArea(shaft, cut)
	if err != nil {
		return 0, bandArea{}, err
	}
	return eps, bandArea{held: held, trueHi: trueHi}, nil
}

// bandTrueArea bounds the exact area of the wall a band stands for. On the
// shaft wall it is the cylinder over the window less T1's ellipse in the
// (y, z) projection, whose area element R/√(R² − y²) is at most R/√(R² − Y²)
// with Y the window's widest |y|; the window's area is at most that of the
// polygon of its corners. On the bore it is the strip between the cut and
// T2, no wider than R' − X_w, around the bore's circumference.
func (f *saddleFillet) bandTrueArea(shaft bool, cut []ringPt) (float64, error) {
	R, r := f.s.key.R, f.s.key.r
	rp, Rp := f.c.Bore+f.c.Rho, f.c.R-f.c.Rho
	piHi := math.Nextafter(math.Pi, math.Inf(1))
	if !shaft {
		xw := math.Inf(1)
		for _, p := range cut {
			xw = math.Min(xw, f.localOf(p.idx).X)
		}
		width := proofbound.AbsSumUpper(Rp, -xw, f.slack)
		return proofbound.ProductUpper(proofbound.ProductUpper(2*piHi, r), width), nil
	}
	poly, yMax := 0.0, 0.0
	for i, p := range cut {
		a, b := f.localOf(p.idx), f.localOf(cut[(i+1)%len(cut)].idx)
		poly += a.Y*b.Z - b.Y*a.Z
		yMax = math.Max(yMax, math.Abs(a.Y))
	}
	poly = proofbound.AbsSumUpper(math.Abs(poly)/2, proofbound.ProductUpper(float64(len(cut))*0x1p-50, yMax*yMax+1))
	// T1's ellipse has semi-axes r'·R/R' and r'.
	ellipse := math.Pi * rp * rp * R / Rp * (1 - 0x1p-48)
	if !(yMax < R) {
		return 0, fmt.Errorf("%w: the saddle fillet's window reaches the side of the shaft", decaderr.ErrUnsupported)
	}
	factor := proofbound.DivUpper(R, math.Sqrt(R*R-yMax*yMax)*(1-0x1p-50))
	return proofbound.ProductUpper(math.Max(0, poly-ellipse), factor), nil
}

// buildSaddle assembles the rewritten payload and the body over it. The
// volume grows by eps times the held and exact areas of the strip and the
// bands (Claim K), and the area slack by the gap between the held area change
// and the exact one, whose exact part is an interval: the fillet added, the
// shaft wall's region between the seam and T1 and the bore's between the seam
// and T2 removed (saddlecanal.Proof).
func (f *saddleFillet) buildSaddle(ctx context.Context, env topo.Env, groups []facetedrecord.Group, stripFrom int,
	proof saddlecanal.Proof, eps, heldBands, trueBands float64, faceIndex map[*topo.Face]int) (*topo.Body, error) {
	fp := f.fp
	gone := map[int]bool{}
	for _, t := range f.removed {
		gone[t] = true
	}
	var facets [][3]int
	var src []int
	for i, t := range fp.Tris {
		if !gone[i] {
			facets = append(facets, t)
			src = append(src, fp.Src[i])
		}
	}
	facets = append(facets, f.out...)
	src = append(src, f.outSrc...)
	verts, beta, facets := compact(f.verts, f.beta, facets)

	oldArea, oldSlop := heldArea(fp.Verts, pick(fp.Tris, f.removed))
	newArea, newSlop := heldArea(f.verts, f.out)
	stripArea, _ := heldArea(f.verts, f.out[stripFrom:])
	deltaHeld := newArea - oldArea
	trueLo := proof.AreaLo - proof.ShaftHi - proof.BoreHi
	trueHi := proof.AreaHi - proof.ShaftLo - proof.BoreLo
	gap := math.Max(math.Abs(deltaHeld-trueLo), math.Abs(deltaHeld-trueHi))
	scale := math.Max(math.Abs(deltaHeld), math.Abs(trueHi)) + oldArea + newArea + proof.AreaHi
	oldGeom, err := f.rewrittenGeom(ctx, faceIndex)
	if err != nil {
		return nil, err
	}
	areaSlack := proofbound.AbsSumUpper(gap, oldSlop, newSlop, 64*0x1p-52*scale, oldGeom)
	volume := proofbound.ProductUpper(eps, proofbound.AbsSumUpper(stripArea, proof.AreaHi, heldBands, trueBands))
	volume = proofbound.AbsSumUpper(volume, 64*0x1p-52*eps*scale)

	meshBound := facetproof.FacetBoundMax(facets, beta)
	next := facetedrecord.Payload{
		Verts: verts, VertexBound: beta, Tris: facets, Src: src, Groups: groups,
		MeshBound:   meshBound,
		VolSymDiff:  proofbound.AbsSumUpper(fp.VolSymDiff, volume),
		AreaSlack:   proofbound.AbsSumUpper(fp.AreaSlack, areaSlack),
		DPair:       facetedrecord.SourceDiameterUpper(verts, meshBound),
		Xform:       fp.Xform,
		ExactPlaced: fp.ExactPlaced,
	}
	if proofbound.IsNonFinite(next.VolSymDiff) || proofbound.IsNonFinite(next.AreaSlack) {
		return nil, fmt.Errorf("%w: the saddle fillet has no finite bound", decaderr.ErrUnsupported)
	}
	body, err := facetedrecord.Build(ctx, env, next)
	if err != nil {
		return nil, fmt.Errorf("%w: the blended mesh fails its held audit: %w", decaderr.ErrUnsupported, err)
	}
	return body, nil
}
