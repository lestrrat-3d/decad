package retainedblend

import (
	"errors"
	"fmt"
	"maps"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file builds the blend of an arc whose end meets a retained cylinder G
// (arc_end_cyl.go): A's chain along G, the wall's band between the radial planes,
// and the cap G gains where the wall's blend ends on it. G is not retriangulated:
// its held facets stay, the ones that keep an edge of A's chain are split where
// the cap's columns fall on the edge, and the cap's facets join them. G's
// unrolled coordinates are x, the arc length along G's circle from V in the sense
// A's chain leaves it, and y = side·height above A's plane.

func wrapPi(x float64) float64 { return x - 2*math.Pi*math.Round(x/(2*math.Pi)) }

// angleAbout is the angle of p about the axis through c, from the rim's u axis.
func (a arcRim) angleAbout(c, p r3.Vec) float64 {
	q := p.Sub(c)
	return math.Atan2(q.Dot(a.w), q.Dot(a.u))
}

// alongG is the arc length from end i's corner V to p along G's circle, signed
// by the sense of A's chain (cylEnd.dir, set by readCylChain).
func (a arcRim) alongG(i int, p r3.Vec) float64 {
	c := a.cyl[i]
	return c.dir * c.rG * wrapPi(a.angleAbout(c.c, p)-a.angleAbout(c.c, a.ends[i].exact))
}

// unrollG is p in end i's unrolled coordinates.
func (a arcRim) unrollG(i int, p r3.Vec) pt2 { return pt2{a.alongG(i, p), a.side * a.height(p)} }

// uncertainty is how far a point within beta of G's circle may lie along it.
func (c *cylEnd) uncertainty(beta float64) float64 {
	return c.rG * math.Asin(math.Min(1, beta/c.rG))
}

// outwardG is the outward normal of G at p, toward G's axis (G is Reversed).
func (a arcRim) outwardG(i int, p r3.Vec) r3.Vec {
	q := p.Sub(a.cyl[i].c)
	q = q.Sub(a.n.Scale(q.Dot(a.n)))
	return q.Scale(-1)
}

// cylFaceLoops is faceLoops for a cylinder end face, which keeps its interior
// vertices: it is not retriangulated.
func (w *work) cylFaceLoops(fi int) (faceLoops, error) {
	f := faceLoops{index: fi, facets: facetsOfFace(w.fp.FaceOf, fi)}
	var err error
	if f.group, err = w.groupOf(f.facets); err != nil {
		return f, err
	}
	f.loops, err = boundaryLoops(w.fp.Tris, f.facets)
	return f, err
}

// chainWalker is the onChain test of locateBy for A's chain along end i's G: a
// vertex joins the chain when the edge from the previous vertex is also an edge
// of a G facet (A's boundary then runs along G) and the vertex lies within its
// bound of G's circle in A's plane. It follows one walk.
func (a arcRim) chainWalker(w *work, i int, g faceLoops) func(int) bool {
	c := a.cyl[i]
	edges := map[[2]int]struct{}{}
	for _, f := range g.facets {
		t := w.fp.Tris[f]
		for k := range 3 {
			edges[[2]int{min(t[k], t[(k+1)%3]), max(t[k], t[(k+1)%3])}] = struct{}{}
		}
	}
	prev := a.ends[i].v
	slack := a.slack()
	return func(v int) bool {
		p := prev
		prev = v
		if _, ok := edges[[2]int{min(p, v), max(p, v)}]; !ok {
			return false
		}
		q := w.verts[v].Sub(c.c)
		plan := math.Hypot(q.Dot(a.u), q.Dot(a.w))
		return math.Hypot(plan-c.rG, a.height(w.verts[v])) <= w.beta[v]+slack
	}
}

// readCylChain sets the sense A's chain leaves end i along G's circle from the
// chain's last vertex, which must lie farther along than its uncertainty.
func (a arcRim) readCylChain(w *work, i int, chain []int, margin float64) error {
	c := a.cyl[i]
	c.dir = 1
	last := chain[len(chain)-1]
	s := a.alongG(i, w.verts[last])
	if !(math.Abs(s) > c.uncertainty(w.beta[last])+margin) {
		return fmt.Errorf("%w: a face leaves an arc end along too short a run to read its direction",
			decaderr.ErrUnsupported)
	}
	if s < 0 {
		c.dir = -1
	}
	return nil
}

// cylSetback is the setback of A's chain at end i, whose P_A is the row 0 point
// p: the chain's vertices before it, which the plane loses.
func (a arcRim) cylSetback(w *work, i int, p r3.Vec, chain []int, margin float64) ([]int, error) {
	c := a.cyl[i]
	tp := a.alongG(i, p)
	if tp <= margin {
		return nil, fmt.Errorf("%w: a setback point lies behind an arc end", decaderr.ErrUnsupported)
	}
	last := chain[len(chain)-1]
	if err := clearance(a.alongG(i, w.verts[last])-tp, c.uncertainty(w.beta[last])+margin,
		"the plane beside the arc"); err != nil {
		// At a cylinder end a chain that ends before the row's meeting means the row runs onto
		// the next face (X9), which is no consumption of the plane: it refuses, it does not degenerate.
		if errors.Is(err, decaderr.ErrDegenerate) {
			return nil, fmt.Errorf("%w: the blend consumes the plane beside the arc", decaderr.ErrUnsupported)
		}
		return nil, err
	}
	var rem []int
	for _, v := range chain {
		tc := a.alongG(i, w.verts[v])
		if math.Abs(tc-tp) <= 4*margin {
			return nil, fmt.Errorf("%w: a setback point falls on a vertex of its face's boundary", decaderr.ErrUnsupported)
		}
		if tc < tp {
			rem = append(rem, v)
		}
	}
	return rem, nil
}

// radialPlane is the plane through the axis and V_i, normal to the rim's plane.
func (a arcRim) radialPlane(i int) (plane, bool) {
	n, ok := a.n.Cross(a.ends[i].exact.Sub(a.center)).Normalize()
	return plane{n: n, o: a.center}, ok
}

// cylBandEnd is the band's end plane at a cylinder end: the radial plane Π_i
// through V_i. On the true wall B the band's boundary is the generator through
// V_i, which lies in Π_i, so the band is the set G would cut on the true wall.
// Π_i holds the axis, so it meets B square. Its on set is V_i and B's chain
// along G, each admitted by provenance and never by a residual: (a) the vertex
// bounds a facet of B and a facet of G, so its true point lies in both closures,
// on B ∩ G, the generators through V and V*, and (b) it lies farther from V*'s
// generator than its bound, so the true point is on V's, in Π_i up to Π_i's own
// rounding, which the band's slack carries (e_V).
func (a arcRim) cylBandEnd(w *work, f faceLoops, i int, run edgeRun) (*bandEnd, error) {
	c := a.cyl[i]
	g, ok := a.radialPlane(i)
	if !ok {
		return nil, fmt.Errorf("%w: an arc end lies on the axis", decaderr.ErrUnsupported)
	}
	sign := 1.0
	if g.offset(a.ends[1-i].exact) < 0 {
		sign = -1
	}
	inFacets := func(facets []int, v int) bool {
		return slices.ContainsFunc(facets, func(t int) bool { return slices.Contains(w.fp.Tris[t][:], v) })
	}
	gFacets := facetsOfFace(w.fp.FaceOf, w.faceIx[a.ends[i].face])
	on := map[int]bool{a.ends[i].v: true}
	slack := proofbound.AbsSumUpper(a.slack(), c.eV)
	c.noise = nil
	for _, v := range run.chain[i] {
		if !inFacets(f.facets, v) || !inFacets(gFacets, v) ||
			!(distToLine(w.verts[v], c.other, a.n) > w.beta[v]+slack+c.eV) {
			return nil, fmt.Errorf("%w: the wall's chain along a cylinder end face does not lie on the end's generator",
				decaderr.ErrUnsupported)
		}
		on[v] = true
	}
	sine, planar := w.wallSine(f.index, g)
	// A boundary vertex of the wall beside V, one of the arc's own, may lie within its
	// bound of Π_i: the held mesh puts a vertex there when the boss's base circle was
	// tessellated near V. Its side is then undecided, and a side read from its held
	// offset would leave the cut along Π_i short of V. It is read as on Π_i: a choice,
	// as a sign reading is (signRead), whose bound the same distance charges: the
	// distance from the held point to the band's boundary on the wall.
	inner := slices.Clone(run.inner)
	if i == 1 {
		slices.Reverse(inner)
	}
	for _, v := range inner {
		off := math.Abs(sign * g.offset(w.verts[v]))
		if !(off <= w.beta[v]+slack+w.tau) {
			break
		}
		if sine == nil {
			return nil, fmt.Errorf("%w: a vertex of the wall lies on a cylinder end's radial plane", decaderr.ErrUnsupported)
		}
		beta := w.inputBeta(v)
		bound, ok := slantBound(proofbound.AbsSumUpper(beta, off, slack), sine)
		if !ok {
			return nil, fmt.Errorf("%w: a vertex of the wall lies on a cylinder end's radial plane", decaderr.ErrUnsupported)
		}
		if !planar {
			bound = proofbound.AbsSumUpper(beta, bound)
		}
		w.beta[v] = max(w.beta[v], bound)
		w.raised = max(w.raised, w.beta[v])
		on[v] = true
		c.noise = append(c.noise, v)
	}
	return &bandEnd{g: g, sign: sign, on: on, sine: sine, planarWall: planar}, nil
}

// cylEndChecks are endFaceRun's gates for a cylinder end: V's loop in G runs
// between A's chain and B's chain, G is convex at V by the exact directions of
// the two chains, and no other part of G comes within reach of the cap.
func (a arcRim) cylEndChecks(w *work, i int, g faceLoops, chainA, chainB []int, reachIDs []int, margin float64) error {
	c := a.cyl[i]
	end := a.ends[i]
	li, ok := g.loopWith(end.v)
	if !ok {
		return fmt.Errorf("%w: an end face does not hold the edge's corner", decaderr.ErrUnsupported)
	}
	loop := g.loops[li]
	n := len(loop)
	at := indexIn(loop, end.v)
	prev, next := loop[(at+n-1)%n], loop[(at+1)%n]
	var aFirst bool
	switch {
	case next == chainA[0] && prev == chainB[0]:
	case next == chainB[0] && prev == chainA[0]:
		aFirst = true
	default:
		return fmt.Errorf("%w: an end face's chains disagree with the faces beside the edge", decaderr.ErrUnsupported)
	}
	// A's chain leaves V along G's circle tangent in the sense of dir, B's chain
	// down the generator, against the way the wall climbs (kindB proved it).
	psi := a.angleAbout(c.c, end.exact)
	tangent := a.u.Scale(-math.Sin(psi)).Add(a.w.Scale(math.Cos(psi))).Scale(c.dir)
	dirA, dirB := tangent, a.n.Scale(-a.side)
	in, out := dirB, dirA
	if aFirst {
		in, out = dirA, dirB
	}
	nG, ok := a.outwardG(i, end.exact).Normalize()
	if !ok {
		return fmt.Errorf("%w: an end face's turn at the edge's corner is undecided", decaderr.ErrUnsupported)
	}
	switch cornerTurn(in, out, nG) {
	case 0:
		return fmt.Errorf("%w: an end face's turn at the edge's corner is undecided", decaderr.ErrUnsupported)
	case -1:
		return fmt.Errorf("%w: an end face's corner does not match the edge's convexity", decaderr.ErrUnsupported)
	}
	reach := 0.0
	for _, v := range reachIDs {
		reach = max(reach, w.verts[v].Sub(end.exact).Len())
	}
	reach += margin
	skip := map[int]bool{end.v: true}
	for _, v := range chainA {
		skip[v] = true
	}
	for _, v := range chainB {
		skip[v] = true
	}
	for _, lp := range g.loops {
		for k, v := range lp {
			q := lp[(k+1)%len(lp)]
			if skip[v] && skip[q] {
				continue
			}
			if distToSegment(end.exact, w.verts[v], w.verts[q]) < reach+w.beta[v]+w.beta[q] {
				return fmt.Errorf("%w: another part of an end face comes within the blend's reach of the corner",
					decaderr.ErrUnsupported)
			}
		}
	}
	return nil
}

// capPoly is the cap's loop in end i's unrolled coordinates, P_A first, the
// points the loop of capSimple reads: the rows' ends, the wall's cut back to V.
func (a arcRim) capPoly(i int, pts []r3.Vec) []pt2 {
	ps := make([]pt2, len(pts))
	for k, p := range pts {
		ps[k] = a.unrollG(i, p)
	}
	for k := len(ps) - 1; k >= 0; k-- {
		ps[k] = pt2{ps[k].u - ps[0].u, ps[k].w - ps[0].w}
	}
	return ps
}

// capArea is the exact area of end i's cap: the unrolled polygon V, the blend's
// end curve on G from P_A up to the wall's generator, and back to V.
func (a arcRim) capArea(p arcProfile, i int) float64 {
	const samples = 4000
	pts := []r3.Vec{a.ends[i].exact}
	ref := [2]float64{a.theta0, a.theta0 + a.span}[i]
	for s := 0; s <= samples; s++ {
		rho, z := p(float64(s) / samples)
		t, ok := a.endAzimuths(i, rho, ref)
		if !ok {
			continue
		}
		ref = t
		pts = append(pts, a.atTheta(rho, t, z))
	}
	ps := make([]pt2, len(pts))
	loop := make([]int, len(pts))
	for k, q := range pts {
		ps[k], loop[k] = a.unrollG(i, q), k
	}
	return math.Abs(signedArea2(ps, loop)) / 2
}

// gArc is the Green's theorem term of the arc of G's circle at end i from p0 to
// p1, in the rim's frame about its axis: ½(c × (p1 − p0) + r_G²·Δψ).
func (a arcRim) gArc(i int, p0, p1 r3.Vec) float64 {
	c := a.cyl[i]
	q := c.c.Sub(a.center)
	d := p1.Sub(p0)
	cross := q.Dot(a.u)*d.Dot(a.w) - q.Dot(a.w)*d.Dot(a.u)
	dpsi := wrapPi(a.angleAbout(c.c, p1) - a.angleAbout(c.c, p0))
	return (cross + c.rG*c.rG*dpsi) / 2
}

// splitEdge is a chain edge of A along G that takes new points, listed by id.
type splitEdge struct {
	u, v int
	pts  []int
}

// capFacets builds end i's cap and the split of the held facets of G it needs,
// emitting both into w.out under G's group. chainA is A's chain and remA its
// vertices the plane loses, endIDs the rows' end vertices from P_A (row 0) to
// the wall's band corner, and wallRun the wall's cut from that corner back to V.
// It returns the cap's range of w.out. Every check only refuses.
func (a arcRim) capFacets(w *work, i int, g faceLoops, chainA, remA, endIDs, wallRun []int) ([2]int, error) {
	c := a.cyl[i]
	slack := a.slack()
	v0 := a.ends[i].v
	m := len(endIDs) - 1
	pa, corner := endIDs[0], endIDs[m]
	x := func(id int) float64 { return a.alongG(i, w.verts[id]) }
	folds := fmt.Errorf("%w: the cap on a cylinder end face folds", decaderr.ErrUnsupported)
	noCol := fmt.Errorf("%w: a cap column falls on a vertex of the end face's top edge", decaderr.ErrUnsupported)

	// The bottom chain's held part and the chain edges the new points fall on.
	base := append([]int{v0}, remA...)
	seq := append(slices.Clone(base), chainA[len(remA)])
	sx := make([]float64, len(seq))
	for k, v := range seq {
		sx[k] = x(v)
		if k > 0 && !(sx[k] > sx[k-1]) {
			return [2]int{}, noCol
		}
	}
	tp := x(pa)
	xs := make([]float64, m+1)
	for j, id := range endIDs {
		xs[j] = x(id)
	}
	// Row end j gets a bottom point beneath it, unless a held vertex of the chain
	// lies within a quarter of the gap to its neighbouring row ends.
	var fresh []int
	for j := 1; j < m; j++ {
		if !(xs[j] > 0 && xs[j] < tp) {
			return [2]int{}, fmt.Errorf("%w: a cap column lies outside the end face's top edge", decaderr.ErrUnsupported)
		}
		gap := min(math.Abs(xs[j]-xs[j-1]), math.Abs(xs[j+1]-xs[j]))
		serves := false
		for _, v := range base {
			serves = serves || math.Abs(x(v)-xs[j]) <= gap/4
		}
		if serves {
			continue
		}
		psi := a.angleAbout(c.c, a.ends[i].exact) + c.dir*xs[j]/c.rG
		p := c.c.Add(a.u.Scale(c.rG * math.Cos(psi))).Add(a.w.Scale(c.rG * math.Sin(psi)))
		fresh = append(fresh, w.vertex(p, 0))
	}
	newPts := append(slices.Clone(fresh), pa)
	edges := map[int]*splitEdge{}
	for _, p := range newPts {
		xp := x(p)
		k := 0
		for ; k+1 < len(sx); k++ {
			if sx[k] < xp && xp < sx[k+1] {
				break
			}
		}
		if k+1 >= len(sx) {
			return [2]int{}, noCol
		}
		if edges[k] == nil {
			edges[k] = &splitEdge{u: seq[k], v: seq[k+1]}
		}
		edges[k].pts = append(edges[k].pts, p)
	}

	// Each held facet that keeps a chain edge with new points is fanned from its
	// apex through them, in order along the edge.
	split := map[int]bool{}
	for _, k := range slices.Sorted(maps.Keys(edges)) {
		e := edges[k]
		var hits []int
		for _, f := range g.facets {
			if t := w.fp.Tris[f]; slices.Contains(t[:], e.u) && slices.Contains(t[:], e.v) {
				hits = append(hits, f)
			}
		}
		if len(hits) != 1 || split[hits[0]] {
			return [2]int{}, fmt.Errorf("%w: an edge of the end face's top boundary does not border one facet",
				decaderr.ErrUnsupported)
		}
		split[hits[0]] = true
		if err := a.splitChainEdge(w, g, hits[0], e, slack); err != nil {
			return [2]int{}, err
		}
	}
	capFrom := len(w.out)

	// The cap: the bottom chain from V to P_A and the top chain from V up the wall's
	// cut, round the band's corner and down the rows' ends to P_A. Where the cut
	// passes vertices beside V that cylBandEnd read as on the radial plane
	// (n₁ … n_k), they lie on the cap's loop beside V, within noise of its bottom
	// edge. The closing triangles (n_{j+1}, n_j, b₁) join them to the bottom chain's
	// first vertex b₁, wound against the held facets that keep the path V, n₁ … n_k,
	// and the cap proper starts at n_k.
	bottom := append(append(slices.Clone(base), fresh...), pa)
	slices.SortFunc(bottom, func(p, q int) int { return cmpFloat(x(p), x(q)) })
	start := v0
	if len(c.noise) > 0 {
		start = c.noise[len(c.noise)-1]
	}
	top := []int{start}
	for _, v := range slices.Backward(wallRun) {
		top = append(top, v)
	}
	top = append(top, corner)
	for j := m - 1; j >= 1; j-- {
		top = append(top, endIDs[j])
	}
	top = append(top, pa)
	bottom[0] = start
	K, L := len(bottom)-1, len(top)-1
	if K < 2 || L < 2 {
		return [2]int{}, folds
	}
	for k := 1; k <= K; k++ {
		if !(x(bottom[k]) > x(bottom[k-1])) {
			return [2]int{}, noCol
		}
	}
	pt := func(id int) pt2 { return a.unrollG(i, w.verts[id]) }
	var ps []pt2
	var loop []int
	for _, id := range bottom {
		ps, loop = append(ps, pt(id)), append(loop, len(loop))
	}
	for k := L - 1; k >= 1; k-- {
		ps, loop = append(ps, pt(top[k])), append(loop, len(loop))
	}
	sgn := 1.0
	if signedArea2(ps, loop) < 0 {
		sgn = -1
	}
	positive := func(p, q, r int) bool {
		a2, b2, c2 := pt(p), pt(q), pt(r)
		scale := 0.0
		for _, s := range []pt2{a2, b2, c2} {
			scale = max(scale, math.Abs(s.u), math.Abs(s.w))
		}
		area := ((b2.u-a2.u)*(c2.w-a2.w) - (b2.w-a2.w)*(c2.u-a2.u)) * sgn
		return area > 32*ulp*scale*scale
	}
	// The noise vertices are the cap's own corners, and G's bound stands for them:
	// their true points lie on the rim, not on G, within the distance of the true V
	// from them, which the held distance and the two bounds cover.
	before := slices.Clone(w.beta)
	var closing [][3]int
	if len(c.noise) > 0 {
		if !slices.ContainsFunc(w.out, func(t [3]int) bool { return hasDirected(t, v0, c.noise[0]) }) &&
			!hasHeld(w, v0, c.noise[0]) {
			return [2]int{}, fmt.Errorf("%w: the wall's boundary beside a cylinder end does not run away from V",
				decaderr.ErrUnsupported)
		}
		seq := append([]int{v0}, c.noise...)
		for j := range len(seq) - 1 {
			closing = append(closing, [3]int{seq[j+1], seq[j], bottom[1]})
		}
		for _, n := range c.noise {
			eff := proofbound.AbsSumUpper(w.inputBeta(n), w.verts[n].Sub(w.verts[v0]).Len(), w.inputBeta(v0), slack)
			w.beta[n] = max(w.beta[n], eff)
			before[n] = max(before[n], eff)
		}
	}
	tris := [][3]int{{start, bottom[1], top[1]}}
	if !positive(tris[0][0], tris[0][1], tris[0][2]) {
		return [2]int{}, folds
	}
	bi, ti := 1, 1
	for bi < K-1 || ti < L-1 {
		type move struct {
			tri  [3]int
			isB  bool
			able bool
		}
		mb := move{[3]int{bottom[bi], bottom[min(bi+1, K)], top[ti]}, true, bi < K-1}
		mt := move{[3]int{bottom[bi], top[min(ti+1, L)], top[ti]}, false, ti < L-1}
		order := []move{mb, mt}
		if mb.able && mt.able && !(x(bottom[bi+1]) < x(top[ti+1])) {
			order = []move{mt, mb}
		}
		took := false
		for _, mv := range order {
			if !mv.able || !positive(mv.tri[0], mv.tri[1], mv.tri[2]) {
				continue
			}
			tris = append(tris, mv.tri)
			if mv.isB {
				bi++
			} else {
				ti++
			}
			took = true
			break
		}
		if !took {
			return [2]int{}, folds
		}
	}
	last := [3]int{bottom[K-1], pa, top[L-1]}
	if !positive(last[0], last[1], last[2]) {
		return [2]int{}, folds
	}
	tris = append(tris, last)

	// Each triangle is wound to G's outward normal at its centroid, and charged: a
	// triangle's points lie within the larger corner bound of the true points of G,
	// and the plane triangle lies within r_G·Δψ²/2 of the cylinder over the true
	// angles' span Δψ. The closing triangles are wound already.
	tilt := c.eps + c.eps*c.eps*c.eps
	charge := func(t [3]int) {
		va, vb, vc := w.verts[t[0]], w.verts[t[1]], w.verts[t[2]]
		beta, hi, zMax := 0.0, t[0], 0.0
		psi := [3]float64{}
		for k, v := range t {
			if before[v] > beta {
				beta, hi = before[v], v
			}
			psi[k] = wrapPi(a.angleAbout(c.c, w.verts[v]) - a.angleAbout(c.c, w.verts[t[0]]))
			zMax = max(zMax, math.Abs(a.height(w.verts[v])))
		}
		span := max(psi[0], psi[1], psi[2]) - min(psi[0], psi[1], psi[2])
		dpsi := proofbound.AbsSumUpper(span, 2*math.Asin(math.Min(1, beta/c.rG)),
			proofbound.DivUpper(proofbound.ProductUpper(2*tilt, zMax), c.rG))
		d := proofbound.AbsSumUpper(beta, proofbound.ProductUpper(c.rG/2, proofbound.ProductUpper(dpsi, dpsi)), slack)
		w.beta[hi] = max(w.beta[hi], d)
		w.capHeld = proofbound.AbsSumUpper(w.capHeld, vb.Sub(va).Cross(vc.Sub(va)).Len()/2)
	}
	for _, t := range closing {
		va, vb, vc := w.verts[t[0]], w.verts[t[1]], w.verts[t[2]]
		if !(vb.Sub(va).Cross(vc.Sub(va)).Len() > 2*proofbound.FacetAreaTermSlop(va, vb, vc)) {
			return [2]int{}, fmt.Errorf("%w: a cap triangle beside a cylinder end has no area", decaderr.ErrUnsupported)
		}
		charge(t)
		w.emit(t, g.group)
	}
	for _, t := range tris {
		va, vb, vc := w.verts[t[0]], w.verts[t[1]], w.verts[t[2]]
		centroid := va.Add(vb).Add(vc).Scale(1.0 / 3)
		if vb.Sub(va).Cross(vc.Sub(va)).Dot(a.outwardG(i, centroid)) < 0 {
			t[1], t[2] = t[2], t[1]
		}
		charge(t)
		w.emit(t, g.group)
	}
	for _, t := range w.out[capFrom:] {
		w.capEps = max(w.capEps, w.beta[t[0]], w.beta[t[1]], w.beta[t[2]])
	}
	return [2]int{capFrom, len(w.out)}, nil
}

// hasDirected reports whether facet t runs from x to y.
func hasDirected(t [3]int, x, y int) bool {
	for k := range 3 {
		if t[k] == x && t[(k+1)%3] == y {
			return true
		}
	}
	return false
}

// hasHeld reports whether a facet the work keeps of the held mesh runs from x to y.
func hasHeld(w *work, x, y int) bool {
	gone := map[int]bool{}
	for _, f := range w.removed {
		gone[f] = true
	}
	for f, t := range w.fp.Tris {
		if !gone[f] && hasDirected(t, x, y) {
			return true
		}
	}
	return false
}

// closeCap is the wall's cut run of end i, listed from the band's corner to V,
// without the vertices cylBandEnd read as on the radial plane beside V, which
// must be its last, and the vertex the cap's loop closes at: the last of them, or V.
func (a arcRim) closeCap(i int, run []int) ([]int, int, error) {
	noise := a.cyl[i].noise
	k := len(noise)
	if k == 0 {
		return run, a.ends[i].v, nil
	}
	bad := fmt.Errorf("%w: the wall's cut along an end face does not pass the arc's own vertices beside its end",
		decaderr.ErrUnsupported)
	if len(run) < k {
		return nil, 0, bad
	}
	for j, v := range noise {
		if run[len(run)-1-j] != v {
			return nil, 0, bad
		}
	}
	return run[:len(run)-k], noise[k-1], nil
}

func cmpFloat(a, b float64) int {
	switch {
	case a < b:
		return -1
	case a > b:
		return 1
	}
	return 0
}

// splitChainEdge fans the held facet f of G, which keeps the chain edge e.u–e.v,
// from its apex through e.pts. Each new point lies off the edge by at most its
// distance to the segment, which with the facet's own bound states its bound;
// every foot must fall inside the edge, clear of its ends, in order. A new facet
// must wind as f does, with an area above its rounding. The facets' volume
// difference from f is within the largest such distance of f's plane over f's
// and the fan's areas (Claim K), which the work adds to its volume bound.
func (a arcRim) splitChainEdge(w *work, g faceLoops, f int, e *splitEdge, slack float64) error {
	t := w.fp.Tris[f]
	k0 := 0
	for k := range 3 {
		if (t[k] == e.u && t[(k+1)%3] == e.v) || (t[k] == e.v && t[(k+1)%3] == e.u) {
			k0 = k
		}
	}
	first, second, apex := t[k0], t[(k0+1)%3], t[(k0+2)%3]
	pu, pv := w.verts[first], w.verts[second]
	edge := pv.Sub(pu)
	el := edge.Len()
	bad := fmt.Errorf("%w: a cap column falls on a vertex of the end face's top edge", decaderr.ErrUnsupported)
	type foot struct {
		id         int
		along, off float64
	}
	var feet []foot
	for _, p := range e.pts {
		q := w.verts[p]
		off := distToSegment(q, pu, pv)
		along := q.Sub(pu).Dot(edge) / el
		if !(along > off+slack) || !(el-along > off+slack) {
			return bad
		}
		feet = append(feet, foot{p, along, off})
	}
	slices.SortFunc(feet, func(p, q foot) int { return cmpFloat(p.along, q.along) })
	for k := 1; k < len(feet); k++ {
		if !(feet[k].along > feet[k-1].along+slack) {
			return bad
		}
	}
	nrm := pv.Sub(pu).Cross(w.verts[apex].Sub(pu))
	nu, ok := nrm.Normalize()
	if !ok || !(nrm.Len() > 2*proofbound.FacetAreaTermSlop(pu, pv, w.verts[apex])) {
		return fmt.Errorf("%w: an end face facet has no plane", decaderr.ErrUnsupported)
	}
	delta := w.facetBound(t)
	maxOff := 0.0
	for _, ft := range feet {
		maxOff = max(maxOff, ft.off)
		w.beta[ft.id] = max(w.beta[ft.id], proofbound.AbsSumUpper(delta, ft.off, slack))
	}
	ids := []int{first}
	for _, ft := range feet {
		ids = append(ids, ft.id)
	}
	ids = append(ids, second)
	area := proofbound.ProductUpper(0.5, nrm.Len())
	fan := 0.0
	for j := range len(ids) - 1 {
		p, q, r := w.verts[ids[j]], w.verts[ids[j+1]], w.verts[apex]
		cr := q.Sub(p).Cross(r.Sub(p))
		if !(cr.Dot(nu) > 2*proofbound.FacetAreaTermSlop(p, q, r)) {
			return fmt.Errorf("%w: a facet of the end face's top edge folds where a cap column splits it",
				decaderr.ErrUnsupported)
		}
		fan = proofbound.AbsSumUpper(fan, proofbound.ProductUpper(0.5, cr.Len()))
		w.emit([3]int{ids[j], ids[j+1], apex}, g.group)
	}
	w.removed = append(w.removed, f)
	w.splitVol = proofbound.AbsSumUpper(w.splitVol,
		proofbound.ProductUpper(maxOff, proofbound.AbsSumUpper(area, fan)))
	return nil
}

// linkNoise settles, in the cut along a cylinder end's radial plane, the vertices
// cylBandEnd read as on it beside V. A held edge between two of them, or between V
// and the first, that a facet of the band recorded as part of the cut is dropped
// first: the cut either reaches V by itself, and then none of them is part of it,
// or it ends at one of them, n_j, and then V, n₁ … n_j are joined by the held edges
// of the wall's boundary between them and the cap closes there (capFacets). The
// cut must otherwise reach one of them: nothing else is read.
func (a arcRim) linkNoise(cuts *bandCuts, kind [2]sideKind) error {
	for i := range 2 {
		c := a.cyl[i]
		if c == nil || kind[i] != extends || len(c.noise) == 0 {
			continue
		}
		g := cuts.runs[i]
		chain := append([]int{a.ends[i].v}, c.noise...)
		drop := func(x, y int) {
			g[x] = slices.DeleteFunc(g[x], func(v int) bool { return v == y })
			g[y] = slices.DeleteFunc(g[y], func(v int) bool { return v == x })
			for _, v := range []int{x, y} {
				if len(g[v]) == 0 {
					delete(g, v)
				}
			}
		}
		for k := range len(chain) - 1 {
			drop(chain[k], chain[k+1])
		}
		if len(g[chain[0]]) == 1 {
			c.noise = nil
			continue
		}
		end := 0
		for k := 1; k < len(chain); k++ {
			if len(g[chain[k]]) == 1 {
				end = k
				break
			}
		}
		if end == 0 {
			return fmt.Errorf("%w: the wall's cut along an end face does not reach the arc's end", decaderr.ErrUnsupported)
		}
		c.noise = c.noise[:end]
		for k := range end {
			g[chain[k]] = append(g[chain[k]], chain[k+1])
			g[chain[k+1]] = append(g[chain[k+1]], chain[k])
		}
	}
	return nil
}
