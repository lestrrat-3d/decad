package retainedblend

import (
	"fmt"
	"math"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// work is one rim's rewrite in progress: the vertices and facets it adds, and
// the facets of the old mesh it replaces.
type work struct {
	fp    facetedrecord.Payload
	r     rim
	spec  blendSpec
	verts []r3.Vec
	beta  []float64
	// out are the facets this rewrite emits, with their source groups.
	out    [][3]int
	outSrc []int
	// removed are the old facets the rewrite replaces.
	removed []int
	// removedTrue is the exact area of the surface the blend removes from the
	// faces it shrinks.
	removedTrue float64
	// tau is the chord tolerance of the blend's grid.
	tau float64
	// dir is +1 when the rim loop runs counter-clockwise about the plane's
	// outward normal, else -1.
	dir float64
	// cuts are the vertices a wall cut made, by the mesh edge it cut.
	cuts map[[2]int]int
	// raised is the largest bound rewriteWall gave a kept inner vertex of a
	// curved wall.
	raised float64
	// cutDelta holds, for each vertex a wall cut made, the facet bound of the
	// held facet it was cut from (noteCut).
	cutDelta map[int]float64
	// sphereWall is the sphere an R6 rim's wall lies on; nil for a wall the
	// cut plane meets square (cutOffset).
	sphereWall *sphereWall
	// share is the part of tau the plane polygon's step spends on a strip
	// triangle's azimuth term, and rowTau the bound the rows between the
	// plane's and the wall's state (§6.3, Strip triangles).
	share, rowTau float64
}

type ringPt struct {
	idx int
	ang float64
}

// newWork starts a rewrite. A straight-edge rewrite has no rim and passes a
// rim that holds only the scale.
func newWork(fp facetedrecord.Payload, r rim, spec blendSpec) *work {
	w := &work{fp: fp, r: r, spec: spec, cuts: map[[2]int]int{}}
	w.verts = append([]r3.Vec(nil), fp.Verts...)
	w.beta = append([]float64(nil), fp.VertexBound...)
	size := spec.r
	if !spec.fillet {
		size = max(spec.dA, spec.dB)
	}
	w.tau = max(math.Ldexp(size, -10), 64*ulp*r.scale)
	w.share, w.rowTau = azimuthShare(spec, w.tau), w.tau
	return w
}

func (w *work) vertex(p r3.Vec, bound float64) int {
	w.verts = append(w.verts, p)
	w.beta = append(w.beta, bound)
	return len(w.verts) - 1
}

func (w *work) emit(t [3]int, src int) {
	w.out = append(w.out, t)
	w.outSrc = append(w.outSrc, src)
}

// groupOf returns the one source group the facets belong to.
func (w *work) groupOf(facets []int) (int, error) {
	if len(facets) == 0 {
		return 0, fmt.Errorf("%w: a rim face has no facets", decaderr.ErrUnsupported)
	}
	g := w.fp.Src[facets[0]]
	for _, f := range facets {
		if w.fp.Src[f] != g {
			return 0, fmt.Errorf("%w: a rim face mixes source groups", decaderr.ErrUnsupported)
		}
	}
	return g, nil
}

// ringCount is the number of vertices a circle of radius rho needs for its
// chord sagitta to stay within tol.
func ringCount(rho, tol float64) int {
	if tol >= rho {
		return 8
	}
	step := 2 * math.Acos(1-tol/rho)
	return min(max(int(math.Ceil(2*math.Pi/step)), 8), 4096)
}

// clearance refuses a blend whose setback does not fit: the bounds say the
// blend consumes the face (SR4, ErrDegenerate), or cannot say it does not
// (SR5, ErrUnsupported). gap is the distance by which a held feature clears
// the removed zone, negative when it lies inside it, and margin is what the
// held bounds allow it to move.
func clearance(gap, margin float64, what string) error {
	switch {
	case gap > margin:
		return nil
	case gap < -margin:
		return fmt.Errorf("%w: the blend consumes %s", decaderr.ErrDegenerate, what)
	}
	return fmt.Errorf("%w: the held bounds cannot show that the blend leaves %s", decaderr.ErrUnsupported, what)
}

// rewritePlane replaces the plane face's facets: the rim loop gives way to a
// polygon on the plane's setback circle, at distance ta from the rim along the
// plane, and the face is triangulated again over its remaining boundary loops
// (§5 step 2, R2 and R6). A vertex no loop reaches is dropped with the old
// facets when it lies exactly on the plane g. The polygon's step keeps the
// azimuth term of a strip triangle within its share of tau on every row out
// to radius rhoHi, the largest of the blend's rows (§6.3).
func (w *work) rewritePlane(fi int, g plane, ta, rhoHi float64) ([]ringPt, int, error) {
	r := w.r
	facets := facetsOfFace(w.fp.FaceOf, fi)
	group, err := w.groupOf(facets)
	if err != nil {
		return nil, 0, err
	}
	loops, err := boundaryLoops(w.fp.Tris, facets)
	if err != nil {
		return nil, 0, err
	}
	slack := r.slack()
	rimAt := -1
	for li, loop := range loops {
		on := true
		for _, v := range loop {
			on = on && r.onRim(w.verts[v], w.beta[v])
		}
		if !on {
			continue
		}
		if rimAt >= 0 {
			return nil, 0, fmt.Errorf("%w: the plane face has two loops on the rim", decaderr.ErrUnsupported)
		}
		rimAt = li
	}
	if rimAt < 0 {
		return nil, 0, fmt.Errorf("%w: the plane face has no loop on the rim", decaderr.ErrUnsupported)
	}
	inLoop := map[int]bool{}
	for _, loop := range loops {
		for _, v := range loop {
			inLoop[v] = true
		}
	}
	for _, f := range facets {
		for _, v := range w.fp.Tris[f] {
			if !inLoop[v] && !g.holds(w.verts[v]) {
				return nil, 0, fmt.Errorf("%w: the plane face holds a vertex off its boundary", decaderr.ErrUnsupported)
			}
		}
	}

	rhoA := r.radius + r.kappa*ta
	lo, hi := min(r.radius, rhoA), max(r.radius, rhoA)
	for li, loop := range loops {
		if li == rimAt {
			continue
		}
		for k, v := range loop {
			q := loop[(k+1)%len(loop)]
			lowRho, highRho := w.segmentRadialRange(w.verts[v], w.verts[q])
			margin := max(w.beta[v], w.beta[q]) + slack + w.tau
			if highRho <= lo-margin || lowRho >= hi+margin {
				continue
			}
			// The segment's radial range meets the removed annulus. It is
			// inside it when the range reaches into the annulus past the margin.
			depth := min(highRho-lo, hi-lowRho)
			if err := clearance(-depth, margin, "the plane face"); err != nil {
				return nil, 0, err
			}
		}
	}

	rimLoop := loops[rimAt]
	w.dir = 1
	rimPoints := make([]pt2, len(rimLoop))
	rimIdx := make([]int, len(rimLoop))
	for i, v := range rimLoop {
		rimPoints[i] = w.plane2(w.verts[v])
		rimIdx[i] = i
	}
	if signedArea2(rimPoints, rimIdx) < 0 {
		w.dir = -1
	}

	n := ringCount(max(r.radius, rhoA, rhoHi), w.share)
	ring := make([]ringPt, n)
	ringBound := w.tau + slack
	for k := range n {
		ang := float64(k) * 2 * math.Pi / float64(n)
		ring[k] = ringPt{idx: w.vertex(r.at(rhoA, w.dir*ang, 0), ringBound), ang: ang}
	}

	cpoly := make([]int, n)
	for k := range n {
		cpoly[k] = ring[k].idx
	}
	var kept [][]int
	for li, loop := range loops {
		if li != rimAt {
			kept = append(kept, loop)
		}
	}
	if err := w.retriangulate(r.n, r.center, append(kept, cpoly), group); err != nil {
		return nil, 0, err
	}
	w.removed = append(w.removed, facets...)
	w.removedTrue += math.Pi * math.Abs(rhoA*rhoA-r.radius*r.radius)
	return ring, group, nil
}

// plane2 is p in the rim plane's (u, w) coordinates.
func (w *work) plane2(p r3.Vec) pt2 {
	q := p.Sub(w.r.center)
	return pt2{q.Dot(w.r.u), q.Dot(w.r.w)}
}

// retriangulate emits the triangulation of the planar face with outward
// normal n through origin over its boundary loops, which wind outward: the
// outer loop counter-clockwise seen from outside and the holes clockwise. Every
// loop vertex appears in the output.
func (w *work) retriangulate(n, origin r3.Vec, loops [][]int, group int) error {
	u, ok := perpendicular(n)
	if !ok {
		return fmt.Errorf("%w: a face has no plane frame", decaderr.ErrUnsupported)
	}
	v := n.Cross(u)
	var global []int
	local := map[int]int{}
	var ps []pt2
	use := func(vi int) int {
		if li, ok := local[vi]; ok {
			return li
		}
		local[vi] = len(global)
		global = append(global, vi)
		q := w.verts[vi].Sub(origin)
		ps = append(ps, pt2{q.Dot(u), q.Dot(v)})
		return local[vi]
	}
	var outer []int
	var holes [][]int
	for _, loop := range loops {
		poly := make([]int, len(loop))
		for i, vi := range loop {
			poly[i] = use(vi)
		}
		if signedArea2(ps, poly) > 0 {
			if outer != nil {
				return fmt.Errorf("%w: a face has two outer loops", decaderr.ErrUnsupported)
			}
			outer = poly
			continue
		}
		holes = append(holes, poly)
	}
	if outer == nil {
		return fmt.Errorf("%w: a face has no outer loop", decaderr.ErrUnsupported)
	}
	tris, err := triangulateRegion(ps, outer, holes)
	if err != nil {
		return err
	}
	for _, t := range tris {
		w.emit([3]int{global[t[0]], global[t[1]], global[t[2]]}, group)
	}
	return nil
}

// segmentRadialRange returns the smallest and largest distance from the rim
// axis over the segment pq.
func (w *work) segmentRadialRange(p, q r3.Vec) (float64, float64) {
	a, b := w.plane2(p), w.plane2(q)
	du, dw := b.u-a.u, b.w-a.w
	t := 0.0
	if d := du*du + dw*dw; d > 0 {
		t = math.Min(1, math.Max(0, -(a.u*du+a.w*dw)/d))
	}
	low := math.Hypot(a.u+t*du, a.w+t*dw)
	return low, math.Max(math.Hypot(a.u, a.w), math.Hypot(b.u, b.w))
}

// rewriteWall cuts the wall by the plane at depth tb below the rim's plane, on
// the wall's side, and keeps the part beyond it (§5 step 2, R2 and R6). The
// removed band is a zone of a cylinder or a sphere of radius bandRadius, whose
// exact area is 2π·bandRadius·tb either way. A cylinder's held facets have
// their vertices on the wall's boundary, so every vertex but the rim's must
// clear the band: a vertex in it is another face's corner the blend would
// meet; blendDist is nil. A sphere's facets also hold vertices inside the
// face, the tessellation of a doubly curved surface and no feature of it: with
// blendDist set, only the boundary loops' vertices must clear the band, and
// the cut removes an inner vertex with its facets or keeps it, by which side
// of the cut plane its held position lies on. A kept inner vertex within its
// bound of the cut plane may have its true point in the removed zone, which
// is no longer boundary; its bound grows to its distance from the blend face,
// blendDist, plus the rounding (§4.5). It returns the cut ring, sorted by
// angle.
func (w *work) rewriteWall(fi int, tb, bandRadius float64, blendDist func(r3.Vec) float64) ([]ringPt, int, error) {
	curved := blendDist != nil
	r := w.r
	facets := facetsOfFace(w.fp.FaceOf, fi)
	group, err := w.groupOf(facets)
	if err != nil {
		return nil, 0, err
	}
	slack := r.slack()
	var inLoop map[int]struct{}
	if curved {
		loops, err := boundaryLoops(w.fp.Tris, facets)
		if err != nil {
			return nil, 0, err
		}
		inLoop = map[int]struct{}{}
		for _, loop := range loops {
			for _, v := range loop {
				inLoop[v] = struct{}{}
			}
		}
	}
	rimVerts := 0
	g := map[int]float64{}
	for _, f := range facets {
		for _, v := range w.fp.Tris[f] {
			if _, seen := g[v]; seen {
				continue
			}
			p := w.verts[v]
			if r.onRim(p, w.beta[v]) {
				g[v] = -tb
				rimVerts++
				continue
			}
			gv := r.side*r.height(p) - tb
			if _, onLoop := inLoop[v]; curved && !onLoop {
				g[v] = gv
				if gv > 0 && gv <= w.beta[v]+slack {
					w.beta[v] = proofbound.AbsSumUpper(max(w.beta[v], blendDist(p)), slack)
					w.raised = max(w.raised, w.beta[v])
				}
				continue
			}
			if err := clearance(gv, w.beta[v]+slack+w.tau, "the wall"); err != nil {
				return nil, 0, err
			}
			g[v] = gv
		}
	}
	if rimVerts == 0 {
		return nil, 0, fmt.Errorf("%w: the wall has no vertex on the rim", decaderr.ErrUnsupported)
	}

	type key struct{ a, b int }
	cuts := map[key]int{}
	facetBound := 0.0
	cut := func(a, b int) int {
		k := key{min(a, b), max(a, b)}
		if c, ok := cuts[k]; ok {
			return c
		}
		t := g[a] / (g[a] - g[b])
		p := w.verts[a].Add(w.verts[b].Sub(w.verts[a]).Scale(t))
		c := w.vertex(p, max(w.beta[a], w.beta[b])+slack+w.tau)
		w.noteCut(c, facetBound)
		cuts[k] = c
		return c
	}
	for _, f := range facets {
		t := w.fp.Tris[f]
		facetBound = w.facetBound(t)
		var above []int
		for k, v := range t {
			if g[v] > 0 {
				above = append(above, k)
			}
		}
		switch len(above) {
		case 0:
		case 3:
			w.emit(t, group)
		case 1:
			x := above[0]
			a, b, c := t[x], t[(x+1)%3], t[(x+2)%3]
			w.emit([3]int{a, cut(a, b), cut(c, a)}, group)
		case 2:
			z := 0
			for k := range 3 {
				if g[t[k]] <= 0 {
					z = k
				}
			}
			x, y, below := t[(z+1)%3], t[(z+2)%3], t[z]
			cyz, czx := cut(y, below), cut(below, x)
			w.emit([3]int{x, y, cyz}, group)
			w.emit([3]int{x, cyz, czx}, group)
		}
	}
	w.removed = append(w.removed, facets...)
	w.removedTrue += 2 * math.Pi * bandRadius * tb

	ring := make([]ringPt, 0, len(cuts))
	seen := map[int]bool{}
	for _, c := range cuts {
		if seen[c] {
			continue
		}
		seen[c] = true
		ring = append(ring, ringPt{idx: c, ang: w.angleOf(w.verts[c])})
	}
	sort.Slice(ring, func(i, j int) bool { return ring[i].ang < ring[j].ang })
	if len(ring) < 3 {
		return nil, 0, fmt.Errorf("%w: the wall's cut leaves fewer than three vertices", decaderr.ErrUnsupported)
	}
	for i, p := range ring {
		next := ring[(i+1)%len(ring)].ang
		if i == len(ring)-1 {
			next += 2 * math.Pi
		}
		if next-p.ang >= math.Pi {
			return nil, 0, fmt.Errorf("%w: the wall's cut does not go round the axis", decaderr.ErrUnsupported)
		}
	}
	return ring, group, nil
}

// angleOf is the angle of p about the axis, measured in the rim loop's own
// sense from r.u, in [0, 2π).
func (w *work) angleOf(p r3.Vec) float64 {
	a := math.Mod(w.dir*w.r.angle(p), 2*math.Pi)
	if a < 0 {
		a += 2 * math.Pi
	}
	return a
}

// blendTag is the exact surface of the blend face and its orientation.
type blendTag struct {
	surface  surfacegeom.Surface
	reversed bool
	// outward returns the outward normal of the blend at p.
	outward func(p r3.Vec) r3.Vec
	// rows are the intermediate rows of the grid, from the plane ring toward
	// the wall ring; each maps an azimuth to a point.
	rows []func(phi float64) r3.Vec
	// trueArea is the exact area of the blend face.
	trueArea float64
	// rhoHi is the largest distance of a row from the axis, and lastSag the
	// profile's sagitta over the gap between the last row and the wall's cut
	// (§6.3, the band).
	rhoHi, lastSag float64
}

// tagGroup completes g, the group of a blend face, with the tag's surface. The
// blend is built from planes that may lie off their tags by up to w.r.charge
// (planeCharge), so its own surface may lie that far off the tag too. A plane
// states that as its AxialDelta. A curved surface has no field for a
// positional offset, so it is not published as a retained tag: the face stays
// faceted. With no charge the tag is published exactly as built.
func (w *work) tagGroup(g facetedrecord.Group, tag blendTag) facetedrecord.Group {
	if w.r.charge == 0 {
		g.Surface, g.Retained, g.Reversed = tag.surface, true, tag.reversed
		return g
	}
	if _, plane := tag.surface.(surfacegeom.Plane); !plane {
		return g
	}
	g.Surface, g.Retained, g.Reversed = tag.surface, true, tag.reversed
	g.AxialDelta, g.HasAxialDelta = w.r.charge, true
	return g
}

func (w *work) tag() blendTag {
	if w.spec.fillet {
		return w.torusTag()
	}
	return w.coneTag()
}

func (w *work) torusTag() blendTag {
	r, rr := w.r, w.spec.r
	rhoC := r.radius + r.kappa*rr
	zetaC := r.side * rr
	tag := blendTag{
		surface: surfacegeom.Torus{
			Center: r.center.Add(r.n.Scale(zetaC)), Axis: r.n,
			Major: units.Millimeters(rhoC), Minor: units.Millimeters(rr),
		},
		reversed: !r.convex,
		trueArea: math.Pi*math.Pi*rr*rhoC - 2*math.Pi*r.kappa*rr*rr,
	}
	tag.outward = func(p r3.Vec) r3.Vec {
		d := p.Sub(r.at(rhoC, r.angle(p), zetaC))
		if !r.convex {
			d = d.Scale(-1)
		}
		return d
	}
	step := rowStep(profileShare(w.tau), rr, 1)
	m := max(2, int(math.Ceil(math.Pi/2/step)))
	tag.rhoHi = max(r.radius, rhoC)
	tag.lastSag = filletSag(w.spec, math.Pi/2/float64(m))
	for j := 1; j < m; j++ {
		alpha := float64(j) * math.Pi / 2 / float64(m)
		tag.rows = append(tag.rows, func(phi float64) r3.Vec {
			return r.at(rhoC-r.kappa*rr*math.Sin(alpha), phi, r.side*rr*(1-math.Cos(alpha)))
		})
	}
	return tag
}

func (w *work) coneTag() blendTag {
	r := w.r
	rho1 := r.radius + r.kappa*w.spec.dA
	zeta2 := r.side * w.spec.dB
	dRho := rho1 - r.radius
	half := math.Atan(math.Abs(dRho) / w.spec.dB)
	cone := surfacegeom.Cone{HalfAngle: units.Radians(half)}
	if rho1 > r.radius {
		cone.Origin, cone.Axis, cone.Radius = r.center.Add(r.n.Scale(zeta2)), r.n.Scale(-r.side), units.Millimeters(r.radius)
	} else {
		cone.Origin, cone.Axis, cone.Radius = r.center, r.n.Scale(r.side), units.Millimeters(rho1)
	}
	// The blend's outward normal leans toward the bisector of the plane's and
	// the wall's outward normals, in the meridian plane.
	chordRho, chordZeta := r.radius-rho1, zeta2
	normRho, normZeta := -chordZeta, chordRho
	nBRho := 1.0
	if r.matOutside {
		nBRho = -1
	}
	if normRho*nBRho+normZeta < 0 {
		normRho, normZeta = -normRho, -normZeta
	}
	outward := func(p r3.Vec) r3.Vec {
		radial := p.Sub(r.center)
		radial = radial.Sub(r.n.Scale(radial.Dot(r.n)))
		unit, ok := radial.Normalize()
		if !ok {
			unit = r.u
		}
		return unit.Scale(normRho).Add(r.n.Scale(normZeta))
	}
	sample := outward(r.at(r.radius, 0, 0))
	natural := r.u.Scale(math.Cos(half)).Sub(cone.Axis.Scale(math.Sin(half)))
	length := math.Hypot(dRho, w.spec.dB)
	return blendTag{
		surface:  cone,
		reversed: sample.Dot(natural) < 0,
		outward:  outward,
		trueArea: math.Pi * (math.Abs(rho1) + r.radius) * length,
		rhoHi:    max(r.radius, rho1),
	}
}

// strip builds the blend face's grid between the plane ring and the wall ring
// and emits it under group.
func (w *work) strip(tag blendTag, ringA, ringB []ringPt, group int) {
	rowBound := w.rowTau + w.r.slack()
	from := len(w.out)
	rows := [][]ringPt{ringA}
	for _, at := range tag.rows {
		row := make([]ringPt, len(ringA))
		for k, p := range ringA {
			row[k] = ringPt{idx: w.vertex(at(w.dir*p.ang), rowBound), ang: p.ang}
		}
		rows = append(rows, row)
	}
	rows = append(rows, ringB)
	for i := 0; i+1 < len(rows); i++ {
		for _, t := range zipper(rows[i], rows[i+1]) {
			a, b, c := w.verts[t[0]], w.verts[t[1]], w.verts[t[2]]
			centroid := a.Add(b).Add(c).Scale(1.0 / 3)
			if b.Sub(a).Cross(c.Sub(a)).Dot(tag.outward(centroid)) < 0 {
				t[1], t[2] = t[2], t[1]
			}
			w.emit(t, group)
		}
	}
	w.chargeBand(from, bandAxis{o: w.r.center, n: w.r.n, ok: true}, tag.rhoHi, tag.lastSag, w.r.slack())
}

// zipper joins two closed rings, each sorted by angle, into a band of
// triangles by advancing whichever ring's next vertex has the smaller angle.
func zipper(a, b []ringPt) [][3]int {
	na, nb := len(a), len(b)
	angA := func(k int) float64 { return a[k%na].ang + 2*math.Pi*float64(k/na) }
	angB := func(k int) float64 { return b[k%nb].ang + 2*math.Pi*float64(k/nb) }
	j0 := 0
	for j0 < nb && angB(j0) < angA(0) {
		j0++
	}
	var out [][3]int
	i, j := 0, j0
	for i < na || j < j0+nb {
		if i < na && (j >= j0+nb || angA(i+1) <= angB(j+1)) {
			out = append(out, [3]int{a[i%na].idx, a[(i+1)%na].idx, b[j%nb].idx})
			i++
			continue
		}
		out = append(out, [3]int{a[i%na].idx, b[(j+1)%nb].idx, b[j%nb].idx})
		j++
	}
	return out
}
