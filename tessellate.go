package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the tessellation half of the export slice (core §11,
// docs/evaluator-design.md §9): per-surface analytic tessellators over the
// evaluator's own prism and cup payloads, with a proven per-facet deviation
// bound and facets that remember their source face. A boolean-faceted body
// restates its held mesh here; the exact-predicate mesh boolean that produces
// it lives in boolean_mesh.go.

// Mesh is a triangle mesh: an OUTPUT of [Body.Tessellate], never the body
// representation (core §3 invariant #1 — the public vocabulary stays
// Body → Face → Edge → Vertex). Vertices lie exactly on the body's analytic
// boundary; the facets between them deviate from it by no more than Bound,
// the proven chord error. A Mesh is immutable: the accessors return copies of
// its slices.
type Mesh struct {
	vertices  []r3.Vec
	triangles [][3]int
	source    []*Face
	bound     float64 // millimetres
	// areaSlack is a proven bound (mm²) on how far the mesh's total facet
	// area falls short of — or, over a hole, overshoots — the body's true
	// boundary area: the chord-versus-arc deficit, closed form per walk.
	// The mesh boolean's area bounds compose from it.
	areaSlack float64
}

// Vertices returns the mesh vertex positions in millimetres (core §5.2).
// Every vertex lies exactly on the body's analytic boundary — the chord error
// lives between the samples, not at them.
func (m *Mesh) Vertices() []r3.Vec { return append([]r3.Vec(nil), m.vertices...) }

// Triangles returns the facets as index triples into Vertices, wound
// counter-clockwise seen from outside the body. The indices describe the
// mesh's own connectivity; they are not selectors (core §3 invariant #3).
func (m *Mesh) Triangles() [][3]int { return append([][3]int(nil), m.triangles...) }

// SourceFaces returns, parallel to Triangles, the analytic face each facet
// approximates — the provenance the mesh boolean groups its output by
// (docs/evaluator-design.md §9). The elements are the body's live faces.
func (m *Mesh) SourceFaces() []*Face { return append([]*Face(nil), m.source...) }

// Bound returns the proven deviation bound: no point of the body's boundary
// lies farther than this from the mesh, and vice versa. It is the largest
// chord sagitta the tessellation actually took — at most the requested
// tolerance, and zero for a body whose boundary is all planes and straight
// edges, which triangulates exactly. A scalar quantity is a units.Value
// (core §5.1): Kind Length, millimetres.
func (m *Mesh) Bound() units.Value { return units.Millimeters(m.bound) }

// Tessellate approximates the body's boundary as a triangle mesh whose facets
// deviate from the analytic faces by no more than tol, the chord tolerance —
// an OUTPUT, not the representation (core §11). tol is a magnitude: Kind
// Length ([ErrUnitKind] otherwise), finite ([ErrNotFinite]), non-negative
// ([ErrNegativeMagnitude]); a zero tolerance asks for a chord that is the
// curve and is [ErrDegenerate].
//
// Planar faces triangulate exactly. Circular boundaries are chorded at
// parameter samples chosen once per boundary curve and shared by every face
// that meets it — a cap and the cylinder wall use the same chording of their
// shared edge — so the mesh is watertight and consistently oriented by
// construction, and Bound carries the largest sagitta actually taken.
//
// Prism, cup and boolean-built bodies tessellate. A boolean-built body only
// RESTATES its held mesh, so tol must be at least the body's own Bound: a
// finer tol is [ErrUnsupported], because the analytic identity is gone and no
// finer mesh can be proven. A revolve body is [ErrUnsupported] here — Revolve
// builds and verifies, but its analytic surfaces have no tessellator, so it
// cannot be meshed or exported. A body this evaluator did not build at all is
// also [ErrUnsupported].
func (b *Body) Tessellate(tol units.Value) (*Mesh, error) {
	return tessellateContext(context.Background(), b, tol)
}

// tessellateContext is the read-only evaluator's cancellable tessellation
// entry. It builds only an unowned Mesh and never touches document state.
func tessellateContext(ctx context.Context, b *Body, tol units.Value) (*Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	chord, err := magnitudeIn(tol, units.Length, units.Millimeter, "the chord tolerance")
	if err != nil {
		return nil, err
	}
	if chord == 0 {
		return nil, fmt.Errorf(`%w: a zero chord tolerance admits no chord`, ErrDegenerate)
	}
	if b.payload == nil {
		return nil, fmt.Errorf(`%w: this evaluator cannot tessellate a body it did not build`, ErrUnsupported)
	}
	if fp, ok := b.payload.(facetedPayload); ok {
		return tessellateFaceted(ctx, b, fp, chord)
	}
	if cp, ok := b.payload.(cupPayload); ok {
		return tessellateCup(ctx, b, cp, chord)
	}
	pp, ok := b.payload.(prismPayload)
	if !ok {
		// Chording is per payload kind; the revolve surfaces are staged —
		// explicit, never a wrong mesh (docs/evaluator-design.md §11).
		return nil, fmt.Errorf(`%w: this evaluator tessellates prism bodies only`, ErrUnsupported)
	}

	// Facets remember their source face (docs/evaluator-design.md §9); the
	// provenance roles are how the payload's walks name the faces evalPrism
	// built from them.
	byRole := map[string]*Face{}
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			byRole[o.Role] = f
		}
	}
	faceOfRole := func(role string) (*Face, error) {
		f, ok := byRole[role]
		if !ok {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, ErrDegenerate, role)
		}
		return f, nil
	}
	capStart, err := faceOfRole(roleCapStart)
	if err != nil {
		return nil, err
	}
	capEnd, err := faceOfRole(roleCapEnd)
	if err != nil {
		return nil, err
	}

	// One boundary polyline per loop: sample j is walk j's own start (the
	// junction shared with the previous walk) plus, for a circular walk, its
	// interior chord samples. Each 2D sample owns one bottom and one top mesh
	// vertex, so every face that meets a boundary curve reuses the SAME
	// chording — watertightness by construction.
	var mesh Mesh
	var pts2 []Point2
	var loopIdx [][]int
	var loopSag []float64
	loops := append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...)
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		samples, faceOf, maxSag, slack, err := chordLoop(ctx, loop, chord, pp.z1-pp.z0, func(w sideWalk) (*Face, error) {
			return faceOfRole(fmt.Sprintf("side(%d,%d)", li, w.segs[0]))
		})
		if err != nil {
			return nil, err
		}
		mesh.bound = math.Max(mesh.bound, maxSag)
		mesh.areaSlack += slack

		base := len(pts2)
		pts2 = append(pts2, samples...)
		idx := make([]int, len(samples))
		for j := range samples {
			idx[j] = base + j
		}
		loopIdx = append(loopIdx, idx)
		loopSag = append(loopSag, maxSag)

		// Side walls: one quad per chord, split into two triangles wound
		// outward (tangent × N is the outward side normal for a CCW outer
		// walk and a CW hole walk alike).
		for j := range samples {
			g0 := base + j
			g1 := base + (j+1)%len(samples)
			mesh.addTriangle([3]int{meshBottom(g0), meshBottom(g1), meshTop(g1)}, faceOf[j])
			mesh.addTriangle([3]int{meshBottom(g0), meshTop(g1), meshTop(g0)}, faceOf[j])
		}
	}

	// The mesh vertices: bottom and top of every boundary sample, placed
	// through the payload — exactly on the analytic boundary.
	mesh.vertices = make([]r3.Vec, 0, 2*len(pts2))
	vertexBudget := newWorkBudget(ctx)
	for _, p := range pts2 {
		if err := vertexBudget.step(); err != nil {
			return nil, err
		}
		mesh.vertices = append(mesh.vertices, pp.point(p.U, p.V, pp.z0), pp.point(p.U, p.V, pp.z1))
	}

	// Loops that touch — a hole tangent to the outline or to another hole —
	// pinch the cap region, and a mesh at this tolerance cannot prove it
	// represents that topology: the chorded loops must clear each other by
	// more than their own sagitta bounds, or the tessellation refuses
	// (an error, never a pinched or cracked mesh).
	if err := requireLoopClearance(ctx, pts2, loopIdx, loopSag); err != nil {
		return nil, err
	}

	// Caps: both share one 2D triangulation of the chorded region — the same
	// non-convex, hole-carrying polygon — mapped to the top vertices as-is
	// (outward +N) and to the bottom vertices reversed (outward −N).
	capTris, err := triangulate2DContext(ctx, pts2, loopIdx)
	if err != nil {
		return nil, err
	}
	for _, tri := range capTris {
		mesh.addTriangle([3]int{meshTop(tri[0]), meshTop(tri[1]), meshTop(tri[2])}, capEnd)
		mesh.addTriangle([3]int{meshBottom(tri[0]), meshBottom(tri[2]), meshBottom(tri[1])}, capStart)
	}

	// A reflected placement flips handedness, turning every counter-clockwise
	// winding clockwise; reversing the windings restores outward orientation.
	if pp.reflected() {
		for i := range mesh.triangles {
			mesh.triangles[i][1], mesh.triangles[i][2] = mesh.triangles[i][2], mesh.triangles[i][1]
		}
	}
	return &mesh, nil
}

// chordLoop chords one boundary loop into 2D samples: sample j is walk j's own
// start (the junction shared with the previous walk) plus, for a circular walk,
// its interior chord samples. It returns the samples, the wall face of the
// chord LEAVING each sample (resolved by wallFace over the coalesced walk it
// belongs to), the largest sagitta the chording took, and the chord-versus-arc
// area slack over the given sweep height. The same chording feeds every face
// that meets the loop — walls and caps alike — so the mesh is watertight by
// construction.
func chordLoop(ctx context.Context, loop LoopRecord, chord, height float64, wallFace func(w sideWalk) (*Face, error)) ([]Point2, []*Face, float64, float64, error) {
	if len(loop.Segments) == 0 {
		return nil, nil, 0, 0, fmt.Errorf(`%w: a recorded loop holds no segments`, ErrDegenerate)
	}
	// One counter spans the segment walk, the walk loop and the sample emission
	// nested under it: a single walk emits many samples, and it is the SAMPLES
	// that are the candidate operations §7.2 counts.
	budget := newWorkBudget(ctx)
	raw := make([]sideWalk, len(loop.Segments))
	for i, seg := range loop.Segments {
		if err := budget.step(); err != nil {
			return nil, nil, 0, 0, err
		}
		w, err := walkOf(seg)
		if err != nil {
			return nil, nil, 0, 0, err
		}
		raw[i] = sideWalk{segmentWalk: w, segs: []int{i}}
	}
	walks := coalesceWalks(raw)

	var samples []Point2
	var faceOf []*Face
	var maxSag, areaSlack float64
	for _, w := range walks {
		if err := budget.step(); err != nil {
			return nil, nil, 0, 0, err
		}
		face, err := wallFace(w)
		if err != nil {
			return nil, nil, 0, 0, err
		}
		if !w.circular {
			samples = append(samples, Point2{U: w.startU, V: w.startV})
			faceOf = append(faceOf, face)
			continue
		}
		n, sag, err := chordCount(w.segmentWalk, chord)
		if err != nil {
			return nil, nil, 0, 0, err
		}
		maxSag = math.Max(maxSag, sag)
		areaSlack += walkAreaSlack(w.segmentWalk, n, height)
		dth := (w.th1 - w.th0) / float64(n)
		for k := range n {
			if err := budget.step(); err != nil {
				return nil, nil, 0, 0, err
			}
			p := Point2{U: w.startU, V: w.startV}
			if k > 0 {
				th := w.th0 + float64(k)*dth
				p = Point2{U: w.cU + w.radius*math.Cos(th), V: w.cV + w.radius*math.Sin(th)}
			}
			samples = append(samples, p)
			faceOf = append(faceOf, face)
		}
	}
	return samples, faceOf, maxSag, areaSlack, nil
}

// tessellateCup meshes a cup (docs/modify-design.md §9, D4): the outer region O
// and the cavity region C, each with k ≥ 0 holes, sharing one rim per region
// loop at the open end. Every region loop is chorded ONCE — the same chording
// feeds its walls, its floor cap and its half of the rim band — so every ring
// vertex is shared and the mesh is watertight and consistently oriented by
// construction, exactly as the prism's is. A hole of O is a tunnel through the
// whole body; a hole of C is a solid POST rising from the floor. The planar
// faces (the kept cap over O, the pocket floor over C, and one rim band per
// loop) triangulate through the shipped cap triangulator.
func tessellateCup(ctx context.Context, b *Body, cp cupPayload, chord float64) (*Mesh, error) {
	byRole := map[string]*Face{}
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			byRole[o.Role] = f
		}
	}
	faceOfRole := func(role string) (*Face, error) {
		f, ok := byRole[role]
		if !ok {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, ErrDegenerate, role)
		}
		return f, nil
	}
	capStart, err := faceOfRole(roleCapStart)
	if err != nil {
		return nil, err
	}
	shellCap, err := faceOfRole("shellCap")
	if err != nil {
		return nil, err
	}

	oLoops := append([]LoopRecord{cp.outer.Outer}, cp.outer.Holes...)
	cLoops := append([]LoopRecord{cp.cavity.Outer}, cp.cavity.Holes...)
	if len(oLoops) != len(cLoops) {
		return nil, fmt.Errorf(`%w: the cup's outer and cavity regions have different loop counts`, ErrDegenerate)
	}

	openIsMax := cp.zOpen > cp.zOuter
	oLo, oHi := math.Min(cp.zOuter, cp.zOpen), math.Max(cp.zOuter, cp.zOpen)
	cLo, cHi := math.Min(cp.zCav, cp.zOpen), math.Max(cp.zCav, cp.zOpen)

	var mesh Mesh
	base := cp.prismLike(0, 0)
	var verts []r3.Vec
	add := func(v r3.Vec) int { verts = append(verts, v); return len(verts) - 1 }

	// One chorded ring per region loop: the walls hang off it and the caps and
	// rims share it, so every vertex is shared and the mesh closes by
	// construction. A ring holds its samples, the lo/hi mesh vertices of each,
	// its wall faces and the sagitta the chording took.
	type ring struct {
		samples []Point2
		loV     []int
		hiV     []int
		faces   []*Face
		sag     float64
	}
	chordRing := func(loop LoopRecord, h, lo, hi float64, role string) (ring, error) {
		samples, faceOf, sag, slack, err := chordLoop(ctx, loop, chord, h, func(w sideWalk) (*Face, error) {
			return faceOfRole(fmt.Sprintf(role, w.segs[0]))
		})
		if err != nil {
			return ring{}, err
		}
		mesh.bound = math.Max(mesh.bound, sag)
		mesh.areaSlack += slack
		r := ring{samples: samples, faces: faceOf, sag: sag}
		r.loV = make([]int, len(samples))
		r.hiV = make([]int, len(samples))
		for i, p := range samples {
			r.loV[i] = add(base.point(p.U, p.V, lo))
			r.hiV[i] = add(base.point(p.U, p.V, hi))
		}
		return r, nil
	}

	// Outer region O: every loop in its natural sense (outer counter-clockwise,
	// holes clockwise), role side(i,j).
	oRings := make([]ring, len(oLoops))
	for i, loop := range oLoops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		oRings[i], err = chordRing(loop, oHi-oLo, oLo, oHi, fmt.Sprintf("side(%d,%%d)", i))
		if err != nil {
			return nil, err
		}
	}

	// Cavity region C: every loop REVERSED (its material lies outside it), so
	// the reversed outer walks clockwise and each reversed hole counter-
	// clockwise (a post), role shellSide(i,j).
	cRings := make([]ring, len(cLoops))
	for i, loop := range cLoops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		rev, err := reverseLoopRecord(loop)
		if err != nil {
			return nil, err
		}
		cRings[i], err = chordRing(rev, cHi-cLo, cLo, cHi, fmt.Sprintf("shellSide(%d,%%d)", i))
		if err != nil {
			return nil, err
		}
	}
	mesh.vertices = verts

	// Side walls: one outward quad per chord — the same winding the prism uses,
	// tangent × N outward for a counter-clockwise walk (O's outer, a post) and a
	// clockwise walk (an O tunnel, C's reversed outer) alike.
	addWalls := func(r ring) {
		n := len(r.loV)
		for j := range r.loV {
			g0, g1 := j, (j+1)%n
			mesh.addTriangle([3]int{r.loV[g0], r.loV[g1], r.hiV[g1]}, r.faces[j])
			mesh.addTriangle([3]int{r.loV[g0], r.hiV[g1], r.hiV[g0]}, r.faces[j])
		}
	}
	for i := range oRings {
		addWalls(oRings[i])
	}
	for i := range cRings {
		addWalls(cRings[i])
	}

	// Every region loop must clear every other by more than their combined
	// chord bounds, or a cap or rim band cannot prove it represents that
	// topology — the same proof the prism's own loop clearance runs, now over
	// all O and C loops (nested rings, tunnels and posts included).
	var clrPts []Point2
	var clrIdx [][]int
	var clrSag []float64
	addClr := func(r ring) {
		base0 := len(clrPts)
		clrPts = append(clrPts, r.samples...)
		idx := make([]int, len(r.samples))
		for k := range r.samples {
			idx[k] = base0 + k
		}
		clrIdx = append(clrIdx, idx)
		clrSag = append(clrSag, r.sag)
	}
	for i := range oRings {
		addClr(oRings[i])
	}
	for i := range cRings {
		addClr(cRings[i])
	}
	if err := requireLoopClearance(ctx, clrPts, clrIdx, clrSag); err != nil {
		return nil, err
	}

	// The vertex each ring contributes to the open end (its rim) and to its own
	// floor cap — whichever ring, lo or hi, that plane falls on.
	openV := func(r ring) []int {
		if openIsMax {
			return r.hiV
		}
		return r.loV
	}
	floorV := func(r ring) []int {
		if openIsMax {
			return r.loV
		}
		return r.hiV
	}

	// A cap whose outward normal is −N reverses the counter-clockwise cap
	// triangulation; +N uses it as-is (the same rule the prism's two caps use).
	emit := func(tris [][3]int, vtx []int, reverse bool, face *Face) {
		for _, tri := range tris {
			a, bb, c := vtx[tri[0]], vtx[tri[1]], vtx[tri[2]]
			if reverse {
				bb, c = c, bb
			}
			mesh.addTriangle([3]int{a, bb, c}, face)
		}
	}

	// triRegion triangulates a polygon-with-holes: each loop's samples go into
	// one pts array (aligned to its floor/open vertex ring), and reverse[i]
	// flips a loop's index order to the sense triangulate2D wants — loops[0]
	// counter-clockwise, holes clockwise.
	triRegion := func(samples [][]Point2, vtxRings [][]int, reverse []bool) ([][3]int, []int, error) {
		var pts []Point2
		var vtx []int
		var loops [][]int
		for i := range samples {
			base0 := len(pts)
			pts = append(pts, samples[i]...)
			vtx = append(vtx, vtxRings[i]...)
			idx := make([]int, len(samples[i]))
			for k := range samples[i] {
				idx[k] = base0 + k
			}
			if reverse[i] {
				for a, z := 0, len(idx)-1; a < z; a, z = a+1, z-1 {
					idx[a], idx[z] = idx[z], idx[a]
				}
			}
			loops = append(loops, idx)
		}
		tris, err := triangulate2DContext(ctx, pts, loops)
		return tris, vtx, err
	}

	// capStart over O — the kept outer floor, outward −N when the open end is on
	// top. O's outer is already counter-clockwise and its holes (tunnels)
	// clockwise, so no loop needs reversing.
	oSamples := make([][]Point2, len(oRings))
	oFloor := make([][]int, len(oRings))
	oReverse := make([]bool, len(oRings))
	for i := range oRings {
		oSamples[i] = oRings[i].samples
		oFloor[i] = floorV(oRings[i])
	}
	capTris, capVtx, err := triRegion(oSamples, oFloor, oReverse)
	if err != nil {
		return nil, err
	}
	emit(capTris, capVtx, openIsMax, capStart)

	// shellCap over C — the pocket floor, outward +N when the open end is on
	// top. C's samples are the reversed loops (outer clockwise, holes/posts
	// counter-clockwise), so reversing every loop's order restores the cap's
	// own sense: outer counter-clockwise, each post a clockwise hole.
	cSamples := make([][]Point2, len(cRings))
	cCav := make([][]int, len(cRings))
	cReverse := make([]bool, len(cRings))
	for i := range cRings {
		cSamples[i] = cRings[i].samples
		cCav[i] = floorV(cRings[i])
		cReverse[i] = true
	}
	shellTris, shellVtx, err := triRegion(cSamples, cCav, cReverse)
	if err != nil {
		return nil, err
	}
	emit(shellTris, shellVtx, !openIsMax, shellCap)

	// Rims: one band per region loop at the open end. rim(0) spans O's outer
	// (counter-clockwise) and C's reversed outer (clockwise hole); a post rim
	// rim(i≥1) spans C's reversed hole (counter-clockwise, the wider post
	// opening) and O's tunnel (clockwise hole) — the outer/hole roles swap, as
	// evalCup's own build does.
	for i := range oLoops {
		rim, err := faceOfRole(fmt.Sprintf("rim(%d)", i))
		if err != nil {
			return nil, err
		}
		var samples [][]Point2
		var vtxRings [][]int
		if i == 0 {
			samples = [][]Point2{oRings[0].samples, cRings[0].samples}
			vtxRings = [][]int{openV(oRings[0]), openV(cRings[0])}
		} else {
			samples = [][]Point2{cRings[i].samples, oRings[i].samples}
			vtxRings = [][]int{openV(cRings[i]), openV(oRings[i])}
		}
		rimTris, rimVtx, err := triRegion(samples, vtxRings, []bool{false, false})
		if err != nil {
			return nil, err
		}
		emit(rimTris, rimVtx, !openIsMax, rim)
	}

	// A reflected placement flips handedness, turning every counter-clockwise
	// winding clockwise; reversing the windings restores outward orientation.
	if base.reflected() {
		for i := range mesh.triangles {
			mesh.triangles[i][1], mesh.triangles[i][2] = mesh.triangles[i][2], mesh.triangles[i][1]
		}
	}

	// The cup's planar faces triangulate through the shipped cap triangulator,
	// which resolves every rim band — including the bridge-collinear one an
	// outward cup or a rectangular post produces, where a rounded outer loop's
	// corner tangent points land on the sharp inner loop's edge lines. The
	// walls and floors close by construction; this proves the assembled mesh is
	// watertight and refuses a cracked one rather than return it, a safety net
	// against any residual chording pathology (core §11, never a wrong mesh).
	if err := requireClosedMesh(&mesh); err != nil {
		return nil, err
	}
	return &mesh, nil
}

// requireClosedMesh proves the mesh is a closed 2-manifold — every directed
// edge is matched by its reverse — refusing a mesh the cap triangulator could
// not close into a watertight band.
func requireClosedMesh(m *Mesh) error {
	directed := make(map[[2]int]int, 3*len(m.triangles))
	for _, tri := range m.triangles {
		for k := range 3 {
			directed[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	for e := range directed {
		if directed[e] != 1 || directed[[2]int{e[1], e[0]}] != 1 {
			return fmt.Errorf(`%w: the chorded cup rim could not be triangulated into a watertight band`, ErrDegenerate)
		}
	}
	return nil
}

// walkAreaSlack is the proven chord-versus-arc area slack one circular walk
// contributes: the wall loses (arc − chord) × height, and each cap gains or
// loses the circular segments between arc and chords — both closed form, both
// padded a hair so float rounding never understates them.
func walkAreaSlack(w segmentWalk, n int, h float64) float64 {
	sweep := math.Abs(w.th1 - w.th0)
	arc := sweep * w.radius
	chord := float64(n) * 2 * w.radius * math.Sin(sweep/(2*float64(n)))
	wall := math.Max(arc-chord, 0) * h
	segs := w.radius * w.radius / 2 * math.Max(sweep-float64(n)*math.Sin(sweep/float64(n)), 0)
	return (wall + 2*segs) * (1 + 1e-9)
}

// tessellateFaceted restates a boolean-built body's held mesh: the polygons
// ARE the boundary this evaluator holds (core §6.1), carrying their own
// proven bound. It cannot refine them — the analytic identity is gone — so a
// tolerance finer than the held bound is ErrUnsupported, never a mesh whose
// bound overstates its trust.
func tessellateFaceted(ctx context.Context, b *Body, fp facetedPayload, chord float64) (*Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if chord < fp.meshBound {
		requested := units.Millimeters(chord)
		minimum := units.Millimeters(fp.meshBound)
		return nil, fmt.Errorf(`%w: requested tolerance %s is below the faceted body's minimum mesh bound %s; retry with a tolerance of at least %s to restate the held mesh`, ErrUnsupported, requested, minimum, minimum)
	}
	faces := b.Faces()
	src := make([]*Face, len(fp.tris))
	budget := newWorkBudget(ctx)
	for i, fi := range fp.faceOf {
		if err := budget.step(); err != nil {
			return nil, err
		}
		if fi < 0 || fi >= len(faces) {
			// An inconsistent source mapping is a broken evaluator, not a
			// staged capability: §7.1 names it an invariant failure, which
			// Verify must return rather than hide as Suspect. ErrUnsupported
			// here would be swallowed into an undecided pair by
			// evaluateBoolean's staging branch.
			return nil, fmt.Errorf(`%w: a facet maps to no face`, ErrBooleanFailed)
		}
		src[i] = faces[fi]
	}
	return &Mesh{
		vertices:  fp.verts,
		triangles: fp.tris,
		source:    src,
		bound:     fp.meshBound,
		areaSlack: fp.areaSlack,
	}, nil
}

// meshBottom and meshTop are the mesh vertex indices of 2D boundary sample g:
// the vertices interleave bottom, top per sample.
func meshBottom(g int) int { return 2 * g }
func meshTop(g int) int    { return 2*g + 1 }

// addTriangle appends one facet and its source face.
func (m *Mesh) addTriangle(tri [3]int, src *Face) {
	m.triangles = append(m.triangles, tri)
	m.source = append(m.source, src)
}

// chordCount picks the number of chords for a circular walk so each chord's
// sagitta r·(1 − cos(Δθ/2)) stays within tol, and returns the sagitta the
// choice proves. A closed walk needs at least three chords to bound a
// polygon; the per-chord angle never exceeds π, so consecutive samples are
// always distinct.
func chordCount(w segmentWalk, tol float64) (int, float64, error) {
	sweep := math.Abs(w.th1 - w.th0)
	maxD := math.Pi
	if tol < w.radius {
		// sagitta(Δθ) = 2r·sin²(Δθ/4), so Δθ_max = 4·asin(√(tol/2r)) — the
		// stable inverse: acos(1 − tol/r) rounds to zero for tiny tol and
		// would seed an unbounded walk-up.
		maxD = 4 * math.Asin(math.Sqrt(tol/(2*w.radius)))
	}
	// A tolerance this fine asks for more chords than any mesh can hold;
	// the intent cannot be built, so it is refused outright (evaluator §2).
	// The cap is enforced again after ceil and inside the walk-up: rounding
	// can push n past a precheck that barely admitted it.
	if maxD == 0 || sweep/maxD > maxChordsPerWalk {
		return 0, 0, errTooManyChords
	}
	n := max(int(math.Ceil(sweep/maxD)), 1)
	if w.closed && n < 3 {
		n = 3
	}
	if n > maxChordsPerWalk {
		return 0, 0, errTooManyChords
	}
	// The returned sagitta is a PROVEN bound, so it may never exceed the
	// asked tolerance: float rounding in the asin/ceil path can land one
	// chord short at a threshold value. Each increment strictly shrinks the
	// sagitta toward zero, so the walk-up terminates.
	// 2r·sin²(Δθ/4) is the same sagitta as r·(1 − cos(Δθ/2)) but stays
	// accurate where 1 − cos rounds a tiny positive sagitta to zero — the
	// bound is PROVEN, so it may be conservative but never understated.
	sagitta := func(n int) float64 {
		s := math.Sin(sweep / float64(n) / 4)
		return 2 * w.radius * s * s
	}
	s := sagitta(n)
	for s > tol && sweep > 0 {
		if n == maxChordsPerWalk {
			return 0, 0, errTooManyChords
		}
		n++
		s = sagitta(n)
	}
	return n, s, nil
}

// errTooManyChords refuses a chord tolerance finer than the mesh cap.
var errTooManyChords = fmt.Errorf(`%w: the chord tolerance asks for more than %d chords on one curve`, ErrUnsupported, maxChordsPerWalk)

// maxChordsPerWalk caps how finely one boundary curve may be chorded. The
// ceiling is set by the cap triangulator, whose ear clipping is quadratic in
// the boundary samples: 2¹⁴ chords keeps the worst cap under a second while
// still admitting sub-micrometre tolerances on any real part (a 10 mm-radius
// circle at the cap carries a sagitta under 2e-7 mm).
//
// The cap is per-curve, and there is deliberately no total across a profile's
// curves. It bounds what ONE curve can ask of the quadratic cap triangulator,
// which is why errTooManyChords reports "more than %d chords on one curve". A
// profile with many loops is proportionally more work because the caller
// modelled proportionally more geometry, and docs/interference-design.md §7
// answers large work with cancellation rather than a work cap: the read-only
// path polls its context at least once per workPollInterval candidate
// operations, so chordLoop, bridgeHole and earClip all abandon a large profile
// promptly. A total cap would instead refuse a profile this evaluator can
// build correctly.
const maxChordsPerWalk = 1 << 14

// requireLoopClearance rejects a profile whose chorded loops come within
// their combined chord bounds of one another. Each chorded loop lies within
// its own sagitta of the true curve, so a polyline clearance beyond the two
// bounds PROVES the true loops are disjoint; anything closer — a hole
// tangent to the outline, or two holes touching — is a pinch this mesh
// cannot prove it represents, and is refused.
func requireLoopClearance(ctx context.Context, pts []Point2, loopIdx [][]int, loopSag []float64) error {
	// Round-off floor, in two translation-honest parts: 1e-9 of the
	// geometry's own span (coordinates carry ~1e-9-relative noise at their
	// own scale, and a span is translation-invariant), plus a few ulps of
	// the largest coordinate magnitude — the arithmetic's actual rounding,
	// which grows with distance from the origin without ever inflating the
	// floor by the position itself.
	minU, maxU := math.Inf(1), math.Inf(-1)
	minV, maxV := math.Inf(1), math.Inf(-1)
	maxAbs := 0.0
	budget := newWorkBudget(ctx)
	for _, p := range pts {
		if err := budget.step(); err != nil {
			return err
		}
		minU, maxU = math.Min(minU, p.U), math.Max(maxU, p.U)
		minV, maxV = math.Min(minV, p.V), math.Max(maxV, p.V)
		maxAbs = math.Max(maxAbs, math.Max(math.Abs(p.U), math.Abs(p.V)))
	}
	span := math.Hypot(maxU-minU, maxV-minV)
	floor := 1e-9*span + 4*(math.Nextafter(maxAbs, math.Inf(1))-maxAbs)
	for i := range loopIdx {
		for j := i + 1; j < len(loopIdx); j++ {
			gate := loopSag[i] + loopSag[j] + floor
			d, err := loopPolylineDistance(ctx, pts, loopIdx[i], loopIdx[j])
			if err != nil {
				return err
			}
			if d <= gate {
				return &tessellationExpectedError{err: fmt.Errorf(`%w: two cap boundary loops come within the chord tolerance of each other`, ErrDegenerate)}
			}
		}
	}
	return nil
}

// loopPolylineDistance is the minimum distance between two closed sample
// polylines.
func loopPolylineDistance(ctx context.Context, pts []Point2, a, b []int) (float64, error) {
	best := math.Inf(1)
	budget := newWorkBudget(ctx)
	for i := range a {
		a0, a1 := pts[a[i]], pts[a[(i+1)%len(a)]]
		for j := range b {
			if err := budget.step(); err != nil {
				return 0, err
			}
			b0, b1 := pts[b[j]], pts[b[(j+1)%len(b)]]
			best = math.Min(best, segSegDistance(a0, a1, b0, b1))
		}
	}
	return best, nil
}

// tessellationExpectedError marks a valid operand whose requested chording
// cannot prove its topology. Public Tessellate still exposes ErrDegenerate
// through Unwrap; read-only interference treats it as an undecided pair.
type tessellationExpectedError struct{ err error }

func (e *tessellationExpectedError) Error() string { return e.err.Error() }
func (e *tessellationExpectedError) Unwrap() error { return e.err }

// segSegDistance is the minimum distance between two 2D segments.
func segSegDistance(a0, a1, b0, b1 Point2) float64 {
	if segmentsCross(a0, a1, b0, b1) {
		return 0
	}
	d := math.Min(pointSegDistance(a0, b0, b1), pointSegDistance(a1, b0, b1))
	d = math.Min(d, pointSegDistance(b0, a0, a1))
	return math.Min(d, pointSegDistance(b1, a0, a1))
}

// segmentsCross reports whether the two segments properly intersect.
func segmentsCross(a0, a1, b0, b1 Point2) bool {
	d1 := cross2(b0, b1, a0)
	d2 := cross2(b0, b1, a1)
	d3 := cross2(a0, a1, b0)
	d4 := cross2(a0, a1, b1)
	return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
		((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))
}

// pointSegDistance is the distance from p to segment (s0, s1).
func pointSegDistance(p, s0, s1 Point2) float64 {
	du, dv := s1.U-s0.U, s1.V-s0.V
	l2 := du*du + dv*dv
	t := 0.0
	if l2 > 0 {
		t = math.Min(1, math.Max(0, ((p.U-s0.U)*du+(p.V-s0.V)*dv)/l2))
	}
	return math.Hypot(p.U-(s0.U+t*du), p.V-(s0.V+t*dv))
}
