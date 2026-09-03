package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

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
// Body → Face → Edge → Vertex). Vertices lie on the evaluator's held boundary;
// curve chording and inherited payload displacement are bounded by Bound. A Mesh
// is immutable: the accessors return copies of its slices.
type Mesh struct {
	vertices  []r3.Vec
	triangles [][3]int
	source    []*Face
	bound     float64 // millimetres
	// faceBound is docs/tessellation-design.md §2's sourceBound(face): the
	// two-sided displacement between the true trimmed face patch and the facets
	// held for it, one entry for EVERY face appearing in source. bound is the
	// maximum over it, so a mesh never publishes a global figure no face
	// accounts for, and the boolean's hidden-tangency pre-pass charges each face
	// what its own construction cost rather than inferring a displacement from
	// the surface kind. A source face missing from this map is an evaluator
	// invariant failure, never a zero: zero is the CLAIM that the held polygon
	// is the true trimmed patch and that its stored coordinates add nothing.
	faceBound map[*Face]float64
	// areaSlack is a proven bound (mm²) on how far the mesh's total facet
	// area falls short of — or, over a hole, overshoots — the body's true
	// boundary area: the chord-versus-arc deficit, closed form per walk, plus
	// the per-facet allowance every coordinate the build itself computed costs.
	// The mesh boolean's area bounds compose from it.
	areaSlack float64
	// volSymDiff bounds volume(TrueBody △ MeshSolid) — occupied volume, never
	// signed volume, so no term of it may cancel another
	// (docs/tessellation-design.md §2). It is meaningful ONLY when symDiffOK:
	// a payload class whose occupied-volume proof has not landed publishes a
	// mesh for export with symDiffOK false, and the mesh boolean refuses that
	// operand rather than substituting bound × held area, which
	// docs/tessellation-design.md §11 forbids outright.
	volSymDiff float64
	symDiffOK  bool
}

// sourceBound reads docs/tessellation-design.md §2's sourceBound(face) for one
// of the mesh's own source faces. A face the record does not carry is an
// evaluator invariant failure — the caller's own sentinel decides how loud —
// and NEVER a zero, which would claim the face is held exactly.
func (m *Mesh) sourceBound(f *Face) (float64, bool) {
	d, ok := m.faceBound[f]
	return d, ok
}

// setFaceBound records one source face's proven displacement, keeping the
// largest a face accumulates over the several walks that can build it (a
// coalesced side face) and lifting bound to match.
func (m *Mesh) setFaceBound(f *Face, delta float64) {
	if m.faceBound == nil {
		m.faceBound = map[*Face]float64{}
	}
	// A zero is a published proof, not an absent one, so the entry is always
	// written: the record's own contract is that every source face is present,
	// and a missing key is an invariant failure the consumers refuse on.
	if prev, ok := m.faceBound[f]; !ok || delta > prev {
		m.faceBound[f] = delta
	}
	if delta > m.bound {
		m.bound = delta
	}
}

// Vertices returns the mesh vertex positions in millimetres (core §5.2).
// Every vertex lies on the held boundary. Its deviation from the denoted body,
// including inherited payload displacement, is covered by Bound.
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
// complete displacement the tessellation used, including curve chording and
// inherited payload coordinate bounds. Its chording component is at most the
// requested tolerance — it is a PROVEN upper bound on the chord-to-arc
// deviation rather than that deviation itself, so it can read above the
// deviation a chord actually takes by the factor [Body.Tessellate] states;
// inherited payload displacement is added on top, so
// Bound can exceed that tolerance. It is zero only when every held boundary
// coordinate is exact. A scalar quantity is a units.Value (core §5.1): Kind
// Length, millimetres.
func (m *Mesh) Bound() units.Value { return units.Millimeters(m.bound) }

// Tessellate approximates the body's boundary as a triangle mesh whose
// chording deviates from the held analytic faces by no more than tol. Inherited
// payload displacement is added to the returned Bound. It is an OUTPUT, not
// the representation (core §11). tol is a magnitude: Kind Length
// ([ErrUnitKind] otherwise), finite ([ErrNotFinite]), non-negative
// ([ErrNegativeMagnitude]); a zero tolerance asks for a chord that is the
// curve and is [ErrDegenerate].
//
// Planar faces triangulate exactly. Circular boundaries are chorded at
// parameter samples chosen once per boundary curve and shared by every face
// that meets it — a cap and the cylinder wall use the same chording of their
// shared edge — so the mesh is watertight and consistently oriented by
// construction, and Bound carries the complete displacement actually taken.
//
// Two things follow from the chording sagitta being PROVEN rather than tight
// (docs/tessellation-design.md §3 derives it; chordSagitta implements it).
// First, the chording component of [Mesh.Bound] sits above the deviation the
// chords take by the factor (x/sin x)² at x = a chord's own quarter angle —
// at most π²/9, about 9.66% high, at the coarsest chording a closed walk
// reaches (three chords over a full circle), and 0.83% at ten. Second, the
// chord count is chosen against that same proven figure, so the finest tol
// this mesh admits is set by it and not by the tighter true sagitta: a tol
// within about 3.06e-9 relative of the finest chording the per-curve cap
// allows leaves no admissible count and is [ErrUnsupported]. Both point the
// same way — a finer mesh, or a refusal, never a coarser mesh under a claim
// this package cannot prove.
//
// Prism, cup, loft and boolean-built bodies tessellate. A lofted body RESTATES
// the flat triangle set its construction already built and audited: nothing is
// chorded here, so tol binds nothing on that path and the returned Bound is the
// payload's own facet departure, which can sit either side of tol.
// A boolean-built body also only
// RESTATES its held mesh, so tol must be at least the body's own Bound: a
// finer tol is [ErrUnsupported], because the analytic identity is gone and no
// finer mesh can be proven. A prism the analytic prism boolean assembled holds
// its section within a proven displacement of the section it denotes
// (docs/prism-boolean-design.md §7); that displacement is reserved from tol
// before any chord is chosen, so a tol it exhausts is [ErrUnsupported] too. A
// revolve body is [ErrUnsupported] here — Revolve builds and verifies, but its
// analytic surfaces have no tessellator, so it cannot be meshed or exported. A
// prism whose section carries a free-form (NURBS) wall builds and reports its
// Volume (docs/spline-design.md §10 P4b) but is [ErrUnsupported] here too:
// free-form chording is its own later increment (§10 P5), and chordLoop
// refuses it the same way it refuses every other staged walk kind. A body
// this evaluator did not build at all is also [ErrUnsupported].
func (b *Body) Tessellate(tol units.Value) (*Mesh, error) {
	return b.TessellateContext(context.Background(), tol)
}

// TessellateContext is [Body.Tessellate] with cancellation. It returns
// ctx.Err() unchanged when ctx is canceled before or during tessellation.
func (b *Body) TessellateContext(ctx context.Context, tol units.Value) (*Mesh, error) {
	return tessellateContext(ctx, b, tol)
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
	if lp, ok := b.payload.(loftPayload); ok {
		// The loft path is an exact restatement of the triangle set the payload
		// already holds, so it takes no chord tolerance at all
		// (tessellate_loft.go's own doc comment owns why).
		return tessellateLoft(ctx, b, lp)
	}
	pp, ok := b.payload.(prismPayload)
	if !ok {
		// Chording is per payload kind. Name both the staged kind and the
		// implemented set so the refusal cannot misstate evaluator reach.
		return nil, fmt.Errorf(`%w: tessellation does not support payload %T; supported payload classes are prism, cup, loft, and faceted`, ErrUnsupported, b.payload)
	}

	// Every mesh vertex lands on the RECORDED section, which a payload carrying a
	// section displacement holds only within that displacement of the section its
	// construction DENOTES (docs/prism-boolean-design.md §7). The displacement is
	// therefore part of the mesh's deviation before a single chord is chosen, so
	// docs/tessellation-design.md §5 RESERVES it from the chord budget here, and a
	// tolerance that cannot pay for it refuses, the same shape as
	// tessellateFaceted's refusal of a tolerance below a held mesh bound. The
	// reservation keeps chording plus this displacement within tol; it covers no
	// other term, so the per-end axial displacement added at the end of the build
	// can still lift the complete Bound above tol, which §1's Tolerance row
	// allows. Both downward nudges are proven margin:
	// the subtraction's own rounding is at most half an ulp, which the first
	// covers, and the second pays for the upward-rounded sum this bound is
	// published through at the end of the build. Every payload a caller draws
	// carries a zero displacement and chords against the requested tolerance
	// unchanged.
	budget := chord
	if pp.sectionDelta > 0 {
		budget = downRound(downRound(chord - pp.sectionDelta))
		if budget <= 0 {
			requested := units.Millimeters(chord)
			displacement := units.Millimeters(pp.sectionDelta)
			return nil, fmt.Errorf(`%w: requested tolerance %s leaves no chord budget above the body's own section displacement %s; retry with a tolerance greater than %s`, ErrUnsupported, requested, displacement, displacement)
		}
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
	// Parallel to pts2: each sample's own plane-local enclosure gap, which every
	// mesh vertex it owns carries into its face's published displacement.
	var sampleBound []walkEndBound
	// The trim displacement a CAP carries: the largest sagitta over every loop
	// bounding it, which is every loop of the section.
	var capTrim float64
	// The section's own walk count and proven perimeter upper bound, for the
	// displacement's area charge below, and the summed circular-segment area its
	// occupied-volume charge reads.
	var walks int
	var perimeterUpper float64
	var segmentArea float64
	// One free-form counter for the whole chorded record (see chordLoop).
	work := newFreeformWork()
	loops := append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...)
	// faceTrim/faceAxial accumulate each face's own trim and axial displacement
	// as the walls are emitted; the two caps' are stated once below.
	faceTrim := map[*Face]float64{}
	faceAxial := map[*Face]float64{}
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		cl, err := chordLoop(ctx, loop, budget, pp.z1-pp.z0, work, func(w sideWalk) (*Face, error) {
			return faceOfRole(fmt.Sprintf("side(%d,%d)", li, w.segs[0]))
		})
		if err != nil {
			return nil, err
		}
		capTrim = math.Max(capTrim, cl.maxSag)
		mesh.areaSlack += cl.areaSlack
		segmentArea = absSumUpper(segmentArea, cl.segmentArea)
		walks += cl.walks
		perimeterUpper = absSumUpper(perimeterUpper, cl.perimeterUpper)

		base := len(pts2)
		pts2 = append(pts2, cl.samples...)
		sampleBound = append(sampleBound, cl.boundOf...)
		idx := make([]int, len(cl.samples))
		for j := range cl.samples {
			idx[j] = base + j
		}
		loopIdx = append(loopIdx, idx)
		loopSag = append(loopSag, cl.maxSag)

		// Side walls: one quad per chord, split into two triangles wound
		// outward (tangent × N is the outward side normal for a CCW outer
		// walk and a CW hole walk alike).
		for j := range cl.samples {
			g0 := base + j
			g1 := base + (j+1)%len(cl.samples)
			mesh.addTriangle([3]int{meshBottom(g0), meshBottom(g1), meshTop(g1)}, cl.faceOf[j])
			mesh.addTriangle([3]int{meshBottom(g0), meshTop(g1), meshTop(g0)}, cl.faceOf[j])
			// A wall spans both ends, so it cannot attribute its axial
			// displacement to one of them and takes the larger.
			f := cl.faceOf[j]
			faceTrim[f] = math.Max(faceTrim[f], cl.sagOf[j])
			faceAxial[f] = pp.axialDelta()
		}
	}
	faceTrim[capStart] = capTrim
	faceTrim[capEnd] = capTrim
	faceAxial[capStart] = pp.z0Delta
	faceAxial[capEnd] = pp.z1Delta

	// The mesh vertices: bottom and top of every boundary sample, placed
	// through the payload — exactly on the analytic boundary.
	mesh.vertices = make([]r3.Vec, 0, 2*len(pts2))
	// vertexStore is docs/tessellation-reach-design.md §3's deltaStore, one
	// entry per mesh vertex: the sample's own plane-local enclosure gap carried
	// through the frame (walkEndBoundAllow) plus the rounding the frame and
	// placement write commits (exactPrismPointRound). Both are zero for a
	// recorded line vertex under an axis-aligned identity payload; neither is
	// ever assumed zero for anything else.
	vertexStore := make([]float64, 0, 2*len(pts2))
	vertexBudget := newWorkBudget(ctx)
	for j, p := range pts2 {
		if err := vertexBudget.step(); err != nil {
			return nil, err
		}
		lo := pp.point(p.U, p.V, pp.z0)
		hi := pp.point(p.U, p.V, pp.z1)
		mesh.vertices = append(mesh.vertices, lo, hi)
		plane := walkEndBoundAllow(sampleBound[j])
		vertexStore = append(vertexStore,
			absSumUpper(plane, exactPrismPointRound(pp, p.U, p.V, pp.z0, lo)),
			absSumUpper(plane, exactPrismPointRound(pp, p.U, p.V, pp.z1, hi)),
		)
	}
	storeMax, err := requireDerivableStore(vertexStore)
	if err != nil {
		return nil, err
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
	// Every vertex sits on the RECORDED boundary at one of the recorded sweep
	// levels — within what its own construction rounded (vertexStore) — and a
	// payload holds each level and each section only within its own displacement
	// of what it denotes: the section's in the plane
	// (docs/prism-boolean-design.md §7), each end's along the normal. So a face
	// deviates by its own trim chording plus its own store, section and axial
	// terms, which is exactly what §3's faceBound composes. The chording above
	// already paid for the section displacement out of the reserved budget, so
	// chording plus that term stays within the requested tolerance; the axial
	// displacement carries no reservation of its own and can make the complete
	// bound larger. The section displacement moves AREA too: the section can
	// differ from the one it denotes by a tube about its own boundary, charged
	// once per cap, and the boundary's own length can differ by that tube's
	// length reading, charged over the sweep height — evalPrism's own composition
	// (2·regionArea + perimeter·height), one dimension at a time. Every such term
	// is zero for a payload a caller draws.
	if err := composeFaceBounds(&mesh, faceTrim, faceAxial, vertexStore, pp.sectionDelta); err != nil {
		return nil, err
	}
	if pp.sectionDelta > 0 {
		capMove := sectionDisplacementArea(pp.sectionDelta, walks, perimeterUpper)
		wallMove := productUpper(sectionDisplacementLength(pp.sectionDelta, walks), math.Abs(pp.z1-pp.z0))
		mesh.areaSlack = absSumUpper(mesh.areaSlack, capMove, capMove, wallMove)
	}
	// Every coordinate the build itself computed can move each facet's own area
	// (docs/tessellation-design.md §5's per-triangle allowance), so the slack
	// carries one such term per facet beside the analytic ones above.
	mesh.areaSlack = absSumUpper(mesh.areaSlack, meshStoreAreaAllow(&mesh, vertexStore))

	// Occupied volume (docs/tessellation-reach-design.md §3). The chorded section
	// differs from the section it denotes by the circular segments it omits or
	// adds, over the sweep height; the recorded section differs from the denoted
	// one by its own displacement tube, again over that height; each end level
	// differs by its own axial displacement over the cap it caps; and every
	// computed coordinate sweeps volume at the rate of the surface it moved.
	// Absolute sums throughout — an occupied-volume bound admits no cancellation.
	height := math.Abs(pp.z1 - pp.z0)
	areaUpper := meshFaceAreaUpper(&mesh, vertexStore)
	terms := []float64{
		productUpper(height, segmentArea),
		productUpper(sectionDisplacementArea(pp.sectionDelta, walks, perimeterUpper), height),
		productUpper(pp.z0Delta, areaUpper[capStart]),
		productUpper(pp.z1Delta, areaUpper[capEnd]),
		sweptVolumeAllow(storeMax, perturbedAreaUpper(mesh.vertices, mesh.triangles, storeMax)),
	}
	if err := publishSymDiff(&mesh, terms); err != nil {
		return nil, err
	}
	return &mesh, nil
}

// requireDerivableStore folds the per-vertex store displacements into the
// payload-wide maximum, refusing a mesh whose own construction it cannot state
// (docs/tessellation-design.md §12: a non-finite proof is a refusal, never an
// infinite bound). chordStationBound's +Inf for an underivable enclosure lands
// here.
func requireDerivableStore(store []float64) (float64, error) {
	worst := 0.0
	for _, d := range store {
		if isNonFinite(d) {
			return 0, fmt.Errorf(`%w: a chorded boundary sample states no enclosure of the point its own record denotes, so this mesh can publish no displacement bound for the faces that meet it`, ErrUnsupported)
		}
		worst = math.Max(worst, d)
	}
	return worst, nil
}

// composeFaceBounds publishes docs/tessellation-design.md §2's sourceBound for
// every face the mesh names, as §3's sum of that face's own trim, store, section
// and axial displacements, and lifts Mesh.bound to their maximum. Every source
// face is present by construction: the walk is over mesh.source itself.
func composeFaceBounds(m *Mesh, trim, axial map[*Face]float64, store []float64, section float64) error {
	faceStore := map[*Face]float64{}
	for i, f := range m.source {
		for _, v := range m.triangles[i] {
			faceStore[f] = math.Max(faceStore[f], store[v])
		}
	}
	for f, s := range faceStore {
		bound := upRound(trim[f] + s + section + axial[f])
		if isNonFinite(bound) {
			return fmt.Errorf(`%w: a face's composed displacement is not finite, so this mesh can state no bound for it`, ErrUnsupported)
		}
		m.setFaceBound(f, bound)
	}
	return nil
}

// meshStoreAreaAllow sums docs/tessellation-design.md §5's per-triangle area
// allowance over the mesh, each facet charged the largest store displacement its
// own three vertices carry.
func meshStoreAreaAllow(m *Mesh, store []float64) float64 {
	total := 0.0
	for _, tri := range m.triangles {
		d := math.Max(store[tri[0]], math.Max(store[tri[1]], store[tri[2]]))
		if d <= 0 {
			continue
		}
		a, b, c := m.vertices[tri[0]], m.vertices[tri[1]], m.vertices[tri[2]]
		total = absSumUpper(total, perturbedTriangleAreaAllow(a, b, c, d))
	}
	return total
}

// meshFaceAreaUpper bounds each source face's own true patch area from the
// facets held for it: their held area plus the per-facet allowance their
// computed coordinates can move it by. It is the yardstick a level's axial
// displacement is charged against — moving a planar patch's level by delta
// displaces at most delta times that patch's own area.
func meshFaceAreaUpper(m *Mesh, store []float64) map[*Face]float64 {
	out := map[*Face]float64{}
	for i, f := range m.source {
		tri := m.triangles[i]
		a, b, c := m.vertices[tri[0]], m.vertices[tri[1]], m.vertices[tri[2]]
		d := math.Max(store[tri[0]], math.Max(store[tri[1]], store[tri[2]]))
		held := b.Sub(a).Cross(c.Sub(a)).Len() / 2
		out[f] = absSumUpper(out[f], held, perturbedTriangleAreaAllow(a, b, c, d))
	}
	return out
}

// publishSymDiff sums an analytic payload's occupied-volume terms into the
// mesh's own volSymDiff and marks the proof complete. A term this build cannot
// state refuses (docs/tessellation-design.md §12) rather than publishing a
// symmetric-difference bound the boolean would then compose into a result.
func publishSymDiff(m *Mesh, terms []float64) error {
	total := absSumUpper(terms...)
	if isNonFinite(total) {
		return fmt.Errorf(`%w: this mesh states no finite bound on the volume it and the body it stands for differ by, so no boolean may consume it`, ErrUnsupported)
	}
	m.volSymDiff = total
	m.symDiffOK = true
	return nil
}

// chordedLoop is one boundary loop's chording, as chordLoop returns it: the 2D
// samples, the wall face of the chord LEAVING each sample, the largest sagitta
// the chording took, the chord-versus-arc area slack over the sweep height, and
// the loop's own coalesced walk count with a proven upper bound on its analytic
// length — the two figures a section displacement's area charge reads
// (docs/tessellation-design.md §5). Beside those it carries the three readings
// the proof record composes per FACE rather than per mesh: each sample's own
// outgoing sagitta and enclosure gap, and the loop's summed circular-segment
// area (docs/tessellation-reach-design.md §3).
type chordedLoop struct {
	samples []Point2
	faceOf  []*Face
	// sagOf is, parallel to samples, the chord sagitta bound of the walk the
	// sample's OUTGOING chord belongs to — the trim displacement its wall face
	// carries, and zero for a straight walk, which chords nothing.
	sagOf []float64
	// boundOf is, parallel to samples, the sample's own proven plane-local
	// enclosure gap: what the recorded curve's certified enclosure at the
	// parameter this sample denotes says about the held (u, v) pair. A walk
	// junction reads the walk's own recorded endpoint bound; an interior
	// circular station reads chordStationBound. A component the record cannot
	// enclose reads +Inf, and the tessellation refuses on it.
	boundOf        []walkEndBound
	maxSag         float64
	areaSlack      float64
	segmentArea    float64
	walks          int
	perimeterUpper float64
}

// chordLoop chords one boundary loop into 2D samples: sample j is walk j's own
// start (the junction shared with the previous walk) plus, for a circular walk,
// its interior chord samples. The wall face of each sample's outgoing chord is
// resolved by wallFace over the coalesced walk it belongs to. The same chording
// feeds every face that meets the loop — walls and caps alike — so the mesh is
// watertight by construction.
// work is the free-form counter of the RECORD being chorded, opened once by the
// caller and shared by every loop of it: chording holds no preflight counter, so
// the ceiling starts at the tessellation entry rather than at each loop.
func chordLoop(ctx context.Context, loop LoopRecord, chord, height float64, work *freeformWork, wallFace func(w sideWalk) (*Face, error)) (chordedLoop, error) {
	if len(loop.Segments) == 0 {
		return chordedLoop{}, fmt.Errorf(`%w: a recorded loop holds no segments`, ErrDegenerate)
	}
	// One counter spans the segment walk, the walk loop and the sample emission
	// nested under it: a single walk emits many samples, and it is the SAMPLES
	// that are the candidate operations §7.2 counts.
	budget := newWorkBudget(ctx)
	raw := make([]sideWalk, len(loop.Segments))
	// The loop's analytic length, upper bound included: buildLoopSidesAs sums the
	// same RAW walk lengths for the body's own perimeter, so both readings of one
	// section speak for the same curve.
	perimeterUpper := 0.0
	for i, seg := range loop.Segments {
		if err := budget.step(); err != nil {
			return chordedLoop{}, err
		}
		w, err := walkOf(seg, work)
		if err != nil {
			return chordedLoop{}, err
		}
		if err := requireAnalyticWalk(w, "chording a boundary loop"); err != nil {
			return chordedLoop{}, err
		}
		perimeterUpper = absSumUpper(perimeterUpper, w.length, w.lengthBound)
		raw[i] = sideWalk{segmentWalk: w, segs: []int{i}}
	}
	walks, err := coalesceWalksContext(ctx, raw)
	if err != nil {
		return chordedLoop{}, err
	}

	var samples []Point2
	var faceOf []*Face
	var sagOf []float64
	var boundOf []walkEndBound
	var maxSag, areaSlack, segmentArea float64
	for _, w := range walks {
		if err := budget.step(); err != nil {
			return chordedLoop{}, err
		}
		face, err := wallFace(w)
		if err != nil {
			return chordedLoop{}, err
		}
		if !w.isCircular() {
			samples = append(samples, Point2{U: w.startU, V: w.startV})
			faceOf = append(faceOf, face)
			sagOf = append(sagOf, 0)
			boundOf = append(boundOf, w.startBound)
			continue
		}
		n, sag, err := chordCount(w.segmentWalk, chord)
		if err != nil {
			return chordedLoop{}, err
		}
		maxSag = math.Max(maxSag, sag)
		areaSlack += walkAreaSlack(w.segmentWalk, n, height)
		segmentArea = absSumUpper(segmentArea, walkSegmentArea(w.segmentWalk, n))
		// A circular walk never coalesces (coalesceWalks), so it covers exactly
		// one recorded segment and every station on it is that segment's own
		// parameter — which is what lets chordStationBound enclose the point the
		// RECORD denotes there rather than the point this walk's float trig
		// landed on.
		seg := loop.Segments[w.segs[0]]
		dth := (w.th1 - w.th0) / float64(n)
		for k := range n {
			if err := budget.step(); err != nil {
				return chordedLoop{}, err
			}
			p := Point2{U: w.startU, V: w.startV}
			bound := w.startBound
			if k > 0 {
				th := w.th0 + float64(k)*dth
				p = Point2{U: w.cU + w.radius*math.Cos(th), V: w.cV + w.radius*math.Sin(th)}
				bound = chordStationBound(seg, k, n, p.U, p.V)
			}
			samples = append(samples, p)
			faceOf = append(faceOf, face)
			sagOf = append(sagOf, sag)
			boundOf = append(boundOf, bound)
		}
	}
	return chordedLoop{
		samples:        samples,
		faceOf:         faceOf,
		sagOf:          sagOf,
		boundOf:        boundOf,
		maxSag:         maxSag,
		areaSlack:      areaSlack,
		segmentArea:    segmentArea,
		walks:          len(walks),
		perimeterUpper: perimeterUpper,
	}, nil
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

	// One free-form counter for the whole chorded record — the cup's outer region
	// and its cavity are the two halves of one section (see chordLoop).
	work := newFreeformWork()
	oLoops := append([]LoopRecord{cp.outer.Outer}, cp.outer.Holes...)
	cLoops := append([]LoopRecord{cp.cavity.Outer}, cp.cavity.Holes...)
	if len(oLoops) != len(cLoops) {
		return nil, fmt.Errorf(`%w: the cup's outer and cavity regions have different loop counts`, ErrDegenerate)
	}

	openIsMax := cp.zOpen > cp.zOuter
	oLo, oHi := math.Min(cp.zOuter, cp.zOpen), math.Max(cp.zOuter, cp.zOpen)
	cLo, cHi := math.Min(cp.zCav, cp.zOpen), math.Max(cp.zCav, cp.zOpen)

	var mesh Mesh
	base := cp.basePrism()
	var verts []r3.Vec
	// vertexStore is docs/tessellation-reach-design.md §3's deltaStore, parallel
	// to verts: the sample's plane-local enclosure gap carried through the frame
	// plus the rounding the frame/placement write commits.
	var vertexStore []float64
	add := func(v r3.Vec, store float64) int {
		verts = append(verts, v)
		vertexStore = append(vertexStore, store)
		return len(verts) - 1
	}
	// Each face's own trim and axial displacement, composed into its published
	// sourceBound once the whole cup is built.
	faceTrim := map[*Face]float64{}
	faceAxial := map[*Face]float64{}
	// The summed circular-segment area of each region's own loops, over the
	// region's own sweep height — the cup's occupied-volume analytic term.
	var oSegmentArea, cSegmentArea float64

	// One chorded ring per region loop: the walls hang off it and the caps and
	// rims share it, so every vertex is shared and the mesh closes by
	// construction. A ring holds its samples, the lo/hi mesh vertices of each,
	// its wall faces and the sagitta bound the chording proved.
	type ring struct {
		samples []Point2
		loV     []int
		hiV     []int
		faces   []*Face
		sag     float64
	}
	chordRing := func(loop LoopRecord, h, lo, hi, loDelta, hiDelta float64, role string, area *float64) (ring, error) {
		cl, err := chordLoop(ctx, loop, chord, h, work, func(w sideWalk) (*Face, error) {
			return faceOfRole(fmt.Sprintf(role, w.segs[0]))
		})
		if err != nil {
			return ring{}, err
		}
		samples := cl.samples
		mesh.areaSlack += cl.areaSlack
		*area = absSumUpper(*area, cl.segmentArea)
		r := ring{samples: samples, faces: cl.faceOf, sag: cl.maxSag}
		r.loV = make([]int, len(samples))
		r.hiV = make([]int, len(samples))
		// A wall spans both of its region's levels, so it cannot attribute its
		// axial displacement to one of them and takes the larger.
		wallAxial := math.Max(loDelta, hiDelta)
		for i, p := range samples {
			plane := walkEndBoundAllow(cl.boundOf[i])
			loV := base.point(p.U, p.V, lo)
			hiV := base.point(p.U, p.V, hi)
			r.loV[i] = add(loV, absSumUpper(plane, exactPrismPointRound(base, p.U, p.V, lo, loV)))
			r.hiV[i] = add(hiV, absSumUpper(plane, exactPrismPointRound(base, p.U, p.V, hi, hiV)))
			f := cl.faceOf[i]
			faceTrim[f] = math.Max(faceTrim[f], cl.sagOf[i])
			faceAxial[f] = wallAxial
		}
		return r, nil
	}
	// Each region's own lo/hi level displacement, in the order chordRing takes
	// them: the open end carries zOpenDelta, each floor its own level's.
	oLoDelta, oHiDelta := cp.zOuterDelta, cp.zOpenDelta
	cLoDelta, cHiDelta := cp.zCavDelta, cp.zOpenDelta
	if !openIsMax {
		oLoDelta, oHiDelta = cp.zOpenDelta, cp.zOuterDelta
		cLoDelta, cHiDelta = cp.zOpenDelta, cp.zCavDelta
	}

	// Outer region O: every loop in its natural sense (outer counter-clockwise,
	// holes clockwise), role side(i,j).
	oRings := make([]ring, len(oLoops))
	for i, loop := range oLoops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		oRings[i], err = chordRing(loop, oHi-oLo, oLo, oHi, oLoDelta, oHiDelta, fmt.Sprintf("side(%d,%%d)", i), &oSegmentArea)
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
		rev, err := reverseLoopRecordContext(ctx, loop)
		if err != nil {
			return nil, err
		}
		cRings[i], err = chordRing(rev, cHi-cLo, cLo, cHi, cLoDelta, cHiDelta, fmt.Sprintf("shellSide(%d,%%d)", i), &cSegmentArea)
		if err != nil {
			return nil, err
		}
	}
	mesh.vertices = verts
	storeMax, err := requireDerivableStore(vertexStore)
	if err != nil {
		return nil, err
	}

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
	// A planar patch's trim displacement is the largest sagitta over the loops
	// that bound it, and its axial displacement its own level's.
	for i := range oRings {
		faceTrim[capStart] = math.Max(faceTrim[capStart], oRings[i].sag)
	}
	faceAxial[capStart] = cp.zOuterDelta

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
	for i := range cRings {
		faceTrim[shellCap] = math.Max(faceTrim[shellCap], cRings[i].sag)
	}
	faceAxial[shellCap] = cp.zCavDelta

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
		faceTrim[rim] = math.Max(faceTrim[rim], math.Max(oRings[i].sag, cRings[i].sag))
		faceAxial[rim] = cp.zOpenDelta
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
	// A cup records its own section verbatim — no analytic reduction re-expresses
	// it — so it carries no section displacement, and each face's bound is its own
	// trim, store and level terms alone (docs/tessellation-design.md §6).
	if err := composeFaceBounds(&mesh, faceTrim, faceAxial, vertexStore, 0); err != nil {
		return nil, err
	}
	mesh.areaSlack = absSumUpper(mesh.areaSlack, meshStoreAreaAllow(&mesh, vertexStore))

	// Occupied volume (docs/tessellation-reach-design.md §3): each region's own
	// chorded-section deficit over its own sweep height, each planar level's
	// displacement over the patch it caps, and the computed coordinates' swept
	// volume.
	areaUpper := meshFaceAreaUpper(&mesh, vertexStore)
	rimArea := 0.0
	for i := range oLoops {
		rim, err := faceOfRole(fmt.Sprintf("rim(%d)", i))
		if err != nil {
			return nil, err
		}
		rimArea = absSumUpper(rimArea, areaUpper[rim])
	}
	terms := []float64{
		productUpper(oHi-oLo, oSegmentArea),
		productUpper(cHi-cLo, cSegmentArea),
		productUpper(cp.zOuterDelta, areaUpper[capStart]),
		productUpper(cp.zCavDelta, areaUpper[shellCap]),
		productUpper(cp.zOpenDelta, rimArea),
		sweptVolumeAllow(storeMax, perturbedAreaUpper(mesh.vertices, mesh.triangles, storeMax)),
	}
	if err := publishSymDiff(&mesh, terms); err != nil {
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
	return (walkWallSlack(w, n, h) + 2*walkSegmentArea(w, n)) * (1 + 1e-9)
}

// walkWallSlack is the WALL half of walkAreaSlack: the area one circular walk's
// side face loses by standing on n chords instead of its arc, (arc − chord) × h.
func walkWallSlack(w segmentWalk, n int, h float64) float64 {
	sweep := math.Abs(w.th1 - w.th0)
	arc := sweep * w.radius
	chord := float64(n) * 2 * w.radius * math.Sin(sweep/(2*float64(n)))
	return math.Max(arc-chord, 0) * h
}

// walkSegmentArea is the PLANAR half of walkAreaSlack, stated on its own
// because two proofs read it: the caps' own area slack reads it twice (one per
// cap), and docs/tessellation-reach-design.md §3's occupied-volume term reads it
// once per walk against the sweep height.
//
// It is the closed-form sum Σ a_c of the n circular segments between one
// circular walk's arc and its chords, r²/2 · (θ − n·sin(θ/n)) for a walk of
// radius r sweeping θ. The chorded section differs from the section it denotes
// by exactly those segments, so the prism between two levels differs from the
// prism it denotes by their sum times the sweep height — an area a hole gains
// and an outline loses, summed in ABSOLUTE value here because occupied volume
// admits no cancellation (docs/tessellation-design.md §2).
func walkSegmentArea(w segmentWalk, n int) float64 {
	sweep := math.Abs(w.th1 - w.th0)
	return w.radius * w.radius / 2 * math.Max(sweep-float64(n)*math.Sin(sweep/float64(n)), 0)
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
	// The payload carries one global composed displacement and no tighter
	// per-face certificate, so every restated face publishes that Delta
	// (docs/tessellation-design.md §2: an incomplete per-face composition falls
	// back to Delta, NEVER to zero).
	faceBound := map[*Face]float64{}
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
		faceBound[src[i]] = fp.meshBound
	}
	return &Mesh{
		vertices:   fp.verts,
		triangles:  fp.tris,
		source:     src,
		bound:      fp.meshBound,
		faceBound:  faceBound,
		areaSlack:  fp.areaSlack,
		volSymDiff: fp.volSymDiff,
		symDiffOK:  true,
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

// chordCount picks the number of chords for a circular walk so chordSagitta's
// PROVEN bound on each chord's sagitta stays within tol, and returns that
// bound. Every accept/reject below reads chordSagitta, never the true
// sagitta r·(1 − cos(Δθ/2)) it encloses. A closed walk needs at least three
// chords to bound a polygon; the per-chord angle never exceeds π, so
// consecutive samples are always distinct.
func chordCount(w segmentWalk, tol float64) (int, float64, error) {
	sweep := math.Abs(w.th1 - w.th0)
	maxD := math.Pi
	if tol < w.radius {
		// The TRUE sagitta 2r·sin²(Δθ/4) inverts to Δθ_max = 4·asin(√(tol/2r))
		// — the stable inverse: acos(1 − tol/r) rounds to zero for tiny tol
		// and would seed an unbounded walk-up. It seeds the search only; the
		// walk-up below decides on chordSagitta's larger proven figure, so
		// this seed can land one chord short and is corrected there.
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
	s := chordSagitta(w.radius, sweep, n)
	for s > tol && sweep > 0 {
		if n == maxChordsPerWalk {
			return 0, 0, errTooManyChords
		}
		n++
		s = chordSagitta(w.radius, sweep, n)
	}
	return n, s, nil
}

// chordSagitta is a PROVEN upper bound on the sagitta 2r·sin²(Δθ/4) a chord
// subtends when a circular walk of the given sweep is split into n equal
// chords — docs/tessellation-design.md §3's published sagitta, and the
// single owner of chordCount's walk-up step, so a later caller measuring the
// SAME quantity for a hand-chorded fixture reads the identical closed form
// rather than a second copy of it.
//
// It never calls math.Sin. sin(x) <= x for every x >= 0 — the tangent line
// to sin at the origin lies above the curve on the whole nonnegative axis,
// an elementary inequality, not a derived one — so with x = sweep/(4n),
// 2r·sin²(x) <= 2r·x² = r·sweep²/(8n²). That bound needs no trig call at
// all, so it carries none of Sin's own missing ulp contract (Go publishes
// none) and none of the FMA-contraction difference between amd64 and arm64
// a Sin-based formula would expose: every operation below is an ordinary
// multiply or divide, each individually outward-rounded, the same
// single-operation proof every other bound in this package rests on.
//
// The denominator 8n² is computed with PLAIN arithmetic, never
// outward-rounded: for every n this package ever calls with (n <=
// maxChordsPerWalk = 1<<14, so 8n² <= 2^31, far under float64's 2^53
// exact-integer range) the computation commits no rounding at all, and
// outward-rounding a DENOMINATOR would move the bound the WRONG way — a
// larger denominator gives a SMALLER, tighter, and here unproven quotient.
// The numerator r·sweep² is outward-rounded through productUpper, and the
// one division is outward-rounded by a final upRound, exactly the
// single-operation margin productUpper already applies to a multiply.
//
// That float chain is a proven upper bound wherever it lands on a positive
// value, and it is the only thing this helper publishes there. What it may
// NOT do is answer ZERO for a positive r·sweep²/(8n²): rounding outward
// cannot rescue an UNDERFLOW, since upRound leaves 0 alone, and both the
// sweep·sweep product and the final quotient can reach 0 from operands that
// are each positive. chordCount hands this figure on as the walk's proven
// sagitta and [Mesh.Bound] publishes it, so a zero there would claim an
// exact chording for a nonzero deviation and would end chordCount's walk-up
// loop on a false reading. A non-positive answer from the float chain
// therefore falls through to exactChordSagitta, which states the same bound
// over the rationals.
//
// The bound sits above the true sagitta by the factor (x/sin x)² — about
// 1.0000126 at a typical fine chording (90 degrees over 64 stations) and
// π²/9 in the coarsest case a closed walk can reach (n=3 on a full circle;
// TestChordSagittaCoarsestClosedWalkStaysProven pins that ratio). Two
// consequences reach a caller, and [Body.Tessellate]'s doc comment owns
// both in the caller's own terms: [Mesh.Bound] reads by that factor above
// the deviation the chords take, and chordCount's walk-up settles on a
// slightly larger n than the true sagitta alone would need — up to and
// including hitting maxChordsPerWalk and returning errTooManyChords for a
// tol within (x/sin x)²−1 of the finest chording the cap admits. Both are
// the safe direction: a finer mesh, or a typed refusal, never a coarser
// mesh under a claim this package cannot prove.
// TestChordCountRefusesTheToleranceWindowAtTheMeshCap pins that refusal at
// its exact boundary.
func chordSagitta(radius, sweep float64, n int) float64 {
	if n <= 0 || sweep < 0 {
		// A non-positive n or a negative sweep would otherwise read as
		// sagitta 0 — productUpper(sweep, sweep) already zeroes a negative
		// sweep — which UNDERSTATES the true, positive sagitta rather than
		// falsifying the claim (cutDisplacementAllow's own rule: an absent
		// bound must never read as a small one).
		return math.Inf(1)
	}
	if radius < 0 {
		// The true sagitta r·(1−cos) is itself non-positive for a negative
		// radius, so 0 is a valid (if unattained) upper bound here — no
		// refusal needed.
		return 0
	}
	if isNonFinite(radius) || isNonFinite(sweep) {
		// A non-finite claim states no bound at all. It refuses with +Inf for
		// the same reason the two arms above do, rather than carrying the NaN
		// the arithmetic below would otherwise publish as a bound.
		return math.Inf(1)
	}
	denom := 8 * float64(n) * float64(n)
	if denom <= 0 || isNonFinite(denom) {
		// 8n² saturated, so the true bound r·sweep²/(8n²) is itself driven to
		// zero and 0 remains an upper bound.
		return 0
	}
	if radius == 0 || sweep == 0 {
		// The true sagitta is exactly zero, and so is its bound.
		return 0
	}
	if s := upRound(productUpper(radius, productUpper(sweep, sweep)) / denom); s > 0 {
		return s
	}
	return exactChordSagitta(radius, sweep, n)
}

// exactChordSagitta states chordSagitta's own r·sweep²/(8n²) over the
// RATIONALS, for the one case its float chain cannot: a POSITIVE bound whose
// float evaluation underflows to zero. Two independent sites in that chain can
// underflow — sweep·sweep can fall below the smallest denormal on its own, and
// the final quotient can land on zero from a numerator that survived — and this
// helper covers both, because it is reached from the PUBLISHED value rather
// than from either site.
//
// radius and sweep are float64 and therefore exact rationals, and 8n² is an
// exact integer for every positive n, so the quotient here is the EXACT bound
// with no rounding anywhere in it. ratFloatUp then rounds that one value
// outward, which answers the smallest positive float64 — never zero — for a
// bound too small for float64 to represent.
func exactChordSagitta(radius, sweep float64, n int) float64 {
	num := new(big.Rat).Mul(floatRat(radius), new(big.Rat).Mul(floatRat(sweep), floatRat(sweep)))
	nRat := new(big.Rat).SetInt64(int64(n))
	denom := new(big.Rat).Mul(new(big.Rat).SetInt64(8), new(big.Rat).Mul(nRat, nRat))
	return ratFloatUp(num.Quo(num, denom))
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
			chordGate := loopSag[i] + loopSag[j]
			gate := chordGate + floor
			d, err := loopPolylineDistance(ctx, pts, loopIdx[i], loopIdx[j])
			if err != nil {
				return err
			}
			if d <= gate {
				msg := fmt.Sprintf(
					`cap boundary loops %d and %d have measured distance %s; distance must exceed the required clearance gate %s`,
					i, j, units.Millimeters(d), units.Millimeters(gate),
				)
				// Chording can only lower the gate toward floor. When the
				// measured gap does not exceed floor, no finer retry can pass.
				if d > floor && chordGate > 0 {
					msg += `; retry with a finer chord tolerance to reduce the gate`
				}
				return &tessellationExpectedError{err: fmt.Errorf(`%w: %s`, ErrDegenerate, msg)}
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
