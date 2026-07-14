package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the tessellation half of the export slice (core §11,
// docs/evaluator-design.md §9/§11 increment 4): per-surface analytic
// tessellators over the evaluator's own prism payload, with a proven
// per-facet deviation bound and facets that remember their source face. The
// exact-predicate mesh boolean that consumes the same machinery is the rest
// of increment 4 and is not here.

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
// construction, and Bound carries the largest sagitta actually taken. A body
// this evaluator did not build is [ErrUnsupported].
func (b *Body) Tessellate(tol units.Value) (*Mesh, error) {
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
	pp := *b.payload

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
	capStart, err := faceOfRole("capStart")
	if err != nil {
		return nil, err
	}
	capEnd, err := faceOfRole("capEnd")
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
	loops := append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...)
	for li, loop := range loops {
		if len(loop.Segments) == 0 {
			return nil, fmt.Errorf(`%w: a recorded loop holds no segments`, ErrDegenerate)
		}
		raw := make([]sideWalk, len(loop.Segments))
		for i, seg := range loop.Segments {
			w, err := walkOf(seg)
			if err != nil {
				return nil, err
			}
			raw[i] = sideWalk{segmentWalk: w, segs: []int{i}}
		}
		walks := coalesceWalks(raw)

		var samples []Point2
		var faceOf []*Face // per sample: the wall face of the chord leaving it
		for _, w := range walks {
			face, err := faceOfRole(fmt.Sprintf("side(%d,%d)", li, w.segs[0]))
			if err != nil {
				return nil, err
			}
			if !w.circular {
				samples = append(samples, Point2{U: w.startU, V: w.startV})
				faceOf = append(faceOf, face)
				continue
			}
			n, sag := chordCount(w.segmentWalk, chord)
			mesh.bound = math.Max(mesh.bound, sag)
			dth := (w.th1 - w.th0) / float64(n)
			for k := range n {
				p := Point2{U: w.startU, V: w.startV}
				if k > 0 {
					th := w.th0 + float64(k)*dth
					p = Point2{U: w.cU + w.radius*math.Cos(th), V: w.cV + w.radius*math.Sin(th)}
				}
				samples = append(samples, p)
				faceOf = append(faceOf, face)
			}
		}

		base := len(pts2)
		pts2 = append(pts2, samples...)
		idx := make([]int, len(samples))
		for j := range samples {
			idx[j] = base + j
		}
		loopIdx = append(loopIdx, idx)

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
	for _, p := range pts2 {
		mesh.vertices = append(mesh.vertices, pp.point(p.U, p.V, pp.z0), pp.point(p.U, p.V, pp.z1))
	}

	// Caps: both share one 2D triangulation of the chorded region — the same
	// non-convex, hole-carrying polygon — mapped to the top vertices as-is
	// (outward +N) and to the bottom vertices reversed (outward −N).
	capTris, err := triangulate2D(pts2, loopIdx)
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
func chordCount(w segmentWalk, tol float64) (int, float64) {
	sweep := math.Abs(w.th1 - w.th0)
	maxD := math.Pi
	if tol < w.radius {
		maxD = 2 * math.Acos(1-tol/w.radius)
	}
	n := max(int(math.Ceil(sweep/maxD-1e-9)), 1)
	if w.closed && n < 3 {
		n = 3
	}
	return n, w.radius * (1 - math.Cos(sweep/float64(n)/2))
}
