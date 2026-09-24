package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/r3"
)

// This file is docs/tessellation-design.md §2's stitchPayload restatement
// row (docs/surface-design.md §10, §14 Table D row 5): an all-planar
// stitched body's mesh, CLOSED or OPEN, is an exact restatement of the
// triangle set Stitch's own build already assembled and audited (stitch.go's
// evalStitchContext), modelled line for line on tessellate_loft.go's exact
// restatement. Nothing is chorded, welded, moved, or retriangulated here, so
// this path takes no chord tolerance at all — there is no chording
// component for one to bind (tessellate_loft.go's own reasoning, restated
// for this payload).
//
// A curved or mixed stitched body carries no recorded triangle set
// (stitchPayload.tris is nil). T10 reuses one complete revolve sheet's
// chording under surface §10.1; T11 also accepts that sheet's identity
// Unstitch siblings re-welded along their original edges (§10.2). Other
// curved sources refuse with ErrUnsupported. An OPEN
// all-planar sheet runs docs/tessellation-design.md §1.2's manifold-with-
// boundary audit (requireSheetMesh, requireSheetVertexLinks) in the
// closed-mesh audit's place, exactly as a surface-result prism or revolve
// sheet's own mesh does (docs/surface-design.md §10).

// tessellateStitch restates an all-planar stitched body's own recorded
// triangle set as a Mesh, CLOSED or OPEN. Its per-face bound is the largest
// Vertex.Bound() over the vertices that face's own triangles touch, zero
// exactly when every one of them is; its areaSlack is the matching
// perturbedTriangleAreaAllow sum. A CLOSED body (b.Kind() == BodySolid)
// whose every vertex carries a proven bound of exactly zero
// (stitchZeroVertexBound, stitch.go) publishes a zero occupied-volume proof
// (symDiffOK == true) — see the comment at that publication below for the
// three-step argument. Every other stitched body — open, curved/mixed,
// placed, or certificate-welded — keeps symDiffOK false, so Union, Cut and
// Intersect keep refusing it (boolean.go's own requireVolumeProvingPayload
// arm), whatever the exact tetrahedron sum itself already proves about
// signed volume.
// stitchLumpFaceGroups returns b's own lumps' constituent faces, one slice
// per lump: sheetLumps builds exactly one shell per lump
// (docs/surface-design.md §2.2), so this is a direct read of b's own
// recorded connectivity, never a second split.
func stitchLumpFaceGroups(b *Body) [][]*Face {
	lumps := b.Lumps()
	groups := make([][]*Face, len(lumps))
	for i, lp := range lumps {
		for _, s := range lp.Shells() {
			groups[i] = append(groups[i], s.Faces()...)
		}
	}
	return groups
}

// stitchSiblingRevolveSource proves that every stitched source face is an
// identity Unstitch copy of one original revolve sheet and that the weld plan
// only rejoins original edges and vertices. The original mesher's shared
// stations then still describe every welded seam.
func stitchSiblingRevolveSource(b *Body, sp stitchPayload) (*Body, map[*Face]*Face, string) {
	if b.Kind() != BodySheet {
		return nil, nil, "a newly welded curved stitch has no open-sheet proof"
	}
	var originalBody *Body
	paired := make(map[*Face]*Face, len(sp.faces))
	originalEdgeOf := map[*Edge]*Edge{}
	originalVertexOf := map[*Vertex]*Vertex{}
	copiesByOriginalEdge := map[*Edge]map[*Edge]struct{}{}
	for i, copied := range sp.faces {
		piece := copied.body
		if piece == nil || len(piece.Faces()) != 1 || piece.Faces()[0] != copied {
			return nil, nil, "a source is not a single-face unstitched sheet"
		}
		up, ok := piece.payload.(unstitchPayload)
		if !ok || up.xform != r3.Identity() || up.delta != 0 || up.face == nil {
			return nil, nil, "an unstitched source has no identity copy proof"
		}
		original := up.face
		if originalBody == nil {
			originalBody = original.body
		}
		if originalBody == nil || original.body != originalBody || sp.liveFaces[i] == nil {
			return nil, nil, "the unstitched faces have no common original"
		}
		if _, exists := paired[original]; exists {
			return nil, nil, "an original face occurs more than once"
		}
		paired[original] = sp.liveFaces[i]
		if len(copied.loops) != len(original.loops) {
			return nil, nil, "an unstitched face changed its loop layout"
		}
		for li, loop := range copied.loops {
			oldLoop := original.loops[li]
			if len(loop.coedges) != len(oldLoop.coedges) || loop.outer != oldLoop.outer {
				return nil, nil, "an unstitched face changed its coedge layout"
			}
			for ci, ce := range loop.coedges {
				old := oldLoop.coedges[ci]
				if ce.forward != old.forward {
					return nil, nil, "an unstitched coedge changed direction"
				}
				if prev, exists := originalEdgeOf[ce.edge]; exists && prev != old.edge {
					return nil, nil, "a copied edge has two original identities"
				}
				originalEdgeOf[ce.edge] = old.edge
				if copiesByOriginalEdge[old.edge] == nil {
					copiesByOriginalEdge[old.edge] = map[*Edge]struct{}{}
				}
				copiesByOriginalEdge[old.edge][ce.edge] = struct{}{}
				for _, pair := range [][2]*Vertex{{ce.Start(), old.Start()}, {ce.End(), old.End()}} {
					if prev, exists := originalVertexOf[pair[0]]; exists && prev != pair[1] {
						return nil, nil, "a copied vertex has two original identities"
					}
					originalVertexOf[pair[0]] = pair[1]
				}
			}
		}
	}
	if originalBody.Kind() != BodySheet {
		return nil, nil, "the original is not a revolve sheet"
	}
	if _, ok := originalBody.payload.(revolvePayload); !ok {
		return nil, nil, "the original has no revolve chording"
	}
	originalFaces := originalBody.Faces()
	if len(originalFaces) != len(paired) {
		return nil, nil, "the unstitched set omits an original face"
	}
	for _, f := range originalFaces {
		if _, ok := paired[f]; !ok {
			return nil, nil, "the unstitched set omits an original face"
		}
	}
	classOriginal := map[int]*Vertex{}
	for v, class := range sp.plan.table.class {
		original, ok := originalVertexOf[v]
		if !ok {
			return nil, nil, "a welded vertex has no original identity"
		}
		if prev, exists := classOriginal[class]; exists && prev != original {
			return nil, nil, "a vertex class merges different original vertices"
		}
		classOriginal[class] = original
	}
	if len(originalVertexOf) != len(sp.plan.table.class) {
		return nil, nil, "an unstitched vertex is absent from the weld plan"
	}
	groupOriginal := map[int]*Edge{}
	groupCount := map[int]int{}
	for copied, gid := range sp.plan.group {
		original, ok := originalEdgeOf[copied]
		if !ok {
			return nil, nil, "a welded edge has no original identity"
		}
		if prev, exists := groupOriginal[gid]; exists && prev != original {
			return nil, nil, "a weld joins different original edges"
		}
		groupOriginal[gid] = original
		groupCount[gid]++
	}
	for gid, count := range groupCount {
		if count != 2 {
			return nil, nil, fmt.Sprintf("weld group %d has %d source edges", gid, count)
		}
	}
	for original, copies := range copiesByOriginalEdge {
		if len(original.faces) == 2 {
			if len(copies) != 2 {
				return nil, nil, "an original two-face edge lacks two copies"
			}
			gid := -1
			for copied := range copies {
				got, ok := sp.plan.group[copied]
				if !ok || (gid >= 0 && got != gid) {
					return nil, nil, "an original two-face edge was not re-welded"
				}
				gid = got
			}
			continue
		}
		if len(copies) != 1 {
			return nil, nil, "an original free edge has multiple copies"
		}
		for copied := range copies {
			if _, welded := sp.plan.group[copied]; welded {
				return nil, nil, "an original free edge was newly welded"
			}
		}
	}
	return originalBody, paired, ""
}

// tessellateStitchCurved reuses one complete revolve sheet's own chording.
// A direct copy adds no weld; a sibling round trip re-welds only original
// edges, so both paths reuse the same source mesh and its proof terms.
func tessellateStitchCurved(ctx context.Context, b *Body, sp stitchPayload, chord float64, verify Verification) (*Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	refuse := func(reason string) (*Mesh, error) {
		if len(sp.faces) == 0 || sp.faces[0] == nil {
			return nil, fmt.Errorf(`%w: %s for a stitched body with no source face`, ErrUnsupported, reason)
		}
		return nil, fmt.Errorf(`%w: %s for a stitched body's %T face`, ErrUnsupported, reason, sp.faces[0].Surface())
	}
	if len(sp.faces) == 0 || len(sp.liveFaces) != len(sp.faces) || sp.plan == nil || sp.plan.table == nil {
		return refuse("this evaluator has no complete source-to-live face pairing")
	}
	for _, f := range sp.faces {
		if f == nil {
			return refuse("this evaluator has no complete source-to-live face pairing")
		}
		switch f.Surface().(type) {
		case Plane, Cylinder, Cone, Sphere, Torus:
		default:
			return nil, fmt.Errorf(`%w: this evaluator has no chording arm for a stitched body's %T face`, ErrUnsupported, f.Surface())
		}
	}
	if sp.xform != r3.Identity() {
		return refuse("this evaluator cannot reuse source chording after stitch placement")
	}
	var sourceBody *Body
	var paired map[*Face]*Face
	if direct := sp.faces[0].body; direct != nil {
		if _, ok := direct.payload.(revolvePayload); ok {
			if direct.Kind() != BodySheet || sp.plan.groups != 0 ||
				len(sp.plan.table.class) != len(sp.plan.table.verts) {
				return refuse("this evaluator cannot reuse source chording after a stitch weld")
			}
			sourceBody = direct
			paired = make(map[*Face]*Face, len(sp.faces))
			for i, f := range sp.faces {
				if f.body != direct || sp.liveFaces[i] == nil {
					return refuse("this evaluator has no complete revolve-sheet face set")
				}
				if _, exists := paired[f]; exists {
					return refuse("this evaluator has a repeated revolve-sheet source face")
				}
				paired[f] = sp.liveFaces[i]
			}
		} else {
			var reason string
			sourceBody, paired, reason = stitchSiblingRevolveSource(b, sp)
			if reason != "" {
				return refuse(reason)
			}
		}
	}
	if sourceBody == nil || len(sourceBody.Faces()) != len(paired) {
		return refuse("this evaluator has no complete revolve-sheet source")
	}
	for _, f := range sourceBody.Faces() {
		if _, ok := paired[f]; !ok {
			return refuse("this evaluator has no complete revolve-sheet face set")
		}
	}
	rp, ok := sourceBody.payload.(revolvePayload)
	if !ok {
		return refuse("this evaluator has no chording arm for the source construction")
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	sourceMesh, err := tessellateRevolve(ctx, sourceBody, rp, chord, verify)
	if err != nil {
		return nil, err
	}
	mesh := &Mesh{
		vertices:  append([]r3.Vec(nil), sourceMesh.vertices...),
		triangles: append([][3]int(nil), sourceMesh.triangles...),
		source:    make([]*Face, len(sourceMesh.source)),
		areaSlack: sourceMesh.areaSlack,
	}
	for i, sourceFace := range sourceMesh.source {
		liveFace, ok := paired[sourceFace]
		if !ok {
			return refuse("the revolve mesh names a face outside the stitched source set")
		}
		mesh.source[i] = liveFace
		if sourceFace.reversed != liveFace.reversed {
			mesh.triangles[i][1], mesh.triangles[i][2] = mesh.triangles[i][2], mesh.triangles[i][1]
		}
	}
	for sourceFace, liveFace := range paired {
		bound, ok := sourceMesh.sourceBound(sourceFace)
		if !ok || isNonFinite(bound) {
			return refuse("the revolve mesh has no finite source-face bound")
		}
		mesh.setFaceBound(liveFace, bound)
	}
	if b.Kind() == BodySheet {
		if err := requireSheetMesh(ctx, b, mesh); err != nil {
			return nil, err
		}
		if err := requireSheetVertexLinks(ctx, mesh); err != nil {
			return nil, err
		}
		return mesh, nil
	}
	if err := requireClosedMesh(mesh); err != nil {
		return nil, fmt.Errorf(`%w: the chorded stitched boundary is not a closed mesh`, ErrUnsupported)
	}
	if err := requireVertexLinks(ctx, mesh); err != nil {
		return nil, err
	}
	if len(mesh.vertices) == 0 || meshOrientationSign(mesh.vertices, mesh.triangles, mesh.vertices[0]) <= 0 {
		return nil, fmt.Errorf(`%w: the chorded stitched boundary does not enclose a positive volume`, ErrUnsupported)
	}
	return mesh, nil
}

func tessellateStitch(ctx context.Context, b *Body, sp stitchPayload) (*Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if sp.tris == nil {
		for _, f := range b.Faces() {
			if !faceIsTetrahedronEligible(f) {
				return nil, fmt.Errorf(`%w: this evaluator has no chording arm for a stitched body's %T face`, ErrUnsupported, f.Surface())
			}
		}
		return nil, fmt.Errorf(`%w: this evaluator cannot restate this stitched body's own triangle set`, ErrUnsupported)
	}

	// Fresh slices: Mesh's accessors copy on the way out, but the held mesh
	// must not alias the payload's own arrays, or a future consumer writing
	// through one would rewrite the body's boundary.
	mesh := &Mesh{
		vertices:  append([]r3.Vec(nil), sp.verts...),
		triangles: append([][3]int(nil), sp.tris...),
		source:    append([]*Face(nil), sp.triFaces...),
	}

	budget := newWorkBudget(ctx)
	slack := 0.0
	for i, tri := range mesh.triangles {
		if err := budget.step(); err != nil {
			return nil, err
		}
		delta := 0.0
		for _, v := range tri {
			delta = max(delta, sp.vertBound[v])
		}
		mesh.setFaceBound(mesh.source[i], delta)
		if delta > 0 {
			a, bb, c := mesh.vertices[tri[0]], mesh.vertices[tri[1]], mesh.vertices[tri[2]]
			slack = absSumUpper(slack, perturbedTriangleAreaAllow(a, bb, c, delta))
		}
	}
	if err := budget.err(); err != nil {
		return nil, err
	}
	mesh.areaSlack = slack

	// A stitched body's mesh publishes an occupied-volume proof of EXACTLY
	// zero — the strongest claim this codebase makes — only for a CLOSED body
	// whose every vertex carries a proven bound of exactly zero. Every other
	// stitched body's exact tetrahedron sum proves only SIGNED volume, never
	// the occupied-volume symmetric-difference bound a boolean requires, so
	// symDiffOK stays false for it (docs/surface-design.md §14 Table D row 5)
	// and boolean.go's requireVolumeProvingPayload refuses it before this
	// mesh is even built.
	//
	// The zero claim rests on three steps, each of which must hold for every
	// triangle this mesh restates, or the claim does not:
	//
	//  1. Every held vertex IS the true boundary vertex, not merely near it.
	//     stitchZeroVertexBound (stitch.go) proves every one of b.Vertices()'s
	//     own proven bounds is exactly zero — the same gate
	//     clearance_geom.go's addStitchFaces dispatch already applies, reused
	//     verbatim rather than re-derived. That rules out both a nonzero
	//     placement delta (stitch.go's rigidRoundAllow, evalStitchContext)
	//     and a certificate-welded class bound (docs/surface-design.md §6.4's
	//     massDelta amendment) — the only two ways this evaluator ever widens
	//     a stitched vertex's bound above zero.
	//  2. The polygon triangulateStitchFaces (stitch.go) built for every face
	//     IS that face's exact boundary, never an approximation of a curved
	//     one. stitchAllTetrahedronEligible — already required for sp.tris to
	//     be non-nil, which is why this function reached this point at all —
	//     requires every face to be a Plane bounded entirely by Line3 edges.
	//     A Line3's whole geometry is its two endpoints, so the coedge-start-
	//     vertex polygon the triangulator reads drops no bulge: there is none
	//     to drop.
	//  3. Ear clipping, with Eberly's hole-bridging ahead of it for a holed
	//     face, exactly tiles that exact polygon — holes excluded once each,
	//     with no gap and no overlap. This is the identical triangulator
	//     loft's own exact restatement already relies on for its cap
	//     triangles (docs/tessellation-design.md §2's loftPayload row), and it
	//     introduces no coordinate the polygon's own vertices did not already
	//     hold: no rounding, no interpolation. checkStitchClosure's
	//     directed-edge parity leg and loftCrossingAudit's own crossing test
	//     — both already run before this mesh is built, in
	//     stitch.go's evalStitchContext — are what prove the several faces'
	//     own triangle sets close into one watertight solid with no
	//     self-contact; that is never this restatement's own concern.
	//
	// Together the three steps mean the held triangle set occupies EXACTLY
	// the same volume the denoted body does: a zero symmetric difference, not
	// merely a small one.
	mesh.volSymDiff = 0
	mesh.symDiffOK = false
	if b.Kind() == BodySolid {
		zeroBound, err := stitchZeroVertexBound(budget, b)
		if err != nil {
			return nil, err
		}
		// The three-step argument above proves each triangle set exact PER
		// LUMP; it says nothing about how the lumps relate to one another.
		// evalStitchContext's own lump-separation gate (stitch.go,
		// docs/surface-design.md Table C/R20) already refuses a nested or
		// interlocking multi-lump assembly before this body could ever
		// exist, but this reading does not lean on that: it re-asks the
		// identical axis-aligned box question directly against b's own
		// Lumps, so the zero claim stays sound on its own terms even if
		// evalStitchContext's gate were ever loosened or bypassed.
		mesh.symDiffOK = zeroBound && stitchLumpsProvenSeparate(stitchLumpFaceGroups(b))
	}

	// Every audit below restates the payload's own invariants over the
	// copied set (tessellate_loft.go's identical reasoning): Stitch's own
	// build already ran checkStitchClosure's directed-edge parity leg over
	// this exact triangle set, whether open or closed, so failing an audit
	// here can only mean a payload that never should have reached this
	// restatement.
	//
	// A BodySheet runs docs/tessellation-design.md §1.2's manifold-with-
	// boundary audit in the closed-mesh audit's place, exactly as a
	// surface-result prism or revolve sheet's own mesh does
	// (docs/surface-design.md §10): requireSheetMesh proves every free
	// directed edge attributes, face by face and chain count by chain
	// count, to the body's own recorded free Edges — the one leg Stitch's
	// own build never itself checked, since checkStitchClosure only ever
	// asked whether an edge has one or two adjacent faces, never which face
	// a mesh triangle attributes to — and requireSheetVertexLinks is its own
	// vertex-link safety net for an open boundary vertex's link, a path
	// rather than requireVertexLinks' cycle.
	if b.Kind() == BodySheet {
		if err := requireSheetMesh(ctx, b, mesh); err != nil {
			return nil, err
		}
		if err := requireSheetVertexLinks(ctx, mesh); err != nil {
			return nil, err
		}
		return mesh, nil
	}

	// requireClosedMesh's own ErrDegenerate is rewrapped as ErrUnsupported to
	// match — this evaluator's own restatement reach, never a claim the
	// body's geometry is bad (docs/tessellation-design.md §12; §1.2's own
	// note that a stitched solid mesh also runs requireVertexLinks beside
	// it).
	if err := requireClosedMesh(mesh); err != nil {
		return nil, fmt.Errorf(`%w: the stitched body's held triangle set is not a closed mesh, so it restates no boundary`, ErrUnsupported)
	}
	if err := requireVertexLinks(ctx, mesh); err != nil {
		return nil, err
	}
	return mesh, nil
}
