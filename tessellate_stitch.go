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
// (stitchPayload.tris is nil) and refuses here with ErrUnsupported: this
// evaluator has not wired a chording arm for it, and it stays staged past
// this increment (docs/surface-design.md §14 Table D row 5). An OPEN
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
		mesh.symDiffOK = zeroBound
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
