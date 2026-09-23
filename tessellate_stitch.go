package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/r3"
)

// This file is docs/tessellation-design.md §2's stitchPayload restatement
// row (docs/surface-design.md §10, §14 Table D row 5): a CLOSED, all-planar
// stitched body's mesh is an exact restatement of the triangle set Stitch's
// own build already assembled and audited (stitch.go's evalStitchContext),
// modelled line for line on tessellate_loft.go's exact restatement. Nothing
// is chorded, welded, moved, or retriangulated here, so this path takes no
// chord tolerance at all — there is no chording component for one to bind
// (tessellate_loft.go's own reasoning, restated for this payload).
//
// A curved, mixed, or OPEN stitched body carries no recorded triangle set
// (stitchPayload.tris is nil) and refuses here with ErrUnsupported: this
// evaluator has not wired a chording arm for either, and both stay staged
// past this increment (docs/surface-design.md §14 Table D row 5).

// tessellateStitch restates a CLOSED, all-planar stitched body's own
// recorded triangle set as a Mesh. Its per-face bound is the largest
// Vertex.Bound() over the vertices that face's own triangles touch, zero
// exactly when every one of them is; its areaSlack is the matching
// perturbedTriangleAreaAllow sum. It publishes no occupied-volume proof —
// symDiffOK stays false — so Union, Cut and Intersect keep refusing a
// stitched operand (boolean.go's own requireVolumeProvingPayload arm) until
// a later increment states that proof, whatever the exact tetrahedron sum
// itself already proves about signed volume.
func tessellateStitch(ctx context.Context, b *Body, sp stitchPayload) (*Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if sp.tris == nil {
		if b.Kind() == BodySheet {
			return nil, fmt.Errorf(`%w: tessellating an open stitched sheet is staged for a later increment`, ErrUnsupported)
		}
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

	// A stitched body's mesh publishes no occupied-volume proof in this
	// increment (docs/surface-design.md §14 Table D row 5): the exact
	// tetrahedron sum proves SIGNED volume, never the occupied-volume
	// symmetric-difference bound a boolean requires. symDiffOK stays false
	// permanently for now, and boolean.go's requireVolumeProvingPayload
	// refuses a stitched operand before this mesh is even built.
	mesh.volSymDiff = 0
	mesh.symDiffOK = false

	// Both audits are the payload's own invariants restated over the copied
	// set (tessellate_loft.go's identical reasoning): Stitch's own build
	// already ran checkStitchClosure and, on the curved path, its own
	// vertex-link audit, so failing either here can only mean a payload
	// that never should have reached this restatement. requireClosedMesh's
	// own ErrDegenerate is rewrapped as ErrUnsupported to match — this
	// evaluator's own restatement reach, never a claim the body's geometry
	// is bad (docs/tessellation-design.md §12; §1.2's own note that a
	// stitched solid mesh also runs requireVertexLinks beside it).
	if err := requireClosedMesh(mesh); err != nil {
		return nil, fmt.Errorf(`%w: the stitched body's held triangle set is not a closed mesh, so it restates no boundary`, ErrUnsupported)
	}
	if err := requireVertexLinks(ctx, mesh); err != nil {
		return nil, err
	}
	return mesh, nil
}
