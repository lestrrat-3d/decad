// Package stitchtess restates Stitch payloads as attributed mesh nodes.
package stitchtess

import (
	"context"
	"fmt"
	"iter"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/revolvetess"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/stitchweld"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// This file is docs/tessellation-design.md §2's stitchPayload restatement
// row (docs/surface-design.md §10, §14 Table D row 5): an all-planar
// stitched body's mesh, CLOSED or OPEN, is an exact restatement of the
// triangle set Stitch's own build already assembled and audited
// (stitchrecord.Build), modelled line for line on internal/loftmesh.RestateLoft's exact
// restatement. Nothing is chorded, welded, moved, or retriangulated here, so
// this path takes no chord tolerance at all — there is no chording
// component for one to bind (RestateLoft's own reasoning, restated
// for this payload).
//
// A curved or mixed stitched body carries no recorded triangle set
// (stitchrecord.Payload.Tris is nil). T10 reuses one complete revolve sheet's
// chording under surface §10.1; T11 also accepts that sheet's identity
// Unstitch siblings re-welded along their original edges (§10.2). Other
// curved sources refuse with decaderr.ErrUnsupported. An OPEN
// all-planar sheet runs docs/tessellation-design.md §1.2's manifold-with-
// boundary audit (topobuild.RequireSheetMesh) in the
// closed-mesh audit's place, exactly as a surface-result prism or revolve
// sheet's own mesh does (docs/surface-design.md §10).

// Restate restates an all-planar stitched body's own recorded triangle set
// as a mesh node, CLOSED or OPEN. Its per-face bound is the largest
// Vertex.Bound() over the vertices that face's own triangles touch, zero
// exactly when every one of them is; its areaSlack is the matching
// proofbound.PerturbedTriangleAreaAllow sum. A CLOSED body (b.Kind == topo.BodySolid)
// whose every vertex carries a proven bound of exactly zero
// (stitchweld.ZeroBound over vertexBounds) publishes a zero occupied-volume proof
// (symDiffOK == true) — see the comment at that publication below for the
// three-step argument. Every other stitched body — open, curved/mixed,
// placed, or certificate-welded — keeps symDiffOK false, so Union, Cut and
// Intersect keep refusing it (boolean.go's own requireVolumeProvingPayload
// arm), whatever the exact tetrahedron sum itself already proves about
// signed volume.
// lumpFaceGroups returns b's own lumps' constituent faces, one slice
// per lump: topobuild.SheetLumps builds exactly one shell per lump
// (docs/surface-design.md §2.2), so this is a direct read of b's own
// recorded connectivity, never a second split.
func lumpFaceGroups(b *topo.Body) [][]*topo.Face {
	lumps := b.Lumps
	groups := make([][]*topo.Face, len(lumps))
	for i, lp := range lumps {
		for _, s := range lp.Shells {
			faces := s.Faces
			groups[i] = slices.Grow(groups[i], len(faces))
			groups[i] = append(groups[i], faces...)
		}
	}
	return groups
}

// vertexBounds yields the same ordered node bounds as root Body.Vertices.
func vertexBounds(b *topo.Body) iter.Seq[float64] {
	return func(yield func(float64) bool) {
		for _, v := range topobuild.Vertices(b) {
			if !yield(v.Bound.Mag()) {
				return
			}
		}
	}
}

// Curved reuses one complete revolve sheet's own chording.
// A direct copy adds no weld; a sibling round trip re-welds only original
// edges, so both paths reuse the same source mesh and its proof terms.
func Curved(ctx context.Context, b *topo.Body, sp stitchrecord.Payload, chord float64,
	verify revolvetess.Level, publicSurface func(surfacegeom.Surface) any) (*topo.Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	refuse := func(reason string) (*topo.Mesh, error) {
		if len(sp.Faces) == 0 || sp.Faces[0] == nil {
			return nil, fmt.Errorf(`%w: %s for a stitched body with no source face`, decaderr.ErrUnsupported, reason)
		}
		return nil, fmt.Errorf(`%w: %s for a stitched body's %T face`, decaderr.ErrUnsupported, reason, publicSurface(sp.Faces[0].Surface))
	}
	if len(sp.Faces) == 0 || len(sp.LiveFaces) != len(sp.Faces) || sp.Plan == nil || sp.Plan.Table == nil {
		return refuse("this evaluator has no complete source-to-live face pairing")
	}
	for _, f := range sp.Faces {
		if f == nil {
			return refuse("this evaluator has no complete source-to-live face pairing")
		}
		switch f.Surface.(type) {
		case surfacegeom.Plane, surfacegeom.Cylinder, surfacegeom.Cone, surfacegeom.Sphere, surfacegeom.Torus:
		default:
			return nil, fmt.Errorf(`%w: this evaluator has no chording arm for a stitched body's %T face`, decaderr.ErrUnsupported, publicSurface(f.Surface))
		}
	}
	if sp.Xform != r3.Identity() {
		return refuse("this evaluator cannot reuse source chording after stitch placement")
	}
	var sourceBody *topo.Body
	var paired map[*topo.Face]*topo.Face
	if direct := sp.Faces[0].Body; direct != nil {
		switch direct.Payload.(type) {
		case revolveplan.Payload:
			if direct.Kind != topo.BodySheet || sp.Plan.Groups != 0 ||
				len(sp.Plan.Table.Classes) != len(sp.Plan.Table.Vertices) {
				return refuse("this evaluator cannot reuse source chording after a stitch weld")
			}
			sourceBody = direct
			paired = make(map[*topo.Face]*topo.Face, len(sp.Faces))
			for i, f := range sp.Faces {
				if f.Body != direct || sp.LiveFaces[i] == nil {
					return refuse("this evaluator has no complete revolve-sheet face set")
				}
				if _, exists := paired[f]; exists {
					return refuse("this evaluator has a repeated revolve-sheet source face")
				}
				paired[f] = sp.LiveFaces[i]
			}
		case stitchrecord.UnstitchPayload:
			sourceNode, matched, reason := stitchrecord.SiblingRevolveSource(b, sp)
			if reason != "" {
				return refuse(reason)
			}
			sourceBody, paired = sourceNode, matched
		default:
			return refuse("this evaluator has no chording arm for the source construction")
		}
	}
	if sourceBody == nil || len(topobuild.Faces(sourceBody)) != len(paired) {
		return refuse("this evaluator has no complete revolve-sheet source")
	}
	for _, f := range topobuild.Faces(sourceBody) {
		if _, ok := paired[f]; !ok {
			return refuse("this evaluator has no complete revolve-sheet face set")
		}
	}
	rp, ok := sourceBody.Payload.(revolveplan.Payload)
	if !ok {
		return refuse("this evaluator has no chording arm for the source construction")
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	sourceNode, err := revolvetess.TessellateRevolve(ctx, sourceBody, rp, chord, verify)
	if err != nil {
		return nil, err
	}
	mesh := &topo.Mesh{
		Vertices:  append([]r3.Vec(nil), sourceNode.Vertices...),
		Triangles: append([][3]int(nil), sourceNode.Triangles...),
		AreaSlack: sourceNode.AreaSlack,
		Source:    make([]*topo.Face, len(sourceNode.Source)),
	}
	for i, sourceFace := range sourceNode.Source {
		liveFace, ok := paired[sourceFace]
		if !ok {
			return refuse("the revolve mesh names a face outside the stitched source set")
		}
		mesh.Source[i] = liveFace
		// Today's identity-copy routes preserve this bit. Keep the winding
		// aligned if an admitted stitch ever reverses a live face.
		if sourceFace.Reversed != liveFace.Reversed {
			mesh.Triangles[i][1], mesh.Triangles[i][2] = mesh.Triangles[i][2], mesh.Triangles[i][1]
		}
	}
	for sourceFace, liveFace := range paired {
		bound, ok := sourceNode.FaceBound[sourceFace]
		if !ok || proofbound.IsNonFinite(bound) {
			return refuse("the revolve mesh has no finite source-face bound")
		}
		topobuild.SetFaceBound(mesh, liveFace, bound)
	}
	if b.Kind == topo.BodySheet {
		if err := topobuild.RequireSheetMesh(ctx, b, mesh); err != nil {
			return nil, err
		}
		return mesh, nil
	}
	if err := tessellation.RequireClosedMesh(mesh.Triangles); err != nil {
		return nil, fmt.Errorf(`%w: the chorded stitched boundary is not a closed mesh`, decaderr.ErrUnsupported)
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.Vertices), mesh.Triangles); err != nil {
		return nil, err
	}
	if len(mesh.Vertices) == 0 || tessellation.OrientationSign(mesh.Vertices, mesh.Triangles, mesh.Vertices[0]) <= 0 {
		return nil, fmt.Errorf(`%w: the chorded stitched boundary does not enclose a positive volume`, decaderr.ErrUnsupported)
	}
	return mesh, nil
}

// Restate copies and audits the held triangles of an all-planar Stitch body.
func Restate(ctx context.Context, b *topo.Body, sp stitchrecord.Payload,
	publicSurface func(surfacegeom.Surface) any) (*topo.Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if sp.Tris == nil {
		for _, f := range topobuild.Faces(b) {
			if !stitchrecord.FaceIsTetrahedronEligible(f) {
				return nil, fmt.Errorf(`%w: this evaluator has no chording arm for a stitched body's %T face`,
					decaderr.ErrUnsupported, publicSurface(f.Surface))
			}
		}
		return nil, fmt.Errorf(`%w: this evaluator cannot restate this stitched body's own triangle set`, decaderr.ErrUnsupported)
	}

	// Fresh slices: Mesh's accessors copy on the way out, but the held mesh
	// must not alias the payload's own arrays, or a future consumer writing
	// through one would rewrite the body's boundary.
	mesh := &topo.Mesh{
		Vertices:  append([]r3.Vec(nil), sp.Verts...),
		Triangles: append([][3]int(nil), sp.Tris...),
		Source:    append([]*topo.Face(nil), sp.TriFaces...),
	}

	budget := proofbound.NewWorkBudget(ctx)
	slack := 0.0
	for i, tri := range mesh.Triangles {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		delta := 0.0
		for _, v := range tri {
			delta = max(delta, sp.VertBound[v])
		}
		topobuild.SetFaceBound(mesh, mesh.Source[i], delta)
		if delta > 0 {
			a, bb, c := mesh.Vertices[tri[0]], mesh.Vertices[tri[1]], mesh.Vertices[tri[2]]
			slack = proofbound.AbsSumUpper(slack, proofbound.PerturbedTriangleAreaAllow(a, bb, c, delta))
		}
	}
	if err := budget.Err(); err != nil {
		return nil, err
	}
	mesh.AreaSlack = slack

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
	//     stitchweld.ZeroBound (internal/stitchweld/bounds.go) proves every
	//     vertex's bound is exactly zero — the same gate
	//     clearance_geom.go's addStitchFaces dispatch already applies, reused
	//     verbatim rather than re-derived. That rules out both a nonzero
	//     placement delta (internal/stitchrecord/build.go's proofbound.RigidRoundAllow, stitchrecord.Build)
	//     and a certificate-welded class bound (docs/surface-design.md §6.4's
	//     massDelta amendment) — the only two ways this evaluator ever widens
	//     a stitched vertex's bound above zero.
	//  2. The polygon stitchrecord.TriangulateFaces (internal/stitchrecord/build.go) built for every face
	//     IS that face's exact boundary, never an approximation of a curved
	//     one. stitchrecord.AllTetrahedronEligible — already required for sp.Tris to
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
	//     hold: no rounding, no interpolation. stitchrecord.CheckClosure's
	//     directed-edge parity leg and loftmesh.LoftCrossingAudit's own crossing test
	//     — both already run before this mesh is built, in
	//     stitchrecord.Build — are what prove the several faces'
	//     own triangle sets close into one watertight solid with no
	//     self-contact; that is never this restatement's own concern.
	//
	// Together the three steps mean the held triangle set occupies EXACTLY
	// the same volume the denoted body does: a zero symmetric difference, not
	// merely a small one.
	mesh.VolSymDiff = 0
	mesh.SymDiffOK = false
	if b.Kind == topo.BodySolid {
		zeroBound, err := stitchweld.ZeroBound(budget, vertexBounds(b))
		if err != nil {
			return nil, err
		}
		// The three-step argument above proves each triangle set exact PER
		// LUMP; it says nothing about how the lumps relate to one another.
		// stitchrecord.Build's own lump-separation gate (internal/stitchrecord/build.go,
		// docs/surface-design.md Table C/R20) already refuses a nested or
		// interlocking multi-lump assembly before this body could ever
		// exist, but this reading does not lean on that: it re-asks the
		// identical axis-aligned box question directly against b's own
		// Lumps, so the zero claim stays sound on its own terms even if
		// stitchrecord.Build's gate were ever loosened or bypassed.
		mesh.SymDiffOK = zeroBound && stitchrecord.FaceLumpsProvenSeparate(lumpFaceGroups(b))
	}

	// Every audit below restates the payload's own invariants over the
	// copied set (RestateLoft's identical reasoning): Stitch's own
	// build already ran stitchrecord.CheckClosure's directed-edge parity leg over
	// this exact triangle set, whether open or closed, so failing an audit
	// here can only mean a payload that never should have reached this
	// restatement.
	//
	// A BodySheet runs docs/tessellation-design.md §1.2's manifold-with-
	// boundary audit in the closed-mesh audit's place, exactly as a
	// surface-result prism or revolve sheet's own mesh does
	// (docs/surface-design.md §10): topobuild.RequireSheetMesh proves every free
	// directed edge attributes, face by face and chain count by chain
	// count, to the body's own recorded free Edges — the one leg Stitch's
	// own build never itself checked, since stitchrecord.CheckClosure only ever
	// asked whether an edge has one or two adjacent faces, never which face
	// a mesh triangle attributes to — and tessellation.RequireSheetVertexLinks is its own
	// vertex-link safety net for an open boundary vertex's link, a path
	// rather than tessellation.RequireVertexLinks' cycle.
	if b.Kind == topo.BodySheet {
		if err := topobuild.RequireSheetMesh(ctx, b, mesh); err != nil {
			return nil, err
		}
		return mesh, nil
	}

	// tessellation.RequireClosedMesh's own Degenerate refusal is rewrapped as ErrUnsupported to
	// match — this evaluator's own restatement reach, never a claim the
	// body's geometry is bad (docs/tessellation-design.md §12; §1.2's own
	// note that a stitched solid mesh also runs tessellation.RequireVertexLinks beside
	// it).
	if err := tessellation.RequireClosedMesh(mesh.Triangles); err != nil {
		return nil, fmt.Errorf(`%w: the stitched body's held triangle set is not a closed mesh, so it restates no boundary`, decaderr.ErrUnsupported)
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.Vertices), mesh.Triangles); err != nil {
		return nil, err
	}
	return mesh, nil
}
