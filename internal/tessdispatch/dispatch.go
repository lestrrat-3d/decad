// Package tessdispatch routes one body to the tessellator of its payload
// (docs/evaluator-design.md §9, docs/tessellation-design.md). It returns the
// mesh node; root wraps it in its handle, caches it and withholds the proofs a
// lower verification level does not ask for.
package tessdispatch

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/alledgechamfermesh"
	"github.com/lestrrat-3d/decad/internal/brepbandmesh"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/compositemesh"
	"github.com/lestrrat-3d/decad/internal/cuptess"
	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/patchmesh"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismtess"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/revolvetess"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/stitchtess"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
)

// Verify is the verification level a tessellation is asked for, with the
// values of root's Verification.
type Verify int

const (
	// None asks for a mesh built to be drawn or exported.
	None Verify = iota
	// Boundary adds the boundary proof.
	Boundary
	// All adds the occupied-volume proofs.
	All
)

// Options carry what root alone can supply: the names of a surface or curve in
// the public API's vocabulary, for the builders that print one into a refusal,
// and the allocator the builders mint topology through.
type Options struct {
	Surface func(surfacegeom.Surface) any
	Curve   func(surfacegeom.Curve) any
	Alloc   topo.Alloc
}

// Body dispatches one body to its payload's own tessellator.
// verify reaches only the paths that would otherwise COMPUTE a proof the level
// withholds, such as prism, brep, cup, revolve, cap-loop chamfer and draft.
// A restatement path (faceted, loft, all-planar stitch) copies its proof terms
// off the payload at no cost and publishes them unconditionally;
// tessellateContext withholds them afterwards.
func Body(ctx context.Context, b *topo.Body, chord float64, verify Verify, opts Options) (*topo.Mesh, error) {
	formatSurface := opts.Surface
	if p, ok := b.Payload.(breprecord.AllEdgeChamferPayload); ok {
		return alledgechamfermesh.Build(ctx, b, p, chord, verify >= Boundary, verify >= All)
	}
	if fp, ok := b.Payload.(facetedrecord.Payload); ok {
		return facetedrecord.Restate(ctx, b, fp, chord)
	}
	if cp, ok := b.Payload.(cupwall.Payload); ok {
		return cuptess.Tessellate(ctx, b, cp.View(), chord, verify >= All)
	}
	if sp, ok := b.Payload.(stackedrecord.Payload); ok {
		return stackedrecord.Tessellate(ctx, b, sp, chord, verify >= All)
	}
	if bp, ok := b.Payload.(breprecord.Payload); ok {
		return brepbandmesh.Tessellate(ctx, b, bp, chord, verify >= All)
	}
	if lp, ok := b.Payload.(loftrecord.Payload); ok {
		// The loft path exactly restates the payload's complete set for a
		// solid or its recorded wall range for a sheet, with no chording
		// (internal/loftmesh.RestateLoft's own doc comment owns why).
		return loftrecord.Tessellate(ctx, b, lp)
	}
	if tp, ok := b.Payload.(twistsweep.Payload); ok {
		if chord < tp.Delta {
			return nil, fmt.Errorf(`%w: the twisted sweep's held boundary is %g mm from its true wall; %g mm is finer`, decaderr.ErrUnsupported, tp.Delta, chord)
		}
		if tp.Mesh != nil {
			mesh := *tp.Mesh
			return &mesh, nil
		}
		node, err := loftrecord.Tessellate(ctx, b, tp.Held, tp.SourceRoles)
		if err != nil {
			return nil, err
		}
		for _, face := range node.Source {
			topobuild.SetFaceBound(node, face, tp.Delta)
		}
		node.Bound = tp.Delta
		node.AreaSlack = tp.AreaSlack
		node.VolSymDiff = tp.VolSymDiff
		node.SymDiffOK = true
		return node, nil
	}
	if tp, ok := b.Payload.(twistsweep.CompositePayload); ok {
		if chord < tp.Delta {
			return nil, fmt.Errorf(`%w: the composite twist's held boundary is %g mm from its true wall; %g mm is finer`,
				decaderr.ErrUnsupported, tp.Delta, chord)
		}
		mesh := *tp.Mesh
		return &mesh, nil
	}
	if pp, ok := b.Payload.(patchchain.Payload); ok {
		return patchmesh.Profile(ctx, b, pp, chord, patchmesh.Level(verify))
	}
	if rp, ok := b.Payload.(revolveplan.Payload); ok {
		return revolvetess.TessellateRevolve(ctx, b, rp, chord, revolvetess.Level(verify))
	}
	if cbp, ok := b.Payload.(capblend.Payload); ok {
		return capBlendMesh(ctx, b, cbp, chord, verify)
	}
	if dp, ok := b.Payload.(draftrecord.Payload); ok {
		return draftrecord.Tessellate(ctx, b, dp, chord, verify >= All)
	}
	if dp, ok := b.Payload.(draftrecord.TwoSidedPayload); ok {
		return draftrecord.TessellateTwoSided(ctx, b, topo.Env{Doc: b.Doc, Ref: b.Origin.Producer, Alloc: opts.Alloc}, dp, chord, verify >= All)
	}
	if sp, ok := b.Payload.(stitchrecord.Payload); ok {
		if sp.Tris == nil {
			return stitchtess.Curved(ctx, b, sp, chord, revolvetess.Level(verify), formatSurface)
		}
		// The all-planar stitch restatement takes no chord tolerance.
		return stitchtess.Restate(ctx, b, sp, formatSurface)
	}
	if pp, ok := b.Payload.(patchchain.BodyPayload); ok {
		return bodyPatchMesh(ctx, b, pp, chord, verify, opts)
	}
	if cp, ok := b.Payload.(chainrecord.Payload); ok {
		// The chain-fed ribbon path takes no chord tolerance either, on the
		// identical reasoning: every wall is already an exact planar quad,
		// so there is no chording decision for a tolerance to bind
		// (internal/chainrecord.Tessellate's own doc comment owns why).
		return chainrecord.Tessellate(ctx, b, cp, formatSurface)
	}
	if sp, ok := b.Payload.(sweeprecord.ChainPayload); ok {
		if len(sp.Spans) != 0 {
			return revolvetess.TessellateChainSweep(ctx, b, sp, chord, revolvetess.Level(verify))
		}
		if sp.Arc {
			return revolvetess.TessellateChainArcSweep(ctx, b, sp, chord, revolvetess.Level(verify))
		}
		// the chain Sweep's one-span straight reduction builds through the
		// identical chainrecord.Build a plain chain Extrude does, over
		// the chainrecord.Payload it wraps unchanged (internal/sweeprecord's
		// FinishChain), so it reuses that same restatement rather
		// than a second one keyed on the wrapper type.
		return chainrecord.Tessellate(ctx, b, sp.Chain, formatSurface)
	}
	if mp, ok := b.Payload.(sweepmitre.Payload); ok {
		// docs/sweep-design.md Table DM row DM2: an exact restatement of the
		// held triangles, refused below the payload's own delta.
		return mitredSweepMesh(ctx, b, mp, chord)
	}
	if cp, ok := b.Payload.(coilshell.Payload); ok {
		// docs/helix-design.md Table CD row CD2: an exact restatement of the
		// held triangles, refused below the payload's own delta.
		return coilshell.Tessellate(ctx, b, cp, chord, verify >= All)
	}
	if sp, ok := b.Payload.(sweeprecord.Payload); ok {
		// One-span sweeps reuse the prism or revolve mesh of their exact
		// analytic reduction. A sheet carries no occupied-volume proof.
		if len(sp.Spans) != 0 {
			return compositeSweepMesh(ctx, b, sp, chord, verify, opts.Alloc)
		}
		if sp.Arc {
			return revolvetess.TessellateRevolveWithRoles(
				ctx, b, sp.Revolve, chord, revolvetess.Level(verify), revolvetess.ArcRole(sp.ReverseArcCaps))
		}
		return prismMesh(ctx, b, sp.Prism, sweepWallRole, chord, verify, nil, 0)
	}
	pp, ok := b.Payload.(prismrecord.Payload)
	if !ok {
		// Chording is per payload kind. Name both the staged kind and the
		// implemented set so the refusal cannot misstate evaluator reach.
		return nil, fmt.Errorf(`%w: tessellation does not support payload %s; supported payload classes are prism, patch, stacked prism, brep, chain-fed prism, sweep, mitred sweep, coil, revolve, cup, loft, cap-loop chamfer, draft (tapered extrude), stitch, and faceted`, decaderr.ErrUnsupported, reportvocab.PayloadTypeName(b.Payload))
	}
	return prismMesh(ctx, b, pp, PrismWallRole, chord, verify, nil, 0)
}

// PrismWallRole names the wall face prismrecord.Build minted for segment seg
// of section loop loop: Table B's side(i,j).
func PrismWallRole(loop, seg int) string { return fmt.Sprintf("side(%d,%d)", loop, seg) }

// sweepWallRole names the same wall on a one-span straight sweep, whose
// reduction rewrote every side(i,j) to carry the path-span index ahead of
// them (internal/sweeprecord's prefixSpanZeroRole): side(0,i,j).
func sweepWallRole(loop, seg int) string { return fmt.Sprintf("side(0,%d,%d)", loop, seg) }

// prismMesh builds the prism mesh of b with the wall roles and count floor the
// caller names.
func prismMesh(ctx context.Context, b *topo.Body, pp prismrecord.Payload,
	wallRole func(loop, seg int) string, chord float64, verify Verify,
	countFloor func(loop, seg int) int, freeformTarget float64) (*topo.Mesh, error) {
	return prismtess.Build(ctx, b, pp, wallRole, chord, verify >= All, countFloor, freeformTarget)
}

// requireMeshAudit dispatches docs/tessellation-design.md §1's mandatory
// mesh audit by body kind. A solid takes the closed-mesh audit; a sheet takes
// topobuild.RequireSheetMesh's free-boundary and vertex-link audits. Angular
// Revolve and chain Sweep builders call that node audit directly.
func requireMeshAudit(ctx context.Context, sheet bool, b *topo.Body, m *topo.Mesh) error {
	if sheet {
		return topobuild.RequireSheetMesh(ctx, b, m)
	}
	return topobuild.RequireClosedMesh(m)
}

// bodyPatchMesh binds source handles and public refusal names to the
// PatchEdges node builder. The circular source mesh is certified first at
// Boundary even when the requested result level is VerifyNone.
func bodyPatchMesh(ctx context.Context, b *topo.Body, pp patchchain.BodyPayload,
	chord float64, verify Verify, opts Options,
) (*topo.Mesh, error) {
	var node *topo.Mesh
	var err error
	if prism, ok := patchchain.CircularPrismSource(pp); ok {
		source, sourceErr := prismMesh(ctx, pp.Faces[0].Body, prism,
			PrismWallRole, chord, Boundary, nil, 0)
		if sourceErr != nil {
			return nil, sourceErr
		}
		node, err = patchmesh.CircularPrism(ctx, b, pp, prism, source, chord, patchmesh.Level(verify))
	} else {
		node, err = patchmesh.Planar(ctx, b, pp, patchmesh.Level(verify),
			opts.Surface, opts.Curve)
	}
	if err != nil {
		return nil, err
	}
	return node, nil
}

// capBlendMesh adapts the node build (capblend.TessellateMesh) to
// public handles and runs the Loft occupied-volume proof, which reads the
// root Body and Mesh.
func capBlendMesh(ctx context.Context, b *topo.Body, cbp capblend.Payload, chord float64, verify Verify) (*topo.Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	mesh := new(topo.Mesh)
	lms, motion, err := capblend.TessellateMesh(ctx, b, cbp, chord, verify >= All, mesh)
	if err != nil {
		return nil, err
	}
	if verify >= All && cbp.LoftSource != nil {
		if err := capblend.ProveLoftBandVolume(ctx, mesh, b, cbp, lms, motion); err != nil {
			return nil, err
		}
	}
	return mesh, nil
}

// compositeSweepMesh assembles one indexed profile grid across the
// reduced spans. The station adapter admits recorded lines and circular walks
// and fitted splines whose span meshes expose the same profile parameter at
// each join. It maps those samples by profile index, never spatial proximity.
func compositeSweepMesh(ctx context.Context, b *topo.Body, sp sweeprecord.Payload, chord float64, verify Verify, alloc topo.Alloc) (*topo.Mesh, error) {
	counts, profileSamples, freeformTarget, err := compositemesh.PlanProfileCounts(ctx, sp, chord, revolvetess.ResolveRevolveWithLimit)
	if err != nil {
		return nil, err
	}
	byRole := map[string]*topo.Face{}
	for _, face := range topobuild.Faces(b) {
		for _, origin := range face.Origins {
			byRole[origin.Role] = face
		}
	}
	mesh := &topo.Mesh{}
	spanWork := sweeprecord.NewCompositeWork(sp.Prism.Profile)
	budget := proofbound.NewWorkBudget(ctx)
	assembly := compositemesh.NewAssembly(mesh, byRole, len(sp.Spans), profileSamples,
		sp.ClosedComposite, sp.SurfaceResult, freeformTarget, budget,
		revolvetess.ResolveRevolveWithLimit, verify >= Boundary, verify >= All)
	for i := range sp.Spans {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		span := sp.Spans[i]
		part, err := span.Build(ctx, topo.Env{Doc: b.Doc, Ref: b.Origin.Producer, Alloc: alloc}, spanWork)
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		// span.Build restores path cap names for topology pairing. The reduced
		// revolve tessellator needs its own cap names while it makes its proof.
		if span.Arc && span.ReverseArcCaps {
			sweeprecord.RestoreArcCapRoles(part, true)
		}
		var piece *topo.Mesh
		if span.Arc {
			floor, floorErr := compositemesh.RevolveFloor(ctx, span.Revolve, counts, freeformTarget, revolvetess.ResolveRevolveWithLimit)
			if floorErr != nil {
				return nil, fmt.Errorf(`sweep path span %d: %w`, i, floorErr)
			}
			var node *topo.Mesh
			node, err = revolvetess.TessellateRevolveWithCountFloor(
				ctx, part, span.Revolve, chord, revolvetess.Level(verify), nil, floor, freeformTarget)
			if err == nil {
				piece = node
			}
		} else {
			piece, err = prismMesh(ctx, part, span.Prism, PrismWallRole, chord, verify,
				func(loop, seg int) int { return counts[compositemesh.StationKey{Loop: loop, Segment: seg}] }, freeformTarget)
		}
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		vertexBounds, err := topobuild.VertexBounds(piece)
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		if err := assembly.Append(ctx, i, span, piece, vertexBounds); err != nil {
			return nil, err
		}
	}
	if err := assembly.BeforeBodyAudit(); err != nil {
		return nil, err
	}
	if err := requireMeshAudit(ctx, sp.SurfaceResult, b, mesh); err != nil {
		return nil, err
	}
	if err := assembly.AfterBodyAudit(ctx); err != nil {
		return nil, err
	}
	return mesh, nil
}

// mitredSweepMesh is Table DM row DM2's exact restatement: the held
// triangles of Table BM, publishing each vertex's own rounding gap as its
// per-vertex bound and every face's source bound as the largest corner gap
// over that face's triangles (docs/faceted-vertex-bounds-design.md §2.1):
// every true face is the exact polygon on the rationals, so each held
// triangle's true piece is the affine triangle on its corners' rationals. A
// tolerance below delta asks for a mesh closer to the body than its held
// vertices are, which no restatement can give (docs/tessellation-design.md
// §7's rule). The
// occupied-volume proof is the swept-volume allowance at delta over the
// perturbed area, because the held set is the exact body with every vertex
// moved by at most delta; the boundary proof is the build's own crossing
// audit, so the mesh needs no facet-contact audit of its own.
func mitredSweepMesh(ctx context.Context, b *topo.Body, mp sweepmitre.Payload, chord float64) (*topo.Mesh, error) {
	if chord < mp.Delta {
		return nil, fmt.Errorf(`%w: a mitred sweep restates its held vertices, which sit up to %g mm from the body; a tolerance of %g mm is finer than that`, decaderr.ErrUnsupported, mp.Delta, chord)
	}
	faceOfRole := map[string]*topo.Face{}
	for _, f := range topobuild.Faces(b) {
		for _, o := range f.Origins {
			faceOfRole[o.Role] = f
		}
	}
	src := make([]*topo.Face, len(mp.Tris))
	for k, slot := range mp.TriFace {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		f, ok := faceOfRole[mp.FaceRoles[slot]]
		if !ok {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, decaderr.ErrDegenerate, mp.FaceRoles[slot])
		}
		src[k] = f
	}
	if err := tessellation.RequireClosedMesh(mp.Tris); err != nil {
		return nil, fmt.Errorf(`%w: the mitred sweep's held triangle set is not a closed mesh`, decaderr.ErrUnsupported)
	}
	mesh := &topo.Mesh{
		Vertices:  append([]r3.Vec(nil), mp.Verts...),
		Triangles: append([][3]int(nil), mp.Tris...),
		Source:    src,
	}
	topobuild.SetVertexBounds(mesh, mp.VertexBound)
	mesh.Bound = mp.Delta
	slack := 0.0
	for _, t := range mp.Tris {
		slack = proofbound.AbsSumUpper(slack, proofbound.PerturbedTriangleAreaAllow(mp.Verts[t[0]], mp.Verts[t[1]], mp.Verts[t[2]], mp.Delta))
	}
	areaUpper, err := proofbound.PerturbedAreaUpperContext(ctx, mp.Verts, mp.Tris, mp.Delta)
	if err != nil {
		return nil, err
	}
	volSymDiff := proofbound.SweptVolumeAllow(mp.Delta, areaUpper)
	if !measurement.FiniteValues(slack, volSymDiff) {
		return nil, fmt.Errorf(`%w: the mitred sweep states no finite proof for a required mesh bound`, decaderr.ErrUnsupported)
	}
	mesh.AreaSlack = slack
	mesh.VolSymDiff = volSymDiff
	mesh.SymDiffOK = true
	return mesh, nil
}
