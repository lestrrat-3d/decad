package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

func crossBoreEdgeHasOrigins(edge *Edge, outer, bore facetGroup) bool {
	if len(edge.n.Faces) != 2 {
		return false
	}
	return (obliqueBossHasOrigin(edge.n.Faces[0], outer) &&
		obliqueBossHasOrigin(edge.n.Faces[1], bore)) ||
		(obliqueBossHasOrigin(edge.n.Faces[1], outer) &&
			obliqueBossHasOrigin(edge.n.Faces[0], bore))
}

// tryCrossCylinderBoreMouthChamfer accepts one complete positive-X mouth
// edge of a source-authenticated perpendicular cylindrical Cut. The selected
// faceted edge chooses a source branch; it never supplies the seam or feet.
func tryCrossCylinderBoreMouthChamfer(ctx context.Context, body *Body,
	fp facetedPayload, edges []*Edge, setback, setbackDelta float64,
	asym *asymmetricChamfer) (*Body, bool, error) {
	rec := fp.CrossCylinderBore
	if rec == nil {
		return nil, false, nil
	}
	if fp.Xform != r3.Identity() || len(edges) != 1 ||
		edges[0].n.Start != edges[0].n.End || len(body.Edges()) != 4 {
		return nil, true, fmt.Errorf("%w: the cross-bore chamfer needs one complete unplaced mouth loop", ErrUnsupported)
	}
	if _, ok := edges[0].n.Curve.(surfacegeom.FacetedCurve); !ok {
		return nil, true, fmt.Errorf("%w: the selected cross-bore mouth has no faceted edge witness", ErrUnsupported)
	}
	outer, outerOK := crossBoreFaceGroup(rec.Target, prismrecord.OuterFirstSideRole)
	bore, boreOK := crossBoreFaceGroup(rec.Tool, prismrecord.OuterFirstSideRole)
	if !outerOK || !boreOK || !crossBoreEdgeHasOrigins(edges[0], outer, bore) ||
		!(edges[0].n.Start.Position.X > rec.Radius/2) {
		return nil, true, fmt.Errorf("%w: the selected edge is not the positive cross-bore mouth", ErrUnsupported)
	}
	if asym != nil || setbackDelta != 0 || !(setback > 0) ||
		proofbound.IsNonFinite(setback) || setback > rec.Bore/2 ||
		rec.CenterZ-rec.Bore-setback <= 0 ||
		rec.CenterZ+rec.Bore+setback >= rec.Height {
		return nil, true, fmt.Errorf("%w: the cross-bore setback exceeds its proven clearance", ErrUnsupported)
	}
	mesh, err := buildCrossBoreChamferMesh(ctx, rec, setback)
	if err != nil {
		return nil, true, err
	}
	bound, volumeGap, areaSlack, err := proveCrossBoreChamfer(ctx, rec, setback, mesh)
	if err != nil {
		return nil, true, err
	}
	// Each cap fan is proposed as a projected XY patch. A cap whose proof
	// fails keeps all its pairs in the exact pairwise audit, and cap-to-cap,
	// cap-to-wall and every other pair is always classified exactly.
	if err := loftmesh.LoftCrossingAuditBoundedProjectedPatchesStreaming(proofbound.NewPollWorkBudget(ctx),
		mesh.verts, mesh.tris, crossBoreCapPatches(mesh.src), loftmesh.MaxLoftAuditTriangles,
		crossBoreAuditScans); err != nil {
		return nil, true, err
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, mesh.verts, mesh.tris)
	if err != nil {
		return nil, true, err
	}
	volume, heldVolume, err := meshVolumeMeasurement(ctx, audit, volumeGap)
	if err != nil {
		return nil, true, err
	}
	if heldVolume.Cmp(proofarith.FloatRat(volumeGap)) <= 0 {
		return nil, true, fmt.Errorf("%w: the cross-bore chamfer has no proven positive volume", ErrUnsupported)
	}
	vertexBound := make([]float64, len(mesh.verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	dPair := facetedSourceDiameterUpper(mesh.verts, bound)
	if proofbound.IsNonFinite(dPair) {
		return nil, true, fmt.Errorf("%w: the cross-bore chamfer has no finite diameter", ErrUnsupported)
	}
	ref := body.n.Doc.NextProducer
	mesh.groups[4].Origins[0].Producer = ref
	result, err := buildFacetedBodyWithProof(ctx, documentHandle(body.n.Doc), ref, facetedPayload{
		Verts: mesh.verts, Tris: mesh.tris, Src: mesh.src, Groups: mesh.groups,
		VertexBound: vertexBound, MeshBound: bound, VolSymDiff: volumeGap,
		AreaSlack: areaSlack, DPair: dPair, ContactAudited: true, Xform: r3.Identity(),
	}, audit, volume, heldVolume)
	return result, true, err
}
