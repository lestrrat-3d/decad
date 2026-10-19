package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/obliqueboss"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func crossCylinderCircleSource(body *Body, u, v r3.Vec) (prismPayload, circleSeg, bool) {
	pp, ok := body.n.Payload.(prismPayload)
	if !ok || !body.n.Solid || body.n.Kind != topo.BodySolid || pp.SurfaceResult ||
		pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		pp.Xform != r3.Identity() || pp.Frame.Origin() != (r3.Vec{}) ||
		pp.Frame.U() != u || pp.Frame.V() != v ||
		len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 ||
		!(pp.Z1 > pp.Z0) {
		return prismPayload{}, circleSeg{}, false
	}
	circle, ok := pp.Profile.Outer.Segments[0].(circleSeg)
	if !ok || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 ||
		!proofbound.FiniteVec(r3.Vec{X: circle.Center.U, Y: circle.Center.V, Z: circle.Radius.Base()}) ||
		!(circle.Radius.Base() > 0) {
		return prismPayload{}, circleSeg{}, false
	}
	return pp, circle, true
}

func crossCylinderCircleBound(body *Body, radius float64) (float64, bool) {
	bound, count := 0.0, 0
	for _, edge := range body.Edges() {
		if _, ok := edge.n.Curve.(surfacegeom.Circle3); !ok {
			continue
		}
		if !edge.n.CurveBounded || proofbound.IsNonFinite(edge.n.CurveBound) {
			return 0, false
		}
		bound = max(bound, edge.n.CurveBound)
		count++
	}
	return bound, count == 2 && bound < radius/1000
}

func recognizeCrossCylinderBore(target, tool *Body) *crossCylinderBoreRecord {
	// The canonical cardinal pose is intentionally narrow. It gives the
	// source intersection two explicit positive/negative X branches.
	tp, outer, ok := crossCylinderCircleSource(target, r3.Vec{X: 1}, r3.Vec{Y: 1})
	if !ok || outer.Center != (planePoint{}) || tp.Z0 != 0 ||
		proofbound.IsNonFinite(tp.Z1) || tp.Z1 <= 0 {
		return nil
	}
	bp, inner, ok := crossCylinderCircleSource(tool, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	if !ok || inner.Center.U != 0 {
		return nil
	}
	R, r, H, c := outer.Radius.Base(), inner.Radius.Base(), tp.Z1, inner.Center.V
	if !proofbound.FiniteVec(r3.Vec{X: R, Y: r, Z: H}) ||
		proofbound.IsNonFinite(c) ||
		R < 2 || R > 20 || r < 0.25 || r > R/4 ||
		H < 2 || H > 40 || c-r <= 0 || c+r >= H ||
		bp.Z0 >= -R-1 || bp.Z1 <= R+1 ||
		math.Abs(bp.Z0) > 100 || math.Abs(bp.Z1) > 100 {
		return nil
	}
	targetBound, ok := crossCylinderCircleBound(target, R)
	if !ok {
		return nil
	}
	toolBound, ok := crossCylinderCircleBound(tool, r)
	if !ok {
		return nil
	}
	bound := proofbound.AbsSumUpper(targetBound, toolBound)
	if !(bound < min(R, r)/1000) {
		return nil
	}
	return &crossCylinderBoreRecord{
		Target: bodyNode(target), Tool: bodyNode(tool), Radius: R, Bore: r,
		Height: H, CenterZ: c, SourceBound: bound,
	}
}

func crossBoreEdgeHasOrigins(edge *Edge, outer, bore facetGroup) bool {
	if len(edge.n.Faces) != 2 {
		return false
	}
	return (obliqueboss.HasOrigin(edge.n.Faces[0], outer) &&
		obliqueboss.HasOrigin(edge.n.Faces[1], bore)) ||
		(obliqueboss.HasOrigin(edge.n.Faces[1], outer) &&
			obliqueboss.HasOrigin(edge.n.Faces[0], bore))
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
	outer, outerOK := crossBoreFaceGroup(bodyHandle(rec.Target), prismrecord.OuterFirstSideRole)
	bore, boreOK := crossBoreFaceGroup(bodyHandle(rec.Tool), prismrecord.OuterFirstSideRole)
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
		mesh.Verts, mesh.Tris, crossbore.CapPatches(mesh.Src), loftmesh.MaxLoftAuditTriangles,
		crossbore.AuditScans); err != nil {
		return nil, true, err
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, mesh.Verts, mesh.Tris)
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
	vertexBound := make([]float64, len(mesh.Verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	dPair := facetedrecord.SourceDiameterUpper(mesh.Verts, bound)
	if proofbound.IsNonFinite(dPair) {
		return nil, true, fmt.Errorf("%w: the cross-bore chamfer has no finite diameter", ErrUnsupported)
	}
	ref := body.n.Doc.NextProducer
	mesh.Groups[4].Origins[0].Producer = ref
	result, err := bodyResult(facetedrecord.BuildWithProof(ctx, envOf(documentNode(documentHandle(body.n.Doc)), ref), facetedPayload{
		Verts: mesh.Verts, Tris: mesh.Tris, Src: mesh.Src, Groups: mesh.Groups,
		VertexBound: vertexBound, MeshBound: bound, VolSymDiff: volumeGap,
		AreaSlack: areaSlack, DPair: dPair, ContactAudited: true, Xform: r3.Identity(),
	}, audit, measurementToInternal(volume), heldVolume))
	return result, true, err
}

func crossBoreFaceGroup(body *Body, role string) (facetGroup, bool) {
	for _, face := range body.Faces() {
		for _, origin := range face.originRefs() {
			if origin.Producer == body.n.Origin.Producer && origin.Role == role {
				return facetGroup{Origins: face.originRefs(), Planar: face.isPlanar()}, true
			}
		}
	}
	return facetGroup{}, false
}

func crossBoreMeshGroups(rec *crossCylinderBoreRecord) ([]facetGroup, error) {
	groups := make([]facetGroup, 0, 5)
	for _, source := range []struct {
		body *Body
		role string
	}{
		{bodyHandle(rec.Target), provenance.RoleCapStart},
		{bodyHandle(rec.Target), provenance.RoleCapEnd},
		{bodyHandle(rec.Target), prismrecord.OuterFirstSideRole},
		{bodyHandle(rec.Tool), prismrecord.OuterFirstSideRole},
	} {
		group, ok := crossBoreFaceGroup(source.body, source.role)
		if !ok {
			return nil, fmt.Errorf("%w: a cross-bore source lost face %q", ErrUnsupported, source.role)
		}
		groups = append(groups, group)
	}
	groups = append(groups, facetGroup{Origins: []provenance.Ref{{Role: "boreMouthChamfer"}}})
	return groups, nil
}

func buildCrossBoreChamferMesh(ctx context.Context,
	rec *crossCylinderBoreRecord, setback float64) (*crossbore.Mesh, error) {
	return buildCrossBoreChamferMeshWorkers(ctx, rec, setback, -1)
}

// buildCrossBoreChamferMeshWorkers is buildCrossBoreChamferMesh with the
// station-error fan-out width given; a negative workers picks
// orderedwork.EachWorkers for the station count. The mesh does not depend on
// workers.
func buildCrossBoreChamferMeshWorkers(ctx context.Context,
	rec *crossCylinderBoreRecord, setback float64, workers int) (*crossbore.Mesh, error) {
	groups, err := crossBoreMeshGroups(rec)
	if err != nil {
		return nil, err
	}
	return crossbore.BuildMesh(ctx, rec, setback, groups, workers)
}

// crossBoreMirrorPose reports whether rec is exactly the record admission
// builds from its own two source bodies. Admission takes only the cardinal
// pose, compared without tolerance: the target circle centred on the Z axis
// in the XY frame, the tool circle centred at Y = 0 in the YZ frame, both
// unplaced. Both source walls, x²+y² = R² and y²+(z−c)² = r², are then
// exactly symmetric under y → −y and z−c → −(z−c), which is what
// crossbore.MirroredCellMaxima's mirror map needs. A pose that is symmetric
// only to rounding fails one of admission's exact comparisons.
func crossBoreMirrorPose(rec *crossCylinderBoreRecord) bool {
	again := recognizeCrossCylinderBore(bodyHandle(rec.Target), bodyHandle(rec.Tool))
	return again != nil && *again == *rec
}

// proveCrossBoreCells proves the 4,096 whole-turn cells of rec's positive
// mouth. It refuses a record that is not exactly admission's mirror-symmetric
// pose before proving any cell. It then proves the 1,025 quarter-turn base
// cells and compares every cell's held samples with its base cell's mirrored
// enclosures (crossbore.MirroredCellMaxima). Both passes run concurrently and
// fold in cell order.
func proveCrossBoreCells(ctx context.Context, rec *crossCylinderBoreRecord,
	setback float64, mesh *crossbore.Mesh) (crossbore.CellResult, error) {
	if !crossBoreMirrorPose(rec) {
		return crossbore.CellResult{}, fmt.Errorf("%w: the cross-bore record is not the exact mirror-symmetric pose",
			ErrUnsupported)
	}
	return crossbore.ProveCells(ctx, rec, setback, mesh)
}

// proveCrossBoreChamfer encloses all source-to-held boundary motion. Its
// volume proof charges the entire positive-mouth collar, where the ideal
// chamfer and mesh can differ arbitrarily, then two radial chord annuli for
// unchanged outer and bore walls. A collar wider than the actual removed
// region is deliberate; it keeps the occupied-volume argument independent
// of the exact band integral. Each source foot is at developed-cylinder
// distance d from its seam point, so its Euclidean distance is at most d;
// the ruled band lies in the resulting positive-mouth collar.
func proveCrossBoreChamfer(ctx context.Context, rec *crossCylinderBoreRecord,
	setback float64, mesh *crossbore.Mesh) (float64, float64, float64, error) {
	maximum, err := proveCrossBoreCells(ctx, rec, setback, mesh)
	if err != nil {
		return 0, 0, 0, err
	}
	return crossbore.ProveChamferBounds(rec, setback, mesh, maximum)
}
