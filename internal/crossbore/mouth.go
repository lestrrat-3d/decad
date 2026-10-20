package crossbore

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/obliqueboss"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// CylinderCircleSource reads a body as one unplaced solid cylinder on a cardinal
// frame: the prism payload and its single counter-clockwise circle.
func CylinderCircleSource(body *topo.Body, u, v r3.Vec) (prismrecord.Payload, sectionrecord.CircleSeg, bool) {
	pp, ok := body.Payload.(prismrecord.Payload)
	if !ok || !body.Solid || body.Kind != topo.BodySolid || pp.SurfaceResult ||
		pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		pp.Xform != r3.Identity() || pp.Frame.Origin() != (r3.Vec{}) ||
		pp.Frame.U() != u || pp.Frame.V() != v ||
		len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 ||
		!(pp.Z1 > pp.Z0) {
		return prismrecord.Payload{}, sectionrecord.CircleSeg{}, false
	}
	circle, ok := pp.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 ||
		!proofbound.FiniteVec(r3.Vec{X: circle.Center.U, Y: circle.Center.V, Z: circle.Radius.Base()}) ||
		!(circle.Radius.Base() > 0) {
		return prismrecord.Payload{}, sectionrecord.CircleSeg{}, false
	}
	return pp, circle, true
}

// CylinderCircleBound is the largest curve bound over the body's two circle
// edges, and whether there are exactly two with a bound below radius/1000.
func CylinderCircleBound(body *topo.Body, radius float64) (float64, bool) {
	bound, count := 0.0, 0
	for _, edge := range topobuild.Edges(body) {
		if _, ok := edge.Curve.(surfacegeom.Circle3); !ok {
			continue
		}
		if !edge.CurveBounded || proofbound.IsNonFinite(edge.CurveBound) {
			return 0, false
		}
		bound = max(bound, edge.CurveBound)
		count++
	}
	return bound, count == 2 && bound < radius/1000
}

// RecognizeBore recognizes a perpendicular cylindrical Cut in the canonical
// cardinal pose and returns the source record, or nil for any other pair.
func RecognizeBore(target, tool *topo.Body) *facetedrecord.CrossCylinderBoreRecord {
	// The canonical cardinal pose is intentionally narrow. It gives the
	// source intersection two explicit positive/negative X branches.
	tp, outer, ok := CylinderCircleSource(target, r3.Vec{X: 1}, r3.Vec{Y: 1})
	if !ok || outer.Center != (sectionrecord.Point2{}) || tp.Z0 != 0 ||
		proofbound.IsNonFinite(tp.Z1) || tp.Z1 <= 0 {
		return nil
	}
	bp, inner, ok := CylinderCircleSource(tool, r3.Vec{Y: 1}, r3.Vec{Z: 1})
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
	targetBound, ok := CylinderCircleBound(target, R)
	if !ok {
		return nil
	}
	toolBound, ok := CylinderCircleBound(tool, r)
	if !ok {
		return nil
	}
	bound := proofbound.AbsSumUpper(targetBound, toolBound)
	if !(bound < min(R, r)/1000) {
		return nil
	}
	return &facetedrecord.CrossCylinderBoreRecord{
		Target: target, Tool: tool, Radius: R, Bore: r,
		Height: H, CenterZ: c, SourceBound: bound,
	}
}

func edgeHasOrigins(edge *topo.Edge, outer, bore facetedrecord.Group) bool {
	if len(edge.Faces) != 2 {
		return false
	}
	return (obliqueboss.HasOrigin(edge.Faces[0], outer) &&
		obliqueboss.HasOrigin(edge.Faces[1], bore)) ||
		(obliqueboss.HasOrigin(edge.Faces[1], outer) &&
			obliqueboss.HasOrigin(edge.Faces[0], bore))
}

// MouthChamfer accepts one complete positive-X mouth
// edge of a source-authenticated perpendicular cylindrical Cut. The selected
// faceted edge chooses a source branch; it never supplies the seam or feet.
func MouthChamfer(ctx context.Context, env topo.Env, body *topo.Body,
	fp facetedrecord.Payload, edges []*topo.Edge, setback, setbackDelta float64,
	asymmetric bool) (*topo.Body, bool, error) {
	rec := fp.CrossCylinderBore
	if rec == nil {
		return nil, false, nil
	}
	if fp.Xform != r3.Identity() || len(edges) != 1 ||
		edges[0].Start != edges[0].End || len(topobuild.Edges(body)) != 4 {
		return nil, true, fmt.Errorf("%w: the cross-bore chamfer needs one complete unplaced mouth loop", decaderr.ErrUnsupported)
	}
	if _, faceted := edges[0].Curve.(surfacegeom.FacetedCurve); !faceted && !edges[0].RetainedCurve {
		return nil, true, fmt.Errorf("%w: the selected cross-bore mouth has no faceted edge witness", decaderr.ErrUnsupported)
	}
	outer, outerOK := FaceGroup(rec.Target, prismrecord.OuterFirstSideRole)
	bore, boreOK := FaceGroup(rec.Tool, prismrecord.OuterFirstSideRole)
	if !outerOK || !boreOK || !edgeHasOrigins(edges[0], outer, bore) ||
		!(edges[0].Start.Position.X > rec.Radius/2) {
		return nil, true, fmt.Errorf("%w: the selected edge is not the positive cross-bore mouth", decaderr.ErrUnsupported)
	}
	if asymmetric || setbackDelta != 0 || !(setback > 0) ||
		proofbound.IsNonFinite(setback) || setback > rec.Bore/2 ||
		rec.CenterZ-rec.Bore-setback <= 0 ||
		rec.CenterZ+rec.Bore+setback >= rec.Height {
		return nil, true, fmt.Errorf("%w: the cross-bore setback exceeds its proven clearance", decaderr.ErrUnsupported)
	}
	mesh, err := BuildChamferMesh(ctx, rec, setback)
	if err != nil {
		return nil, true, err
	}
	bound, volumeGap, areaSlack, err := proveChamfer(ctx, rec, setback, mesh)
	if err != nil {
		return nil, true, err
	}
	// Each cap fan is proposed as a projected XY patch. A cap whose proof
	// fails keeps all its pairs in the exact pairwise audit, and cap-to-cap,
	// cap-to-wall and every other pair is always classified exactly.
	if err := loftmesh.LoftCrossingAuditBoundedProjectedPatchesStreaming(proofbound.NewPollWorkBudget(ctx),
		mesh.Verts, mesh.Tris, CapPatches(mesh.Src), loftmesh.MaxLoftAuditTriangles,
		AuditScans); err != nil {
		return nil, true, err
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, mesh.Verts, mesh.Tris)
	if err != nil {
		return nil, true, err
	}
	volume, volumeBound, heldVolume, err := booleaneval.VolumeReading(ctx, audit, volumeGap)
	if err != nil {
		return nil, true, err
	}
	if heldVolume.Cmp(proofarith.FloatRat(volumeGap)) <= 0 {
		return nil, true, fmt.Errorf("%w: the cross-bore chamfer has no proven positive volume", decaderr.ErrUnsupported)
	}
	vertexBound := make([]float64, len(mesh.Verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	dPair := facetedrecord.SourceDiameterUpper(mesh.Verts, bound)
	if proofbound.IsNonFinite(dPair) {
		return nil, true, fmt.Errorf("%w: the cross-bore chamfer has no finite diameter", decaderr.ErrUnsupported)
	}
	mesh.Groups[4].Origins[0].Producer = env.Ref
	result, err := facetedrecord.BuildWithProof(ctx, env, facetedrecord.Payload{
		Verts: mesh.Verts, Tris: mesh.Tris, Src: mesh.Src, Groups: mesh.Groups,
		VertexBound: vertexBound, MeshBound: bound, VolSymDiff: volumeGap,
		AreaSlack: areaSlack, DPair: dPair, ContactAudited: true, Xform: r3.Identity(),
	}, audit, measurement.Measurement{Value: units.CubicMillimeters(volume),
		Exactness: measurement.ExactnessOf(volumeBound), Bound: units.CubicMillimeters(volumeBound)}, heldVolume)
	return result, true, err
}

// FaceGroup is the facet group of the body's face stamped with role by the
// body's own producer.
func FaceGroup(body *topo.Body, role string) (facetedrecord.Group, bool) {
	for _, face := range topobuild.Faces(body) {
		for _, origin := range face.Origins {
			if origin.Producer == body.Origin.Producer && origin.Role == role {
				return facetedrecord.Group{Origins: slices.Clone(face.Origins), Planar: face.Surface.Kind() == surfacegeom.KindPlane || face.HeldPlanar}, true
			}
		}
	}
	return facetedrecord.Group{}, false
}

func meshGroups(rec *facetedrecord.CrossCylinderBoreRecord) ([]facetedrecord.Group, error) {
	groups := make([]facetedrecord.Group, 0, 5)
	for _, source := range []struct {
		body *topo.Body
		role string
	}{
		{rec.Target, provenance.RoleCapStart},
		{rec.Target, provenance.RoleCapEnd},
		{rec.Target, prismrecord.OuterFirstSideRole},
		{rec.Tool, prismrecord.OuterFirstSideRole},
	} {
		group, ok := FaceGroup(source.body, source.role)
		if !ok {
			return nil, fmt.Errorf("%w: a cross-bore source lost face %q", decaderr.ErrUnsupported, source.role)
		}
		groups = append(groups, group)
	}
	groups = append(groups, facetedrecord.Group{Origins: []provenance.Ref{{Role: "boreMouthChamfer"}}})
	return groups, nil
}

func BuildChamferMesh(ctx context.Context,
	rec *facetedrecord.CrossCylinderBoreRecord, setback float64) (*Mesh, error) {
	return BuildChamferMeshWorkers(ctx, rec, setback, -1)
}

// BuildChamferMeshWorkers is BuildChamferMesh with the
// station-error fan-out width given; a negative workers picks
// orderedwork.EachWorkers for the station count. The mesh does not depend on
// workers.
func BuildChamferMeshWorkers(ctx context.Context,
	rec *facetedrecord.CrossCylinderBoreRecord, setback float64, workers int) (*Mesh, error) {
	groups, err := meshGroups(rec)
	if err != nil {
		return nil, err
	}
	return BuildMesh(ctx, rec, setback, groups, workers)
}

// mirrorPose reports whether rec is exactly the record admission
// builds from its own two source bodies. Admission takes only the cardinal
// pose, compared without tolerance: the target circle centred on the Z axis
// in the XY frame, the tool circle centred at Y = 0 in the YZ frame, both
// unplaced. Both source walls, x²+y² = R² and y²+(z−c)² = r², are then
// exactly symmetric under y → −y and z−c → −(z−c), which is what
// MirroredCellMaxima's mirror map needs. A pose that is symmetric
// only to rounding fails one of admission's exact comparisons.
func mirrorPose(rec *facetedrecord.CrossCylinderBoreRecord) bool {
	again := RecognizeBore(rec.Target, rec.Tool)
	return again != nil && *again == *rec
}

// proveCells proves the 4,096 whole-turn cells of rec's positive
// mouth. It refuses a record that is not exactly admission's mirror-symmetric
// pose before proving any cell. It then proves the 1,025 quarter-turn base
// cells and compares every cell's held samples with its base cell's mirrored
// enclosures (MirroredCellMaxima). Both passes run concurrently and
// fold in cell order.
func proveCells(ctx context.Context, rec *facetedrecord.CrossCylinderBoreRecord,
	setback float64, mesh *Mesh) (CellResult, error) {
	if !mirrorPose(rec) {
		return CellResult{}, fmt.Errorf("%w: the cross-bore record is not the exact mirror-symmetric pose",
			decaderr.ErrUnsupported)
	}
	return ProveCells(ctx, rec, setback, mesh)
}

// proveChamfer encloses all source-to-held boundary motion. Its
// volume proof charges the entire positive-mouth collar, where the ideal
// chamfer and mesh can differ arbitrarily, then two radial chord annuli for
// unchanged outer and bore walls. A collar wider than the actual removed
// region is deliberate; it keeps the occupied-volume argument independent
// of the exact band integral. Each source foot is at developed-cylinder
// distance d from its seam point, so its Euclidean distance is at most d;
// the ruled band lies in the resulting positive-mouth collar.
func proveChamfer(ctx context.Context, rec *facetedrecord.CrossCylinderBoreRecord,
	setback float64, mesh *Mesh) (float64, float64, float64, error) {
	maximum, err := proveCells(ctx, rec, setback, mesh)
	if err != nil {
		return 0, 0, 0, err
	}
	return ProveChamferBounds(rec, setback, mesh, maximum)
}
