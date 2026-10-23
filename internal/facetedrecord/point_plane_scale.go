package facetedrecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// ScaleImage returns the payload of body's positive uniform image about
// anchor by factor s, from the certified similarity source fp. It refuses a
// source that is neither an authenticated point-loft Split slab, a similarity
// source, nor a Boolean result whose proof is complete, and a source that
// retains a denoted or curved surface. Source roles are dropped; the held mesh
// and its bounds follow the image.
func ScaleImage(ctx context.Context, body *topo.Body, fp Payload, anchor r3.Vec, s float64) (Payload, error) {
	pointSlab := fp.PointPlane != nil && fp.PointPlane.Base.PointSection != nil
	booleanSource := fp.BooleanSimilaritySource && body.Kind == topo.BodySolid && body.Solid
	if (!pointSlab && !fp.SimilaritySource && !booleanSource) || !fp.Xform.IsValid() ||
		len(fp.Verts) == 0 || len(fp.VertexBound) != len(fp.Verts) {
		return Payload{}, fmt.Errorf("%w: this body has no certified similarity source", decaderr.ErrUnsupported)
	}
	if booleanSource && !CertifiedBooleanSimilaritySource(fp, len(topobuild.Faces(body))) {
		return Payload{}, fmt.Errorf("%w: the Boolean similarity source has no complete proof", decaderr.ErrUnsupported)
	}
	if fp.PointSimilarity != nil && fp.PointSimilarity.Slice != fp.PointPlane {
		return Payload{}, fmt.Errorf("%w: the slab similarity source does not match its Split source", decaderr.ErrUnsupported)
	}
	for _, group := range fp.Groups {
		if group.Denoted != nil && !group.Retained {
			return Payload{}, fmt.Errorf("%w: a denoted surface has no similarity proof", decaderr.ErrUnsupported)
		}
		if _, ok := group.Surface.(surfacegeom.Plane); group.Surface != nil && !ok && !group.Retained {
			return Payload{}, fmt.Errorf("%w: a retained curved surface has no similarity proof", decaderr.ErrUnsupported)
		}
	}
	budget := proofbound.NewWorkBudget(ctx)
	mesh, err := facetproof.ScaleMesh(ctx, budget, fp.Verts, fp.Tris, fp.VertexBound,
		fp.VolSymDiff, fp.AreaSlack, anchor, s)
	if err != nil {
		return Payload{}, err
	}
	next := fp
	next.SmoothRectSection = nil
	next.SmoothHomotheticSection = nil
	next.Verts, next.Tris = mesh.Verts, mesh.Tris
	next.VertexBound, next.MeshBound = mesh.VertexBound, mesh.MeshBound
	next.VolSymDiff, next.AreaSlack = mesh.VolSymDiff, mesh.AreaSlack
	// Scaling the old diameter does not include the held vertices' new
	// rounding or an embedding adjustment. The centroid's yardstick must
	// cover both the true body and this rounded held body.
	next.DPair = max(proofbound.ProductUpper(fp.DPair, s), SourceDiameterUpper(next.Verts, next.MeshBound))
	next.Xform = r3.Identity()
	next.PointPlaneSourceEligible = false
	next.AxialSectionJoin = false
	next.PrismPatternJoin = nil
	next.ObliqueBoss = nil
	next.PrismUnionFillet = nil
	next.CrossCylinderBore = nil
	next.LoftCylinderJoin = nil
	next.LoftCylinderBore = nil
	next.LoftMirrorJoin = nil
	next.LoftMirrorCopy = nil
	next.MirrorCapGroups = nil
	next.LoftMirrorCylinderJoin = nil
	next.LoftMirrorCylinderBore = nil
	next.RevolveLoftJoin = nil
	next.RevolveLoftSector = nil
	next.LoftFacesPrimitive = false
	next.LoftFacesCap = nil
	next.LoftFacesToe = nil
	next.LoftFacesTrim = nil
	next.CopyLineage = nil
	next.PointSection, next.PointCone = nil, nil
	next.ExactSourceVerts, next.ExactSourceTris = nil, nil
	next.LowerSupport = nil
	next.Groups = append([]Group(nil), fp.Groups...)
	for i := range next.Groups {
		group := &next.Groups[i]
		if err := budget.Step(); err != nil {
			return Payload{}, err
		}
		if group.HasCentroid {
			centroid, centroidErr := SimilarCapCentroid(group.Centroid, anchor, s)
			group.Centroid, err = centroid, centroidErr
			if err != nil {
				return Payload{}, err
			}
		}
		if _, plane := group.Surface.(surfacegeom.Plane); group.Retained && (!plane || group.Denoted != nil) {
			// A scaled retained curved or denoted face reads Faceted: its tag
			// has no similarity image here.
			group.Surface, group.Denoted, group.Retained, group.NormalBound = nil, nil, false, 0
		}
		if plane, ok := group.Surface.(surfacegeom.Plane); ok {
			origin := anchor.Add(plane.Frame.Origin().Sub(anchor).Scale(s))
			frame, frameErr := r3.NewFrame(origin, plane.Frame.U(), plane.Frame.V())
			if frameErr != nil {
				return Payload{}, fmt.Errorf("%w: a scaled cap frame is not representable: %s", decaderr.ErrUnsupported, frameErr)
			}
			group.Surface = surfacegeom.Plane{Frame: frame}
			// NewFrame may round the held axes again. The exact similar plane
			// keeps its old normal; two unit normals differ by at most 2.
			group.NormalBound = 2
			group.CapRecord, group.AxialDelta, group.HasAxialDelta = nil, 0, false
		}
	}
	if pointSlab {
		CarryPointPlaneScale(fp, &next, anchor, s, mesh.Round)
	}
	if proofbound.IsNonFinite(next.DPair) || proofbound.IsNonFinite(next.PointPlaneCapDisplacement) {
		return Payload{}, fmt.Errorf("%w: the scaled source proof is not finite", decaderr.ErrUnsupported)
	}
	return next, nil
}

// CarryPointPlaneScale records the new image of an authenticated Split slab.
// The caller has already scaled and audited the held mesh and its source groups.
func CarryPointPlaneScale(source Payload, scaled *Payload, anchor r3.Vec, factor, imageRound float64) {
	scaled.PointPlaneCapDisplacement = proofbound.AbsSumUpper(
		proofbound.ProductUpper(source.PointPlaneCapDisplacement, factor), imageRound)
	record := PointPlaneSimilarityRecord{Slice: source.PointPlane}
	if source.PointSimilarity == nil {
		if source.Xform != r3.Identity() {
			record.Steps = append(record.Steps, PointPlaneSimilarityStep{Motion: source.Xform,
				MeshBound: source.MeshBound, CapDisplacement: source.PointPlaneCapDisplacement})
		}
	} else {
		record.Steps = append(record.Steps, source.PointSimilarity.Steps...)
	}
	record.Steps = append(record.Steps, PointPlaneSimilarityStep{Anchor: anchor, Factor: factor,
		MeshBound: scaled.MeshBound, CapDisplacement: scaled.PointPlaneCapDisplacement})
	scaled.PointSimilarity = &record
}
