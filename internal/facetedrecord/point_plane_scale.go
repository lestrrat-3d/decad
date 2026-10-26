package facetedrecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// ScaleImage returns the payload of body's positive uniform image about
// anchor by factor s, from the certified similarity source fp. A source is an
// authenticated point-loft Split slab, a marked similarity source, a marked
// Boolean result whose proof fields are complete, or an unmarked faceted
// solid whose proof fields are complete and whose held mesh passes
// massmoment.AuditMesh now. Every other source refuses.
// A retained Plane, Cylinder, Sphere or Torus tag keeps its tag at ScaleTag's image; a
// retained Cone, any denoted tag and any unretained curved tag read Faceted.
// Source records are dropped; the held mesh and its bounds follow the image.
func ScaleImage(ctx context.Context, body *topo.Body, fp Payload, anchor r3.Vec, s float64) (Payload, error) {
	next, _, err := ScaleImageWithSummary(ctx, body, fp, anchor, s)
	return next, err
}

// ScaleImageWithSummary is ScaleImage with actual-use evidence for the new
// image's accepted positive-reach repair. No source warning list is inherited.
func ScaleImageWithSummary(ctx context.Context, body *topo.Body, fp Payload, anchor r3.Vec,
	s float64) (Payload, *meshbool.EmbeddingReach, error) {
	pointSlab := fp.PointPlane != nil && fp.PointPlane.Base.PointSection != nil
	solid := body.Kind == topo.BodySolid && body.Solid
	booleanSource := fp.BooleanSimilaritySource && solid
	// An unmarked faceted solid has no similarity mark, but its held mesh
	// carries the same proof fields every faceted reading trusts. The mesh
	// audit below supplies the embedding that ScaleMesh assumes.
	facetedSource := !pointSlab && !fp.SimilaritySource && !fp.BooleanSimilaritySource && solid
	if (!pointSlab && !fp.SimilaritySource && !booleanSource && !facetedSource) || !fp.Xform.IsValid() ||
		len(fp.Verts) == 0 || len(fp.VertexBound) != len(fp.Verts) {
		return Payload{}, nil, fmt.Errorf("%w: this body has no certified similarity source", decaderr.ErrUnsupported)
	}
	if booleanSource && !CertifiedFacetedSimilaritySource(fp, len(topobuild.Faces(body))) {
		return Payload{}, nil, fmt.Errorf("%w: the Boolean similarity source has no complete proof", decaderr.ErrUnsupported)
	}
	if facetedSource {
		if !CertifiedFacetedSimilaritySource(fp, len(topobuild.Faces(body))) {
			return Payload{}, nil, fmt.Errorf("%w: the faceted similarity source has no complete proof", decaderr.ErrUnsupported)
		}
		if err := massmoment.AuditMesh(ctx, fp.Verts, fp.Tris, fp.ContactAudited); err != nil {
			return Payload{}, nil, fmt.Errorf("the faceted similarity source failed its mesh audit: %w", err)
		}
	}
	if fp.PointSimilarity != nil && fp.PointSimilarity.Slice != fp.PointPlane {
		return Payload{}, nil, fmt.Errorf("%w: the slab similarity source does not match its Split source", decaderr.ErrUnsupported)
	}
	budget := proofbound.NewWorkBudget(ctx)
	mesh, err := facetproof.ScaleMesh(ctx, budget, fp.Verts, fp.Tris, fp.VertexBound,
		fp.VolSymDiff, fp.AreaSlack, anchor, s)
	if err != nil {
		return Payload{}, nil, err
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
	next.ExactCopyMoments = nil
	next.PointSection, next.PointCone = nil, nil
	next.ExactSourceVerts, next.ExactSourceTris = nil, nil
	next.LowerSupport = nil
	next.Groups = append([]Group(nil), fp.Groups...)
	for i := range next.Groups {
		group := &next.Groups[i]
		if err := budget.Step(); err != nil {
			return Payload{}, nil, err
		}
		if group.HasCentroid {
			centroid, centroidErr := SimilarCapCentroid(group.Centroid, anchor, s)
			group.Centroid, err = centroid, centroidErr
			if err != nil {
				return Payload{}, nil, err
			}
		}
		if _, plane := group.Surface.(surfacegeom.Plane); !group.Retained &&
			(group.Denoted != nil || group.Surface != nil && !plane) {
			// An unretained curved or denoted tag has no similarity image
			// here. Dropping it to Faceted claims less than the source did.
			group.Surface, group.Denoted, group.NormalBound, group.Reversed = nil, nil, 0, false
		}
		if group.Retained && group.Denoted == nil {
			// A uniform scale keeps every normal, so a retained Plane,
			// Cylinder, Sphere or Torus keeps its tag at the image of ScaleTag.
			// The group's own departure carries over (an angle survives the
			// scale) and the image adds only its rounding.
			if moved, add, stated := ScaleTag(group.Surface, anchor, s); stated {
				group.Surface = moved
				group.NormalBound = min(2, proofbound.AbsSumUpper(group.NormalBound, add))
				group.CapRecord, group.AxialDelta, group.HasAxialDelta = nil, 0, false
				continue
			}
		}
		if _, plane := group.Surface.(surfacegeom.Plane); group.Retained && (!plane || group.Denoted != nil) {
			// A scaled retained Cone, denoted face, or tag with no stated
			// bound reads Faceted: its tag has no similarity image here.
			group.Surface, group.Denoted, group.Retained, group.NormalBound = nil, nil, false, 0
		}
		if plane, ok := group.Surface.(surfacegeom.Plane); ok {
			origin := anchor.Add(plane.Frame.Origin().Sub(anchor).Scale(s))
			frame, frameErr := r3.NewFrame(origin, plane.Frame.U(), plane.Frame.V())
			if frameErr != nil {
				return Payload{}, nil, fmt.Errorf("%w: a scaled cap frame is not representable: %s", decaderr.ErrUnsupported, frameErr)
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
	} else {
		// Only a Split slab carries its rational cut maps through a scale.
		next.PointPlane, next.PointSimilarity, next.PointPlaneCapDisplacement = nil, nil, 0
	}
	if proofbound.IsNonFinite(next.DPair) || proofbound.IsNonFinite(next.PointPlaneCapDisplacement) {
		return Payload{}, nil, fmt.Errorf("%w: the scaled source proof is not finite", decaderr.ErrUnsupported)
	}
	return next, mesh.EmbeddingReach, nil
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
