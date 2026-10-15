package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// pointPlaneSimilarityRecord retains the exact source cut maps and the order
// of their later rigid and positive uniform images. The source slice is never
// reconstructed from the rounded cap chords.
type pointPlaneSimilarityRecord struct {
	slice *pointPlaneSliceRecord
	steps []pointPlaneSimilarityStep
}

type pointPlaneSimilarityStep struct {
	motion          r3.Transform
	anchor          r3.Vec
	factor          float64 // zero means a rigid step
	meshBound       float64
	capDisplacement float64
}

// ScaledAt returns the positive uniform image of an authenticated point-loft
// Split slab, exact line/circular solid prism, line-only stack or stack-backed
// BRep, complete solid Revolve, or audited faceted Boolean result about anchor,
// then retires b. A nil context or a nil b is ErrDegenerate.
// Source roles and certified boundary and occupied-volume bounds follow the
// image. A refused or canceled call leaves the document unchanged.
func ScaledAt(ctx context.Context, b *Body, anchor r3.Vec, factor units.Value) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf("%w: a nil context cannot control a scale", ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf("%w: the body belongs to no document", ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if factor.Kind() != units.Dimensionless {
		return nil, fmt.Errorf("%w: the scale factor must be dimensionless", ErrUnitKind)
	}
	s := factor.Base()
	if proofbound.IsNonFinite(s) || !proofbound.FiniteVec(anchor) {
		return nil, fmt.Errorf("%w: scale factor and anchor must be finite", ErrNotFinite)
	}
	if s <= 0 {
		return nil, fmt.Errorf("%w: the scale factor must be positive", ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	fp, ok := b.n.Payload.(facetedPayload)
	if !ok {
		var err error
		switch source := b.n.Payload.(type) {
		case prismPayload:
			fp, err = prismScaleSource(ctx, b, source, s)
		case revolvePayload:
			fp, err = revolveScaleSource(ctx, b, source, s)
		case stackedPrismPayload:
			fp, err = stackedScaleSource(ctx, b, source)
		case brepPayload:
			fp, err = brepScaleSource(ctx, b, source)
		default:
			return nil, fmt.Errorf("%w: this body has no certified similarity source", ErrUnsupported)
		}
		if err != nil {
			return nil, err
		}
	}
	pointSlab := fp.pointPlane != nil && fp.pointPlane.base.pointSection != nil
	booleanSource := fp.booleanSimilaritySource && b.Kind() == BodySolid && b.IsSolid()
	if (!pointSlab && !fp.similaritySource && !booleanSource) || !fp.Xform.IsValid() ||
		len(fp.Verts) == 0 || len(fp.VertexBound) != len(fp.Verts) {
		return nil, fmt.Errorf("%w: this body has no certified similarity source", ErrUnsupported)
	}
	if booleanSource && !certifiedBooleanSimilaritySource(fp, len(b.Faces())) {
		return nil, fmt.Errorf("%w: the Boolean similarity source has no complete proof", ErrUnsupported)
	}
	if fp.pointSimilarity != nil && fp.pointSimilarity.slice != fp.pointPlane {
		return nil, fmt.Errorf("%w: the slab similarity source does not match its Split source", ErrUnsupported)
	}
	for _, group := range fp.groups {
		if group.denoted != nil {
			return nil, fmt.Errorf("%w: a denoted surface has no similarity proof", ErrUnsupported)
		}
		if _, ok := group.surface.(surfacegeom.Plane); group.surface != nil && !ok {
			return nil, fmt.Errorf("%w: a retained curved surface has no similarity proof", ErrUnsupported)
		}
	}
	budget := proofbound.NewWorkBudget(ctx)
	mesh, err := facetproof.ScaleMesh(ctx, budget, fp.Verts, fp.Tris, fp.VertexBound,
		fp.VolSymDiff, fp.AreaSlack, anchor, s)
	if err != nil {
		return nil, err
	}
	next := fp
	next.smoothRectSection = nil
	next.Verts, next.Tris = mesh.Verts, mesh.Tris
	next.VertexBound, next.MeshBound = mesh.VertexBound, mesh.MeshBound
	next.VolSymDiff, next.AreaSlack = mesh.VolSymDiff, mesh.AreaSlack
	// Scaling the old diameter does not include the held vertices' new
	// rounding or an embedding adjustment. The centroid's yardstick must
	// cover both the true body and this rounded held body.
	next.DPair = max(proofbound.ProductUpper(fp.DPair, s),
		facetedSourceDiameterUpper(next.Verts, next.MeshBound))
	next.Xform = r3.Identity()
	next.pointPlaneSourceEligible = false
	next.axialSectionJoin = false
	next.prismPatternJoin = nil
	next.obliqueBoss = nil
	next.prismUnionFillet = nil
	next.crossCylinderBore = nil
	next.loftCylinderJoin = nil
	next.loftCylinderBore = nil
	next.loftMirrorJoin = nil
	next.loftMirrorCopy = nil
	next.mirrorCapGroups = nil
	next.loftMirrorCylinderJoin = nil
	next.loftMirrorCylinderBore = nil
	next.revolveLoftJoin = nil
	next.revolveLoftSector = nil
	next.LoftFacesPrimitive = false
	next.loftFacesCap = nil
	next.loftFacesToe = nil
	next.loftFacesTrim = nil
	next.copyLineage = nil
	next.spiralTools = nil
	if pointSlab {
		next.pointPlaneCapDisplacement = proofbound.AbsSumUpper(
			proofbound.ProductUpper(fp.pointPlaneCapDisplacement, s), mesh.Round)
	}
	next.pointSection, next.pointCone = nil, nil
	next.exactSourceVerts, next.exactSourceTris = nil, nil
	next.lowerSupport = nil
	next.groups = append([]facetGroup(nil), fp.groups...)
	for i := range next.groups {
		group := &next.groups[i]
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if group.hasCentroid {
			group.centroid, err = similarCapCentroid(group.centroid, anchor, s)
			if err != nil {
				return nil, err
			}
		}
		if plane, ok := group.surface.(surfacegeom.Plane); ok {
			origin := anchor.Add(plane.Frame.Origin().Sub(anchor).Scale(s))
			frame, frameErr := r3.NewFrame(origin, plane.Frame.U(), plane.Frame.V())
			if frameErr != nil {
				return nil, fmt.Errorf("%w: a scaled cap frame is not representable: %s", ErrUnsupported, frameErr)
			}
			group.surface = surfacegeom.Plane{Frame: frame}
			// NewFrame may round the held axes again. The exact similar plane
			// keeps its old normal; two unit normals differ by at most 2.
			group.normalBound = 2
		}
	}
	if proofbound.IsNonFinite(next.DPair) || proofbound.IsNonFinite(next.pointPlaneCapDisplacement) {
		return nil, fmt.Errorf("%w: the scaled source proof is not finite", ErrUnsupported)
	}
	if pointSlab {
		record := pointPlaneSimilarityRecord{slice: fp.pointPlane}
		if fp.pointSimilarity == nil {
			if fp.Xform != r3.Identity() {
				record.steps = append(record.steps, pointPlaneSimilarityStep{motion: fp.Xform,
					meshBound: fp.MeshBound, capDisplacement: fp.pointPlaneCapDisplacement})
			}
		} else {
			record.steps = append(record.steps, fp.pointSimilarity.steps...)
		}
		record.steps = append(record.steps, pointPlaneSimilarityStep{anchor: anchor, factor: s,
			meshBound: next.MeshBound, capDisplacement: next.pointPlaneCapDisplacement})
		next.pointSimilarity = &record
	}
	ref := d.nextProducerID()
	scaled, err := buildFacetedBody(ctx, d, ref, next)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(scaled, b)
	return scaled, nil
}

// certifiedBooleanSimilaritySource checks the private Boolean result's
// geometry, proof terms, and source-group mapping before ScaleMesh consumes it.
func certifiedBooleanSimilaritySource(fp facetedPayload, faceCount int) bool {
	if len(fp.Tris) == 0 || len(fp.Tris) != len(fp.Src) ||
		len(fp.Tris) != len(fp.FaceOf) || len(fp.groups) == 0 || faceCount == 0 ||
		fp.MeshBound < 0 || fp.VolSymDiff < 0 || fp.AreaSlack < 0 || fp.DPair <= 0 ||
		proofbound.IsNonFinite(fp.MeshBound) || proofbound.IsNonFinite(fp.VolSymDiff) ||
		proofbound.IsNonFinite(fp.AreaSlack) || proofbound.IsNonFinite(fp.DPair) {
		return false
	}
	for _, beta := range fp.VertexBound {
		if beta < 0 || proofbound.IsNonFinite(beta) {
			return false
		}
	}
	if fp.MeshBound < facetproof.FacetBoundMax(fp.Tris, fp.VertexBound) {
		return false
	}
	for _, source := range fp.Src {
		if source < 0 || source >= len(fp.groups) {
			return false
		}
	}
	for _, face := range fp.FaceOf {
		if face < 0 || face >= faceCount {
			return false
		}
	}
	for _, group := range fp.groups {
		if len(group.origins) == 0 {
			return false
		}
	}
	return true
}

func similarCapCentroid(source VecMeasurement, anchor r3.Vec, factor float64) (VecMeasurement, error) {
	p := [3]float64{source.Value.X, source.Value.Y, source.Value.Z}
	a := [3]float64{anchor.X, anchor.Y, anchor.Z}
	var exact [3]*big.Rat
	for i := range exact {
		exact[i] = pointPlaneAdd(ratOfPointPlane(a[i]),
			pointPlaneMul(ratOfPointPlane(factor), pointPlaneSub(ratOfPointPlane(p[i]), ratOfPointPlane(a[i]))))
	}
	value, round, err := pointPlaneRound(exact)
	if err != nil {
		return VecMeasurement{}, err
	}
	bound := proofbound.AbsSumUpper(proofbound.ProductUpper(source.Bound.Base(), factor), round)
	if proofbound.IsNonFinite(bound) {
		return VecMeasurement{}, fmt.Errorf("%w: the scaled cap centroid has no finite bound", ErrUnsupported)
	}
	return VecMeasurement{Value: value, Exactness: exactnessOf(bound), Bound: units.Millimeters(bound)}, nil
}
