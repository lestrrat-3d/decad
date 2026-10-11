package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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
// Split slab about anchor and retires the receiver. Its cap roles, source cut
// maps, and certified boundary and occupied-volume bounds follow the image.
// The operation refuses bodies for which this evaluator has no similarity
// proof. A refused or canceled call leaves the document unchanged.
func (b *Body) ScaledAt(ctx context.Context, anchor r3.Vec, factor units.Value) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf("%w: a nil context cannot control a scale", ErrDegenerate)
	}
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf("%w: the body belongs to no document", ErrDegenerate)
	}
	d := b.doc
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
	fp, ok := b.payload.(facetedPayload)
	if !ok || fp.pointPlane == nil || fp.pointPlane.base.pointSection == nil || !fp.xform.IsValid() ||
		len(fp.verts) == 0 || len(fp.vertexBound) != len(fp.verts) {
		return nil, fmt.Errorf("%w: this body has no certified point-loft Split slab to scale", ErrUnsupported)
	}
	if fp.pointSimilarity != nil && fp.pointSimilarity.slice != fp.pointPlane {
		return nil, fmt.Errorf("%w: the slab similarity source does not match its Split source", ErrUnsupported)
	}
	for _, group := range fp.groups {
		if group.denoted != nil {
			return nil, fmt.Errorf("%w: a denoted surface has no similarity proof", ErrUnsupported)
		}
		if _, ok := group.surface.(Plane); group.surface != nil && !ok {
			return nil, fmt.Errorf("%w: a retained curved surface has no similarity proof", ErrUnsupported)
		}
	}
	budget := proofbound.NewWorkBudget(ctx)
	mesh, err := facetproof.ScaleMesh(ctx, budget, fp.verts, fp.tris, fp.vertexBound,
		fp.volSymDiff, fp.areaSlack, anchor, s)
	if err != nil {
		return nil, err
	}
	next := fp
	next.verts, next.tris = mesh.Verts, mesh.Tris
	next.vertexBound, next.meshBound = mesh.VertexBound, mesh.MeshBound
	next.volSymDiff, next.areaSlack = mesh.VolSymDiff, mesh.AreaSlack
	next.dPair = proofbound.ProductUpper(fp.dPair, s)
	next.xform = r3.Identity()
	next.pointPlaneSourceEligible = false
	next.pointPlaneCapDisplacement = proofbound.AbsSumUpper(
		proofbound.ProductUpper(fp.pointPlaneCapDisplacement, s), mesh.Round)
	next.pointSection, next.pointCone = nil, nil
	next.exactSourceVerts, next.exactSourceTris = nil, nil
	next.lowerSupport = nil
	next.bevelJoin, next.bevelTooth = nil, nil
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
		if plane, ok := group.surface.(Plane); ok {
			origin := anchor.Add(plane.Frame.Origin().Sub(anchor).Scale(s))
			frame, frameErr := r3.NewFrame(origin, plane.Frame.U(), plane.Frame.V())
			if frameErr != nil {
				return nil, fmt.Errorf("%w: a scaled cap frame is not representable: %s", ErrUnsupported, frameErr)
			}
			group.surface = Plane{Frame: frame}
			// NewFrame may round the held axes again. The exact similar plane
			// keeps its old normal; two unit normals differ by at most 2.
			group.normalBound = 2
		}
	}
	if proofbound.IsNonFinite(next.dPair) || proofbound.IsNonFinite(next.pointPlaneCapDisplacement) {
		return nil, fmt.Errorf("%w: the scaled source proof is not finite", ErrUnsupported)
	}
	record := pointPlaneSimilarityRecord{slice: fp.pointPlane}
	if fp.pointSimilarity == nil {
		if fp.xform != r3.Identity() {
			record.steps = append(record.steps, pointPlaneSimilarityStep{motion: fp.xform,
				meshBound: fp.meshBound, capDisplacement: fp.pointPlaneCapDisplacement})
		}
	} else {
		record.steps = append(record.steps, fp.pointSimilarity.steps...)
	}
	record.steps = append(record.steps, pointPlaneSimilarityStep{anchor: anchor, factor: s,
		meshBound: next.meshBound, capDisplacement: next.pointPlaneCapDisplacement})
	next.pointSimilarity = &record
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
