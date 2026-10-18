package facetedrecord

import (
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

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
