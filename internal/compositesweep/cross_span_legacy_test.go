package compositesweep_test

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/tessellation"
)

// legacyCrossSpanContactAudit is the all-pairs walk CrossSpanContactAudit
// replaces, kept as the reference its differential test compares against.
func legacyCrossSpanContactAudit(budget *proofbound.WorkBudget, data []revolvemesh.RevolveAuditTri,
	triangles [][3]int, spanOf []int, delta float64) error {
	if len(data) != len(triangles) || len(spanOf) != len(triangles) {
		return fmt.Errorf(`%w: the composite sweep's facet audit has inconsistent triangle records`, decaderr.ErrUnsupported)
	}
	pairs, ok := proofbound.WallChoose2(uint64(len(triangles)))
	if !ok || pairs > proofbound.MaxFacetPairTestsPerCall {
		err := fmt.Errorf(`%w: the composite sweep's facet-pair audit exceeds its fixed work ceiling`, decaderr.ErrUnsupported)
		if ok {
			return proofbound.WithWorkLimit(err, "facet_pairs", pairs, proofbound.MaxFacetPairTestsPerCall)
		}
		return err
	}
	margin := proofbound.ProductUpper(2, delta)
	for i := range triangles {
		for j := i + 1; j < len(triangles); j++ {
			if spanOf[i] == spanOf[j] {
				continue
			}
			if err := budget.Step(); err != nil {
				return err
			}
			shared, count := tessellation.SharedVertexIndices(triangles[i], triangles[j])
			if count == 0 && revolvemesh.BoxGapExceeds(data[i].Box, data[j].Box, margin) {
				continue
			}
			if err := revolvemesh.AuditRevolvePair(data, triangles, shared, count, i, j, delta); err != nil {
				return err
			}
		}
	}
	return budget.Err()
}
