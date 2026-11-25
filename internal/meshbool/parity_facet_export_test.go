package meshbool

import "github.com/lestrrat-3d/decad/internal/proof"

// ParityFacetOutside reports whether classifyFacet, the exact per-facet step
// of the parity scan, classifies facet ti of pm as strictly outside the query
// p along AxisRays[ray].
func ParityFacetOutside(pm *ParityMesh, p proof.Xpt, ray, ti int) bool {
	r := AxisRays[ray]
	pa := NewXP2(RatCoordOf(p, r.U), RatCoordOf(p, r.V))
	return pm.classifyFacet(p, pa, r.Axis, r.U, r.V, r.Dir, ti) == facetOutside
}
