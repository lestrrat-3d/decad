package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// pointFarCapCentroid lifts the authenticated profile's area centroid through
// its recorded plane. Its local bound covers the true Sketch region, not the
// triangulated far cap. A missing source proof leaves the loft usable.
func pointFarCapCentroid(profile profileRecord, plane planeRecord) (VecMeasurement, bool) {
	local, err := profile.Centroid()
	if err != nil || !proofbound.FiniteVec(local.Value) ||
		proofbound.IsNonFinite(local.Bound.Base()) || local.Bound.Base() < 0 {
		return VecMeasurement{}, false
	}
	origin := [3]float64{plane.Origin.X, plane.Origin.Y, plane.Origin.Z}
	u := [3]float64{plane.U.X, plane.U.Y, plane.U.Z}
	v := [3]float64{plane.V.X, plane.V.Y, plane.V.Z}
	var exact [3]*big.Rat
	for i := range exact {
		exact[i] = pointPlaneAdd(ratOfPointPlane(origin[i]),
			pointPlaneAdd(pointPlaneMul(ratOfPointPlane(local.Value.X), ratOfPointPlane(u[i])),
				pointPlaneMul(ratOfPointPlane(local.Value.Y), ratOfPointPlane(v[i]))))
	}
	value, round, err := facetedrecord.PointPlaneRound(exact)
	if err != nil {
		return VecMeasurement{}, false
	}
	basisU := proofbound.DvLenUpper(proofbound.HeldDelta(plane.U, r3.Vec{}))
	basisV := proofbound.DvLenUpper(proofbound.HeldDelta(plane.V, r3.Vec{}))
	bound := proofbound.AbsSumUpper(
		proofbound.ProductUpper(local.Bound.Base(), proofbound.AbsSumUpper(basisU, basisV)), round)
	if proofbound.IsNonFinite(bound) {
		return VecMeasurement{}, false
	}
	return VecMeasurement{Value: value, Exactness: exactnessOf(bound),
		Bound: units.Millimeters(bound)}, true
}
