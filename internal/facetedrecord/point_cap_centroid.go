package facetedrecord

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// PointFarCapCentroid lifts the authenticated profile's area centroid through
// its recorded plane. Its local bound covers the true Sketch region, not the
// triangulated far cap. A missing source proof leaves the loft usable.
func PointFarCapCentroid(profile momentinput.Profile, plane sectionrecord.PlaneRecord) (measurement.VecMeasurement, bool) {
	local, err := profile.Centroid()
	if err != nil || !proofbound.FiniteVec(local.Value) ||
		proofbound.IsNonFinite(local.Bound.Base()) || local.Bound.Base() < 0 {
		return measurement.VecMeasurement{}, false
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
	value, round, err := PointPlaneRound(exact)
	if err != nil {
		return measurement.VecMeasurement{}, false
	}
	basisU := proofbound.DvLenUpper(proofbound.HeldDelta(plane.U, r3.Vec{}))
	basisV := proofbound.DvLenUpper(proofbound.HeldDelta(plane.V, r3.Vec{}))
	bound := proofbound.AbsSumUpper(
		proofbound.ProductUpper(local.Bound.Base(), proofbound.AbsSumUpper(basisU, basisV)), round)
	if proofbound.IsNonFinite(bound) {
		return measurement.VecMeasurement{}, false
	}
	return measurement.VecMeasurement{Value: value, Exactness: measurement.ExactnessOf(bound),
		Bound: units.Millimeters(bound)}, true
}

// SimilarCapCentroid applies a positive uniform scale about anchor to a cap
// centroid and bounds the rounded point and scaled source uncertainty.
func SimilarCapCentroid(source measurement.VecMeasurement, anchor r3.Vec, factor float64) (measurement.VecMeasurement, error) {
	p := [3]float64{source.Value.X, source.Value.Y, source.Value.Z}
	a := [3]float64{anchor.X, anchor.Y, anchor.Z}
	var exact [3]*big.Rat
	for i := range exact {
		exact[i] = pointPlaneAdd(ratOfPointPlane(a[i]),
			pointPlaneMul(ratOfPointPlane(factor), pointPlaneSub(ratOfPointPlane(p[i]), ratOfPointPlane(a[i]))))
	}
	value, round, err := PointPlaneRound(exact)
	if err != nil {
		return measurement.VecMeasurement{}, err
	}
	bound := proofbound.AbsSumUpper(proofbound.ProductUpper(source.Bound.Base(), factor), round)
	if proofbound.IsNonFinite(bound) {
		return measurement.VecMeasurement{}, fmt.Errorf("%w: the scaled cap centroid has no finite bound", decaderr.ErrUnsupported)
	}
	return measurement.VecMeasurement{Value: value, Exactness: measurement.ExactnessOf(bound),
		Bound: units.Millimeters(bound)}, nil
}
