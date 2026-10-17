package facetedrecord

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// PointPlaneMapExact maps a source ray to the exact split-plane intersection.
func PointPlaneMapExact(apex, far r3.Vec, plane *PointPlaneLimit) ([3]*big.Rat, error) {
	den := plane.Denominator(apex, far)
	if den.Sign() <= 0 {
		return [3]*big.Rat{}, fmt.Errorf("%w: a source ray does not meet the split plane", decaderr.ErrUnsupported)
	}
	alpha := new(big.Rat).Quo(plane.D, den)
	a := [3]float64{apex.X, apex.Y, apex.Z}
	p := [3]float64{far.X, far.Y, far.Z}
	var q [3]*big.Rat
	for i := range q {
		q[i] = pointPlaneAdd(ratOfPointPlane(a[i]),
			pointPlaneMul(alpha, pointPlaneSub(ratOfPointPlane(p[i]), ratOfPointPlane(a[i]))))
	}
	return q, nil
}

// PointPlaneCapCentroid integrates the rational image of the ORIGINAL far
// station polygon. The projective ray map is a homeomorphism on the admitted
// cap, so its source contour and the mapped chord walk have the same interior.
// Every true contour point is within delta of its mapped chord; the symmetric
// difference is inside the chord walk's delta tube. If A is the polygon area,
// Δ the tube area and D its greatest possible distance from the polygon
// centroid, the true centroid differs by at most ΔD/(A-Δ).
func PointPlaneCapCentroid(ctx context.Context, base Payload,
	plane *PointPlaneLimit) (measurement.VecMeasurement, bool, error) {
	if len(base.Verts) < 4 || base.PointSection == nil {
		return measurement.VecMeasurement{}, false, nil
	}
	polygon := make([][3]*big.Rat, len(base.Verts)-1)
	for i, far := range base.Verts[1:] {
		if i%32 == 0 {
			if err := ctx.Err(); err != nil {
				return measurement.VecMeasurement{}, false, err
			}
		}
		q, err := PointPlaneMapExact(base.PointSection.Apex, far, plane)
		if err != nil {
			return measurement.VecMeasurement{}, false, nil //nolint:nilerr // failed map withholds the optional centroid reading
		}
		polygon[i] = q
	}
	// A vertical cut plane has nx*x+ny*y=constant. Drop the coordinate
	// with the larger normal component to keep the projected area useful.
	u, v := 0, 2
	if pointPlaneAbs(plane.Nx).Cmp(pointPlaneAbs(plane.Ny)) > 0 {
		u = 1
	}
	twiceArea := new(big.Rat)
	var moments [3]*big.Rat
	moments[u], moments[v] = new(big.Rat), new(big.Rat)
	perimeter := 0.0
	for i, a := range polygon {
		if i%32 == 0 {
			if err := ctx.Err(); err != nil {
				return measurement.VecMeasurement{}, false, err
			}
		}
		b := polygon[(i+1)%len(polygon)]
		perimeter = proofbound.AbsSumUpper(perimeter, PointCapDistanceUpper(a, b))
	}
	// Signed triangles from one boundary vertex integrate the projected
	// coordinates. The exact plane relation recovers the third coordinate.
	anchor := polygon[0]
	for i := 1; i+1 < len(polygon); i++ {
		a, b := polygon[i], polygon[i+1]
		cross := pointPlaneSub(
			pointPlaneMul(pointPlaneSub(a[u], anchor[u]), pointPlaneSub(b[v], anchor[v])),
			pointPlaneMul(pointPlaneSub(b[u], anchor[u]), pointPlaneSub(a[v], anchor[v])))
		twiceArea.Add(twiceArea, cross)
		for _, k := range [2]int{u, v} {
			three := pointPlaneAdd(pointPlaneAdd(anchor[k], a[k]), b[k])
			moments[k].Add(moments[k], pointPlaneMul(three, cross))
		}
	}
	if twiceArea.Sign() == 0 || proofbound.IsNonFinite(perimeter) {
		return measurement.VecMeasurement{}, false, nil
	}
	den := pointPlaneMul(big.NewRat(3, 1), twiceArea)
	var center [3]*big.Rat
	for _, k := range [2]int{u, v} {
		// Lowest-terms operands: moments and den are results of the
		// pointPlane helpers and raw Add (see pointPlaneAdd).
		center[k] = proofarith.QuoRat(new(big.Rat), moments[k], den)
	}
	center[1-u] = PointPlaneCapDroppedCoordinate(center, anchor, plane, u)
	// The projected area is a conservative lower bound on physical area:
	// dropping the dominant normal component cannot enlarge a planar patch.
	areaLower := proofbound.RatFloatDown(new(big.Rat).Quo(pointPlaneAbs(twiceArea), big.NewRat(2, 1)))
	delta := proofbound.ProductUpper(plane.MapStretch, base.MeshBound)
	tube := proofbound.SectionDisplacementArea(delta, len(polygon), perimeter)
	if areaLower <= tube || proofbound.IsNonFinite(delta) || proofbound.IsNonFinite(tube) {
		return measurement.VecMeasurement{}, false, nil
	}
	radius := 0.0
	for i, q := range polygon {
		if i%32 == 0 {
			if err := ctx.Err(); err != nil {
				return measurement.VecMeasurement{}, false, err
			}
		}
		radius = max(radius, PointCapDistanceUpper(q, center))
	}
	floor := math.Nextafter(areaLower-tube, math.Inf(-1))
	if floor <= 0 || proofbound.IsNonFinite(radius) {
		return measurement.VecMeasurement{}, false, nil
	}
	value, round, err := PointPlaneRound(center)
	if err != nil {
		return measurement.VecMeasurement{}, false, nil //nolint:nilerr // failed rounding withholds the optional centroid reading
	}
	shift := proofbound.DivUpper(proofbound.ProductUpper(tube, proofbound.AbsSumUpper(radius, delta)), floor)
	bound := proofbound.AbsSumUpper(shift, round)
	if proofbound.IsNonFinite(bound) {
		return measurement.VecMeasurement{}, false, nil
	}
	return measurement.VecMeasurement{Value: value, Exactness: measurement.ExactnessOf(bound),
		Bound: units.Millimeters(bound)}, true, nil
}

// PointPlaneCapDroppedCoordinate uses the mapped anchor's exact plane offset
// to recover the horizontal coordinate omitted from the area moments.
func PointPlaneCapDroppedCoordinate(center, anchor [3]*big.Rat,
	plane *PointPlaneLimit, keep int) *big.Rat {
	drop := 1 - keep
	normal := [2]*big.Rat{plane.Nx, plane.Ny}
	// QuoRat needs lowest-terms operands; plane.Nx and plane.Ny come from
	// FloatRat or NewRat (see pointPlaneAdd), so the slope is in lowest terms.
	slope := proofarith.QuoRat(new(big.Rat), normal[keep], normal[drop])
	return pointPlaneSub(anchor[drop],
		pointPlaneMul(slope, pointPlaneSub(center[keep], anchor[keep])))
}

// PointCapDistanceUpper rounds each coordinate's exact difference up once by
// proofbound.RatDistanceUp, which never reduces the difference to lowest terms:
// the centroid's denominator has about 15,000 bits, and the reduction's GCD
// costs far more than the rounding. The shared dyadic length proof then avoids
// a square root of the large projective rational denominator while keeping a
// tight 3D distance bound.
func PointCapDistanceUpper(a, b [3]*big.Rat) float64 {
	var up [3]float64
	for i := range a {
		up[i] = proofbound.RatDistanceUp(a[i], b[i])
		if proofbound.IsNonFinite(up[i]) {
			return math.Inf(1)
		}
	}
	return proofbound.DvLenUpper(proofbound.HeldDelta(r3.NewVec(up[0], up[1], up[2]), r3.Vec{}))
}
