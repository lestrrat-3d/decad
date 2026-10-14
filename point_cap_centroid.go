package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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
	value, round, err := pointPlaneRound(exact)
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

// pointPlaneCapCentroid integrates the rational image of the ORIGINAL far
// station polygon. The projective ray map is a homeomorphism on the admitted
// cap, so its source contour and the mapped chord walk have the same interior.
// Every true contour point is within delta of its mapped chord; the symmetric
// difference is inside the chord walk's delta tube. If A is the polygon area,
// Δ the tube area and D its greatest possible distance from the polygon
// centroid, the true centroid differs by at most ΔD/(A-Δ).
func pointPlaneCapCentroid(ctx context.Context, base facetedPayload,
	plane *pointPlaneLimit) (VecMeasurement, bool, error) {
	if len(base.verts) < 4 || base.pointSection == nil {
		return VecMeasurement{}, false, nil
	}
	polygon := make([][3]*big.Rat, len(base.verts)-1)
	for i, far := range base.verts[1:] {
		if i%32 == 0 {
			if err := ctx.Err(); err != nil {
				return VecMeasurement{}, false, err
			}
		}
		q, err := pointPlaneMapExact(base.pointSection.apex, far, plane)
		if err != nil {
			return VecMeasurement{}, false, nil //nolint:nilerr // failed map withholds the optional centroid reading
		}
		polygon[i] = q
	}
	// A vertical cut plane has nx*x+ny*y=constant. Drop the coordinate
	// with the larger normal component to keep the projected area useful.
	u, v := 0, 2
	if pointPlaneAbs(plane.nx).Cmp(pointPlaneAbs(plane.ny)) > 0 {
		u = 1
	}
	twiceArea := new(big.Rat)
	var moments [3]*big.Rat
	moments[u], moments[v] = new(big.Rat), new(big.Rat)
	perimeter := 0.0
	for i, a := range polygon {
		if i%32 == 0 {
			if err := ctx.Err(); err != nil {
				return VecMeasurement{}, false, err
			}
		}
		b := polygon[(i+1)%len(polygon)]
		perimeter = proofbound.AbsSumUpper(perimeter, pointCapDistanceUpper(a, b))
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
		return VecMeasurement{}, false, nil
	}
	den := pointPlaneMul(big.NewRat(3, 1), twiceArea)
	var center [3]*big.Rat
	for _, k := range [2]int{u, v} {
		center[k] = proofarith.QuoRat(new(big.Rat), moments[k], den)
	}
	center[1-u] = pointPlaneCapDroppedCoordinate(center, anchor, plane, u)
	// The projected area is a conservative lower bound on physical area:
	// dropping the dominant normal component cannot enlarge a planar patch.
	areaLower := proofbound.RatFloatDown(new(big.Rat).Quo(pointPlaneAbs(twiceArea), big.NewRat(2, 1)))
	delta := proofbound.ProductUpper(plane.mapStretch, base.meshBound)
	tube := proofbound.SectionDisplacementArea(delta, len(polygon), perimeter)
	if areaLower <= tube || proofbound.IsNonFinite(delta) || proofbound.IsNonFinite(tube) {
		return VecMeasurement{}, false, nil
	}
	radius := 0.0
	for i, q := range polygon {
		if i%32 == 0 {
			if err := ctx.Err(); err != nil {
				return VecMeasurement{}, false, err
			}
		}
		radius = max(radius, pointCapDistanceUpper(q, center))
	}
	floor := math.Nextafter(areaLower-tube, math.Inf(-1))
	if floor <= 0 || proofbound.IsNonFinite(radius) {
		return VecMeasurement{}, false, nil
	}
	value, round, err := pointPlaneRound(center)
	if err != nil {
		return VecMeasurement{}, false, nil //nolint:nilerr // failed rounding withholds the optional centroid reading
	}
	shift := proofbound.DivUpper(proofbound.ProductUpper(tube, proofbound.AbsSumUpper(radius, delta)), floor)
	bound := proofbound.AbsSumUpper(shift, round)
	if proofbound.IsNonFinite(bound) {
		return VecMeasurement{}, false, nil
	}
	return VecMeasurement{Value: value, Exactness: exactnessOf(bound),
		Bound: units.Millimeters(bound)}, true, nil
}

// pointPlaneCapDroppedCoordinate uses the mapped anchor's exact plane offset
// to recover the horizontal coordinate omitted from the area moments.
func pointPlaneCapDroppedCoordinate(center, anchor [3]*big.Rat,
	plane *pointPlaneLimit, keep int) *big.Rat {
	drop := 1 - keep
	normal := [2]*big.Rat{plane.nx, plane.ny}
	slope := proofarith.QuoRat(new(big.Rat), normal[keep], normal[drop])
	return pointPlaneSub(anchor[drop],
		pointPlaneMul(slope, pointPlaneSub(center[keep], anchor[keep])))
}

// Each coordinate's exact difference is rounded up once by
// proofbound.RatDistanceUp, which never reduces the difference to lowest terms:
// the centroid's denominator has about 15,000 bits, and the reduction's GCD
// costs far more than the rounding. The shared dyadic length proof then avoids
// a square root of the large projective rational denominator while keeping a
// tight 3D distance bound.
func pointCapDistanceUpper(a, b [3]*big.Rat) float64 {
	var up [3]float64
	for i := range a {
		up[i] = proofbound.RatDistanceUp(a[i], b[i])
		if proofbound.IsNonFinite(up[i]) {
			return math.Inf(1)
		}
	}
	return proofbound.DvLenUpper(proofbound.HeldDelta(r3.NewVec(up[0], up[1], up[2]), r3.Vec{}))
}

// placedCapCentroid applies the recorded affine motion to the true cap
// centroid. Rational evaluation rounds the reported point once; the placement
// matrix's proved stretch maps the source radius to the new one.
func placedCapCentroid(source VecMeasurement, motion r3.Transform) (VecMeasurement, error) {
	linear, err := massmoment.PlacementRotation(motion)
	if err != nil {
		return VecMeasurement{}, err
	}
	tr := motion.Translation()
	t := [3]*big.Rat{ratOfPointPlane(tr.X), ratOfPointPlane(tr.Y), ratOfPointPlane(tr.Z)}
	return placedCapCentroidBy(source, linear, t)
}

// placedCapCentroidBy applies the exact affine map p -> linear*p + t to the
// true cap centroid. Rational evaluation rounds the reported point once; the
// map's proved stretch, zero for an exactly orthogonal linear part, carries the
// source radius to the new one.
func placedCapCentroidBy(source VecMeasurement, linear [3][3]*big.Rat, t [3]*big.Rat) (VecMeasurement, error) {
	charge, err := massmoment.MapChargeOf(linear)
	if err != nil {
		return VecMeasurement{}, err
	}
	p := [3]float64{source.Value.X, source.Value.Y, source.Value.Z}
	var exact [3]*big.Rat
	for i := range exact {
		exact[i] = new(big.Rat).Set(t[i])
		for j := range p {
			exact[i].Add(exact[i], pointPlaneMul(linear[i][j], ratOfPointPlane(p[j])))
		}
	}
	value, round, err := pointPlaneRound(exact)
	if err != nil {
		return VecMeasurement{}, err
	}
	bound := proofbound.AbsSumUpper(proofbound.ProductUpper(source.Bound.Base(),
		proofbound.AbsSumUpper(1, charge.Stretch)), round)
	return VecMeasurement{Value: value, Exactness: exactnessOf(bound),
		Bound: units.Millimeters(bound)}, nil
}
