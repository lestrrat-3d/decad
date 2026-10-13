package decad

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// cardinalConeSectionRecord cuts one authenticated full right cone. The three
// complete meridian lines must join a base-axis point, base-rim point and apex.
// All comparisons use their exact recorded binary values. A rational radius
// that cannot be written exactly in the returned Sketch is refused.
func cardinalConeSectionRecord(rp revolvePayload, plane r3.Frame) (profileRecord, bool) {
	if !rp.full || rp.surfaceResult || rp.sectionDelta != 0 ||
		len(rp.profile.Holes) != 0 || len(rp.profile.Outer.Segments) != 3 ||
		!box.CardinalBasis(rp.frame.U(), rp.frame.V(), rp.frame.N()) ||
		!box.CardinalBasis(plane.U(), plane.V(), plane.N()) ||
		!signedAxisTransform(rp.xform) || !proofbound.FiniteVec(rp.frame.Origin()) ||
		rp.ax.AUBound != 0 || rp.ax.AVBound != 0 ||
		rp.ax.DUBound != 0 || rp.ax.DVBound != 0 ||
		!((math.Abs(rp.ax.DU) == 1 && rp.ax.DV == 0) ||
			(rp.ax.DU == 0 && math.Abs(rp.ax.DV) == 1)) ||
		!finiteMeasurementValues(rp.ax.AU, rp.ax.AV) {
		return profileRecord{}, false
	}

	var corners [3]Point2
	for i, segment := range rp.profile.Outer.Segments {
		line, ok := segment.(lineSeg)
		if !ok || line.TStart != 0 || line.TEnd != 1 ||
			!finiteMeasurementValues(line.Start.U, line.Start.V, line.End.U, line.End.V) {
			return profileRecord{}, false
		}
		corners[i] = line.Start
		next, ok := rp.profile.Outer.Segments[(i+1)%3].(lineSeg)
		if !ok || line.End != next.Start {
			return profileRecord{}, false
		}
	}
	if corners[0] == corners[1] || corners[1] == corners[2] || corners[2] == corners[0] {
		return profileRecord{}, false
	}

	// The resolved axis has a zero error bound, and the original meridian has
	// zero section displacement. Exactly two triangle corners must lie on it;
	// the third must share the base corner's axial coordinate.
	au, av := proofarith.DyOfFinite(rp.ax.AU), proofarith.DyOfFinite(rp.ax.AV)
	du, dv := proofarith.DyOfFinite(rp.ax.DU), proofarith.DyOfFinite(rp.ax.DV)
	var axial, radial [3]proofarith.Dyadic
	axisCorners := make([]int, 0, 2)
	outer := -1
	for i, p := range corners {
		u := proofarith.DySubScalar(proofarith.DyOfFinite(p.U), au)
		v := proofarith.DySubScalar(proofarith.DyOfFinite(p.V), av)
		axial[i] = proofarith.DyAdd(proofarith.DyMul(u, du), proofarith.DyMul(v, dv))
		radial[i] = proofarith.DySubScalar(proofarith.DyMul(v, du), proofarith.DyMul(u, dv))
		if radial[i].IsZero() {
			axisCorners = append(axisCorners, i)
		} else if outer < 0 {
			outer = i
		} else {
			return profileRecord{}, false
		}
	}
	if len(axisCorners) != 2 || outer < 0 {
		return profileRecord{}, false
	}
	base, apex := axisCorners[0], axisCorners[1]
	if proofarith.DyCmp(axial[base], axial[outer]) != 0 {
		base, apex = apex, base
	}
	if proofarith.DyCmp(axial[base], axial[outer]) != 0 ||
		proofarith.DyCmp(axial[base], axial[apex]) == 0 {
		return profileRecord{}, false
	}

	// Compare the placed base and apex with the requested cardinal plane.
	// Opposite strict signs prove an interior cut; the cone radius varies
	// linearly along that axis, so the cut is one complete disk.
	point := func(p Point2) proofarith.DyV3 {
		local := proofarith.DvAdd(proofarith.DyVec(rp.frame.Origin()),
			proofarith.DvAdd(
				proofarith.DvScale(proofarith.DyVec(rp.frame.U()), proofarith.DyOfFinite(p.U)),
				proofarith.DvScale(proofarith.DyVec(rp.frame.V()), proofarith.DyOfFinite(p.V)),
			))
		return proofarith.DvTransform(rp.xform, local)
	}
	baseWorld, apexWorld := point(corners[base]), point(corners[apex])
	u, v, n := proofarith.DyVec(plane.U()), proofarith.DyVec(plane.V()), proofarith.DyVec(plane.N())
	if !proofarith.DvEqual(proofarith.DvCross(u, v), n) ||
		!proofarith.DvIsZero(proofarith.DvCross(proofarith.DvSub(apexWorld, baseWorld), n)) {
		return profileRecord{}, false
	}
	baseDelta := proofarith.DvSub(baseWorld, proofarith.DyVec(plane.Origin()))
	apexDelta := proofarith.DvSub(apexWorld, proofarith.DyVec(plane.Origin()))
	baseSide, apexSide := proofarith.DvDot(baseDelta, n), proofarith.DvDot(apexDelta, n)
	if baseSide.Sign() == 0 || apexSide.Sign() == 0 || baseSide.Sign() == apexSide.Sign() {
		return profileRecord{}, false
	}
	height := proofarith.DySubScalar(apexSide, baseSide)
	cutFromBase := new(big.Rat).Neg(baseSide.Rat())
	cutFromBase.Quo(cutFromBase, height.Rat())
	radius := radial[outer].Rat()
	radius.Abs(radius)
	radius.Mul(radius, new(big.Rat).Sub(big.NewRat(1, 1), cutFromBase))
	radiusMM, radiusExact := radius.Float64()
	centerU, exactU := proofarith.DvDot(baseDelta, u).Float64()
	centerV, exactV := proofarith.DvDot(baseDelta, v).Float64()
	if !radiusExact || radiusMM <= 0 || math.IsInf(radiusMM, 0) ||
		!exactU || !exactV {
		return profileRecord{}, false
	}
	return profileRecord{Outer: loopRecord{Segments: []curveSegment{circleSeg{
		Center: Point2{U: centerU, V: centerV}, Radius: units.Millimeters(radiusMM),
		CCW: true, TStart: 0, TEnd: 1,
	}}}}, true
}
