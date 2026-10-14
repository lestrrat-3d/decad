package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// A cap endpoint remains an interval-certified true point. The held point
// projects its box center onto the exact cap circle, then charges the full
// distance back to every point in the box. Wall and cap facets will also
// need their own source-curve and chord bounds before a weld can admit Union.
type revolveLoftFacesCapSeamVertex struct {
	boundary              revolveLoftFacesToolCapBoundary
	point                 r3.Vec
	blankBound, loftBound float64
}

func revolveLoftFacesCapArcSeamVertices(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	arc *revolveLoftFacesToolCapOccupiedArc,
) ([2]revolveLoftFacesCapSeamVertex, bool, error) {
	return revolveLoftFacesCapArcSeamVerticesIn(ctx, proof, nil, arc)
}

// revolveLoftFacesCapArcSeamVerticesIn takes the census under the contract of
// revolveLoftFacesHeelArcSideTiesIn; a nil census rederives the arc.
func revolveLoftFacesCapArcSeamVerticesIn(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, census *revolveLoftFacesPairCensus,
	arc *revolveLoftFacesToolCapOccupiedArc,
) ([2]revolveLoftFacesCapSeamVertex, bool, error) {
	var out [2]revolveLoftFacesCapSeamVertex
	current, err := revolveLoftFacesToolCapOccupiedArcCurrentIn(ctx, proof, census, arc)
	if err != nil || !current {
		return out, false, err
	}
	for i, boundary := range [...]revolveLoftFacesToolCapBoundary{
		arc.negativeZ, arc.positiveZ,
	} {
		vertex, ok := revolveLoftFacesCapBoundarySeamVertex(arc.circle, boundary)
		if !ok {
			return [2]revolveLoftFacesCapSeamVertex{}, false, nil
		}
		out[i] = vertex
	}
	return out, true, nil
}

func revolveLoftFacesCapBoundarySeamVertex(circle revolveLoftFacesToolCapCircle,
	boundary revolveLoftFacesToolCapBoundary,
) (revolveLoftFacesCapSeamVertex, bool) {
	var out revolveLoftFacesCapSeamVertex
	if circle.x == nil || circle.radius == nil || circle.radius.Sign() <= 0 ||
		boundary.parameter.Lo == nil || boundary.parameter.Hi == nil ||
		boundary.parameter.Lo.Cmp(boundary.parameter.Hi) >= 0 {
		return out, false
	}
	var middle [3]*big.Rat
	for axis, interval := range boundary.box {
		if interval.Lo == nil || interval.Hi == nil ||
			interval.Lo.Cmp(interval.Hi) > 0 {
			return out, false
		}
		middle[axis] = lfDiv(lfAdd(interval.Lo, interval.Hi), big.NewRat(2, 1))
	}
	if boundary.box[0].Lo.Cmp(circle.x) > 0 ||
		boundary.box[0].Hi.Cmp(circle.x) < 0 ||
		lfAdd(lfMul(middle[1], middle[1]),
			lfMul(middle[2], middle[2])).Sign() <= 0 {
		return out, false
	}
	point, _, ok := revolveLoftFacesMeridianRayPoint(
		[2]*big.Rat{middle[1], middle[2]}, circle.x, circle.radius)
	if !ok {
		return out, false
	}
	coordinates := [3]float64{point.X, point.Y, point.Z}
	distance := lfZero()
	for axis, held := range coordinates {
		interval := boundary.box[axis]
		lower := lfAbs(lfSub(lfF(held), interval.Lo))
		upper := lfAbs(lfSub(lfF(held), interval.Hi))
		if lower.Cmp(upper) > 0 {
			distance = lfAdd(distance, lower)
		} else {
			distance = lfAdd(distance, upper)
		}
	}
	bound := proofbound.RatFloatUp(distance)
	if proofbound.IsNonFinite(bound) || bound < 0 {
		return out, false
	}
	return revolveLoftFacesCapSeamVertex{
		boundary: boundary, point: point,
		blankBound: bound, loftBound: bound}, true
}
