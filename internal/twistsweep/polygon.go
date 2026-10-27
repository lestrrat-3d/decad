package twistsweep

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// TwistPolygon is a strictly convex chord outline admitted for a twisted
// sweep, with its exact area and perimeter and radius bounds.
type TwistPolygon struct {
	Points         []planePoint
	Area           *big.Rat
	PerimeterLower float64
	PerimeterUpper float64
	MaxRadiusL1    float64
	MaxEdgeL1      float64
	HasArc         bool
	AreaInterval   proofbound.RatInterval
	// CentroidU and CentroidV are the exact section centroid relative to the
	// twist axis. A curved section proves them zero by central symmetry.
	CentroidU, CentroidV *big.Rat
}

// OffAxis reports whether the section centroid is not on the twist axis.
func (p TwistPolygon) OffAxis() bool { return p.CentroidU.Sign() != 0 || p.CentroidV.Sign() != 0 }

// Opposite source edges have the same parameter walk after a half-turn.
// This proves the exact curved section's first moments vanish, rather than
// inferring its centroid from the chord polygon.
func twistOppositeSegments(a, b curveSegment) bool {
	negative := func(p, q planePoint) bool { return p.U == -q.U && p.V == -q.V }
	switch a := a.(type) {
	case lineSeg:
		b, ok := b.(lineSeg)
		return ok && a.TStart == b.TStart && a.TEnd == b.TEnd &&
			negative(a.Start, b.Start) && negative(a.End, b.End)
	case arcSeg:
		b, ok := b.(arcSeg)
		return ok && a.TStart == b.TStart && a.TEnd == b.TEnd &&
			negative(a.Center, b.Center) && negative(a.Start, b.Start) && negative(a.End, b.End)
	default:
		return false
	}
}

// TwistRat returns x as an exact rational.
func TwistRat(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }

// TwistPolygonOf admits a strictly convex chord outline. A curved outline
// must also have exact opposite source edges, which proves its true centroid
// lies on the axis. A straight outline may lie anywhere relative to the axis:
// every section is a rigid rotation about it, and monotone Z keeps different
// true sections disjoint.
func TwistPolygonOf(profile profileRecord) (TwistPolygon, error) {
	if len(profile.Holes) != 0 {
		return TwistPolygon{}, fmt.Errorf(`%w: a twisted sweep with holes is not implemented`, decaderr.ErrUnsupported)
	}
	segments := profile.Outer.Segments
	if len(segments) < 3 || len(segments) > 256 {
		return TwistPolygon{}, fmt.Errorf(`%w: a twisted sweep needs 3 to 256 profile edges`, decaderr.ErrUnsupported)
	}
	points := make([]planePoint, len(segments))
	hasArc := false
	for i, segment := range segments {
		switch edge := segment.(type) {
		case lineSeg:
			if edge.TStart != 0 || edge.TEnd != 1 {
				return TwistPolygon{}, fmt.Errorf(`%w: a twisted sweep needs whole profile edges`, decaderr.ErrUnsupported)
			}
			points[i] = edge.Start
		case arcSeg:
			hasArc = true
			if edge.TStart != 0 || edge.TEnd != 1 {
				return TwistPolygon{}, fmt.Errorf(`%w: a twisted sweep needs whole profile edges`, decaderr.ErrUnsupported)
			}
			startU := new(big.Rat).Sub(TwistRat(edge.Start.U), TwistRat(edge.Center.U))
			startV := new(big.Rat).Sub(TwistRat(edge.Start.V), TwistRat(edge.Center.V))
			endU := new(big.Rat).Sub(TwistRat(edge.End.U), TwistRat(edge.Center.U))
			endV := new(big.Rat).Sub(TwistRat(edge.End.V), TwistRat(edge.Center.V))
			startR2 := new(big.Rat).Add(new(big.Rat).Mul(startU, startU), new(big.Rat).Mul(startV, startV))
			endR2 := new(big.Rat).Add(new(big.Rat).Mul(endU, endU), new(big.Rat).Mul(endV, endV))
			if startR2.Sign() <= 0 || startR2.Cmp(endR2) != 0 {
				return TwistPolygon{}, fmt.Errorf(`%w: a twisted arc needs equal pinned endpoint radii`, decaderr.ErrUnsupported)
			}
			points[i] = edge.Start
		default:
			return TwistPolygon{}, fmt.Errorf(`%w: a twisted sweep needs line or arc profile edges`, decaderr.ErrUnsupported)
		}
	}
	if hasArc {
		if len(segments)%2 != 0 {
			return TwistPolygon{}, fmt.Errorf(`%w: a mixed twisted section needs paired opposite edges`, decaderr.ErrUnsupported)
		}
		for i := range len(segments) / 2 {
			if !twistOppositeSegments(segments[i], segments[i+len(segments)/2]) {
				return TwistPolygon{}, fmt.Errorf(`%w: a mixed twisted section needs exact central symmetry`, decaderr.ErrUnsupported)
			}
		}
	}
	area2, momentX6, momentY6 := new(big.Rat), new(big.Rat), new(big.Rat)
	perimeter, perimeterLow := new(big.Rat), new(big.Rat)
	maxRadius, maxEdge := new(big.Rat), new(big.Rat)
	sign := 0
	for i, p := range points {
		q, r := points[(i+1)%len(points)], points[(i+2)%len(points)]
		px, py, qx, qy := TwistRat(p.U), TwistRat(p.V), TwistRat(q.U), TwistRat(q.V)
		dx, dy := new(big.Rat).Sub(qx, px), new(big.Rat).Sub(qy, py)
		cross := new(big.Rat).Sub(new(big.Rat).Mul(px, qy), new(big.Rat).Mul(qx, py))
		area2.Add(area2, cross)
		momentX6.Add(momentX6, new(big.Rat).Mul(new(big.Rat).Add(px, qx), cross))
		momentY6.Add(momentY6, new(big.Rat).Mul(new(big.Rat).Add(py, qy), cross))
		rx, ry := new(big.Rat).Sub(TwistRat(r.U), qx), new(big.Rat).Sub(TwistRat(r.V), qy)
		turn := new(big.Rat).Sub(new(big.Rat).Mul(dx, ry), new(big.Rat).Mul(dy, rx))
		if turn.Sign() == 0 || (sign != 0 && turn.Sign() != sign) {
			return TwistPolygon{}, fmt.Errorf(`%w: a twisted sweep needs a strictly convex profile`, decaderr.ErrUnsupported)
		}
		sign = turn.Sign()
		if arc, curved := segments[i].(arcSeg); curved {
			converted := circularbounds.RecordSegment(arc)
			radiusIv, sweepIv, ok := circularbounds.WalkEnclosures(converted)
			if !ok || radiusIv.Lo.Sign() <= 0 || sweepIv.Lo.Sign() <= 0 ||
				sweepIv.Hi.Cmp(proofbound.PiUpper) > 0 {
				return TwistPolygon{}, fmt.Errorf(`%w: a twisted arc needs a bounded positive turn up to a half-circle`, decaderr.ErrUnsupported)
			}
			length, ok := circularbounds.LengthInterval(converted)
			if !ok || length.Lo.Sign() <= 0 {
				return TwistPolygon{}, fmt.Errorf(`%w: the twisted arc perimeter has no bound`, decaderr.ErrUnsupported)
			}
			perimeter.Add(perimeter, length.Hi)
			perimeterLow.Add(perimeterLow, length.Lo)
			arcRadius := new(big.Rat).Add(
				new(big.Rat).Add(new(big.Rat).Abs(TwistRat(arc.Center.U)),
					new(big.Rat).Abs(TwistRat(arc.Center.V))),
				new(big.Rat).Mul(big.NewRat(2, 1), radiusIv.Hi))
			if arcRadius.Cmp(maxRadius) > 0 {
				maxRadius = arcRadius
			}
		} else {
			length2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
			length, ok := proofbound.SqrtFixed(length2)
			if !ok {
				return TwistPolygon{}, fmt.Errorf(`%w: the twisted profile perimeter has no bound`, decaderr.ErrUnsupported)
			}
			perimeter.Add(perimeter, length.Hi)
			perimeterLow.Add(perimeterLow, length.Lo)
			edgeL1 := new(big.Rat).Add(new(big.Rat).Abs(dx), new(big.Rat).Abs(dy))
			if edgeL1.Cmp(maxEdge) > 0 {
				maxEdge = edgeL1
			}
		}
		radiusL1 := new(big.Rat).Add(new(big.Rat).Abs(px), new(big.Rat).Abs(py))
		if radiusL1.Cmp(maxRadius) > 0 {
			maxRadius = radiusL1
		}
	}
	if area2.Sign() == 0 || (hasArc && (momentX6.Sign() != 0 || momentY6.Sign() != 0)) {
		return TwistPolygon{}, fmt.Errorf(`%w: the twisted profile must have its centroid on the path axis`, decaderr.ErrUnsupported)
	}
	// The polygon centroid is the shoelace first moment over three times
	// twice the signed area.
	denom := new(big.Rat).Mul(big.NewRat(3, 1), area2)
	centroidU, centroidV := new(big.Rat), new(big.Rat)
	if !hasArc {
		centroidU.Quo(momentX6, denom)
		centroidV.Quo(momentY6, denom)
	}
	area := new(big.Rat).Quo(new(big.Rat).Abs(area2), big.NewRat(2, 1))
	areaInterval := proofbound.PointInterval(area)
	if hasArc {
		measured, err := profile.Area()
		if err != nil {
			return TwistPolygon{}, err
		}
		value, bound := TwistRat(measured.Value.Base()), TwistRat(measured.Bound.Base())
		areaInterval = proofbound.Interval(new(big.Rat).Sub(value, bound), new(big.Rat).Add(value, bound))
		if areaInterval.Lo.Sign() <= 0 {
			return TwistPolygon{}, fmt.Errorf(`%w: a mixed twisted section has no positive area proof`, decaderr.ErrUnsupported)
		}
	}
	return TwistPolygon{
		Points: points, Area: area, AreaInterval: areaInterval,
		PerimeterLower: proofbound.RatFloatDown(perimeterLow),
		PerimeterUpper: proofbound.RatFloatUp(perimeter),
		MaxRadiusL1:    proofbound.RatFloatUp(maxRadius),
		MaxEdgeL1:      proofbound.RatFloatUp(maxEdge),
		HasArc:         hasArc,
		CentroidU:      centroidU, CentroidV: centroidV,
	}, nil
}
