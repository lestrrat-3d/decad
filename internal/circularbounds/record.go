package circularbounds

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// RecordSegment transfers the recorded fields used by circular proofs.
func RecordSegment(segment sectionrecord.CurveSegment) CurveSegment {
	switch segment := segment.(type) {
	case sectionrecord.CircleSeg:
		return CircleSeg{
			Center: RecordPoint(segment.Center), Radius: segment.Radius, CCW: segment.CCW,
			TStart: segment.TStart, TEnd: segment.TEnd,
		}
	case sectionrecord.ArcSeg:
		return ArcSeg{
			Center: RecordPoint(segment.Center), Start: RecordPoint(segment.Start), End: RecordPoint(segment.End),
			TStart: segment.TStart, TEnd: segment.TEnd,
		}
	default:
		return nil
	}
}

// RecordPoint transfers a recorded plane-local point to a circular proof.
func RecordPoint(point sectionrecord.Point2) Point2 { return Point2{U: point.U, V: point.V} }

// ArcRadialResidualUpper is an upper bound on | ‖End − Center‖ − ‖Start − Center‖ |
// for a recorded arc: how far the recorded End sits from the point the arc
// denotes at t == 1, which lies at Start's radius and End's own angle
// (docs/evaluator-design.md §4).
//
// The bound is exact-rational throughout and never a float subtraction of two
// square roots. |r1 − r0| is |r1² − r0²| / (r1 + r0); the numerator is the
// exact rational difference of the two recorded squared distances, and the
// denominator is replaced by a rounded-DOWN sum of the two radii
// (proofbound.RatSqrtDown), which can only enlarge the quotient.
// proofbound.RatFloatUp rounds the result out once. Equal squared radii answer
// exactly zero.
//
// A denominator that cannot be shown positive answers +Inf, which the caller
// refuses on rather than publishing a substitute. It is defensive: it needs
// both recorded radii to round down to zero while their exact squares differ.
func ArcRadialResidualUpper(arc sectionrecord.ArcSeg) float64 {
	dx0 := ExactCoordinateDelta(arc.Start.U, arc.Center.U)
	dy0 := ExactCoordinateDelta(arc.Start.V, arc.Center.V)
	dx1 := ExactCoordinateDelta(arc.End.U, arc.Center.U)
	dy1 := ExactCoordinateDelta(arc.End.V, arc.Center.V)
	r0 := new(big.Rat).Add(new(big.Rat).Mul(dx0, dx0), new(big.Rat).Mul(dy0, dy0))
	r1 := new(big.Rat).Add(new(big.Rat).Mul(dx1, dx1), new(big.Rat).Mul(dy1, dy1))

	diff := new(big.Rat).Sub(r1, r0)
	if diff.Sign() == 0 {
		return 0
	}
	diff.Abs(diff)

	den := new(big.Rat).Add(proofarith.FloatRat(proofbound.RatSqrtDown(r0)), proofarith.FloatRat(proofbound.RatSqrtDown(r1)))
	if den.Sign() <= 0 {
		return math.Inf(1)
	}
	up := proofbound.RatFloatUp(new(big.Rat).Quo(diff, den))
	if proofbound.IsNonFinite(up) {
		return math.Inf(1)
	}
	return up
}
