package momentregion

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/units"
)

// AddSegment integrates a normalized record in its recorded walk direction.
func (s State) AddSegment(segment sectionrecord.CurveSegment, plan Plan, anchor sectionrecord.Point2,
	order freeform.MomentIntegralOrder) error {
	segment, err := sectionrecord.NormalizeSegment(segment)
	if err != nil {
		return err
	}
	if order == freeform.MomentThirdOrder {
		// Before the switch: the free-form arm below shifts its spans to the
		// anchor in place, and the third-order sum is kept about the origin.
		s.AddThird(SegmentThirdMoments(segment, plan))
	}
	switch segment := segment.(type) {
	case sectionrecord.LineSeg:
		s.AddLine(segment, anchor, order)
		return nil
	case sectionrecord.CircleSeg:
		if segment.Radius.Kind() != units.Length {
			return fmt.Errorf(`%w: a circle segment's radius must be a %s, got %s`, decaderr.ErrUnitKind, units.Length, segment.Radius.Kind())
		}
		radius, err := segment.Radius.In(units.Millimeter)
		if err != nil {
			return fmt.Errorf(`%w: a circle segment's radius is not representable: %s`, decaderr.ErrNotFinite, err)
		}
		if segment.CCW != (segment.TStart < segment.TEnd) {
			return fmt.Errorf(`%w: a circle segment's CCW flag contradicts its range order`, decaderr.ErrDegenerate)
		}
		circle := circularbounds.RecordSegment(segment)
		point := circularbounds.RecordPoint(anchor)
		areaProof, haveAreaProof := circularbounds.AreaInterval(circle, point)
		muProof, mvProof, haveMomentProof := circularbounds.FirstMomentInterval(circle, point)
		var muuProof, muvProof, mvvProof proofbound.RatInterval
		var haveSecondMomentProof bool
		if order >= freeform.MomentSecondOrder {
			muuProof, muvProof, mvvProof, haveSecondMomentProof = circularbounds.SecondMomentInterval(circle, point)
		}
		envelope := circularL1Upper(segment.Center, anchor, math.Abs(radius))
		segment.Center = sectionrecord.Point2{U: segment.Center.U - anchor.U, V: segment.Center.V - anchor.V}
		// The arrangement's normalized t is the angle 2π·t from +u
		// (geom.BoundaryEdge); the recorded range order is the walk.
		s.AddCircular(
			segment.Center,
			radius,
			2*math.Pi*segment.TStart,
			2*math.Pi*segment.TEnd,
			math.Abs(radius),
			proofbound.CircularSweepUpper(segment.TStart, segment.TEnd),
			envelope,
			areaProof,
			haveAreaProof,
			muProof,
			mvProof,
			haveMomentProof,
			muuProof,
			muvProof,
			mvvProof,
			haveSecondMomentProof,
			order,
		)
		return nil
	case sectionrecord.ArcSeg:
		circle := circularbounds.RecordSegment(segment)
		point := circularbounds.RecordPoint(anchor)
		areaProof, haveAreaProof := circularbounds.AreaInterval(circle, point)
		muProof, mvProof, haveMomentProof := circularbounds.FirstMomentInterval(circle, point)
		var muuProof, muvProof, mvvProof proofbound.RatInterval
		var haveSecondMomentProof bool
		if order >= freeform.MomentSecondOrder {
			muuProof, muvProof, mvvProof, haveSecondMomentProof = circularbounds.SecondMomentInterval(circle, point)
		}
		envelope := circularL1Upper(segment.Center, anchor, arcRadiusUpper(segment))
		segment.Center = sectionrecord.Point2{U: segment.Center.U - anchor.U, V: segment.Center.V - anchor.V}
		segment.Start = sectionrecord.Point2{U: segment.Start.U - anchor.U, V: segment.Start.V - anchor.V}
		segment.End = sectionrecord.Point2{U: segment.End.U - anchor.U, V: segment.End.V - anchor.V}
		radius := math.Hypot(segment.Start.U-segment.Center.U, segment.Start.V-segment.Center.V)
		a0 := math.Atan2(segment.Start.V-segment.Center.V, segment.Start.U-segment.Center.U)
		a1 := math.Atan2(segment.End.V-segment.Center.V, segment.End.U-segment.Center.U)
		sweep := math.Mod(a1-a0, 2*math.Pi)
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		// normalized t maps to angle = a0 + t·sweep; the range order is the walk.
		s.AddCircular(
			segment.Center,
			radius,
			a0+segment.TStart*sweep,
			a0+segment.TEnd*sweep,
			proofbound.AbsSumUpper(segment.Start.U, segment.Center.U, segment.Start.V, segment.Center.V),
			proofbound.CircularSweepUpper(segment.TStart, segment.TEnd),
			envelope,
			areaProof,
			haveAreaProof,
			muProof,
			mvProof,
			haveMomentProof,
			muuProof,
			muvProof,
			mvvProof,
			haveSecondMomentProof,
			order,
		)
		return nil
	default:
		if !splinebezier.IsFreeformSegment(segment) || len(plan.Spans) == 0 {
			return fmt.Errorf(`%w: this evaluator computes mass properties over line, arc, circle and Tier A free-form profile segments only; the profile has a %T segment`, decaderr.ErrUnsupported, segment)
		}
		if err := splinebezier.ShiftFreeformSpans(plan.Spans, anchor); err != nil {
			return err
		}
		s.AddFreeform(plan.Spans, plan.Reversed, order)
		return nil
	}
}

// circularL1Upper bounds |u| + |v| over the whole circle about center of
// radius at most rUpper, in coordinates relative to anchor: every point there
// is (center − anchor) + R·(cos θ, sin θ), so the sum is at most
// |center − anchor|₁ + R(|cos θ| + |sin θ|) ≤ |center − anchor|₁ + √2·R. The
// offset is taken exactly over the rationals, since the integrator's own
// shifted centre is a rounded difference, and math.Sqrt2 rounds above √2. A
// recorded arc or trimmed circle lies on its whole circle. A coordinate that
// denotes no rational answers +Inf, which leaves the integrator's own
// envelope in place.
func circularL1Upper(center, anchor sectionrecord.Point2, rUpper float64) float64 {
	cu, cv := proofarith.FloatRat(center.U), proofarith.FloatRat(center.V)
	au, av := proofarith.FloatRat(anchor.U), proofarith.FloatRat(anchor.V)
	if cu == nil || cv == nil || au == nil || av == nil || proofbound.IsNonFinite(rUpper) {
		return math.Inf(1)
	}
	du := proofbound.RatFloatUp(new(big.Rat).Abs(new(big.Rat).Sub(cu, au)))
	dv := proofbound.RatFloatUp(new(big.Rat).Abs(new(big.Rat).Sub(cv, av)))
	return proofbound.AbsSumUpper(du, dv, proofbound.ProductUpper(math.Sqrt2, rUpper))
}

// arcRadiusUpper is an upper bound on a recorded arc's radius, the exact
// distance from Center to Start (the circle the arc denotes), rounded up. A
// coordinate that denotes no rational answers +Inf.
func arcRadiusUpper(seg sectionrecord.ArcSeg) float64 {
	su, sv := proofarith.FloatRat(seg.Start.U), proofarith.FloatRat(seg.Start.V)
	cu, cv := proofarith.FloatRat(seg.Center.U), proofarith.FloatRat(seg.Center.V)
	if su == nil || sv == nil || cu == nil || cv == nil {
		return math.Inf(1)
	}
	du, dv := new(big.Rat).Sub(su, cu), new(big.Rat).Sub(sv, cv)
	return proofbound.RatSqrtUp(new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv)))
}
