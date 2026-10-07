package momentregion

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
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
