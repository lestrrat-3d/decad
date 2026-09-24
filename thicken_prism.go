package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/units"
)

// thickenPrism builds a through-height wall around a recorded prism sheet.
// This arm first proves that its generated section coordinates denote the
// exact offset of the source section; evalPrismContext can then keep its
// zero-displacement section certificate.
func thickenPrism(ctx context.Context, d *Document, pp prismPayload, side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if !pp.surfaceResult || pp.sectionDelta != 0 || len(pp.profile.Holes) != 0 {
		return nil, fmt.Errorf(`%w: this prism sheet has no admitted Thicken section`, ErrUnsupported)
	}
	if admitAbove(boundedSub(pp.z1Scalar(), pp.z0Scalar()), 0) != survAdmit {
		return nil, fmt.Errorf(`%w: the prism sheet has no proven positive sweep height`, ErrUnsupported)
	}
	if tDelta != 0 {
		return nil, fmt.Errorf(`%w: the prism thickness cannot be generated exactly in millimetres`, ErrUnsupported)
	}
	amount := tmm
	if side == ThickenCentered {
		half := boundedQuotient(tmm, 0, 2, 0)
		if half.bound != 0 || half.value <= 0 {
			return nil, fmt.Errorf(`%w: the half-thickness is not exactly representable`, ErrUnsupported)
		}
		amount = half.value
	}
	budget := newWorkBudget(ctx)
	if err := budget.err(); err != nil {
		return nil, err
	}
	if len(pp.profile.Outer.Segments) != 1 {
		return thickenPrismAxis(ctx, d, pp, side, amount, budget)
	}
	circle, ok := pp.profile.Outer.Segments[0].(CircleSeg)
	if !ok {
		return thickenPrismAxis(ctx, d, pp, side, amount, budget)
	}
	if !circle.CCW {
		return nil, fmt.Errorf(`%w: this circle does not have the required outer-loop winding`, ErrUnsupported)
	}
	radius, rDelta, err := magnitudeInBounded(circle.Radius, units.Length, units.Millimeter, "the circle radius")
	if err != nil || rDelta != 0 {
		return nil, fmt.Errorf(`%w: the circle radius is not exact in millimetres`, ErrUnsupported)
	}
	var outer, inner ProfileRecord
	switch side {
	case ThickenPositive:
		outer, err = prismCircleOffset(budget, pp.profile, radius, -1, amount)
		inner = pp.profile
	case ThickenNegative:
		inner, err = prismCircleOffset(budget, pp.profile, radius, +1, amount)
		outer = pp.profile
	case ThickenCentered:
		outer, err = prismCircleOffset(budget, pp.profile, radius, -1, amount)
		if err == nil {
			inner, err = prismCircleOffset(budget, pp.profile, radius, +1, amount)
		}
	}
	if err != nil {
		return nil, err
	}
	// The two circles are concentric, and their radii were certified as
	// exact. Their strict radius order proves separation for every offset
	// parameter from zero through the requested endpoint.
	outerRadius := radius
	innerRadius := radius
	if side != ThickenNegative {
		outerRadius += amount
	}
	if side != ThickenPositive {
		innerRadius -= amount
	}
	if outerRadius <= innerRadius || innerRadius <= 0 {
		return nil, fmt.Errorf(`%w: the circle offsets do not bound a wall`, ErrUnsupported)
	}
	if side == ThickenPositive {
		return evalTubeContext(ctx, d, d.nextProducerID(), pp, outer, -1)
	}
	if side == ThickenNegative {
		return evalTubeContext(ctx, d, d.nextProducerID(), pp, inner, +1)
	}
	hole, err := reverseLoopRecordContext(ctx, inner.Outer)
	if err != nil {
		return nil, err
	}
	pp.profile = ProfileRecord{Outer: outer.Outer, Holes: []LoopRecord{hole}}
	pp.surfaceResult = false
	pp.walks = nil
	return evalPrismContext(ctx, d, d.nextProducerID(), pp, newFreeformWork())
}

func prismCircleOffset(budget *workBudget, source ProfileRecord, radius, sense, amount float64) (ProfileRecord, error) {
	offset, err := offsetProfile(budget, source, sense, amount)
	if err != nil {
		return ProfileRecord{}, err
	}
	if len(offset.Outer.Segments) != 1 {
		return ProfileRecord{}, fmt.Errorf(`%w: the circle offset changed feature count`, ErrUnsupported)
	}
	generated, ok := offset.Outer.Segments[0].(CircleSeg)
	sourceCircle, sourceOK := source.Outer.Segments[0].(CircleSeg)
	if !ok || !sourceOK || generated.Center != sourceCircle.Center || generated.CCW != sourceCircle.CCW {
		return ProfileRecord{}, fmt.Errorf(`%w: the circle offset changed feature kind`, ErrUnsupported)
	}
	got, delta, err := magnitudeInBounded(generated.Radius, units.Length, units.Millimeter, "the generated circle radius")
	if err != nil || delta != 0 {
		return ProfileRecord{}, fmt.Errorf(`%w: the generated circle radius is not exact`, ErrUnsupported)
	}
	want := new(big.Rat).Set(floatRat(radius))
	if sense < 0 {
		want.Add(want, floatRat(amount))
	} else {
		want.Sub(want, floatRat(amount))
	}
	if want.Sign() <= 0 || rationalFloatError(want, got) != 0 {
		return ProfileRecord{}, fmt.Errorf(`%w: the circle offset is not exactly representable`, ErrUnsupported)
	}
	if err := thickenAuditRefusal(auditOffsetSectionBudget(budget, source, offset)); err != nil {
		return ProfileRecord{}, err
	}
	return offset, nil
}

// The shared Shell audit can classify a flipped offset as ErrDegenerate.
// Thicken's topology-changing offset is R26, always ErrUnsupported; a
// cancelled call still returns its context error unchanged.
func thickenAuditRefusal(err error) error {
	if err == nil || errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) ||
		errors.Is(err, ErrUnsupported) {
		return err
	}
	return fmt.Errorf(`%w: the thicken offset audit refused: %v`, ErrUnsupported, err)
}
