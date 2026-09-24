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
	amount, err := thickenAmount(tmm, tDelta, side)
	if err != nil {
		return nil, err
	}
	budget := newWorkBudget(ctx)
	if err := budget.err(); err != nil {
		return nil, err
	}
	sec, err := thickenSectionOf(ctx, pp.profile, side, amount, budget, nil)
	if err != nil {
		return nil, err
	}
	if side != ThickenCentered {
		return evalTubeContext(ctx, d, d.nextProducerID(), pp, sec.generated(side), sec.sense(side))
	}
	annulus, err := thickenAnnulus(ctx, sec)
	if err != nil {
		return nil, err
	}
	pp.profile = annulus
	pp.surfaceResult = false
	pp.walks = nil
	return evalPrismContext(ctx, d, d.nextProducerID(), pp, newFreeformWork())
}

// thickenAmount is the per-side offset magnitude: the whole thickness for a
// one-sided call, half of it for a centered one. A thickness no millimetre
// conversion can state exactly, and a half-thickness that is not itself
// representable, are both R27 (docs/surface-design.md §16.2).
func thickenAmount(tmm, tDelta float64, side ThickenSide) (float64, error) {
	if tDelta != 0 {
		return 0, fmt.Errorf(`%w: the thickness cannot be generated exactly in millimetres`, ErrUnsupported)
	}
	if side != ThickenCentered {
		return tmm, nil
	}
	half := boundedQuotient(tmm, 0, 2, 0)
	if half.bound != 0 || half.value <= 0 {
		return 0, fmt.Errorf(`%w: the half-thickness is not exactly representable`, ErrUnsupported)
	}
	return half.value, nil
}

// thickenSection is the certified offset pair one Thicken arm sweeps: the
// outer section and the inner one it strictly encloses. One of the two IS the
// source section for a one-sided call; a centered call generates both.
type thickenSection struct {
	source       ProfileRecord
	outer, inner ProfileRecord
}

// generated names the offset section a one-sided call built, and sense the
// erosion sign evalTubeContext reads it with.
func (s thickenSection) generated(side ThickenSide) ProfileRecord {
	if side == ThickenNegative {
		return s.inner
	}
	return s.outer
}

func (s thickenSection) sense(side ThickenSide) float64 {
	if side == ThickenNegative {
		return +1
	}
	return -1
}

// thickenAnnulus assembles the certified pair into one hole-free-outer,
// one-hole section: the outer loop with the inner loop reversed into its hole
// walk.
func thickenAnnulus(ctx context.Context, sec thickenSection) (ProfileRecord, error) {
	hole, err := reverseLoopRecordContext(ctx, sec.inner.Outer)
	if err != nil {
		return ProfileRecord{}, err
	}
	return ProfileRecord{Outer: sec.outer.Outer, Holes: []LoopRecord{hole}}, nil
}

// thickenSectionOf certifies both offsets of one recorded closed section: the
// whole-circle arm where the section is a single CircleSeg, the axis-parallel
// line arm otherwise. radial is the revolve arm's radial gate and nil for a
// prism, whose walls never turn about an axis.
func thickenSectionOf(ctx context.Context, profile ProfileRecord, side ThickenSide,
	amount float64, budget *workBudget, radial *thickenRadial) (thickenSection, error) {
	if len(profile.Outer.Segments) == 1 {
		if circle, ok := profile.Outer.Segments[0].(CircleSeg); ok {
			return thickenCircleSection(profile, circle, side, amount, budget, radial)
		}
	}
	return thickenAxisSection(ctx, profile, side, amount, budget, radial)
}

// thickenCircleSection certifies the concentric offsets of a whole-circle
// section. The two circles share a center and their radii were certified
// exact, so their strict radius order proves separation for every offset
// parameter from zero through the requested endpoint.
func thickenCircleSection(profile ProfileRecord, circle CircleSeg, side ThickenSide,
	amount float64, budget *workBudget, radial *thickenRadial) (thickenSection, error) {
	if !circle.CCW {
		return thickenSection{}, fmt.Errorf(`%w: this circle does not have the required outer-loop winding`, ErrUnsupported)
	}
	radius, rDelta, err := magnitudeInBounded(circle.Radius, units.Length, units.Millimeter, "the circle radius")
	if err != nil || rDelta != 0 {
		return thickenSection{}, fmt.Errorf(`%w: the circle radius is not exact in millimetres`, ErrUnsupported)
	}
	sec := thickenSection{source: profile, outer: profile, inner: profile}
	if side != ThickenNegative {
		sec.outer, err = prismCircleOffset(budget, profile, radius, -1, amount)
		if err != nil {
			return thickenSection{}, err
		}
	}
	if side != ThickenPositive {
		sec.inner, err = prismCircleOffset(budget, profile, radius, +1, amount)
		if err != nil {
			return thickenSection{}, err
		}
	}
	outerRadius, innerRadius := radius, radius
	if side != ThickenNegative {
		outerRadius += amount
	}
	if side != ThickenPositive {
		innerRadius -= amount
	}
	if outerRadius <= innerRadius || innerRadius <= 0 {
		return thickenSection{}, fmt.Errorf(`%w: the circle offsets do not bound a wall`, ErrUnsupported)
	}
	if radial != nil {
		// The whole swept family is the concentric circles of radius at most
		// outerRadius about one fixed center, so the least radius any of them
		// reaches from the axis is the center's own less that outermost radius.
		least := new(big.Rat).Sub(radial.rho(floatRat(circle.Center.U), floatRat(circle.Center.V)), floatRat(outerRadius))
		if err := radial.require(least); err != nil {
			return thickenSection{}, err
		}
	}
	return sec, nil
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
