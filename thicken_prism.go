package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/extent"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/thickenaxis"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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
	if proofbound.AdmitAbove(proofbound.BoundedSub(pp.z1Scalar(), pp.z0Scalar()), 0) != proofbound.SurvAdmit {
		return nil, fmt.Errorf(`%w: the prism sheet has no proven positive sweep height`, ErrUnsupported)
	}
	amount, err := thickenAmount(tmm, tDelta, side)
	if err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	sec, err := thickenSectionOf(ctx, pp.profile, side, amount, budget, nil)
	if err != nil {
		return nil, err
	}
	if side != ThickenCentered {
		negative := side == ThickenNegative
		return evalTubeContext(ctx, d, d.nextProducerID(), pp, sec.Generated(negative), sec.Sense(negative))
	}
	annulus, err := sec.Annulus(ctx)
	if err != nil {
		return nil, err
	}
	pp.profile = annulus
	pp.surfaceResult = false
	pp.walks = nil
	return evalPrismContext(ctx, d, d.nextProducerID(), pp, freeform.NewFreeformWork())
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
	half := proofbound.BoundedQuotient(tmm, 0, 2, 0)
	if half.Bound != 0 || half.Value <= 0 {
		return 0, fmt.Errorf(`%w: the half-thickness is not exactly representable`, ErrUnsupported)
	}
	return half.Value, nil
}

// thickenSectionOf certifies both offsets of one recorded closed section: the
// whole-circle arm where the section is a single CircleSeg, the axis-parallel
// line arm otherwise. radial is the revolve arm's radial gate and nil for a
// prism, whose walls never turn about an axis.
func thickenSectionOf(ctx context.Context, profile profileRecord, side ThickenSide,
	amount float64, budget *proofbound.WorkBudget, radial *thickenRadial) (thickenaxis.SectionPair, error) {
	if len(profile.Outer.Segments) == 1 {
		if circle, ok := profile.Outer.Segments[0].(circleSeg); ok {
			return thickenCircleSection(profile, circle, side, amount, budget, radial)
		}
	}
	return thickenAxisSection(ctx, profile, side, amount, budget, radial)
}

// thickenCircleSection certifies the concentric offsets of a whole-circle
// section. The two circles share a center and their radii were certified
// exact, so their strict radius order proves separation for every offset
// parameter from zero through the requested endpoint.
func thickenCircleSection(profile profileRecord, circle circleSeg, side ThickenSide,
	amount float64, budget *proofbound.WorkBudget, radial *thickenRadial) (thickenaxis.SectionPair, error) {
	if !circle.CCW {
		return thickenaxis.SectionPair{}, fmt.Errorf(`%w: this circle does not have the required outer-loop winding`, ErrUnsupported)
	}
	radius, rDelta, err := extent.MagnitudeInBounded(circle.Radius, units.Length, units.Millimeter, "the circle radius")
	if err != nil || rDelta != 0 {
		return thickenaxis.SectionPair{}, fmt.Errorf(`%w: the circle radius is not exact in millimetres`, ErrUnsupported)
	}
	sec := thickenaxis.NewSectionPair(profile)
	if side != ThickenNegative {
		sec.Outer, err = prismCircleOffset(budget, profile, radius, -1, amount)
		if err != nil {
			return thickenaxis.SectionPair{}, err
		}
	}
	if side != ThickenPositive {
		sec.Inner, err = prismCircleOffset(budget, profile, radius, +1, amount)
		if err != nil {
			return thickenaxis.SectionPair{}, err
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
		return thickenaxis.SectionPair{}, fmt.Errorf(`%w: the circle offsets do not bound a wall`, ErrUnsupported)
	}
	if radial != nil {
		// The whole swept family is the concentric circles of radius at most
		// outerRadius about one fixed center, so the least radius any of them
		// reaches from the axis is the center's own less that outermost radius.
		least := new(big.Rat).Sub(radial.Rho(proofarith.FloatRat(circle.Center.U), proofarith.FloatRat(circle.Center.V)), proofarith.FloatRat(outerRadius))
		if err := radial.Require(least); err != nil {
			return thickenaxis.SectionPair{}, err
		}
	}
	return sec, nil
}

func prismCircleOffset(budget *proofbound.WorkBudget, source profileRecord, radius, sense, amount float64) (profileRecord, error) {
	offset, err := offsetProfile(budget, source, sense, amount)
	if err != nil {
		return profileRecord{}, err
	}
	if len(offset.Outer.Segments) != 1 {
		return profileRecord{}, fmt.Errorf(`%w: the circle offset changed feature count`, ErrUnsupported)
	}
	generated, ok := offset.Outer.Segments[0].(circleSeg)
	sourceCircle, sourceOK := source.Outer.Segments[0].(circleSeg)
	if !ok || !sourceOK || generated.Center != sourceCircle.Center || generated.CCW != sourceCircle.CCW {
		return profileRecord{}, fmt.Errorf(`%w: the circle offset changed feature kind`, ErrUnsupported)
	}
	got, delta, err := extent.MagnitudeInBounded(generated.Radius, units.Length, units.Millimeter, "the generated circle radius")
	if err != nil || delta != 0 {
		return profileRecord{}, fmt.Errorf(`%w: the generated circle radius is not exact`, ErrUnsupported)
	}
	want := new(big.Rat).Set(proofarith.FloatRat(radius))
	if sense < 0 {
		want.Add(want, proofarith.FloatRat(amount))
	} else {
		want.Sub(want, proofarith.FloatRat(amount))
	}
	if want.Sign() <= 0 || proofarith.RationalFloatError(want, got) != 0 {
		return profileRecord{}, fmt.Errorf(`%w: the circle offset is not exactly representable`, ErrUnsupported)
	}
	if err := thickenaxis.AuditRefusal(auditOffsetSectionBudget(budget, source, offset)); err != nil {
		return profileRecord{}, err
	}
	return offset, nil
}
