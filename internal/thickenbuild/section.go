package thickenbuild

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/thickenaxis"
	"github.com/lestrrat-3d/units"
)

// Amount is the per-side offset magnitude: the whole thickness for a
// one-sided call, half of it for a centered one. A thickness no millimetre
// conversion can state exactly, and a half-thickness that is not itself
// representable, are both R27 (docs/surface-design.md §16.2).
func Amount(tmm, tDelta float64, side Side) (float64, error) {
	if tDelta != 0 {
		return 0, fmt.Errorf(`%w: the thickness cannot be generated exactly in millimetres`, decaderr.ErrUnsupported)
	}
	if side != Centered {
		return tmm, nil
	}
	half := proofbound.BoundedQuotient(tmm, 0, 2, 0)
	if half.Bound != 0 || half.Value <= 0 {
		return 0, fmt.Errorf(`%w: the half-thickness is not exactly representable`, decaderr.ErrUnsupported)
	}
	return half.Value, nil
}

// SectionOf certifies both offsets of one recorded closed section: the
// whole-circle arm where the section is a single CircleSeg, the axis-parallel
// line arm otherwise. radial is the revolve arm's radial gate and nil for a
// prism, whose walls never turn about an axis.
func SectionOf(ctx context.Context, profile momentinput.Profile, side Side,
	amount float64, budget *proofbound.WorkBudget, radial *thickenaxis.Radial) (thickenaxis.SectionPair, error) {
	if len(profile.Outer.Segments) == 1 {
		if circle, ok := profile.Outer.Segments[0].(sectionrecord.CircleSeg); ok {
			return circleSection(profile, circle, side, amount, budget, radial)
		}
	}
	return thickenaxis.AxisSection(ctx, profile,
		side != Negative, side != Positive,
		amount, offset2d.ShellTol, budget, radial)
}

// circleSection certifies the concentric offsets of a whole-circle
// section. The two circles share a center and their radii were certified
// exact, so their strict radius order proves separation for every offset
// parameter from zero through the requested endpoint.
func circleSection(profile momentinput.Profile, circle sectionrecord.CircleSeg, side Side,
	amount float64, budget *proofbound.WorkBudget, radial *thickenaxis.Radial) (thickenaxis.SectionPair, error) {
	if !circle.CCW {
		return thickenaxis.SectionPair{}, fmt.Errorf(`%w: this circle does not have the required outer-loop winding`, decaderr.ErrUnsupported)
	}
	radius, rDelta, err := extent.MagnitudeInBounded(circle.Radius, units.Length, units.Millimeter, "the circle radius")
	if err != nil || rDelta != 0 {
		return thickenaxis.SectionPair{}, fmt.Errorf(`%w: the circle radius is not exact in millimetres`, decaderr.ErrUnsupported)
	}
	sec := thickenaxis.NewSectionPair(profile)
	if side != Negative {
		sec.Outer, err = thickenaxis.CircleOffset(budget, profile, radius, -1, amount, offset2d.ShellTol)
		if err != nil {
			return thickenaxis.SectionPair{}, err
		}
	}
	if side != Positive {
		sec.Inner, err = thickenaxis.CircleOffset(budget, profile, radius, +1, amount, offset2d.ShellTol)
		if err != nil {
			return thickenaxis.SectionPair{}, err
		}
	}
	outerRadius, innerRadius := radius, radius
	if side != Negative {
		outerRadius += amount
	}
	if side != Positive {
		innerRadius -= amount
	}
	if outerRadius <= innerRadius || innerRadius <= 0 {
		return thickenaxis.SectionPair{}, fmt.Errorf(`%w: the circle offsets do not bound a wall`, decaderr.ErrUnsupported)
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
