package thickenaxis

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

// This file builds the offset sections of a prism sheet for docs/surface-design.md
// §16.2 (Thicken) and §17.2 (Offset). The shell tolerance the offset construction
// binds is the tol parameter of each builder; the root passes its shellTol.

// OffsetSection constructs and certifies the one section the result is
// built over, in §17.2's fixed order: construct the offset, certify its
// generated values exactly, run the shared §5 audit, then prove the whole
// offset interval 0 < τ ≤ d clear. It dispatches on the two admitted section
// shapes exactly as thickenPrism does, and refuses every other one.
func OffsetSection(ctx context.Context, profile momentinput.Profile, sense int, amount, tol float64, budget *proofbound.WorkBudget) (momentinput.Profile, error) {
	if len(profile.Outer.Segments) == 1 {
		if circle, ok := profile.Outer.Segments[0].(sectionrecord.CircleSeg); ok {
			return offsetCircleSection(profile, circle, sense, amount, tol, budget)
		}
	}
	return offsetAxisSection(ctx, profile, sense, amount, tol, budget)
}

// offsetCircleSection offsets a whole circle to a concentric one through
// CircleOffset, which constructs the circle, certifies
// its generated centre and radius against the exact rational sum or difference,
// and runs the offset audit — the same gate bundle §16.2's own circle arm runs,
// unchanged.
//
// Its positive-radius certificate IS the whole-interval proof for this shape. A
// concentric offset radius is R(τ) = r − sense·τ, affine and monotone in τ, and
// a whole circle has no other contact event (§16.2), so a certified positive
// endpoint radius proves every intermediate radius positive too: growing
// (sense −1) increases it from r, and eroding (sense +1) reaches its minimum at
// the endpoint CircleOffset already refused a non-positive value for.
func offsetCircleSection(source momentinput.Profile, circle sectionrecord.CircleSeg, sense int, amount, tol float64, budget *proofbound.WorkBudget) (momentinput.Profile, error) {
	if !circle.CCW {
		return momentinput.Profile{}, fmt.Errorf(`%w: this circle does not have the required outer-loop winding`, decaderr.ErrUnsupported)
	}
	radius, rDelta, err := extent.MagnitudeInBounded(circle.Radius, units.Length, units.Millimeter, "the circle radius")
	if err != nil || rDelta != 0 {
		return momentinput.Profile{}, fmt.Errorf(`%w: the circle radius is not exact in millimetres`, decaderr.ErrUnsupported)
	}
	offset, err := CircleOffset(budget, source, radius, float64(sense), amount, tol)
	if err != nil {
		return momentinput.Profile{}, offsetSectionRefusal(err)
	}
	return offset, nil
}

// offsetAxisSection offsets the axis-parallel line class. It reuses
// this package's three gates unchanged — AxisDirections for the
// class itself, CertifyAxisOffset for the exact-generation gate over
// every generated endpoint, corner centre and radius, and
// AxisIntervalClear for the whole-interval Sturm certification — with
// offset2d.OffsetProfile supplying the section and offset2d.AuditOffsetSectionBudget checking it.
//
// The interval proof is NOT optional on this shape and is never inferred from
// the endpoint: an endpoint that looks simple cannot prove an earlier offset did
// not pinch and change which boundary the construction denotes (§17.2).
func offsetAxisSection(ctx context.Context, profile momentinput.Profile, sense int, amount, tol float64, budget *proofbound.WorkBudget) (momentinput.Profile, error) {
	for _, seg := range profile.Outer.Segments {
		if err := ctx.Err(); err != nil {
			return momentinput.Profile{}, err
		}
		if _, ok := seg.(sectionrecord.LineSeg); !ok {
			return momentinput.Profile{}, fmt.Errorf(`%w: the prism sheet requires line-only axis-parallel walks`, decaderr.ErrUnsupported)
		}
	}
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, profile)
	if err != nil {
		return momentinput.Profile{}, err
	}
	if len(loops) != 1 {
		return momentinput.Profile{}, fmt.Errorf(`%w: the prism sheet requires one outer loop`, decaderr.ErrUnsupported)
	}
	loop := loops[0]
	dirs, err := AxisDirections(loop.Walks, budget)
	if err != nil {
		return momentinput.Profile{}, err
	}
	offset, err := offset2d.OffsetProfile(budget, profile, float64(sense), amount, tol)
	if err != nil {
		return momentinput.Profile{}, offsetSectionRefusal(err)
	}
	if err := CertifyAxisOffset(loop.Walks, dirs, offset.Outer, sense, amount, budget); err != nil {
		return momentinput.Profile{}, err
	}
	if err := offsetSectionRefusal(offset2d.AuditOffsetSectionBudget(budget, profile, offset)); err != nil {
		return momentinput.Profile{}, err
	}
	// radial is nil: a prism sheet's walls sweep along a fixed normal and turn
	// about no axis, so there is no radius for the scan to keep positive
	// (thickenbuild's radial gate is for the revolve arm alone).
	if err := AxisIntervalClear(ctx, loop.Walks, dirs, sense, amount, budget, nil); err != nil {
		return momentinput.Profile{}, err
	}
	return offset, nil
}

// offsetSectionRefusal lands every section-construction refusal on R38. The
// shared Shell audit can classify a flipped offset as ErrDegenerate; an offset
// that changes the section's topology can still denote a sheet after trimming,
// so R38 is this evaluator's reach (ErrUnsupported) rather than a claim that
// the caller's positive distance is malformed. A cancelled call still returns
// its context error unchanged.
func offsetSectionRefusal(err error) error {
	if err == nil || errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) ||
		errors.Is(err, decaderr.ErrUnsupported) {
		return err
	}
	return fmt.Errorf(`%w: the offset section audit refused: %v`, decaderr.ErrUnsupported, err)
}

// AxisOffset is the exact offset of one axis-parallel outer loop by amount on
// the given sense, certified exact against dirs and audited.
func AxisOffset(budget *proofbound.WorkBudget, source momentinput.Profile, loop survey2d.CornerLoop,
	dirs []AxisDir, sense int, amount, tol float64) (momentinput.Profile, error) {
	offset, err := offset2d.OffsetProfile(budget, source, float64(sense), amount, tol)
	if err != nil {
		return momentinput.Profile{}, AuditRefusal(err)
	}
	if err := CertifyAxisOffset(loop.Walks, dirs, offset.Outer, sense, amount, budget); err != nil {
		return momentinput.Profile{}, err
	}
	if err := offset2d.AuditOffsetSectionBudget(budget, source, offset); err != nil {
		return momentinput.Profile{}, AuditRefusal(err)
	}
	return offset, nil
}

// CircleOffset is the concentric offset of a whole-circle section: it
// constructs the circle, certifies its generated centre and radius against the
// exact rational sum or difference of the source radius and amount, and runs
// the offset audit.
func CircleOffset(budget *proofbound.WorkBudget, source momentinput.Profile, radius, sense, amount, tol float64) (momentinput.Profile, error) {
	offset, err := offset2d.OffsetProfile(budget, source, sense, amount, tol)
	if err != nil {
		return momentinput.Profile{}, err
	}
	if len(offset.Outer.Segments) != 1 {
		return momentinput.Profile{}, fmt.Errorf(`%w: the circle offset changed feature count`, decaderr.ErrUnsupported)
	}
	generated, ok := offset.Outer.Segments[0].(sectionrecord.CircleSeg)
	sourceCircle, sourceOK := source.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || !sourceOK || generated.Center != sourceCircle.Center || generated.CCW != sourceCircle.CCW {
		return momentinput.Profile{}, fmt.Errorf(`%w: the circle offset changed feature kind`, decaderr.ErrUnsupported)
	}
	got, delta, err := extent.MagnitudeInBounded(generated.Radius, units.Length, units.Millimeter, "the generated circle radius")
	if err != nil || delta != 0 {
		return momentinput.Profile{}, fmt.Errorf(`%w: the generated circle radius is not exact`, decaderr.ErrUnsupported)
	}
	want := new(big.Rat).Set(proofarith.FloatRat(radius))
	if sense < 0 {
		want.Add(want, proofarith.FloatRat(amount))
	} else {
		want.Sub(want, proofarith.FloatRat(amount))
	}
	if want.Sign() <= 0 || proofarith.RationalFloatError(want, got) != 0 {
		return momentinput.Profile{}, fmt.Errorf(`%w: the circle offset is not exactly representable`, decaderr.ErrUnsupported)
	}
	if err := AuditRefusal(offset2d.AuditOffsetSectionBudget(budget, source, offset)); err != nil {
		return momentinput.Profile{}, err
	}
	return offset, nil
}
