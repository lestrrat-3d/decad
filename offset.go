package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is Body.Offset of docs/surface-design.md §17: a SECOND sheet whose
// every face lies at a stated normal distance from the source face it came
// from. It admits §16's two receiver families and reads the same recorded
// generator Thicken reads, and it differs from Thicken in exactly two ways.
//
// The receiver stays LIVE. An offset depends on its source and consumes
// nothing — the two are the pair a caller stitches, thickens or measures a
// clearance between — so the result registers on the non-consuming terms
// PlacedCopy and Duplicate already take (document.go's copyUnder,
// docs/api-design.md §6). Nothing sums the pair: Volume and Centroid are
// ErrNotSolid on a sheet by kind (§8), and Union/Cut/Intersect refuse a sheet
// permanently (Table X), so the two live bodies can never be combined into a
// reading that counts the same material twice.
//
// The result holds ONE region, bounded by the offset loop alone. Thicken
// assembles an annulus from an outer and an inner loop, so it must first prove
// the source and offset loops disjoint and strictly nested; this build
// publishes no such relation and denotes nothing whatever about the source, so
// that audit has no subject here and reverseLoopRecordContext and
// evalTubeContext are not on this path (§17.2). Everything else §16.2 states —
// offsetProfile, auditOffsetSectionBudget, the exact-generation gate and the
// whole-interval certification of thicken_axis.go — runs unchanged.

// OffsetOption configures which side of a sheet the offset is taken on.
type OffsetOption interface{ offsetOption() }

type offsetSideOption struct{ side OffsetSide }

func (offsetSideOption) offsetOption() {}

// OffsetSide names the side of the source sheet the offset sheet is placed on.
// It is its own two-value type rather than a reuse of [ThickenSide]: an offset
// publishes ONE surface and two surfaces are two calls, so there is no centered
// value, and ThickenCentered can never enter an operation that would have to
// refuse it at run time (docs/surface-design.md §17.1).
type OffsetSide int

const (
	// OffsetPositive places the offset along the sheet's positive normal and is
	// the default.
	OffsetPositive OffsetSide = iota
	// OffsetNegative places the offset opposite the sheet's positive normal.
	OffsetNegative
)

// WithOffsetSide selects the side the offset sheet is placed on.
func WithOffsetSide(side OffsetSide) OffsetOption { return offsetSideOption{side: side} }

// Offset returns a second sheet whose every face lies at the stated normal
// distance from the source face it came from, LEAVING THE RECEIVER LIVE.
//
// distance is a strictly positive [units.Value] of kind Length; its sign never
// chooses a side, and [WithOffsetSide] does, the last occurrence winning. A
// recorded planar patch offsets by translation along its own frame normal; a
// profile-fed prism sheet offsets by the certified per-feature section offset
// of docs/modify-design.md §8, growing a radius-distance arc at each convex
// corner and mitering each concave one. Every other sheet family, and every
// solid, is ErrUnsupported (docs/surface-design.md Table R row R37).
//
// A nil context or an option this package did not mint is ErrDegenerate, as is
// a zero distance; a wrong-kind, non-finite or negative distance keeps
// magnitudeIn's own sentinel (R40). A retired receiver is ErrRetiredBody (R41).
// A refusal registers nothing and retires nothing, so the receiver is live
// after a refusal exactly as it is after a success.
func (b *Body) Offset(ctx context.Context, distance units.Value, opts ...OffsetOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control an offset`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := b.doc
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	side := OffsetPositive
	for _, raw := range opts {
		o, ok := raw.(offsetSideOption)
		if !ok {
			return nil, fmt.Errorf(`%w: the offset option is not a decad offset option (%T)`, ErrDegenerate, raw)
		}
		if o.side < OffsetPositive || o.side > OffsetNegative {
			return nil, fmt.Errorf(`%w: unknown offset side %d`, ErrDegenerate, o.side)
		}
		side = o.side
	}
	dmm, dDelta, err := magnitudeInBounded(distance, units.Length, units.Millimeter, "the offset distance")
	if err != nil {
		return nil, err
	}
	if dmm == 0 {
		return nil, fmt.Errorf(`%w: a zero distance names no offset surface`, ErrDegenerate)
	}
	if b.Kind() != BodySheet {
		return nil, fmt.Errorf(`%w: Offset requires a sheet body`, ErrUnsupported)
	}
	var result *Body
	switch payload := b.payload.(type) {
	case patchPayload:
		result, err = offsetPatch(ctx, d, payload, side, dmm, dDelta)
	case prismPayload:
		result, err = offsetPrism(ctx, d, payload, side, dmm, dDelta)
	default:
		return nil, fmt.Errorf(`%w: this sheet has no admitted Offset generator`, ErrUnsupported)
	}
	if err != nil {
		return nil, err
	}
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	// The source is DEPENDED ON, never consumed: commit with no consumed input,
	// exactly as copyUnder does, so the receiver stays in Document.Bodies()
	// beside its offset (docs/surface-design.md §17.1).
	d.commit(result)
	return result, nil
}

// offsetPatch is §17.2's patch arm: the receiver's own payload rebuilt under
// the composed motion that translates it by ±d·N, N the placed frame normal the
// sheet's one face already publishes. It mints no geometry — a planar region
// carried along its own normal is congruent to itself — so the recorded
// profile, its frame axes and every plane-local coordinate travel unchanged and
// the rebuild is the identical one PlacedCopy performs, charging the identical
// placement rounding.
//
// A millimetre conversion that displaces the distance is R39. patchPayload
// carries no displacement term — no sectionDelta, no z0Delta/z1Delta — so a
// translation built from a rescaled float would move every published
// coordinate off the translation the call denotes with nothing to charge it
// against, and a zero bound is a positive claim of exactness. A distance stated
// in millimetres converts by a factor of one and reports zero.
func offsetPatch(ctx context.Context, d *Document, pp patchPayload, side OffsetSide, dmm, dDelta float64) (*Body, error) {
	if dDelta != 0 {
		return nil, fmt.Errorf(
			`%w: the offset distance's millimetre conversion displaces the translation, and a patch payload carries no term for it`,
			ErrUnsupported)
	}
	signed := dmm
	if side == OffsetNegative {
		signed = -dmm
	}
	motion, err := r3.Translation(pp.prism().dir(0, 0, 1).Scale(signed))
	if err != nil {
		return nil, fmt.Errorf(`%w: the offset translation is not a placement: %s`, ErrUnsupported, err)
	}
	composed, err := pp.transform().Then(motion)
	if err != nil {
		return nil, fmt.Errorf(`%w: composing the offset placement failed: %s`, ErrUnsupported, err)
	}
	return pp.placed(ctx, d, d.nextProducerID(), composed)
}

// offsetPrism is §17.2's prism arm: the source's own sweep over the offset
// section. The frame, both levels and their displacements, the accumulated
// placement and surfaceResult all travel unchanged; only the section is
// replaced, and the exact-generation gate below is what keeps the payload's
// sectionDelta == 0 truthful, so evalPrismContext's existing moment and extent
// proofs apply to the offset section as they do to a recorded one.
func offsetPrism(ctx context.Context, d *Document, pp prismPayload, side OffsetSide, dmm, dDelta float64) (*Body, error) {
	if !pp.surfaceResult || pp.sectionDelta != 0 || len(pp.profile.Holes) != 0 {
		return nil, fmt.Errorf(`%w: this prism sheet has no admitted Offset section`, ErrUnsupported)
	}
	if admitAbove(boundedSub(pp.z1Scalar(), pp.z0Scalar()), 0) != survAdmit {
		return nil, fmt.Errorf(`%w: the prism sheet has no proven positive sweep height`, ErrUnsupported)
	}
	if dDelta != 0 {
		return nil, fmt.Errorf(`%w: the offset distance cannot be generated exactly in millimetres`, ErrUnsupported)
	}
	// sense is offsetProfile's own: −1 grows the section (P ⊕ d) and +1 erodes
	// it (P ⊖ d), so the positive side of a profile-fed prism sheet — the
	// profile region's exterior on every loop wall (§17.1) — grows.
	sense := -1
	if side == OffsetNegative {
		sense = +1
	}
	budget := newWorkBudget(ctx)
	if err := budget.err(); err != nil {
		return nil, err
	}
	section, err := offsetPrismSection(ctx, pp, sense, dmm, budget)
	if err != nil {
		return nil, err
	}
	pp.profile = section
	// The walk cache resolves THIS payload's old profile; the new section
	// resolves afresh (prism_payload.go's walks doc comment).
	pp.walks = nil
	// No source face role is inherited (§17.3): a blend descriptor names (loop,
	// segment) indices of the SOURCE record, and the offset mints its own
	// segment list, so carrying them over would name arbitrary result walls.
	pp.blendSegs, pp.blendKind = nil, ""
	return evalPrismContext(ctx, d, d.nextProducerID(), pp, newFreeformWork())
}

// offsetPrismSection constructs and certifies the one section the result is
// built over, in §17.2's fixed order: construct the offset, certify its
// generated values exactly, run the shared §5 audit, then prove the whole
// offset interval 0 < τ ≤ d clear. It dispatches on the two admitted section
// shapes exactly as thickenPrism does, and refuses every other one.
func offsetPrismSection(ctx context.Context, pp prismPayload, sense int, amount float64, budget *workBudget) (ProfileRecord, error) {
	if len(pp.profile.Outer.Segments) == 1 {
		if circle, ok := pp.profile.Outer.Segments[0].(CircleSeg); ok {
			return offsetCircleSection(pp.profile, circle, sense, amount, budget)
		}
	}
	return offsetAxisSection(ctx, pp, sense, amount, budget)
}

// offsetCircleSection offsets a whole circle to a concentric one through
// prismCircleOffset (thicken_prism.go), which constructs the circle, certifies
// its generated centre and radius against the exact rational sum or difference,
// and runs the offset audit — the same gate bundle §16.2's own circle arm runs,
// unchanged.
//
// Its positive-radius certificate IS the whole-interval proof for this shape. A
// concentric offset radius is R(τ) = r − sense·τ, affine and monotone in τ, and
// a whole circle has no other contact event (§16.2), so a certified positive
// endpoint radius proves every intermediate radius positive too: growing
// (sense −1) increases it from r, and eroding (sense +1) reaches its minimum at
// the endpoint prismCircleOffset already refused a non-positive value for.
func offsetCircleSection(source ProfileRecord, circle CircleSeg, sense int, amount float64, budget *workBudget) (ProfileRecord, error) {
	if !circle.CCW {
		return ProfileRecord{}, fmt.Errorf(`%w: this circle does not have the required outer-loop winding`, ErrUnsupported)
	}
	radius, rDelta, err := magnitudeInBounded(circle.Radius, units.Length, units.Millimeter, "the circle radius")
	if err != nil || rDelta != 0 {
		return ProfileRecord{}, fmt.Errorf(`%w: the circle radius is not exact in millimetres`, ErrUnsupported)
	}
	offset, err := prismCircleOffset(budget, source, radius, float64(sense), amount)
	if err != nil {
		return ProfileRecord{}, offsetSectionRefusal(err)
	}
	return offset, nil
}

// offsetAxisSection offsets the axis-parallel line class. It reuses
// thicken_axis.go's own three gates unchanged — thickenAxisDirections for the
// class itself, thickenCertifyAxisOffset for the exact-generation gate over
// every generated endpoint, corner centre and radius, and
// thickenAxisIntervalClear for the whole-interval Sturm certification — with
// offsetProfile supplying the section and auditOffsetSectionBudget checking it.
//
// The interval proof is NOT optional on this shape and is never inferred from
// the endpoint: an endpoint that looks simple cannot prove an earlier offset did
// not pinch and change which boundary the construction denotes (§17.2).
func offsetAxisSection(ctx context.Context, pp prismPayload, sense int, amount float64, budget *workBudget) (ProfileRecord, error) {
	for _, seg := range pp.profile.Outer.Segments {
		if err := ctx.Err(); err != nil {
			return ProfileRecord{}, err
		}
		if _, ok := seg.(LineSeg); !ok {
			return ProfileRecord{}, fmt.Errorf(`%w: the prism sheet requires line-only axis-parallel walks`, ErrUnsupported)
		}
	}
	loops, err := prismCornerLoopsBudget(budget, pp)
	if err != nil {
		return ProfileRecord{}, err
	}
	if len(loops) != 1 {
		return ProfileRecord{}, fmt.Errorf(`%w: the prism sheet requires one outer loop`, ErrUnsupported)
	}
	loop := loops[0]
	dirs, err := thickenAxisDirections(loop, budget)
	if err != nil {
		return ProfileRecord{}, err
	}
	offset, err := offsetProfile(budget, pp.profile, float64(sense), amount)
	if err != nil {
		return ProfileRecord{}, offsetSectionRefusal(err)
	}
	if err := thickenCertifyAxisOffset(loop, dirs, offset.Outer, sense, amount, budget); err != nil {
		return ProfileRecord{}, err
	}
	if err := offsetSectionRefusal(auditOffsetSectionBudget(budget, pp.profile, offset)); err != nil {
		return ProfileRecord{}, err
	}
	// radial is nil: a prism sheet's walls sweep along a fixed normal and turn
	// about no axis, so there is no radius for the scan to keep positive
	// (thicken.go's thickenRadial is the revolve arm's gate alone).
	if err := thickenAxisIntervalClear(ctx, loop, dirs, sense, amount, budget, nil); err != nil {
		return ProfileRecord{}, err
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
		errors.Is(err, ErrUnsupported) {
		return err
	}
	return fmt.Errorf(`%w: the offset section audit refused: %v`, ErrUnsupported, err)
}
