package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/prismrecord"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/thickenaxis"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is Offset of docs/surface-design.md §17: a SECOND sheet whose
// every face lies at a stated normal distance from the source face it came
// from. It admits §16's two receiver families and reads the same recorded
// generator Thicken reads, and it differs from Thicken in exactly two ways.
//
// The receiver stays LIVE. An offset depends on its source and consumes
// nothing — the two are the pair a caller stitches, thickens or measures a
// clearance between — so the result registers on the non-consuming terms
// Copy already takes (document.go's copyUnder,
// docs/api-design.md §6). Nothing sums the pair: Volume and Centroid are
// ErrNotSolid on a sheet by kind (§8), and Union/Cut/Intersect refuse a sheet
// permanently (Table X), so the two live bodies can never be combined into a
// reading that counts the same material twice.
//
// The result holds ONE region, bounded by the offset loop alone. Thicken
// assembles an annulus from an outer and an inner loop, so it must first prove
// the source and offset loops disjoint and strictly nested; this build
// publishes no such relation and denotes nothing whatever about the source, so
// that audit has no subject here and offset2d.ReverseLoopRecordContext and
// prismshell.BuildTube are not on this path (§17.2). Everything else §16.2 states —
// offset2d.OffsetProfile, offset2d.AuditOffsetSectionBudget, the exact-generation gate and the
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
// distance from the source face of b it came from, LEAVING b LIVE.
//
// distance is a strictly positive [units.Value] of kind Length; its sign never
// chooses a side, and [WithOffsetSide] does, the last occurrence winning. A
// recorded planar patch offsets by translation along its own frame normal; a
// profile-fed prism sheet offsets by the certified per-feature section offset
// of docs/modify-design.md §8, growing a radius-distance arc at each convex
// corner and mitering each concave one. Every other sheet family, and every
// solid, is ErrUnsupported (docs/surface-design.md Table R row R37).
//
// A nil context, a nil b or an option this package did not mint is
// ErrDegenerate, as is a zero distance; a wrong-kind, non-finite or negative distance keeps
// sectionrecord.MagnitudeIn's own sentinel (R40). A retired b is ErrRetiredBody (R41).
// A refusal registers nothing and retires nothing, so b is live
// after a refusal exactly as it is after a success.
func Offset(ctx context.Context, b *Body, distance units.Value, opts ...OffsetOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control an offset`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
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
	dmm, dDelta, err := extent.MagnitudeInBounded(distance, units.Length, units.Millimeter, "the offset distance")
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
	switch payload := b.n.Payload.(type) {
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
// the rebuild is the identical one Copy performs, charging the identical
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
	motion, err := r3.Translation(pp.Prism().Dir(0, 0, 1).Scale(signed))
	if err != nil {
		return nil, fmt.Errorf(`%w: the offset translation is not a placement: %s`, ErrUnsupported, err)
	}
	composed, err := pp.Transform().Then(motion)
	if err != nil {
		return nil, fmt.Errorf(`%w: composing the offset placement failed: %s`, ErrUnsupported, err)
	}
	return placePayload(ctx, pp, d, d.nextProducerID(), composed)
}

// offsetPrism is §17.2's prism arm: the source's own sweep over the offset
// section. The frame, both levels and their displacements, the accumulated
// placement and surfaceResult all travel unchanged; only the section is
// replaced, and the exact-generation gate below is what keeps the payload's
// sectionDelta == 0 truthful, so prismrecord.Build's existing moment and extent
// proofs apply to the offset section as they do to a recorded one.
func offsetPrism(ctx context.Context, d *Document, pp prismPayload, side OffsetSide, dmm, dDelta float64) (*Body, error) {
	if !pp.SurfaceResult || pp.SectionDelta != 0 || len(pp.Profile.Holes) != 0 {
		return nil, fmt.Errorf(`%w: this prism sheet has no admitted Offset section`, ErrUnsupported)
	}
	if proofbound.AdmitAbove(proofbound.BoundedSub(pp.Z1Scalar(), pp.Z0Scalar()), 0) != proofbound.SurvAdmit {
		return nil, fmt.Errorf(`%w: the prism sheet has no proven positive sweep height`, ErrUnsupported)
	}
	if dDelta != 0 {
		return nil, fmt.Errorf(`%w: the offset distance cannot be generated exactly in millimetres`, ErrUnsupported)
	}
	// sense is offset2d.OffsetProfile's own: −1 grows the section (P ⊕ d) and +1 erodes
	// it (P ⊖ d), so the positive side of a profile-fed prism sheet — the
	// profile region's exterior on every loop wall (§17.1) — grows.
	sense := -1
	if side == OffsetNegative {
		sense = +1
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	section, err := thickenaxis.OffsetSection(ctx, pp.Profile, sense, dmm, shellTol, budget)
	if err != nil {
		return nil, err
	}
	pp.Profile = section
	// The walk cache resolves THIS payload's old profile; the new section
	// resolves afresh (internal/prismrecord's walks doc comment).
	pp.Walks = nil
	// No source face role is inherited (§17.3): a blend descriptor names (loop,
	// segment) indices of the SOURCE record, and the offset mints its own
	// segment list, so carrying them over would name arbitrary result walls.
	pp.BlendSegs, pp.BlendKind = nil, ""
	return bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), d.nextProducerID()), pp, freeform.NewFreeformWork()))
}
