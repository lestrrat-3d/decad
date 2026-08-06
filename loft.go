package decad

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-go/option/v3"
)

// This file is docs/loft-design.md PR 1b: the public entry point over PR 1a's
// evaluator (loft_build.go). It owns the LoftOption surface, WithLoftAlignment,
// gates S9-S11 and S4's arity half (§2/§4), and the atomic record->evaluate->
// commit tail (§10). Every other gate (S1-S8) is validateLoftRecords' and
// loftCrossingAudit's, run inside evalLoft in §4's stated order.

// LoftOption configures Loft.
type LoftOption interface {
	option.Interface
	loftOption()
}

type loftOption struct{ option.Interface }

func (loftOption) loftOption() {}

type identLoftAlignment struct{}

// WithLoftAlignment records, per loop, which recorded segment index of the
// SECOND profile pairs with segment index 0 of the FIRST profile's
// corresponding loop. offsets[0] is the outer loop; offsets[1+h] is
// p1.Holes[h]. Omitting the option means every offset is 0 — the natural
// case where both profiles were authored with matching segment order
// (docs/loft-design.md §2/§3 Table P row P4).
//
// The option is accepted at most once: two payloads are two different
// correspondences, so a repeat is [ErrDegenerate] rather than last-wins
// (Table S row S4).
func WithLoftAlignment(offsets ...int) LoftOption {
	out := make([]int, len(offsets))
	copy(out, offsets)
	return loftOption{option.New(identLoftAlignment{}, out)}
}

// Loft calls [Document.LoftContext] with [context.Background].
func (d *Document) Loft(s0 *sketch.Sketch, p0 *sketch.Profile, s1 *sketch.Sketch, p1 *sketch.Profile, opts ...LoftOption) (*Body, error) {
	return d.LoftContext(context.Background(), s0, p0, s1, p1, opts...)
}

// LoftContext builds a solid ruled between two profiles recorded on distinct
// geometric planes (docs/loft-design.md), registers it, and returns the new
// body. s0/p0 is the FROM section (capStart); s1/p1 is the TO section
// (capEnd) — the same naming Extrude already uses for its two caps. Both
// profiles pass the unmodified core §7 seam gates independently, in argument
// order (p0 of s0, then p1 of s1): a foreign, stale, invalid, or
// unrecordable profile is the seam's own sentinel.
//
// Every corresponding segment pair must be a LineSeg on both sides — a
// same-kind CircleSeg or ArcSeg pair is also [ErrUnsupported] in this
// increment (§1). The two profiles must lie on distinct geometric planes;
// coplanar sections are [ErrDegenerate] (§4, S5), since every wall vertex
// would then lie in one plane and the solid has zero volume by construction.
// A hole-count or per-loop segment-count mismatch has no positional or
// one-to-one pairing and is [ErrUnsupported] (S1/S2). A build-time audit
// proves the ruled walls do not cross or self-touch anywhere but their
// recorded shared edges and vertices (§6); a proven crossing is
// [ErrDegenerate] (S7), and exhausting the audit's fixed pair-test budget is
// [ErrUnsupported] (S8). A section point whose world coordinate runs past the
// representable float64 range is [ErrUnsupported] (S13) — the body exists,
// and this evaluator cannot hold its vertex table.
//
// A failed call leaves the recipe and the document untouched.
func (d *Document) LoftContext(ctx context.Context, s0 *sketch.Sketch, p0 *sketch.Profile, s1 *sketch.Sketch, p1 *sketch.Profile, opts ...LoftOption) (*Body, error) {
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	// S10: an explicit nil check ahead of the seam, so the message names
	// which of the four arguments was nil (RecordProfile would answer the
	// same sentinel, but less specifically).
	if s0 == nil || p0 == nil || s1 == nil || p1 == nil {
		return nil, fmt.Errorf(`%w: Loft requires two non-nil sketches and two non-nil profiles`, ErrDegenerate)
	}

	// S11 (option ownership) and S4's ARITY half (a repeated
	// WithLoftAlignment), both pre-gates decidable with no record at all
	// (docs/loft-design.md §4, amended). decad owns the option vocabulary, so
	// a foreign concrete type — including one that embeds LoftOption to
	// promote the sealed marker — is rejected before its Ident() ever runs.
	var alignment []int
	haveAlignment := false
	for _, raw := range opts {
		if raw == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		o, ok := raw.(loftOption)
		if !ok {
			return nil, fmt.Errorf(`%w: the loft option is not a decad loft option (%T)`, ErrDegenerate, raw)
		}
		switch ident := o.Ident().(type) {
		case identLoftAlignment:
			if haveAlignment {
				return nil, fmt.Errorf(`%w: WithLoftAlignment was passed more than once; two alignment payloads name two different correspondences`, ErrDegenerate)
			}
			v, ok := option.Get[[]int](o)
			if !ok {
				return nil, fmt.Errorf(`%w: WithLoftAlignment carries no offsets`, ErrDegenerate)
			}
			// option.Get is a plain type assertion: it hands back the
			// option's own stored slice header, not a copy. A caller that
			// retains the LoftOption value and mutates that slice through
			// option.Get (or Option[[]int].Value()) later would otherwise
			// reach into this step's recorded Alignment and loftPayload's
			// alignment (docs/loft-design.md §10) after the fact — clone
			// here, at ingress, so no caller-owned slice survives into
			// either.
			alignment = slices.Clone(v)
			haveAlignment = true
		default:
			return nil, fmt.Errorf(`%w: unknown loft option identifier %T`, ErrDegenerate, ident)
		}
	}

	// S9: both profiles through the unmodified seam gates, in argument order.
	profile0, plane0, area0, err := recordProfile(s0, p0)
	if err != nil {
		return nil, err
	}
	profile1, plane1, area1, err := recordProfile(s1, p1)
	if err != nil {
		return nil, err
	}

	// evaluator §1's one live-profile read, reject-only (not a new gate —
	// docs/loft-design.md §2 states no new gate here; both Extrude and
	// Revolve run the identical falsifier on their own recorded profile). One
	// counter per profile, threaded through to evalLoft so a single loft
	// operation opens exactly two R7 ceilings — one per record — rather than
	// four (docs/spline-design.md §5.2).
	work0 := newFreeformWork()
	work1 := newFreeformWork()
	if err := falsifyRecordedArea(profile0, area0, work0); err != nil {
		return nil, err
	}
	if err := falsifyRecordedArea(profile1, area1, work1); err != nil {
		return nil, err
	}

	frame0, err := r3.NewFrame(plane0.Origin, plane0.U, plane0.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the first recorded plane is degenerate: %s`, ErrDegenerate, err)
	}
	frame1, err := r3.NewFrame(plane1.Origin, plane1.U, plane1.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the second recorded plane is degenerate: %s`, ErrDegenerate, err)
	}

	// The recorded step (§10): Profile/Plane carry the FROM section exactly
	// as Extrude/Revolve record theirs; the TO section and the alignment ride
	// in LoftOpts. Alignment records the caller's supplied payload verbatim
	// (nil when the option was omitted) — never the normalized offsets
	// validateLoftRecords resolves inside evalLoft — since the decoder
	// restores an absent Alignment to the same all-zero intent (§10).
	step := Step{
		Op:      OpLoft,
		Profile: profile0,
		Plane:   plane0,
		Opts: LoftOpts{
			Profile2:  profile1,
			Plane2:    plane1,
			Alignment: alignment,
		},
	}
	ref := d.nextStepRef()

	body, err := evalLoft(ctx, d, ref, loftPayload{
		profile0: profile0, profile1: profile1,
		plane0: plane0, plane1: plane1,
		frame0: frame0, frame1: frame1,
		alignment: alignment,
		xform:     r3.Identity(),
	}, newWorkBudget(ctx), work0, work1)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(step, body)
	return body, nil
}
