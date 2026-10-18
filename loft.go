package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/sectioncert"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-go/option/v3"
)

// This file is docs/loft-design.md PR 1b: the public entry point over PR 1a's
// evaluator (internal/loftrecord). It owns the LoftOption surface, gates S9-S10,
// and the atomic record->evaluate->commit tail (§10). internal/featureoption
// owns S11 and S4's arity half. The other gates — S1-S8 and S13's coordinate-range
// gate — is loftmesh.ValidateLoftRecords', loftmesh.AssembleTriangulated's and loftmesh.LoftCrossingAuditStructured's, run
// inside loftrecord.Build in §4's stated order.

// LoftOption configures Loft.
type LoftOption interface {
	option.Interface
	loftOption()
}

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
	return loftOptionValue{featureoption.WithLoftAlignment(offsets...)}
}

// Loft builds a solid ruled between two profiles recorded on distinct
// geometric planes (docs/loft-design.md), registers it, and returns the new
// body. s0/p0 is the FROM section (capStart); s1/p1 is the TO section
// (capEnd) — the same naming Extrude already uses for its two caps. Both
// profiles pass the unmodified core §7 seam gates independently, in argument
// order (p0 of s0, then p1 of s1): a foreign, stale, invalid, or
// unrecordable profile is the seam's own sentinel.
//
// Every corresponding segment pair must be same-kind, over the recorded
// segment type: both LineSeg, both ArcSeg, both CircleSeg, or both the same
// Tier A free-form type (SplineSeg, ClosedSplineSeg, a NURBSSeg with equal
// weights, or FitSplineSeg). An ArcSeg paired against a CircleSeg, or a
// SplineSeg against a FitSplineSeg, is a mixed-kind pairing like any other,
// and a mixed-kind pairing is [ErrUnsupported] (§1, P5, S3). A free-form pair
// whose two curves convert to different Bézier span counts uses exact common
// subdivision (§5.1). A circular or free-form pair's walls are chorded
// (§5.1), which carries three refusals of its own, each [ErrUnsupported]: a
// pair the fixed station cap cannot chord to its chord target (S15), a build
// whose certified sagitta or station displacement has no derivation from the
// two records (S14), and a chord cell that collapses to one point on exactly
// one of the two sections (S16).
// The two profiles must lie on distinct geometric planes;
// coplanar sections are [ErrDegenerate] (§4, S5), since every wall vertex
// would then lie in one plane and the solid has zero volume by construction.
// A hole-count or per-loop segment-count mismatch has no positional or
// one-to-one pairing and is [ErrUnsupported] (S1/S2). A build-time audit
// proves the ruled walls do not cross or self-touch anywhere but their
// recorded shared edges and vertices (§6); a proven crossing is
// [ErrDegenerate] (S7's audit arm). A circular pair whose two sides walk in
// opposite senses is that same crossing — the correspondence walls each side
// against the other's reversed walk — and is the same [ErrDegenerate] (S7's
// structural arm), decided from the two records before any triangle is built.
// Exhausting the audit's fixed pair-test budget is
// [ErrUnsupported] (S8). A section point whose world coordinate runs past the
// representable float64 range is [ErrUnsupported] (S13) — the body exists,
// and this evaluator cannot hold its vertex table. WithSurfaceResult() omits
// both section caps and publishes a sheet body instead of a solid
// (docs/surface-design.md §4).
//
// A failed call leaves the document untouched.
func Loft(ctx context.Context, d *Document, s0 *sketch.Sketch, p0 *sketch.Profile, s1 *sketch.Sketch, p1 *sketch.Profile, opts ...LoftOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a loft`, ErrDegenerate)
	}
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
	cfg, err := decodeLoftOptions(opts)
	if err != nil {
		return nil, err
	}
	alignment, surfaceResult := cfg.Alignment, cfg.SurfaceResult

	// S9: both profiles through the unmodified seam gates, in argument order.
	profile0, plane0, area0, err := sectioncert.RecordProfile(s0, p0)
	if err != nil {
		return nil, err
	}
	profile1, plane1, area1, err := sectioncert.RecordProfile(s1, p1)
	if err != nil {
		return nil, err
	}

	// evaluator §1's one live-profile read, reject-only (not a new gate —
	// docs/loft-design.md §2 states no new gate here; both Extrude and
	// Revolve run the identical falsifier on their own recorded profile). One
	// counter per profile, threaded through to loftrecord.Build so a single loft
	// operation opens exactly two R7 ceilings — one per record — rather than
	// four (docs/spline-design.md §5.2).
	// The two integrals the falsifier computes are also the records' own
	// areas the chord target reads (docs/loft-gear-bounds-design.md §5), so
	// they ride on the payload rather than being integrated again in loftrecord.Build.
	work0 := freeform.NewFreeformWork()
	work1 := freeform.NewFreeformWork()
	// The profile area falsifier spends the same record-wide counter as the
	// later station walk. A many-flank profile can exhaust the default R7
	// ceiling during that first integral, before ValidateLoftRecords raises it.
	// Give both authenticated records their existing station-scaled ceiling
	// here; the later gate keeps that ceiling and the already spent work.
	p := uint64(len(profile0.Outer.Segments))
	for _, hole := range profile0.Holes {
		p += uint64(len(hole.Segments))
	}
	limit := loftmesh.StationWorkLimit(0, p)
	work0.RaiseLimit(limit)
	work1.RaiseLimit(limit)
	work0.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
	work1.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
	recordArea0, err := profile0.FalsifyLoftRecordedArea(area0, work0)
	if err != nil {
		return nil, err
	}
	recordArea1, err := profile1.FalsifyLoftRecordedArea(area1, work1)
	if err != nil {
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

	ref := d.nextProducerID()

	body, err := bodyResult(loftrecord.Build(ctx, envOf(documentNode(d), ref), loftPayload{
		Profile0: profile0, Profile1: profile1,
		Plane0: plane0, Plane1: plane1,
		Frame0: frame0, Frame1: frame1,
		Alignment:                   alignment,
		Xform:                       r3.Identity(),
		AuthenticatedReconstruction: true,
		SurfaceResult:               surfaceResult,
		RecordArea:                  [2]float64{recordArea0, recordArea1},
	}, proofbound.NewWorkBudget(ctx), work0, work1))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}

// This section is LoftChain of docs/loft-design.md §16: the ruled ribbon
// between two OPEN sketch chains. §16.1 states how Table P reads over two open
// walks — P1 and P2 collapse onto one walk with no holes, P3 and P5 survive
// verbatim, P4 loses its modular wrap so segment j pairs with segment j, and
// P6's intrinsic winding is replaced by the walk direction sketch itself
// publishes. §16.2 states the one thing a closed shell supplied that two open
// walks do not — the positive side — and the exactly-parallel plane gate that
// replaces it.

// ChainLoftOption configures LoftChain. It is its own sealed tier rather than
// [LoftOption]: WithSurfaceResult() does not implement it, so the compiler
// refuses that option outright rather than accepting it as a no-op — a
// chain-fed loft always returns a sheet, so there is no cap for the option to
// omit (docs/surface-design.md §13.2, docs/loft-design.md §16.3).
// WithLoftAlignment is not a member either: Table P's own P4 row forces a
// chain pair's offset to 0, so the option would name a correspondence that
// does not exist. This placeholder tier has no concrete option yet; it reserves the
// call for a later chain-only option.
type ChainLoftOption interface {
	option.Interface
	chainLoftOption()
}

// LoftChain builds a sheet ruled between the open chains c0 (of s0) and c1 (of
// s1), registers it, and returns the new body. c0 is the FROM walk and c1 the
// TO walk, the same argument order Loft uses for its two sections and the
// order §16.2's plane gate reads its positive normal from. Both chains pass
// the identical seam gates chain Extrude runs (RecordChain,
// docs/surface-design.md §13.3), in argument order, and the seam's own
// sentinel wins before any shape gate.
//
// The two walks MUST carry the same recorded segment count (ErrUnsupported),
// and segment j of c0 pairs with segment j of c1 — there is no alignment
// offset for an open walk, since a nonzero one would pair a segment across a
// free end. Every paired segment MUST be a LineSeg pair in this increment: a
// mixed-kind pairing and a same-kind circular pairing are each ErrUnsupported
// (docs/loft-design.md Table SL rows SL4 and SL7). The two recorded planes
// MUST be exactly parallel with the to-plane strictly on the from-plane's
// positive side, which is what states the ribbon's positive side
// (ErrUnsupported, SL5); coplanar sections are ErrDegenerate (S5). A build-time
// audit proves the ruled walls cross nowhere but their own shared edges and
// vertices, and a proven crossing is ErrDegenerate (S7) — which is what
// refuses a pair whose two published walks run opposite ways.
//
// The result is always a sheet — Kind() == BodySheet — two flat triangles per
// chord cell with no cap and no closing face, so WithSurfaceResult() does not
// compile against this call. A failed call leaves the document untouched.
func LoftChain(ctx context.Context, d *Document, s0 *sketch.Sketch, c0 *sketch.Chain, s1 *sketch.Sketch, c1 *sketch.Chain, opts ...ChainLoftOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a loft`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if s0 == nil || c0 == nil || s1 == nil || c1 == nil {
		return nil, fmt.Errorf(`%w: LoftChain requires two non-nil sketches and two non-nil chains`, ErrDegenerate)
	}
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	// SL2 before every shape gate, in argument order: a seam refusal names a
	// repair the caller makes in the sketch, and a shape refusal reported
	// first would hide it (docs/loft-design.md Table SL).
	chain0, plane0, err := recordChain(s0, c0)
	if err != nil {
		return nil, err
	}
	chain1, plane1, err := recordChain(s1, c1)
	if err != nil {
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

	ref := d.nextProducerID()
	body, err := bodyResult(loftrecord.BuildChain(ctx, envOf(documentNode(d), ref), chainLoftPayload{
		Chain0: chain0, Chain1: chain1,
		Plane0: plane0, Plane1: plane1,
		Frame0: frame0, Frame1: frame1,
		Xform: r3.Identity(),
	}, proofbound.NewWorkBudget(ctx), freeform.NewFreeformWork(), freeform.NewFreeformWork()))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}
