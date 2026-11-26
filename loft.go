package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/opcommand"

	"github.com/lestrrat-3d/sketch"
)

// This file is agent-docs/loft-design.md PR 1b: the public entry point over PR 1a's
// evaluator (internal/loftrecord). It owns the LoftCommand surface, gates S9-S10,
// and the atomic record->evaluate->commit tail (§10). The loft methods replace
// a repeated call, so S4's arity half and S11 have no counterpart. The other gates — S1-S8 and S13's coordinate-range
// gate — is loftmesh.ValidateLoftRecords', loftmesh.AssembleTriangulated's and loftmesh.LoftCrossingAuditStructured's, run
// inside loftrecord.Build in §4's stated order.

// LoftSplit names one recorded segment of one loop and the fractions of its
// parameter range at which Splits cuts it. Loop 0 is the outer loop and
// loop 1+h is Holes[h]; Segment is the recorded segment index within that loop.
// Every fraction is strictly inside (0, 1), and they increase.
type LoftSplit struct {
	Loop    int
	Segment int
	At      []float64
}

// Loft builds a solid ruled between two profiles recorded on distinct
// geometric planes (agent-docs/loft-design.md), registers it, and returns the new
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
// and this evaluator cannot hold its vertex table. SurfaceResult omits
// both section caps and publishes a sheet body instead of a solid
// (agent-docs/surface-design.md §4).
//
// A failed call leaves the document untouched.
func Loft(d *Document, s0 *sketch.Sketch, p0 *sketch.Profile, s1 *sketch.Sketch, p1 *sketch.Profile) *LoftCommand {
	c := &LoftCommand{d: d, s0: s0, p0: p0, s1: s1, p1: p1}
	if d == nil {
		c.state.Record("d", errNilDocument)
	}
	// S10: the message names the four arguments, and the subject the first nil one.
	for _, a := range []struct {
		name string
		nil  bool
	}{{"s0", s0 == nil}, {"p0", p0 == nil}, {"s1", s1 == nil}, {"p1", p1 == nil}} {
		if a.nil {
			c.state.Record(a.name, fmt.Errorf(`%w: Loft requires two non-nil sketches and two non-nil profiles`, ErrDegenerate))
			break
		}
	}
	return c
}

// LoftCommand builds a solid ruled between two profiles. Build it with [Loft].
type LoftCommand struct {
	state     opcommand.State               // consumed flag and first recorded fault
	d         *Document                     // stored as given
	s0, s1    *sketch.Sketch                // stored as given
	p0, p1    *sketch.Profile               // stored as given
	surface   surfaceResultSetting          // set by SurfaceResult
	alignment opcommand.Slot[[]int]         // set by Alignment
	splits    opcommand.Slot[loftSplitPair] // set by Splits
}

// loftSplitPair holds the copied splits of the first and the second section.
type loftSplitPair struct{ first, second []featureoption.LoftSplit }

// Alignment records, per loop, which recorded segment index of the
// SECOND profile pairs with segment index 0 of the FIRST profile's
// corresponding loop. offsets[0] is the outer loop; offsets[1+h] is
// p1.Holes[h]. Omitting the method means every offset is 0 — the natural
// case where both profiles were authored with matching segment order
// (agent-docs/loft-design.md §2/§3 Table P row P4). A second call replaces the
// first.
func (c *LoftCommand) Alignment(offsets ...int) *LoftCommand {
	// An empty call stores an empty, non-nil list: the call names a payload of no offsets, which the
	// pairing gate refuses, and never means no alignment.
	c.alignment.Set(append([]int{}, offsets...), nil)
	return c
}

// Splits cuts recorded segments of the first and of the second section
// before the loft pairs them, so two sections whose loops hold different numbers
// of segments can be paired one to one. Each piece is the same recorded curve over
// a sub-range of the segment's own parameter range, the way a trimmed fragment is
// recorded, so a split changes no curve. A fraction f resolves to the parameter
// TStart + f·(TEnd − TStart) of the segment, which is proportional to length on a
// line and a parameter of the curve on any other kind. The pieces follow each
// other in the loop's walk order. [LoftCommand.Alignment] then applies to the
// split loops, and a wall's source role names the split segment index.
//
// A second call replaces the first. A split index outside its loop, a fraction
// outside (0, 1) or not increasing, a segment split twice, and a fraction that
// rounds onto its neighbour are [ErrDegenerate]. Loops still holding different
// segment counts, or a pair still of different kinds, after the splits are
// [ErrUnsupported] as without the method, and a correspondence that twists is
// refused by the crossing audit.
func (c *LoftCommand) Splits(first, second []LoftSplit) *LoftCommand {
	conv := func(in []LoftSplit) []featureoption.LoftSplit {
		out := make([]featureoption.LoftSplit, len(in))
		for i, s := range in {
			out[i] = featureoption.LoftSplit{Loop: s.Loop, Segment: s.Segment, At: s.At}
		}
		return featureoption.CloneSplits(out)
	}
	c.splits.Set(loftSplitPair{first: conv(first), second: conv(second)}, nil)
	return c
}

// SurfaceResult omits both section caps and publishes a sheet body instead of
// a solid (agent-docs/surface-design.md §4). Calling it twice is the same as
// calling it once.
func (c *LoftCommand) SurfaceResult() *LoftCommand { c.surface.set(); return c }

// Do builds the loft, registers the new body and returns it, or returns the
// error of the first gate that refuses, leaving the document unchanged. A nil
// ctx is ErrDegenerate, and a second call returns ErrConsumed.
func (c *LoftCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationLoft, c.run)
}

func (c *LoftCommand) run(ctx context.Context) (*Body, error) {
	cfg := featureoption.LoftConfig{SurfaceResult: c.surface.on}
	if c.alignment.IsSet() {
		cfg.Alignment, _ = c.alignment.Get()
	}
	if c.splits.IsSet() {
		pair, _ := c.splits.Get()
		cfg.Splits = [2][]featureoption.LoftSplit{pair.first, pair.second}
	}
	return loft(ctx, c.d, c.s0, c.p0, c.s1, c.p1, cfg)
}

// loft is Loft's implementation. ctx is not nil, and the errors it returns
// carry no operation tag (agent-docs/command-api-design.md §11).
func loft(ctx context.Context, d *Document, s0 *sketch.Sketch, p0 *sketch.Profile,
	s1 *sketch.Sketch, p1 *sketch.Profile, cfg featureoption.LoftConfig) (*Body, error) {
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	// S10: an explicit nil check ahead of the seam, so the message names
	// which of the four arguments was nil (RecordProfile would answer the
	// same sentinel, but less specifically).
	if s0 == nil || p0 == nil || s1 == nil || p1 == nil {
		return nil, fmt.Errorf(`%w: Loft requires two non-nil sketches and two non-nil profiles`, ErrDegenerate)
	}

	if err := d.requireSameContainer(nil, s0, s1); err != nil {
		return nil, err
	}
	// S9: both profiles through the unmodified seam gates, in argument order.
	node, err := loftrecord.FromProfiles(ctx, envOf(documentNode(d), d.producerAt(ctx)), s0, p0, s1, p1, cfg)
	if err != nil {
		return nil, err
	}
	body := bodyHandle(node)
	if err := d.commitOperationIn(ctx, d.ContainerOfSketch(s0), OperationLoft, nil, body); err != nil {
		return nil, err
	}
	return body, nil
}

// This section is LoftChain of agent-docs/loft-design.md §16: the ruled ribbon
// between two OPEN sketch chains. §16.1 states how Table P reads over two open
// walks — P1 and P2 collapse onto one walk with no holes, P3 and P5 survive
// verbatim, P4 loses its modular wrap so segment j pairs with segment j, and
// P6's intrinsic winding is replaced by the walk direction sketch itself
// publishes. §16.2 states the one thing a closed shell supplied that two open
// walks do not — the positive side — and the exactly-parallel plane gate that
// replaces it.

// LoftChain builds a sheet ruled between the open chains c0 (of s0) and c1 (of
// s1), registers it, and returns the new body. c0 is the FROM walk and c1 the
// TO walk, the same argument order Loft uses for its two sections and the
// order §16.2's plane gate reads its positive normal from. Both chains pass
// the identical seam gates chain Extrude runs (RecordChain,
// agent-docs/surface-design.md §13.3), in argument order, and the seam's own
// sentinel wins before any shape gate.
//
// The two walks MUST carry the same recorded segment count (ErrUnsupported),
// and segment j of c0 pairs with segment j of c1 — there is no alignment
// offset for an open walk, since a nonzero one would pair a segment across a
// free end. Every paired segment MUST be a LineSeg pair in this increment: a
// mixed-kind pairing and a same-kind circular pairing are each ErrUnsupported
// (agent-docs/loft-design.md Table SL rows SL4 and SL7). The two recorded planes
// MUST be exactly parallel with the to-plane strictly on the from-plane's
// positive side, which is what states the ribbon's positive side
// (ErrUnsupported, SL5); coplanar sections are ErrDegenerate (S5). A build-time
// audit proves the ruled walls cross nowhere but their own shared edges and
// vertices, and a proven crossing is ErrDegenerate (S7) — which is what
// refuses a pair whose two published walks run opposite ways.
//
// The result is always a sheet — Kind() == BodySheet — two flat triangles per
// chord cell with no cap and no closing face, so SurfaceResult does not
// compile against this call. A failed call leaves the document untouched.
func LoftChain(d *Document, s0 *sketch.Sketch, c0 *sketch.Chain, s1 *sketch.Sketch, c1 *sketch.Chain) *LoftChainCommand {
	c := &LoftChainCommand{d: d, s0: s0, c0: c0, s1: s1, c1: c1}
	if d == nil {
		c.state.Record("d", errNilDocument)
	}
	for _, a := range []struct {
		name string
		nil  bool
	}{{"s0", s0 == nil}, {"c0", c0 == nil}, {"s1", s1 == nil}, {"c1", c1 == nil}} {
		if a.nil {
			c.state.Record(a.name, fmt.Errorf(`%w: LoftChain requires two non-nil sketches and two non-nil chains`, ErrDegenerate))
			break
		}
	}
	return c
}

// LoftChainCommand builds a sheet ruled between two open chains. Build it with
// [LoftChain]. It has no method: a chain loft always returns a sheet, so it has
// no SurfaceResult, and its segment pairing leaves no alignment.
type LoftChainCommand struct {
	state  opcommand.State // consumed flag and first recorded fault
	d      *Document       // stored as given
	s0, s1 *sketch.Sketch  // stored as given
	c0, c1 *sketch.Chain   // stored as given
}

// Do builds the ribbon, registers the new body and returns it, or returns the
// error of the first gate that refuses, leaving the document unchanged. A nil
// ctx is ErrDegenerate, and a second call returns ErrConsumed.
func (c *LoftChainCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationLoftChain, c.run)
}

func (c *LoftChainCommand) run(ctx context.Context) (*Body, error) {
	return loftChain(ctx, c.d, c.s0, c.c0, c.s1, c.c1)
}

// loftChain is LoftChain's implementation. ctx is not nil, and the errors it
// returns carry no operation tag (agent-docs/command-api-design.md §11).
func loftChain(ctx context.Context, d *Document, s0 *sketch.Sketch, c0 *sketch.Chain, s1 *sketch.Sketch,
	c1 *sketch.Chain) (*Body, error) {
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if s0 == nil || c0 == nil || s1 == nil || c1 == nil {
		return nil, fmt.Errorf(`%w: LoftChain requires two non-nil sketches and two non-nil chains`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	// SL2 before every shape gate, in argument order: a seam refusal names a
	// repair the caller makes in the sketch, and a shape refusal reported
	// first would hide it (agent-docs/loft-design.md Table SL).
	if err := d.requireSameContainer(nil, s0, s1); err != nil {
		return nil, err
	}
	chain0, plane0, err := recordChain(s0, c0)
	if err != nil {
		return nil, err
	}
	chain1, plane1, err := recordChain(s1, c1)
	if err != nil {
		return nil, err
	}

	node, err := loftrecord.FromChains(ctx, envOf(documentNode(d), d.producerAt(ctx)), chain0, plane0, chain1, plane1)
	if err != nil {
		return nil, err
	}
	body := bodyHandle(node)
	if err := d.commitOperationIn(ctx, d.ContainerOfSketch(s0), OperationLoftChain, nil, body); err != nil {
		return nil, err
	}
	return body, nil
}
