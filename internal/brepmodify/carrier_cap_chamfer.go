package brepmodify

import (
	"context"
	"errors"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// capLoopRouteDisabled switches the cap-loop route off so a test can compare
// today's answer with the route's; nothing in production sets it.
var capLoopRouteDisabled atomic.Bool

// CarrierCapLoopChamfer is the cap-loop route of
// agent-docs/prism-boolean-design.md Part K §8.1: a Chamfer whose selection is every
// edge of one or more complete cap loops of a prism receiver whose valid
// carrier record cr marks every selected loop exact builds the chamfer of the
// section the receiver denotes, as a capblend.DisplacedPayload that charges
// each other loop's record displacement LoopH. The setbacks are the exact
// path's (one setback on both caps, or asym's per-cap pair).
//
// matched is false for every selection or refusal the route does not own: a
// single straight cap edge, a lateral or malformed selection, a selected loop
// with a nonzero entry, a failing gate, offset audit or transfer condition
// (§8.3), and SX6's drop. Only cancellation is returned as an error, so every
// other refusal falls through to RequireExactSection and brep-modify §2.2's
// faceted view as before the route existed.
func CarrierCapLoopChamfer(ctx context.Context, env topo.Env, pp prismrecord.Payload, caps PrismCaps,
	cr prismrecord.CarrierRecord, sel string, edges []*topo.Edge, dmm, dDelta float64, asym *Asymmetric) (Result, bool, error) {
	if capLoopRouteDisabled.Load() || straightPrismCapEdge(caps, edges) {
		return Result{}, false, nil
	}
	startLoops, endLoops, lateral, err := ClassifyChamferSelection(ctx, pp, caps, sel, edges)
	if err != nil {
		return Result{}, false, cancellation(ctx, err)
	}
	if lateral {
		return Result{}, false, nil
	}
	for _, loops := range []map[int]bool{startLoops, endLoops} {
		for l, on := range loops {
			if on && !cr.ExactLoop(l) {
				return Result{}, false, nil
			}
		}
	}
	start := capblend.Setback{Dc: dmm, DcDelta: dDelta, Ds: dmm, DsDelta: dDelta}
	end := start
	if asym != nil {
		if start, end, err = asym.CapSetbacks(caps, edges, startLoops, endLoops); err != nil {
			return Result{}, false, cancellation(ctx, err)
		}
	}
	n := 1 + len(pp.Profile.Holes)
	loopDelta := make([]float64, n)
	for l := range loopDelta {
		loopDelta[l] = cr.LoopH(l)
	}
	node, err := capblend.BuildCheckedDisplaced(ctx, env, pp, start, end, startLoops, endLoops, loopDelta)
	if err != nil {
		return Result{}, false, cancellation(ctx, err)
	}
	return Result{Body: node}, true, nil
}

// cancellation returns err when it is a cancellation and nil otherwise, the
// only refusal the cap-loop route lets through (§8.1).
func cancellation(ctx context.Context, err error) error {
	if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) || ctx.Err() != nil {
		return err
	}
	return nil
}
