package revolveplan

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// MeridianClearOfAxis reads Trim and Extend's S4 clearance clause off the axis
// snap that has ALREADY run: axisFrame.walk assigns a walk endpoint ρ exactly
// zero when the resolved axis's own snap tolerance covers it, so "clear" is
// the published structural fact that no endpoint carries that zero, never a
// fresh measurement decad takes on the geometry it was handed. A meridian
// touching the axis sweeps a pole, and a cut fragment ending at one would
// need pole topology this arm does not place.
func MeridianClearOfAxis(ctx context.Context, rp Payload) (bool, error) {
	work := freeform.NewFreeformWork()
	for _, loop := range append([]sectionrecord.LoopRecord{rp.Profile.Outer}, rp.Profile.Holes...) {
		resolved, err := revolveaxis.ResolveLoop(ctx, loop, work, "the trim axis-clearance gate",
			rp.ChargedWalk, rp.Ax.SnapTol)
		if err != nil {
			return false, err
		}
		for _, w := range resolved.Walks {
			if w.StartV == 0 || w.EndV == 0 {
				return false, nil
			}
		}
	}
	return true, nil
}

// ChainClearOfAxis is MeridianClearOfAxis over an OPEN meridian walk set,
// read through the chain resolver rather than the loop one: an open walk's
// last segment does not continue into its first, so coalescing it as a loop
// could merge the two free ends into one walk and hide an endpoint the
// clearance clause has to see.
func ChainClearOfAxis(ctx context.Context, rp ChainPayload) (bool, error) {
	work := freeform.NewFreeformWork()
	for ci := range rp.Chains {
		view := rp.WalkView(ci)
		resolved, err := revolveaxis.ResolveChain(ctx, rp.Chains[ci], work, "the revolve wall build",
			view.ChargedWalk, view.Ax.SnapTol)
		if err != nil {
			return false, err
		}
		for _, w := range resolved.Walks {
			if w.StartV == 0 || w.EndV == 0 {
				return false, nil
			}
		}
	}
	return true, nil
}
