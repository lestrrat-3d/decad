package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// tryAxialLoftPrismUnion admits a Loft only when its authenticated source
// denotes an exact straight prism. The prism arranger then certifies every
// planar crossing and seam through sketch's exact fragments. This first
// subcase has no segment-count, radial-placement, or source-identity gate.
// A varying or curved Loft remains on the existing paths until its own wall
// intersection and source-boundary proofs are implemented.
func tryAxialLoftPrismUnion(ctx context.Context, op meshbool.OperationKind, a, b *Body) (prismPayload, bool, error) {
	if op != meshbool.OpUnion {
		return prismPayload{}, false, nil
	}
	pa, aPrism := a.n.Payload.(prismPayload)
	pb, bPrism := b.n.Payload.(prismPayload)
	la, aLoft := a.n.Payload.(loftPayload)
	lb, bLoft := b.n.Payload.(loftPayload)
	if aPrism && bLoft {
		var ok bool
		var err error
		pb, ok, err = loftrecord.ExactAxialPrism(ctx, lb)
		if err != nil || !ok {
			return prismPayload{}, false, err
		}
	} else if aLoft && bPrism {
		var ok bool
		var err error
		pa, ok, err = loftrecord.ExactAxialPrism(ctx, la)
		if err != nil || !ok {
			return prismPayload{}, false, err
		}
	} else {
		return prismPayload{}, false, nil
	}
	left := bornBody(&Body{n: topo.Body{Payload: pa}})
	right := bornBody(&Body{n: topo.Body{Payload: pb}})
	// A shared edge with no interior overlap is a tangent contact. Require
	// the same arranger to resolve a positive-area intersection before
	// asking it to merge the two outlines.
	_, overlap, err := tryPrismBoolean(ctx, meshbool.OpIntersect, left, right)
	if err != nil || !overlap {
		return prismPayload{}, false, err
	}
	return tryPrismBoolean(ctx, op, left, right)
}
