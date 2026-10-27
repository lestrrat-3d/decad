package pairkernel

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/gatediameter"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tolerance"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// BodyGateDiameter reads a lower bound on the body's own diameter for
// Verify's tolerance gate (verification §3). This package owns the faceted
// record and exact carrier build; internal/gatediameter owns the payload
// witnesses and the fallback after the carrier decision. A nil body reads no
// diameter.
func BodyGateDiameter(ctx context.Context, body *topo.Body) (float64, bool, error) {
	if body == nil {
		return 0, false, nil
	}
	if payload, ok := body.Payload.(facetedrecord.Payload); ok {
		return payload.Diameter, tolerance.UsableMagnitude(payload.Diameter), nil
	}
	if d, ok, handled, err := gatediameter.DirectBodyGateDiameter(ctx, body); handled || err != nil {
		return d, ok, err
	}
	// The carrier build and the gate readers use this budget only through
	// Step, StepN and Err, so the poll budget returns what NewWorkBudget's
	// would at every step (see PairGateDiameter).
	budget := proofbound.NewPollWorkBudget(ctx)
	_, carrierOK, err := newBodyGeomBudget(budget, body)
	if err != nil {
		return 0, false, err
	}
	return gatediameter.CarrierOrFallbackGateDiameter(ctx, budget, body, carrierOK)
}

// bodyGatePoints reads the faceted payload's build-time vertex table here.
// Other payloads read their witnesses from internal/gatediameter.
func bodyGatePoints(ctx context.Context, budget *proofbound.WorkBudget, body *topo.Body,
	stations *diameter.StationMemo) (diameter.GatePoints, bool, error) {
	if pl, ok := body.Payload.(facetedrecord.Payload); ok {
		return diameter.GatePoints{Points: pl.Verts}, true, nil
	}
	return gatediameter.BodyGatePoints(ctx, budget, body, stations)
}

// PairGateDiameter reads the diameter D of the pair a and b (verification
// design §3): the greatest distance between two points drawn from either
// body. It reads bodyGatePoints for both bodies together, so the reading
// ranges over pairs across the two bodies as well as within each, and
// shrinks the maximum by twice the larger allow. Every point is held within
// that allow of a point of one of the two bodies, so the reading is a lower
// bound on the pair's own diameter. A body with no point set leaves the pair
// with the other body's points alone, and a pair with neither reads zero: an
// understated D only tightens the gate.
//
// Each body's points come through Cache.gatePoints, so a body read in an
// earlier pair of the same call is not read again. The joined set g starts
// empty and Join only appends into it, so a stored reading's slice is never
// appended to; keep it that way. A nil cache reads every body fresh.
func PairGateDiameter(ctx context.Context, a, b *topo.Body, cache *Cache) (float64, error) {
	// The gate readers use this budget only through Step, StepN and Err. The
	// poll budget counts steps as NewWorkBudget's does and calls ctx.Err at
	// the same multiples of WorkPollInterval, so every call returns what
	// NewWorkBudget's would, and the reading and the step at which a cancel
	// stops it are the same. Its StepN charges the diameter scan's skipped
	// blocks in one move per poll instead of one call per pair.
	budget := proofbound.NewPollWorkBudget(ctx)
	var g diameter.GatePoints
	for _, body := range [2]*topo.Body{a, b} {
		points, ok, err := cache.gatePoints(ctx, budget, body)
		if err != nil {
			return 0, err
		}
		if ok {
			g.Join(points)
		}
	}
	d, ok, err := g.Diameter(budget)
	if err != nil || !ok {
		return 0, err
	}
	return d, nil
}
