package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/gatediameter"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tolerance"
)

// bodyGateDiameter reads a lower bound on the body's own diameter. The root
// owns the faceted record and exact carrier build; internal/gatediameter owns the
// payload witnesses and the fallback after the carrier decision.
func bodyGateDiameter(ctx context.Context, body *Body) (float64, bool, error) {
	if body == nil {
		return 0, false, nil
	}
	if payload, ok := body.n.Payload.(facetedPayload); ok {
		return payload.Diameter, tolerance.UsableMagnitude(payload.Diameter), nil
	}
	if d, ok, handled, err := gatediameter.DirectBodyGateDiameter(ctx, &body.n); handled || err != nil {
		return d, ok, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	_, carrierOK, err := newBodyGeomBudget(budget, body)
	if err != nil {
		return 0, false, err
	}
	return gatediameter.CarrierOrFallbackGateDiameter(ctx, budget, &body.n, carrierOK)
}

type gatePoints = diameter.GatePoints

// bodyGatePoints keeps the faceted payload's build-time vertex table in the
// root boundary. Other payloads read their witnesses from internal/gatediameter.
func bodyGatePoints(ctx context.Context, budget *proofbound.WorkBudget, body *Body,
	stations *diameter.StationMemo) (gatePoints, bool, error) {
	if pl, ok := body.n.Payload.(facetedPayload); ok {
		return gatePoints{Points: pl.Verts}, true, nil
	}
	return gatediameter.BodyGatePoints(ctx, budget, &body.n, stations)
}

// pairGateDiameterCached reads the diameter D of the pair a and b (verification
// design §3): the greatest distance between two points drawn from either
// body. It reads bodyGatePoints for both bodies together, so the reading
// ranges over pairs across the two bodies as well as within each, and
// shrinks the maximum by twice the larger allow. Every point is held within
// that allow of a point of one of the two bodies, so the reading is a lower
// bound on the pair's own diameter. A body with no point set leaves the pair
// with the other body's points alone, and a pair with neither reads zero: an
// understated D only tightens the gate.
//
// Each body's points come through cache.gatePoints, so a body read in an
// earlier pair of the same call is not read again. The joined set g starts
// empty and Join only appends into it, so a stored reading's slice is never
// appended to; keep it that way. A nil cache reads every body fresh.
func pairGateDiameterCached(ctx context.Context, a, b *Body, cache *bodyGeomCache) (float64, error) {
	budget := proofbound.NewWorkBudget(ctx)
	var g gatePoints
	for _, body := range [2]*Body{a, b} {
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
