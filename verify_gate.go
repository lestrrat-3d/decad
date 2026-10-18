package decad

import (
	"context"

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
