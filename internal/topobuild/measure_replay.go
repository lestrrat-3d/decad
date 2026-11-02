package topobuild

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ValidateAnalyticMeasurementsReplay checks the same four finite readings
// under the caller's meter and returns a static error on a failed reading.
func ValidateAnalyticMeasurementsReplay(ctx context.Context, body *topo.Body,
	budget *proofbound.ReplayBudget) error {
	if budget == nil || body == nil {
		return decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 4); err != nil {
		return err
	}
	if !measurement.FiniteValues(body.Volume.Value.Base(), body.Volume.Bound.Base()) ||
		!measurement.FiniteValues(body.Area.Value.Base(), body.Area.Bound.Base()) ||
		!measurement.FiniteValues(body.Centroid.Value.X, body.Centroid.Value.Y,
			body.Centroid.Value.Z, body.Centroid.Bound.Base()) ||
		!measurement.FiniteValues(body.Bounds.Min.X, body.Bounds.Min.Y, body.Bounds.Min.Z,
			body.Bounds.Max.X, body.Bounds.Max.Y, body.Bounds.Max.Z, body.Bounds.Bound.Base()) {
		return decaderr.ErrNotFinite
	}
	return nil
}
