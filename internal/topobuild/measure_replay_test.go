package topobuild_test

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

func TestValidateAnalyticMeasurementsReplayLimits(t *testing.T) {
	body := &topo.Body{}
	if err := topobuild.ValidateAnalyticMeasurements(body); err != nil {
		t.Fatal(err)
	}
	exact, err := proofbound.NewReplayBudgetLimits(4, 1, 1)
	if err != nil {
		t.Fatal(err)
	}
	if err := topobuild.ValidateAnalyticMeasurementsReplay(t.Context(), body, exact); err != nil {
		t.Fatal(err)
	}
	short, err := proofbound.NewReplayBudgetLimits(3, 1, 1)
	if err != nil {
		t.Fatal(err)
	}
	if err := topobuild.ValidateAnalyticMeasurementsReplay(t.Context(), body, short); err == nil {
		t.Fatal("one-short work admitted")
	}
	body.Bounds.Min = r3.Vec{X: math.NaN()}
	if err := topobuild.ValidateAnalyticMeasurementsReplay(t.Context(), body,
		proofbound.NewReplayTrialBudget()); !errors.Is(err, decaderr.ErrNotFinite) {
		t.Fatalf("nonfinite result err=%v", err)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	meter := proofbound.NewReplayTrialBudget()
	if err := topobuild.ValidateAnalyticMeasurementsReplay(ctx, body, meter); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancelled err=%v", err)
	}
	work, records := meter.Usage()
	if work != 0 || records != 0 || meter.Bytes() != 0 {
		t.Fatalf("cancelled charges work=%d records=%d bytes=%d", work, records, meter.Bytes())
	}
}
