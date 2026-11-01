package tessellation_test

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/units"
)

func TestCircleSamplesReplay(t *testing.T) {
	seg := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 4, V: -2}, Radius: units.Millimeters(3), CCW: true, TEnd: 1}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) (tessellation.ChordSamples[int], error) {
		return tessellation.CircleSamplesReplay(ctx, seg, 17, .2, 7, b)
	}
	meter := proofbound.NewReplayTrialBudget()
	out, err := run(t.Context(), meter)
	if err != nil {
		t.Fatal(err)
	}
	n := len(out.Samples)
	if n < 3 || out.MaxSag > .2 || out.CapSlack <= 0 || out.WallSlack <= 0 {
		t.Fatalf("count=%d sag=%g cap=%g wall=%g", n, out.MaxSag, out.CapSlack, out.WallSlack)
	}
	for i, p := range out.Samples {
		if out.FaceOf[i] != 17 || !out.BoundOf[i].Derivable() || math.IsNaN(p.U) || math.IsNaN(p.V) {
			t.Fatalf("invalid station %d", i)
		}
		u, v, ok := circularbounds.EndpointInterval(circularbounds.RecordSegment(seg), big.NewRat(int64(i), int64(n)))
		if !ok || out.BoundOf[i].U < proofbound.IntervalFloatError(u, p.U) || out.BoundOf[i].V < proofbound.IntervalFloatError(v, p.V) {
			t.Fatalf("station %d bound does not contain independent interval", i)
		}
	}
	w, r := meter.Usage()
	bytes := meter.Bytes()
	for _, mode := range []string{replayBudgetTestExact, replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		b := proofbound.NewReplayTrialBudget()
		ww, rr, bb := proofbound.ReplayTrialWork-w, proofbound.ReplayTrialRecords-r, proofbound.ReplayTrialBytes-bytes
		switch mode {
		case replayBudgetTestWork:
			ww++
		case replayBudgetTestRecords:
			rr++
		case replayBudgetTestBytes:
			bb++
		}
		if err = b.SpendWork(t.Context(), ww); err != nil {
			t.Fatal(err)
		}
		if err = b.ReserveRecords(t.Context(), rr); err != nil {
			t.Fatal(err)
		}
		if err = b.ReserveBytes(t.Context(), bb); err != nil {
			t.Fatal(err)
		}
		_, err = run(t.Context(), b)
		if mode == replayBudgetTestExact && err != nil || mode != replayBudgetTestExact && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("mode=%s error=%v", mode, err)
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, err = run(ctx, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancel=%v", err)
	}
	seg.TEnd = .5
	if _, err = run(t.Context(), proofbound.NewReplayTrialBudget()); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("partial=%v", err)
	}
}
