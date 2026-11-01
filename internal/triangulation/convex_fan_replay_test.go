package triangulation

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestConvexFanReplay(t *testing.T) {
	for _, n := range []int{3, 4, 41, 300, 1068} {
		pts := make([]Point2, n)
		poly := make([]int, n)
		for i := range pts {
			a := 2 * math.Pi * float64(i) / float64(n)
			pts[i] = Point2{U: 5 + 2*math.Cos(a), V: -7 + 2*math.Sin(a)}
			poly[i] = i
		}
		want, err := EarClip(t.Context(), pts, poly)
		if err != nil {
			t.Fatal(err)
		}
		meter := proofbound.NewReplayTrialBudget()
		got, err := ConvexFanReplay(t.Context(), pts, poly, meter)
		if err != nil {
			t.Fatalf("n=%d: %v", n, err)
		}
		if len(got) != len(want) {
			t.Fatalf("n=%d count", n)
		}
		for i := range got {
			if got[i] != want[i] {
				t.Fatalf("n=%d triangle=%d", n, i)
			}
		}
		if n != 1068 {
			continue
		}
		work, records := meter.Usage()
		bytes := meter.Bytes()
		for _, short := range []string{"", "work", "records", "bytes"} {
			b := proofbound.NewReplayTrialBudget()
			w, r, z := proofbound.ReplayTrialWork-work, proofbound.ReplayTrialRecords-records, proofbound.ReplayTrialBytes-bytes
			switch short {
			case "work":
				w++
			case "records":
				r++
			case "bytes":
				z++
			}
			if err := b.SpendWork(t.Context(), w); err != nil {
				t.Fatal(err)
			}
			if err := b.ReserveRecords(t.Context(), r); err != nil {
				t.Fatal(err)
			}
			if err := b.ReserveBytes(t.Context(), z); err != nil {
				t.Fatal(err)
			}
			_, err := ConvexFanReplay(t.Context(), pts, poly, b)
			if short == "" && err != nil || short != "" && !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("%q: %v", short, err)
			}
		}
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		if _, err := ConvexFanReplay(ctx, pts, poly, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
			t.Fatalf("cancellation: %v", err)
		}
	}
}

func TestConvexFanReplayRefuses(t *testing.T) {
	pts := []Point2{{U: 0, V: 0}, {U: 2, V: 0}, {U: 1, V: .1}, {U: 2, V: 2}, {U: 0, V: 2}}
	for _, poly := range [][]int{{0, 1, 2, 3, 4}, {0, 1, 1, 3, 4}, {0, 4, 3, 2, 1}, {0, 1, 2, 3, 5}} {
		if _, err := ConvexFanReplay(t.Context(), pts, poly, proofbound.NewReplayTrialBudget()); err == nil {
			t.Fatalf("admitted invalid polygon")
		}
	}
	pts[2].U = math.NaN()
	if _, err := ConvexFanReplay(t.Context(), pts, []int{0, 1, 2, 3, 4}, proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("admitted nonfinite polygon")
	}
}

// stepCancelContext cancels after the budget has polled an active traversal.
//
//nolint:containedctx // The context is confined to this synchronous replay.
type stepCancelContext struct {
	context.Context
	polls, after int
}

func (c *stepCancelContext) Err() error {
	c.polls++
	if c.polls > c.after {
		return context.Canceled
	}
	return nil
}

func TestConvexFanReplayCancelDuringTraversal(t *testing.T) {
	const n = 1068
	pts := make([]Point2, n)
	poly := make([]int, n)
	for i := range pts {
		a := 2 * math.Pi * float64(i) / float64(n)
		pts[i] = Point2{U: math.Cos(a), V: math.Sin(a)}
		poly[i] = i
	}
	for _, tc := range []struct {
		name  string
		after int
	}{
		{"radial", n + 20}, {"turn", 2*n + 20}, {"output", 3*n + 20},
	} {
		t.Run(tc.name, func(t *testing.T) {
			ctx := &stepCancelContext{Context: t.Context(), after: tc.after}
			got, err := ConvexFanReplay(ctx, pts, poly, proofbound.NewReplayTrialBudget())
			if !errors.Is(err, context.Canceled) || len(got) != 0 {
				t.Fatalf("cancel during traversal: %v, count=%d", err, len(got))
			}
			if ctx.polls <= tc.after {
				t.Fatal("traversal did not poll")
			}
		})
	}
}

func TestConvexFanReplayDifficultInputs(t *testing.T) {
	cases := []struct {
		name string
		pts  []Point2
		poly []int
	}{
		{"near-collinear", []Point2{{U: 0, V: 0}, {U: 1, V: 0}, {U: 2, V: math.SmallestNonzeroFloat64}, {U: 2, V: 1}, {U: 0, V: 1}}, []int{0, 1, 2, 3, 4}},
		{"overflow", []Point2{{U: -math.MaxFloat64, V: 0}, {U: 0, V: -math.MaxFloat64}, {U: math.MaxFloat64, V: 0}, {U: 0, V: math.MaxFloat64}}, []int{0, 1, 2, 3}},
		{"star", []Point2{{U: 0, V: 3}, {U: 3, V: 1}, {U: 2, V: -3}, {U: -2, V: -3}, {U: -3, V: 1}}, []int{0, 2, 4, 1, 3}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			_, err := ConvexFanReplay(t.Context(), tc.pts, tc.poly, proofbound.NewReplayTrialBudget())
			if err == nil {
				t.Fatal("unproved input accepted")
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, err := ConvexFanReplay(ctx, nil, nil, nil); !errors.Is(err, context.Canceled) {
		t.Fatalf("invalid cancelled input: %v", err)
	}
}
