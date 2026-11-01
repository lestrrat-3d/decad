package triangulation

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestEarClipReplay(t *testing.T) {
	cases := [][]Point2{{{U: 0, V: 0}, {U: 4, V: 0}, {U: 4, V: 3}, {U: 0, V: 3}}, {{U: 0, V: 0}, {U: 4, V: 0}, {U: 2, V: 1}, {U: 4, V: 3}, {U: 0, V: 3}}}
	var circle []Point2
	for i := range 41 {
		a := 2 * math.Pi * float64(i) / 41
		circle = append(circle, Point2{U: math.Cos(a), V: math.Sin(a)})
	}
	cases = append(cases, circle)
	for c, pts := range cases {
		idx := make([]int, len(pts))
		for i := range idx {
			idx[i] = i
		}
		want, err := EarClip(t.Context(), pts, idx)
		if err != nil {
			t.Fatalf("case %d ordinary: %v", c, err)
		}
		meter := proofbound.NewReplayTrialBudget()
		got, err := EarClipReplay(t.Context(), pts, idx, meter)
		if err != nil {
			t.Fatalf("case %d replay: %v", c, err)
		}
		if len(got) != len(want) {
			t.Fatalf("case %d triangle count", c)
		}
		for i := range got {
			if got[i] != want[i] {
				t.Fatalf("case %d triangle %d", c, i)
			}
		}
		if c != 0 {
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
			_, err := EarClipReplay(t.Context(), pts, idx, b)
			if short == "" && err != nil || short != "" && !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("%q limit: %v", short, err)
			}
		}
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, err = EarClipReplay(ctx, pts, idx, proofbound.NewReplayTrialBudget())
		if !errors.Is(err, context.Canceled) {
			t.Fatalf("cancel: %v", err)
		}
	}
}

type cancelAtPoll struct {
	context.Context //nolint:containedctx // The context is confined to one synchronous replay or test.
	stop, polls     int
}

func (c *cancelAtPoll) Err() error {
	c.polls++
	if c.polls >= c.stop {
		return context.Canceled
	}
	return nil
}

func TestEarClipReplayCancelsAtFirstBlockerVisit(t *testing.T) {
	pts := []Point2{{U: 0, V: 0}, {U: 4, V: 0}, {U: 4, V: 3}, {U: 0, V: 3}}
	// Point validation, five preflight charges, a pass poll, and a candidate
	// charge precede the first blocker visit. Poll 12 stops its predicate.
	ctx := &cancelAtPoll{Context: context.Background(), stop: 12}
	tris, err := EarClipReplay(ctx, pts, []int{0, 1, 2, 3}, proofbound.NewReplayTrialBudget())
	if !errors.Is(err, context.Canceled) || len(tris) != 0 || ctx.polls != ctx.stop {
		t.Fatalf("polls=%d triangles=%d error=%v", ctx.polls, len(tris), err)
	}
}

func TestEarClipReplayRejectsNonfinitePoint(t *testing.T) {
	pts := []Point2{{U: 0, V: 0}, {U: math.NaN(), V: 0}, {U: 0, V: 2}}
	triangles, err := EarClipReplay(t.Context(), pts, []int{0, 1, 2}, proofbound.NewReplayTrialBudget())
	if !errors.Is(err, decaderr.ErrUnsupported) || len(triangles) != 0 {
		t.Fatalf("triangles=%d error=%v", len(triangles), err)
	}
}
