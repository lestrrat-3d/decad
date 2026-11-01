package tessellation

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestCircleTurnCacheReplayParity(t *testing.T) {
	for _, n := range []int{3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 17, 31, 41, 63, 100, 128, 199, 200, 1068} {
		b := proofbound.NewReplayTrialBudget()
		cache, err := newCircleTurnCache(t.Context(), n, b)
		if err != nil {
			t.Fatal(err)
		}
		for k := 1; k < n; k++ {
			s, c, err := cache.station(t.Context(), k, b)
			if err != nil {
				t.Fatalf("n=%d k=%d: %v", n, k, err)
			}
			ws, wc, err := proofbound.CircleTurnFractionReplay(t.Context(), k, n, proofbound.NewReplayTrialBudget())
			if err != nil {
				t.Fatal(err)
			}
			if s.Lo.Cmp(ws.Lo) != 0 || s.Hi.Cmp(ws.Hi) != 0 || c.Lo.Cmp(wc.Lo) != 0 || c.Hi.Cmp(wc.Hi) != 0 {
				t.Fatalf("n=%d k=%d interval differs", n, k)
			}
		}
	}
}
func TestCircleTurnCacheReplayLimits(t *testing.T) {
	run := func(ctx context.Context, b *proofbound.ReplayBudget) error {
		c, err := newCircleTurnCache(ctx, 1068, b)
		if err != nil {
			return err
		}
		for k := 1; k < 1068; k++ {
			_, _, err = c.station(ctx, k, b)
			if err != nil {
				return err
			}
		}
		return nil
	}
	base := proofbound.NewReplayTrialBudget()
	if err := run(t.Context(), base); err != nil {
		t.Fatal(err)
	}
	work, records := base.Usage()
	bytes := base.Bytes()
	for _, mode := range []string{"exact", "work", "records", "bytes"} {
		b := proofbound.NewReplayTrialBudget()
		w, r, z := proofbound.ReplayTrialWork-work, proofbound.ReplayTrialRecords-records, proofbound.ReplayTrialBytes-bytes
		switch mode {
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
		err := run(t.Context(), b)
		if mode == "exact" && err != nil || mode != "exact" && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("%s: %v", mode, err)
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if err := run(ctx, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancel: %v", err)
	}
	b := proofbound.NewReplayTrialBudget()
	c, err := newCircleTurnCache(t.Context(), 41, b)
	if err != nil {
		t.Fatal(err)
	}
	if _, _, err = c.station(t.Context(), 2, b); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("order: %v", err)
	}
	if _, _, err = c.station(t.Context(), 1, proofbound.NewReplayTrialBudget()); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("meter: %v", err)
	}
}

//nolint:containedctx // The context is confined to this synchronous replay.
type turnCancelContext struct {
	context.Context
	polls int
}

func (c *turnCancelContext) Err() error {
	c.polls++
	if c.polls > 1000 {
		return context.Canceled
	}
	return nil
}
func TestCircleTurnCacheReplayCancelDuring(t *testing.T) {
	ctx := &turnCancelContext{Context: t.Context()}
	b := proofbound.NewReplayTrialBudget()
	c, err := newCircleTurnCache(ctx, 1068, b)
	if err != nil {
		t.Fatal(err)
	}
	for k := 1; k < 1068; k++ {
		_, _, err = c.station(ctx, k, b)
		if err != nil {
			break
		}
	}
	if !errors.Is(err, context.Canceled) || ctx.polls <= 1000 {
		t.Fatalf("cancel during traversal: %v polls=%d", err, ctx.polls)
	}
}
