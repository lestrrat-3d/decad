package prismtess

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func TestPointRoundReplayCache(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{X: 4.3, Y: -1.8, Z: 9.1}, r3.Vec{X: 1, Y: 1}, r3.Vec{X: -1, Y: 1})
	if err != nil {
		t.Fatal(err)
	}
	turn, err := r3.Rotation(r3.Vec{X: 1, Y: 2, Z: 3}, units.Radians(.71))
	if err != nil {
		t.Fatal(err)
	}
	p := prismrecord.Payload{Frame: frame, Xform: turn}
	b := proofbound.NewReplayTrialBudget()
	c, err := newPointRoundReplayCache(t.Context(), p, b)
	if err != nil {
		t.Fatal(err)
	}
	for i, q := range [][3]float64{{1.6, -.03, 6}, {-1e-30, 17.1, -4.5}, {0, 0, 0}, {2, 3, 4}} {
		held := p.Point(q[0], q[1], q[2])
		got, err := c.bound(t.Context(), p, q[0], q[1], q[2], held, b)
		if err != nil {
			t.Fatal(err)
		}
		want := p.PointRound(q[0], q[1], q[2], held)
		if math.Float64bits(got) != math.Float64bits(want) {
			t.Fatalf("sample %d got %.17g want %.17g", i, got, want)
		}
		if _, err := c.bound(t.Context(), p, q[0], q[1], q[2], held, proofbound.NewReplayTrialBudget()); !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("wrong meter: %v", err)
		}
	}
	changed := p
	changed.Xform = r3.Identity()
	if _, err := c.bound(t.Context(), changed, 1, 2, 3, changed.Point(1, 2, 3), b); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("changed placement: %v", err)
	}
	before := c.coefficient[0][0]
	c.coefficient[0][0] = proof.DyZero()
	if _, err := c.bound(t.Context(), p, 1, 2, 3, p.Point(1, 2, 3), b); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("changed cached coefficient: %v", err)
	}
	c.coefficient[0][0] = before
	run := func(ctx context.Context, b *proofbound.ReplayBudget) error {
		c, err := newPointRoundReplayCache(ctx, p, b)
		if err != nil {
			return err
		}
		_, err = c.bound(ctx, p, 1.6, -.03, 6, p.Point(1.6, -.03, 6), b)
		return err
	}
	base := proofbound.NewReplayTrialBudget()
	if err := run(t.Context(), base); err != nil {
		t.Fatal(err)
	}
	work, records := base.Usage()
	bytes := base.Bytes()
	for _, mode := range []string{replayBudgetTestExact, replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		b := proofbound.NewReplayTrialBudget()
		w, r, z := proofbound.ReplayTrialWork-work, proofbound.ReplayTrialRecords-records, proofbound.ReplayTrialBytes-bytes
		switch mode {
		case replayBudgetTestWork:
			w++
		case replayBudgetTestRecords:
			r++
		case replayBudgetTestBytes:
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
		if mode == replayBudgetTestExact && err != nil || mode != replayBudgetTestExact && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("%s: %v", mode, err)
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if err := run(ctx, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancel: %v", err)
	}
}
