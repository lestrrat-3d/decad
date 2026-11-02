package facetproof

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestBrakeReplayRepairsAndChargesSliver(t *testing.T) {
	ctx := t.Context()
	run := func(ctx context.Context, meter *proofbound.ReplayBudget) ([]float64, []r3.Vec, proof.Xpt, error) {
		w := new(big.Int).Lsh(big.NewInt(1), 61)
		x := new(big.Int).Rsh(new(big.Int).Set(w), 1)
		y := new(big.Int).Add(new(big.Int).Set(x), big.NewInt(2))
		p := proof.Xpt{X: x, Y: y, Z: big.NewInt(0), W: w}
		held := []r3.Vec{{}, {X: 1, Y: 1}, {X: 1}, p.Vec()}
		exact := []proof.Xpt{proof.XptOf(held[0]), proof.XptOf(held[1]), proof.XptOf(held[2]), p}
		tris := [][3]int{{0, 2, 1}, {0, 1, 3}}
		charges, summary, err := keepPlacedEmbeddedWithAxesReplay(ctx, exact, held, tris, [3]r3.Vec{}, nil, meter.WorkBudget(ctx))
		if summary != nil {
			t.Fatal("float-box repair produced a reach summary")
		}
		return charges, held, p, err
	}
	meter := proofbound.NewBrakeReplayBudget()
	charges, held, point, err := run(ctx, meter)
	if err != nil || len(charges) != len(held) {
		t.Fatalf("repair: charges=%d error=%v", len(charges), err)
	}
	if held[3].Y != math.Nextafter(.5, 1) {
		t.Fatalf("repaired apex y=%.17g", held[3].Y)
	}
	gap, _ := meshbool.CoordDistance(point, held[3]).Float64()
	if charges[3] < gap || !(charges[3] > 0) {
		t.Fatalf("repair bound=%.17g gap=%.17g", charges[3], gap)
	}
	work, records := meter.Usage()
	bytes := meter.Bytes()
	for _, kind := range []string{"exact", "work", "records", "bytes"} {
		t.Run(kind, func(t *testing.T) {
			budget := proofbound.NewBrakeReplayBudget()
			w, r, b := proofbound.BrakeReplayWork-work, proofbound.BrakeReplayRecords-records, proofbound.BrakeReplayBytes-bytes
			switch kind {
			case "work":
				w++
			case "records":
				r++
			case "bytes":
				b++
			}
			if err := budget.SpendWork(ctx, w); err != nil {
				t.Fatal(err)
			}
			if err := budget.ReserveRecords(ctx, r); err != nil {
				t.Fatal(err)
			}
			if err := budget.ReserveBytes(ctx, b); err != nil {
				t.Fatal(err)
			}
			_, _, _, err := run(ctx, budget)
			if kind == "exact" {
				if err != nil {
					t.Fatalf("exact: %v", err)
				}
				return
			}
			if _, ok := errors.AsType[*proofbound.WorkLimitError](err); !ok {
				t.Fatalf("one-short %s: %v", kind, err)
			}
		})
	}
	cancelled, cancel := context.WithCancel(ctx)
	cancel()
	_, _, _, err = run(cancelled, proofbound.NewBrakeReplayBudget()) //nolint:dogsled // Only the cancellation error is needed.
	if !errors.Is(err, context.Canceled) {
		t.Fatalf("cancelled repair: %v", err)
	}
}
