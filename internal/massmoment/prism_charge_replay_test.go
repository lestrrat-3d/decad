package massmoment_test

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func TestReplayPlacementCharge(t *testing.T) {
	transforms := []r3.Transform{r3.Identity()}
	for _, angle := range []float64{1, 37, 89, 179} {
		r, err := r3.Rotation(r3.Vec{X: 1, Y: 2, Z: 3}, units.Degrees(angle))
		if err != nil {
			t.Fatal(err)
		}
		transforms = append(transforms, r)
	}
	for i, x := range transforms {
		linear, err := massmoment.PlacementRotation(x)
		if err != nil {
			t.Fatal(err)
		}
		want, err := massmoment.MapChargeOf(linear)
		if err != nil {
			t.Fatal(err)
		}
		got, err := massmoment.ReplayPlacementCharge(context.Background(), x, proofbound.NewReplayTrialBudget())
		if err != nil {
			t.Fatal(err)
		}
		if math.Float64bits(got.Volume) != math.Float64bits(want.Volume) || math.Float64bits(got.Stretch) != math.Float64bits(want.Stretch) {
			t.Fatalf("transform %d charge mismatch: got %+v want %+v", i, got, want)
		}
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if _, err := massmoment.ReplayPlacementCharge(ctx, r3.Identity(), proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("canceled charge succeeded")
	}
	budget := proofbound.NewReplayTrialBudget()
	if err := budget.ReserveBytes(context.Background(), proofbound.ReplayTrialBytes); err != nil {
		t.Fatal(err)
	}
	if _, err := massmoment.ReplayPlacementCharge(context.Background(), r3.Identity(), budget); err == nil {
		t.Fatal("charge succeeded with no byte allowance")
	}
}

func TestReplayPrismCharge(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{X: 4, Y: -2, Z: 7}, r3.Vec{X: 1, Y: 1}, r3.Vec{X: -1, Y: 1})
	if err != nil {
		t.Fatal(err)
	}
	x, err := r3.Rotation(r3.Vec{X: 1, Y: 2, Z: 3}, units.Radians(.71))
	if err != nil {
		t.Fatal(err)
	}
	matrix, err := massmoment.PrismRotation(frame, x)
	if err != nil {
		t.Fatal(err)
	}
	want, err := massmoment.MapChargeOf(matrix)
	if err != nil {
		t.Fatal(err)
	}
	b := proofbound.NewReplayTrialBudget()
	got, err := massmoment.ReplayPrismCharge(t.Context(), frame, x, b)
	if err != nil {
		t.Fatal(err)
	}
	if math.Float64bits(got.Stretch) != math.Float64bits(want.Stretch) || math.Float64bits(got.Volume) != math.Float64bits(want.Volume) {
		t.Fatalf("got %+v want %+v", got, want)
	}
	work, records := b.Usage()
	bytes := b.Bytes()
	t.Logf("charge=%+v work=%d records=%d bytes=%d", got, work, records, bytes)
	for _, short := range []string{"", "work", "records", "bytes"} {
		m := proofbound.NewReplayTrialBudget()
		w, n, z := proofbound.ReplayTrialWork-work, proofbound.ReplayTrialRecords-records, proofbound.ReplayTrialBytes-bytes
		switch short {
		case "work":
			w++
		case "records":
			n++
		case "bytes":
			z++
		}
		if err := m.SpendWork(t.Context(), w); err != nil {
			t.Fatal(err)
		}
		if err := m.ReserveRecords(t.Context(), n); err != nil {
			t.Fatal(err)
		}
		if err := m.ReserveBytes(t.Context(), z); err != nil {
			t.Fatal(err)
		}
		_, err := massmoment.ReplayPrismCharge(t.Context(), frame, x, m)
		if short == "" && err != nil || short != "" && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("%s: %v", short, err)
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, err := massmoment.ReplayPrismCharge(ctx, frame, x, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancel: %v", err)
	}
}

func TestReplayPrismChord(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1, Y: 1}, r3.Vec{X: -1, Y: 1})
	if err != nil {
		t.Fatal(err)
	}
	x, err := r3.Rotation(r3.Vec{X: 1, Y: 2, Z: 3}, units.Radians(.71))
	if err != nil {
		t.Fatal(err)
	}
	if _, err := massmoment.ReplayPrismCharge(t.Context(), frame, r3.Transform{}, proofbound.NewReplayTrialBudget()); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("collapsed transform: %v", err)
	}
	chord := 0.05
	local, lift, charge, err := massmoment.ReplayPrismChord(t.Context(), frame, x, chord, proofbound.NewReplayTrialBudget())
	if err != nil {
		t.Fatal(err)
	}
	product := new(big.Rat).Mul(new(big.Rat).SetFloat64(local), new(big.Rat).SetFloat64(lift))
	if product.Cmp(new(big.Rat).SetFloat64(chord)) > 0 || !(charge.Stretch > 0) {
		t.Fatalf("bad chord %g lift %g charge %g", local, lift, charge.Stretch)
	}
	for _, c := range []float64{0, -1, math.NaN(), math.Inf(1)} {
		if _, _, _, err := massmoment.ReplayPrismChord(t.Context(), frame, x, c, proofbound.NewReplayTrialBudget()); !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("invalid chord %g: %v", c, err)
		}
	}
}
