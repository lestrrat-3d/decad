package proofbound

import (
	"math/big"
	"math/rand"
	"testing"
)

func TestReplayTrigArenaArithmetic(t *testing.T) {
	rng := rand.New(rand.NewSource(23))
	limit := new(big.Int).Lsh(big.NewInt(1), 1024)
	for i := range 80 {
		x := new(big.Int).Rand(rng, limit)
		y := new(big.Int).Rand(rng, limit)
		a, err := newReplayTrigArena(t.Context(), NewReplayTrialBudget())
		if err != nil {
			t.Fatal(err)
		}
		if err = a.load(0, x); err != nil {
			t.Fatal(err)
		}
		if err = a.load(1, y); err != nil {
			t.Fatal(err)
		}
		if err = a.mul(2, 0, 1); err != nil {
			t.Fatal(err)
		}
		words := make([]big.Word, replayTrigWords)
		for j, w := range a.slots[2] {
			words[j] = big.Word(w)
		}
		if new(big.Int).SetBits(words).Cmp(new(big.Int).Mul(x, y)) != 0 {
			t.Fatalf("product differs at %d", i)
		}
		if err = a.add(3, 0, 1); err != nil {
			t.Fatal(err)
		}
		if err = a.sub(4, 3, 1); err != nil {
			t.Fatal(err)
		}
		if y.Sign() != 0 {
			if err = a.div(5, 6, 2, 1); err != nil {
				t.Fatal(err)
			}
			if replayTrigCmp(&a.slots[5], &a.slots[0]) != 0 || replayTrigBitLen(&a.slots[6]) != 0 {
				t.Fatalf("division differs at %d", i)
			}
			if err = a.div(5, 6, 0, 1); err != nil {
				t.Fatal(err)
			}
			q, r := new(big.Int), new(big.Int)
			q.QuoRem(x, y, r)
			if err = a.load(7, q); err != nil {
				t.Fatal(err)
			}
			if err = a.load(8, r); err != nil {
				t.Fatal(err)
			}
			if replayTrigCmp(&a.slots[5], &a.slots[7]) != 0 || replayTrigCmp(&a.slots[6], &a.slots[8]) != 0 {
				t.Fatalf("quotient or remainder differs at %d", i)
			}
		}
		if replayTrigCmp(&a.slots[4], &a.slots[0]) != 0 {
			t.Fatalf("sum or difference differs at %d", i)
		}
	}
	exhausted := NewReplayTrialBudget()
	if err := exhausted.ReserveBytes(t.Context(), ReplayTrialBytes); err != nil {
		t.Fatal(err)
	}
	if _, err := newReplayTrigArena(t.Context(), exhausted); err == nil {
		t.Fatal("arena exceeded byte limit")
	}
}

func TestReplayTrigArenaLimits(t *testing.T) {
	ctx := t.Context()
	probe := NewReplayTrialBudget()
	a, err := newReplayTrigArena(ctx, probe)
	if err != nil {
		t.Fatal(err)
	}
	used := probe.Bytes()
	for _, extra := range []uint64{0, 1} {
		b := NewReplayTrialBudget()
		if err := b.ReserveBytes(ctx, ReplayTrialBytes-used+extra); err != nil {
			t.Fatal(err)
		}
		_, err := newReplayTrigArena(ctx, b)
		if (err != nil) != (extra == 1) {
			t.Fatalf("arena byte boundary extra=%d: %v", extra, err)
		}
	}
	tooWide := new(big.Int).Lsh(big.NewInt(1), 4096)
	if err := a.load(0, tooWide); err == nil {
		t.Fatal("accepted integer above 4096 bits")
	}
}

func TestReplayTrigFixedSeries(t *testing.T) {
	rng := rand.New(rand.NewSource(101))
	bound := new(big.Int).Rsh(new(big.Int).Set(TrigFixedOne), 1)
	for i := range 20 {
		x := new(big.Int).Rand(rng, bound)
		a, err := newReplayTrigArena(t.Context(), NewReplayTrialBudget())
		if err != nil {
			t.Fatal(err)
		}
		if err := a.load(0, x); err != nil {
			t.Fatal(err)
		}
		if err := a.fixedSeries(0, 1, 2, 3); err != nil {
			t.Fatal(err)
		}
		s, c, e := TrigFixedSeries(x)
		for j, want := range []*big.Int{s, c, e} {
			if err := a.load(4, want); err != nil {
				t.Fatal(err)
			}
			if replayTrigCmp(&a.slots[j+1], &a.slots[4]) != 0 {
				t.Fatalf("case %d result %d differs", i, j)
			}
		}
	}
}

func TestReplayTrigSignedAdd(t *testing.T) {
	for _, x := range []int64{-12, -1, 0, 1, 12} {
		for _, y := range []int64{-13, -1, 0, 1, 13} {
			a, err := newReplayTrigArena(t.Context(), NewReplayTrialBudget())
			if err != nil {
				t.Fatal(err)
			}
			ax, ay := big.NewInt(x), big.NewInt(y)
			if err = a.load(0, new(big.Int).Abs(ax)); err != nil {
				t.Fatal(err)
			}
			if err = a.load(1, new(big.Int).Abs(ay)); err != nil {
				t.Fatal(err)
			}
			z, err := a.signedAdd(2, replayTrigSigned{0, x < 0}, replayTrigSigned{1, y < 0})
			if err != nil {
				t.Fatal(err)
			}
			want := big.NewInt(x + y)
			if z.negative != (want.Sign() < 0) {
				t.Fatalf("%d + %d sign", x, y)
			}
			if err = a.load(3, new(big.Int).Abs(want)); err != nil {
				t.Fatal(err)
			}
			if replayTrigCmp(&a.slots[2], &a.slots[3]) != 0 {
				t.Fatalf("%d + %d magnitude", x, y)
			}
		}
	}
}
