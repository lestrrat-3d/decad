package proof

import (
	"context"
	"errors"
	"math/big"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

func TestReplayProductDifferenceMatches(t *testing.T) {
	r := rand.New(rand.NewSource(77))
	for i := range 400 {
		v := [4]*big.Int{}
		for j := range v {
			v[j] = new(big.Int).Rand(r, new(big.Int).Lsh(big.NewInt(1), uint(1+(i*11+j*37)%1200)))
			if (i+j)%3 == 0 {
				v[j].Neg(v[j])
			}
		}
		want := new(big.Int).Sub(new(big.Int).Mul(v[0], v[1]), new(big.Int).Mul(v[2], v[3]))
		got, err := ReplayProductDifference(t.Context(), v[0], v[1], v[2], v[3], new(keyTestBudget))
		if err != nil || got.Cmp(want) != 0 {
			t.Fatalf("case %d: %v", i, err)
		}
	}
}

func TestReplayXlerpMatches(t *testing.T) {
	r := rand.New(rand.NewSource(104))
	for i := range 100 {
		point := func() Xpt {
			v := [4]*big.Int{}
			for j := range v {
				v[j] = new(big.Int).Rand(r, new(big.Int).Lsh(big.NewInt(1), uint(1+(i*3+j*17)%300)))
				if j < 3 && (i+j)%2 == 0 {
					v[j].Neg(v[j])
				}
			}
			if v[3].Sign() == 0 {
				v[3].SetInt64(1)
			}
			return Xpt{X: v[0], Y: v[1], Z: v[2], W: v[3]}
		}
		a, b := point(), point()
		n := big.NewInt(int64(i - 50))
		d := big.NewInt(int64(i + 1))
		want := Xlerp(a, b, n, d)
		got, err := ReplayXlerp(t.Context(), a, b, n, d, new(keyTestBudget))
		if err != nil || want.X.Cmp(got.X) != 0 || want.Y.Cmp(got.Y) != 0 || want.Z.Cmp(got.Z) != 0 || want.W.Cmp(got.W) != 0 {
			t.Fatalf("case %d: %v", i, err)
		}
	}
}

type smallProductTestBudget struct{ *keyTestBudget }

func (b *smallProductTestBudget) ReserveProductSmall(ctx context.Context, size uint64) error {
	return b.ReserveBytes(ctx, 2*size)
}

var smallProductTestSink *big.Int

func TestReplayProductSmallOwnedResult(t *testing.T) {
	ctx := t.Context()
	budget := &smallProductTestBudget{new(keyTestBudget)}
	arena := &ReplayProductArena{}
	vals := [4]*big.Int{big.NewInt(-31), big.NewInt(7), big.NewInt(11), big.NewInt(3)}
	first, err := arena.ProductDifference(ctx, vals[0], vals[1], vals[2], vals[3], budget)
	if err != nil || first.Int64() != -250 {
		t.Fatalf("first product: %v", err)
	}
	second, err := arena.ProductDifference(ctx, vals[2], vals[3], vals[1], vals[0], budget)
	if err != nil || second.Int64() != 250 || first.Int64() != -250 {
		t.Fatalf("product results did not retain their values: %v", err)
	}
	allocs := testing.AllocsPerRun(100, func() {
		var e error
		smallProductTestSink, e = arena.ProductDifference(ctx, vals[0], vals[1], vals[2], vals[3], budget)
		if e != nil {
			panic(e)
		}
	})
	if allocs != 1 {
		t.Fatalf("small product allocated %.0f objects, expected one", allocs)
	}
}

func TestReplayProductArenaRetainsMixedWidths(t *testing.T) {
	rng := rand.New(rand.NewSource(91))
	budget := &smallProductTestBudget{new(keyTestBudget)}
	arena := &ReplayProductArena{}
	var got, want []*big.Int
	for i := range 200 {
		var values [4]*big.Int
		for j := range values {
			width := uint(1 + (i*13+j*17)%150)
			values[j] = new(big.Int).Rand(rng, new(big.Int).Lsh(big.NewInt(1), width))
			if (i+j)%2 == 0 {
				values[j].Neg(values[j])
			}
		}
		value, err := arena.ProductDifference(t.Context(), values[0], values[1], values[2], values[3], budget)
		if err != nil {
			t.Fatalf("case %d: %v", i, err)
		}
		got = append(got, value)
		want = append(want, new(big.Int).Sub(new(big.Int).Mul(values[0], values[1]), new(big.Int).Mul(values[2], values[3])))
	}
	for i := range got {
		if got[i].Cmp(want[i]) != 0 {
			t.Fatalf("retained product %d changed", i)
		}
	}
}

func TestReplayInvalidProductDoesNotAllocate(t *testing.T) {
	ctx := t.Context()
	budget := &smallProductTestBudget{new(keyTestBudget)}
	value := big.NewInt(1)
	run := func() error {
		_, err := ReplayProductDifference(ctx, nil, value, value, value, budget)
		return err
	}
	if !errors.Is(run(), decaderr.ErrUnsupported) {
		t.Fatal("invalid product did not retain its refusal")
	}
	if allocs := testing.AllocsPerRun(100, func() {
		if run() == nil {
			panic("missing refusal")
		}
	}); allocs != 0 {
		t.Fatalf("invalid product refusal allocated %.0f objects", allocs)
	}
}
