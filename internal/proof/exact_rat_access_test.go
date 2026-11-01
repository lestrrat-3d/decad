package proof

import (
	"context"
	"errors"
	"math/big"
	"testing"
)

var errRatAccessLimit = errors.New("rational access budget exceeded")

type ratAccessBudget struct {
	work, records, bytes              uint64
	workLimit, recordLimit, byteLimit uint64
}

func (b *ratAccessBudget) SpendWork(_ context.Context, n uint64) error {
	if n > b.workLimit-b.work {
		return errRatAccessLimit
	}
	b.work += n
	return nil
}

func (b *ratAccessBudget) ReserveRecords(_ context.Context, n uint64) error {
	if n > b.recordLimit-b.records {
		return errRatAccessLimit
	}
	b.records += n
	return nil
}

func (b *ratAccessBudget) ReserveBytes(_ context.Context, n uint64) error {
	if n > b.byteLimit-b.bytes {
		return errRatAccessLimit
	}
	b.bytes += n
	return nil
}

func TestReplayRatPartToWordsCopiesUnderExactWorkLimit(t *testing.T) {
	q := new(big.Rat).SetFrac(big.NewInt(-13), big.NewInt(8))
	run := func(budget ReplayKeyBudget) ([64]uint64, error) {
		var words [64]uint64
		return words, ReplayRatPartToWords(t.Context(), q, true, words[:], budget)
	}
	base := &ratAccessBudget{workLimit: 64}
	words, err := run(base)
	if err != nil {
		t.Fatal(err)
	}
	if words[0] != 13 || words[1] != 0 || base.work != 64 || base.records != 0 || base.bytes != 0 {
		t.Fatalf("copied words=%v, budget=%+v", words[:2], base)
	}
	words[0] = 0
	if q.Cmp(new(big.Rat).SetFrac(big.NewInt(-13), big.NewInt(8))) != 0 {
		t.Fatal("destination mutation changed source rational")
	}
	if _, err = run(&ratAccessBudget{workLimit: 63}); !errors.Is(err, errRatAccessLimit) {
		t.Fatalf("one-short work limit: %v", err)
	}
}

func TestReplayDyadicRatFromWordsExactAndOneShortBudgets(t *testing.T) {
	inputs := []struct {
		words []uint64
		exp   int
		neg   bool
		want  *big.Rat
	}{
		{nil, -19, true, new(big.Rat)},
		{[]uint64{13}, -3, true, new(big.Rat).SetFrac(big.NewInt(-13), big.NewInt(8))},
		{[]uint64{12}, -3, false, new(big.Rat).SetFrac(big.NewInt(3), big.NewInt(2))},
		{[]uint64{7}, 4, false, new(big.Rat).SetInt64(112)},
	}
	for i, input := range inputs {
		t.Run(big.NewInt(int64(i)).String(), func(t *testing.T) {
			run := func(budget *ratAccessBudget) (*big.Rat, error) {
				return ReplayDyadicRatFromWords(t.Context(), input.words, input.exp, input.neg, budget)
			}
			base := &ratAccessBudget{workLimit: ^uint64(0), recordLimit: ^uint64(0), byteLimit: ^uint64(0)}
			got, err := run(base)
			if err != nil {
				t.Fatal(err)
			}
			if got.Cmp(input.want) != 0 || got.Num().BitLen() > 4096 || got.Denom().BitLen() > 4096 {
				t.Fatalf("got %s, want %s", got, input.want)
			}
			gcd := new(big.Int).GCD(nil, nil, got.Num(), got.Denom())
			if gcd.Cmp(big.NewInt(1)) != 0 {
				t.Fatalf("result is not reduced: %s", got)
			}
			limits := [][3]uint64{
				{base.work, base.records, base.bytes},
				{base.work - 1, base.records, base.bytes},
				{base.work, base.records - 1, base.bytes},
				{base.work, base.records, base.bytes - 1},
			}
			for j, limit := range limits {
				b := &ratAccessBudget{workLimit: limit[0], recordLimit: limit[1], byteLimit: limit[2]}
				v, e := run(b)
				if j == 0 && (e != nil || v.Cmp(input.want) != 0) {
					t.Fatalf("exact budget: value=%v error=%v", v, e)
				}
				if j != 0 && !errors.Is(e, errRatAccessLimit) {
					t.Fatalf("one-short budget %d: %v", j, e)
				}
			}
		})
	}
}
