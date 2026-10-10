package freeform

import (
	"math/big"
	"testing"
)

func TestRefineSpanChainPreservesQuadratic(t *testing.T) {
	span := BezierSpan{
		{U: big.NewRat(0, 1), V: big.NewRat(0, 1)},
		{U: big.NewRat(1, 1), V: big.NewRat(2, 1)},
		{U: big.NewRat(2, 1), V: big.NewRat(0, 1)},
	}
	parts, err := RefineSpanChain([]BezierSpan{span}, 3, NewFreeformWork())
	if err != nil {
		t.Fatal(err)
	}
	if len(parts) != 3 {
		t.Fatalf("got %d parts, want 3", len(parts))
	}
	for i, part := range parts {
		for _, sample := range []int{0, 1, 2} {
			local := big.NewRat(int64(sample), 2)
			global := big.NewRat(int64(2*i+sample), 6)
			got := quadraticPoint(part, local)
			want := RatPoint{
				U: new(big.Rat).Mul(big.NewRat(2, 1), global),
				V: new(big.Rat).Mul(big.NewRat(4, 1), new(big.Rat).Mul(global,
					new(big.Rat).Sub(big.NewRat(1, 1), global))),
			}
			if got.U.Cmp(want.U) != 0 || got.V.Cmp(want.V) != 0 {
				t.Fatalf("part %d sample %d: got (%s, %s), want (%s, %s)",
					i, sample, got.U, got.V, want.U, want.V)
			}
		}
	}
}

func quadraticPoint(span BezierSpan, t *big.Rat) RatPoint {
	u := new(big.Rat).Sub(big.NewRat(1, 1), t)
	u2 := new(big.Rat).Mul(u, u)
	ut2 := new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Mul(u, t))
	t2 := new(big.Rat).Mul(t, t)
	coord := func(first, middle, last *big.Rat) *big.Rat {
		out := new(big.Rat).Mul(u2, first)
		out.Add(out, new(big.Rat).Mul(ut2, middle))
		return out.Add(out, new(big.Rat).Mul(t2, last))
	}
	return RatPoint{
		U: coord(span[0].U, span[1].U, span[2].U),
		V: coord(span[0].V, span[1].V, span[2].V),
	}
}
