package linkagebound

import (
	"fmt"
	"math/big"
	"strings"
)

// FoldDigits is the number of decimals used to print a fold's bounds outward.
const FoldDigits = 12

// DecimalDown prints x with the fold's outward lower rounding.
func DecimalDown(x *big.Rat) string { return decimalRounded(x, false) }

// DecimalUp prints x with the fold's outward upper rounding.
func DecimalUp(x *big.Rat) string { return decimalRounded(x, true) }

func decimalRounded(x *big.Rat, up bool) string {
	scale := new(big.Int).Exp(big.NewInt(10), big.NewInt(FoldDigits), nil)
	scaled := new(big.Rat).Mul(x, new(big.Rat).SetInt(scale))
	q, m := new(big.Int).DivMod(scaled.Num(), scaled.Denom(), new(big.Int))
	if up && m.Sign() != 0 {
		q.Add(q, big.NewInt(1))
	}
	return new(big.Rat).SetFrac(q, scale).FloatString(FoldDigits)
}

// GridDepth is the deepest dyadic level walked by the canonical chain.
const GridDepth = 30

// ChainStart finds the predecessor cell's start without crossing near.
func ChainStart(lo, near, s *big.Rat, readingFloor int64) *big.Rat {
	increasing := near.Cmp(lo) == 0
	start := gridStart(near, s, readingFloor, increasing)
	if increasing && start.Cmp(near) < 0 || !increasing && start.Cmp(near) > 0 {
		return new(big.Rat).Set(near)
	}
	return start
}

func gridStart(near, s *big.Rat, readingFloor int64, increasing bool) *big.Rat {
	den := s.Denom()
	depth := den.BitLen() - 1
	dyadic := new(big.Int).Lsh(big.NewInt(1), uint(depth)).Cmp(den) == 0
	if dyadic && depth <= GridDepth {
		step := new(big.Rat).SetFrac(big.NewInt(1), den)
		if increasing {
			return step.Sub(s, step)
		}
		return step.Add(s, step)
	}
	scaled := new(big.Rat).Mul(s, big.NewRat(readingFloor, 1))
	q := new(big.Int).Div(scaled.Num(), scaled.Denom())
	if !increasing {
		q.Add(q, big.NewInt(1))
	}
	anchor := new(big.Rat).SetFrac(q, big.NewInt(readingFloor))
	if anchor.Cmp(s) == 0 {
		return new(big.Rat).Set(near)
	}
	return anchor
}

// AskKey names a cached ask by sub-segment, kind, and exact endpoint fractions.
func AskKey(idx int, kind string, ends ...*big.Rat) string {
	var b strings.Builder
	fmt.Fprintf(&b, "%d:%s", idx, kind)
	for n, e := range ends {
		if n > 0 {
			b.WriteByte(',')
		}
		b.WriteString(e.RatString())
	}
	return b.String()
}
