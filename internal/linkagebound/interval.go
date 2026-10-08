package linkagebound

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Span holds a dependent joint's values at a piece's ends and over the piece.
type Span struct {
	Start, End, Hull proofbound.RatInterval
}

// ValueInterval encloses one dependent joint's value in radians or millimetres.
func ValueInterval(lo, hi motionbound.MotionParam) proofbound.RatInterval {
	return proofbound.IntervalOwned(motionbound.ParamLower(lo), motionbound.ParamUpper(hi))
}

// Hull is the smallest interval holding every supplied interval.
func Hull(ivs ...proofbound.RatInterval) proofbound.RatInterval {
	lo, hi := ivs[0].Lo, ivs[0].Hi
	for _, iv := range ivs[1:] {
		if iv.Lo.Cmp(lo) < 0 {
			lo = iv.Lo
		}
		if iv.Hi.Cmp(hi) > 0 {
			hi = iv.Hi
		}
	}
	return proofbound.IntervalOwned(new(big.Rat).Set(lo), new(big.Rat).Set(hi))
}

// Magnitude returns the largest absolute value in an interval.
func Magnitude(iv proofbound.RatInterval) *big.Rat {
	m := new(big.Rat).Abs(iv.Lo)
	if hi := new(big.Rat).Abs(iv.Hi); hi.Cmp(m) > 0 {
		m = hi
	}
	return m
}

// SpanUpper bounds a dependent joint's travel through one piece, including
// a reversal inside its hull.
func SpanUpper(sp Span) *big.Rat {
	low := new(big.Rat).Add(sp.Start.Hi, sp.End.Hi)
	low.Sub(low, new(big.Rat).Mul(big.NewRat(2, 1), sp.Hull.Lo))
	high := new(big.Rat).Mul(big.NewRat(2, 1), sp.Hull.Hi)
	high.Sub(high, sp.Start.Lo)
	high.Sub(high, sp.End.Lo)
	if high.Cmp(low) > 0 {
		return high
	}
	return low
}

// SumSpanUpper adds one dependent's travel over every interval piece.
func SumSpanUpper(pieces [][]Span, dependent int) *big.Rat {
	sum := new(big.Rat)
	for _, piece := range pieces {
		sum.Add(sum, SpanUpper(piece[dependent]))
	}
	return sum
}

// HullOfPieces encloses one dependent's value over every interval piece.
func HullOfPieces(pieces [][]Span, dependent int) proofbound.RatInterval {
	ivs := make([]proofbound.RatInterval, len(pieces))
	for n, piece := range pieces {
		ivs[n] = piece[dependent].Hull
	}
	return Hull(ivs...)
}

// IntervalKey identifies a verification interval by its exact endpoints.
func IntervalKey(a, b *big.Rat) string { return a.RatString() + "," + b.RatString() }
