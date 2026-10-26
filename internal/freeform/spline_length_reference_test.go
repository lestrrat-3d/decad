package freeform_test

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file keeps an independent copy of the fixed-depth arc-length bracket in
// its earlier integer form: the span's whole least common denominator kept as
// Den, and every de Casteljau midpoint aligning its two operands' exponents
// itself. The differential test holds the production bracket to this copy bit
// for bit.

func refSpanLengthBracket(span freeform.BezierSpan, depth int) (float64, float64) {
	s, _ := freeform.DyadicSpanOf(nil, span)
	s.DenSq = new(big.Int).Mul(s.Den, s.Den)
	if depth <= 0 || len(s.Points) < 2 {
		return s.ChordLower(), s.PolygonUpper()
	}
	frames := make([]refSplitScratch, depth)
	for i := range frames {
		frames[i].init(len(s.Points))
	}
	return refLengthBracket(s, frames, &refDistanceScratch{})
}

type refSplitScratch struct {
	work, left, right []freeform.DyadicPoint
	tmp               big.Int
}

func (f *refSplitScratch) init(n int) {
	f.work = make([]freeform.DyadicPoint, n)
	f.left = make([]freeform.DyadicPoint, n)
	f.right = make([]freeform.DyadicPoint, n)
	for _, points := range [][]freeform.DyadicPoint{f.work, f.left, f.right} {
		for i := range points {
			points[i].U = new(big.Int)
			points[i].V = new(big.Int)
		}
	}
}

func refLengthBracket(s freeform.DyadicSpan, frames []refSplitScratch, distance *refDistanceScratch) (float64, float64) {
	if len(frames) == 0 {
		return refChordLower(s, distance), refPolygonUpper(s, distance)
	}
	f := &frames[len(frames)-1]
	f.split(s.Points)
	left := freeform.DyadicSpan{Points: f.left, Den: s.Den, DenSq: s.DenSq}
	right := freeform.DyadicSpan{Points: f.right, Den: s.Den, DenSq: s.DenSq}
	leftLo, leftHi := refLengthBracket(left, frames[:len(frames)-1], distance)
	rightLo, rightHi := refLengthBracket(right, frames[:len(frames)-1], distance)
	return freeform.DownRound(leftLo + rightLo), proofbound.UpRound(leftHi + rightHi)
}

func refSetPoint(dst *freeform.DyadicPoint, src freeform.DyadicPoint) {
	dst.U.Set(src.U)
	dst.V.Set(src.V)
	dst.Exp = src.Exp
}

func (f *refSplitScratch) split(points []freeform.DyadicPoint) {
	n := len(points)
	for i, p := range points {
		refSetPoint(&f.work[i], p)
	}
	refSetPoint(&f.left[0], f.work[0])
	refSetPoint(&f.right[n-1], f.work[n-1])
	for round := n - 1; round > 0; round-- {
		for i := range round {
			refMidpointInto(&f.work[i], f.work[i], f.work[i+1], &f.tmp)
		}
		refSetPoint(&f.left[n-round], f.work[0])
		refSetPoint(&f.right[round-1], f.work[round-1])
	}
}

func refMidpointInto(dst *freeform.DyadicPoint, a, b freeform.DyadicPoint, tmp *big.Int) {
	exp := max(a.Exp, b.Exp)
	dst.U.Lsh(a.U, exp-a.Exp)
	tmp.Lsh(b.U, exp-b.Exp)
	dst.U.Add(dst.U, tmp)
	dst.V.Lsh(a.V, exp-a.Exp)
	tmp.Lsh(b.V, exp-b.Exp)
	dst.V.Add(dst.V, tmp)
	dst.Exp = exp + 1
}

type refDistanceScratch struct {
	du, dv, tmp, num, lhs, rhs, squareMant big.Int
	lastUpper                              float64
	hasLastUpper                           bool
}

func refChordLower(s freeform.DyadicSpan, scratch *refDistanceScratch) float64 {
	return refSqrtDown(refDistanceSquared(s, s.Points[0], s.Points[len(s.Points)-1], scratch), scratch)
}

func refPolygonUpper(s freeform.DyadicSpan, scratch *refDistanceScratch) float64 {
	total := 0.0
	for i := 0; i+1 < len(s.Points); i++ {
		edge := scratch.lastUpper
		if i != 0 || !scratch.hasLastUpper {
			edge = refSqrtUp(refDistanceSquared(s, s.Points[i], s.Points[i+1], scratch), scratch)
		}
		total = proofbound.UpRound(total + edge)
		if i+2 == len(s.Points) {
			scratch.lastUpper = edge
			scratch.hasLastUpper = true
		}
	}
	return total
}

func refDistanceSquared(
	s freeform.DyadicSpan, a, b freeform.DyadicPoint, scratch *refDistanceScratch,
) freeform.SpanSquaredDistance {
	exp := max(a.Exp, b.Exp)
	scratch.du.Lsh(b.U, exp-b.Exp)
	scratch.tmp.Lsh(a.U, exp-a.Exp)
	scratch.du.Sub(&scratch.du, &scratch.tmp)
	scratch.dv.Lsh(b.V, exp-b.Exp)
	scratch.tmp.Lsh(a.V, exp-a.Exp)
	scratch.dv.Sub(&scratch.dv, &scratch.tmp)
	scratch.num.Mul(&scratch.du, &scratch.du)
	scratch.tmp.Mul(&scratch.dv, &scratch.dv)
	scratch.num.Add(&scratch.num, &scratch.tmp)
	return freeform.SpanSquaredDistance{Num: &scratch.num, DenSq: s.DenSq, Exp: exp}
}

func refSquareCmp(f float64, d freeform.SpanSquaredDistance, scratch *refDistanceScratch) int {
	if math.IsNaN(f) || math.IsInf(f, 0) {
		return 1
	}
	if f == 0 {
		return -d.Num.Sign()
	}
	square := proofarith.DyOfFinite(f)
	mant := square.MantInto(&scratch.squareMant)
	scratch.lhs.Mul(mant, mant)
	scratch.lhs.Mul(&scratch.lhs, d.DenSq)
	shift := 2 * (square.Exp() + int(d.Exp))
	if shift >= 0 {
		return scratch.lhs.Lsh(&scratch.lhs, uint(shift)).Cmp(d.Num)
	}
	return scratch.lhs.Cmp(scratch.rhs.Lsh(d.Num, uint(-shift)))
}

func refSqrtDown(d freeform.SpanSquaredDistance, scratch *refDistanceScratch) float64 {
	if d.Num.Sign() <= 0 {
		return 0
	}
	f := refSqrtSeed(d)
	if proofbound.IsNonFinite(f) {
		f = math.MaxFloat64
	}
	for range proofbound.SqrtAdjustLimit {
		if refSquareCmp(f, d, scratch) <= 0 {
			return f
		}
		f = math.Nextafter(f, 0)
	}
	return 0
}

func refSqrtUp(d freeform.SpanSquaredDistance, scratch *refDistanceScratch) float64 {
	if d.Num.Sign() <= 0 {
		return 0
	}
	f := refSqrtSeed(d)
	if proofbound.IsNonFinite(f) {
		f = math.MaxFloat64
	}
	for range proofbound.SqrtAdjustLimit {
		if refSquareCmp(f, d, scratch) >= 0 {
			return f
		}
		f = math.Nextafter(f, math.Inf(1))
	}
	return math.Inf(1)
}

func refSqrtSeed(d freeform.SpanSquaredDistance) float64 {
	if seed, ok := proofarith.RatioSqrtSeed(d.Num, d.DenSq, 2*int(d.Exp)); ok {
		return seed
	}
	var mant, ratio big.Float
	mant.SetPrec(64)
	ratio.SetPrec(64).Quo(new(big.Float).SetInt(d.Num), new(big.Float).SetInt(d.DenSq))
	exp := ratio.MantExp(&mant) - 2*int(d.Exp)
	if exp%2 != 0 {
		exp--
		mant.SetMantExp(&mant, 1)
	}
	m, _ := mant.Float64()
	return math.Ldexp(math.Sqrt(m), exp/2)
}
