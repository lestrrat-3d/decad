package decad

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"
)

// requireSameFloatBits asserts two floats are the same IEEE-754 value, bit for
// bit: a published bound that moved by one ulp is a changed result.
func requireSameFloatBits(t *testing.T, want, got float64, msgAndArgs ...any) {
	t.Helper()
	require.Equal(t, math.Float64bits(want), math.Float64bits(got), msgAndArgs...)
}

// The three oracles below are the line-walk bounds as they were computed over
// big.Rat before the dyadic rewrite, kept verbatim so the rewrite is pinned to
// the values it replaced rather than to a model of them.

func ratLineWalkTangentBound(seg LineSeg, heldU, heldV float64) float64 {
	u0 := ratLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0 := ratLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1 := ratLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1 := ratLerp(seg.Start.V, seg.End.V, seg.TEnd)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil {
		return math.Inf(1)
	}
	return math.Max(
		rationalFloatError(new(big.Rat).Sub(u1, u0), heldU),
		rationalFloatError(new(big.Rat).Sub(v1, v0), heldV),
	)
}

func ratLineWalkEndBound(seg LineSeg, t, heldU, heldV float64) walkEndBound {
	return walkEndBound{
		u: rationalFloatError(ratLerp(seg.Start.U, seg.End.U, t), heldU),
		v: rationalFloatError(ratLerp(seg.Start.V, seg.End.V, t), heldV),
	}
}

func ratLineWalkBounds(seg LineSeg, held float64) (float64, float64, float64) {
	u0 := ratLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0 := ratLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1 := ratLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1 := ratLerp(seg.Start.V, seg.End.V, seg.TEnd)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil {
		return math.Inf(1), math.Inf(1), math.Inf(1)
	}
	du := new(big.Rat).Sub(u1, u0)
	dv := new(big.Rat).Sub(v1, v0)
	lengthSquared := new(big.Rat).Add(
		new(big.Rat).Mul(du, du),
		new(big.Rat).Mul(dv, dv),
	)
	heldRat := floatRat(held)
	coordUpper := math.Max(ratL1Upper(u0, v0), ratL1Upper(u1, v1))
	if heldRat != nil && new(big.Rat).Mul(heldRat, heldRat).Cmp(lengthSquared) == 0 {
		return 0, held, coordUpper
	}
	l1 := new(big.Rat).Add(new(big.Rat).Abs(du), new(big.Rat).Abs(dv))
	upper, exact := l1.Float64()
	if !exact {
		upper = math.Nextafter(upper, math.Inf(1))
	}
	bound := math.Min(conservativeValueError(held, upper), sqrtIntervalError(lengthSquared, held))
	return bound, upper, coordUpper
}

// TestLineWalkBoundsDyadicMatchRational pins lineWalkBounds,
// lineWalkTangentBound and lineWalkEndBound, computed over dyadics, to the
// big.Rat computation they replaced, bit for bit, on every output. The
// segments mix integer, eighth-unit, 1e-6, 1e2 and 1e6 coordinates, natural,
// midpoint and random parameters, and a non-finite coordinate now and then.
// Each is read at the walk's own held values and at a held length one ulp
// high, which exercises the inexact rounding paths.
//
// Shown to fail: publishing through dyFloatUp instead of dyNearestUp turns the
// test red on an inexact value whose nearest float rounded up, in
// dyRoundedFloatError alone (the tangent leg) and in dyL1Upper alone (the
// length-upper leg); deleting lineWalkBounds' dySquareEquals early return
// turns it red on an exact-length segment.
func TestLineWalkBoundsDyadicMatchRational(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 9))
	coord := func() float64 {
		switch rng.IntN(5) {
		case 0:
			return float64(rng.IntN(200) - 100)
		case 1:
			return float64(rng.IntN(2000)-1000) / 8
		case 2:
			return rng.NormFloat64() * 100
		case 3:
			return rng.NormFloat64() * 1e-6
		default:
			return rng.NormFloat64() * 1e6
		}
	}
	param := func() float64 {
		switch rng.IntN(4) {
		case 0:
			return 0
		case 1:
			return 1
		case 2:
			return 0.5
		default:
			return rng.Float64()
		}
	}
	for i := range 40000 {
		seg := LineSeg{
			Start: Point2{U: coord(), V: coord()}, End: Point2{U: coord(), V: coord()},
			TStart: param(), TEnd: param(),
		}
		if i%97 == 0 {
			seg.Start.U = math.Inf(1)
		}
		u0, v0 := lerp2(seg.Start, seg.End, seg.TStart)
		u1, v1 := lerp2(seg.Start, seg.End, seg.TEnd)
		du, dv := u1-u0, v1-v0
		length := math.Hypot(du, dv)
		for _, held := range []float64{length, math.Nextafter(length, math.Inf(1))} {
			wb, wu, wc := ratLineWalkBounds(seg, held)
			gb, gu, gc := lineWalkBounds(seg, held)
			requireSameFloatBits(t, wb, gb, "length bound of %+v at %v", seg, held)
			requireSameFloatBits(t, wu, gu, "length upper of %+v at %v", seg, held)
			requireSameFloatBits(t, wc, gc, "coordinate upper of %+v at %v", seg, held)
		}
		requireSameFloatBits(t, ratLineWalkTangentBound(seg, du, dv), lineWalkTangentBound(seg, du, dv),
			"tangent bound of %+v", seg)
		for _, end := range [][3]float64{{seg.TStart, u0, v0}, {seg.TEnd, u1, v1}} {
			want := ratLineWalkEndBound(seg, end[0], end[1], end[2])
			got := lineWalkEndBound(seg, end[0], end[1], end[2])
			requireSameFloatBits(t, want.u, got.u, "end u of %+v at t=%v", seg, end[0])
			requireSameFloatBits(t, want.v, got.v, "end v of %+v at t=%v", seg, end[0])
		}
	}
}
