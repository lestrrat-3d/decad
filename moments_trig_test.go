package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestTurnSinCosIntervalEnclosesMathSincos is turnSinCosInterval's own
// enclosure proof (design A7 §5.1): every turn checked must have
// math.Sincos's own float64 answer land inside the returned interval, widened
// by a few ulps to absorb the reference libm call's own (undocumented, but
// necessarily tiny) rounding — and the interval itself must be far tighter
// than float64 resolution, so the fixed-point series is not merely correct
// but not the bottleneck of the bracket it serves.
func TestTurnSinCosIntervalEnclosesMathSincos(t *testing.T) {
	const widthCeiling = 1e-40 // trigFixedSeries's own margin measures in the 1e-58 range
	const ulpSlack = 1e-15

	turns := make([]float64, 0, 400)
	turns = append(turns, 0, 1)
	for k := range 8 {
		boundary := float64(k) / 8
		turns = append(turns, boundary)
		turns = append(turns, math.Nextafter(boundary, math.Inf(1)))
		turns = append(turns, math.Nextafter(boundary, math.Inf(-1)))
	}
	for i := range 400 {
		turns = append(turns, float64(i)/400)
	}

	for _, turn := range turns {
		tr := new(big.Rat).SetFloat64(turn)
		require.NotNil(t, tr, "turn=%g", turn)
		sinIv, cosIv := turnSinCosInterval(tr)

		sinLo, _ := sinIv.lo.Float64()
		sinHi, _ := sinIv.hi.Float64()
		cosLo, _ := cosIv.lo.Float64()
		cosHi, _ := cosIv.hi.Float64()
		require.LessOrEqualf(t, sinLo, sinHi, "turn=%g: sin interval inverted", turn)
		require.LessOrEqualf(t, cosLo, cosHi, "turn=%g: cos interval inverted", turn)

		wantSin, wantCos := math.Sincos(2 * math.Pi * turn)
		require.GreaterOrEqualf(t, wantSin, sinLo-ulpSlack, "turn=%g: sin below the enclosure", turn)
		require.LessOrEqualf(t, wantSin, sinHi+ulpSlack, "turn=%g: sin above the enclosure", turn)
		require.GreaterOrEqualf(t, wantCos, cosLo-ulpSlack, "turn=%g: cos below the enclosure", turn)
		require.LessOrEqualf(t, wantCos, cosHi+ulpSlack, "turn=%g: cos above the enclosure", turn)

		sinWidth := new(big.Rat).Sub(sinIv.hi, sinIv.lo)
		cosWidth := new(big.Rat).Sub(cosIv.hi, cosIv.lo)
		sinWidthF, _ := sinWidth.Float64()
		cosWidthF, _ := cosWidth.Float64()
		require.LessOrEqualf(t, sinWidthF, widthCeiling, "turn=%g: sin interval too wide", turn)
		require.LessOrEqualf(t, cosWidthF, widthCeiling, "turn=%g: cos interval too wide", turn)

		// sin^2+cos^2 must enclose 1: the Pythagorean identity holds exactly
		// for the true value, so a sound enclosure of both factors must
		// enclose their sum of squares at 1.
		one := big.NewRat(1, 1)
		sq := func(iv ratInterval) ratInterval { return intervalMul(iv, iv) }
		pyth := intervalAdd(sq(sinIv), sq(cosIv))
		require.LessOrEqualf(t, pyth.lo.Cmp(one), 0, "turn=%g: sin^2+cos^2 lower bound above 1", turn)
		require.GreaterOrEqualf(t, pyth.hi.Cmp(one), 0, "turn=%g: sin^2+cos^2 upper bound below 1", turn)
	}
}

// TestTurnSinCosIntervalMatchesKnownTurns checks a handful of turns whose
// sine/cosine are exactly representable rationals or simple radicals, against
// their own closed forms rather than against math.Sincos, so this test does
// not merely check turnSinCosInterval against the same libm call it is meant
// to replace.
func TestTurnSinCosIntervalMatchesKnownTurns(t *testing.T) {
	sqrt2over2 := math.Sqrt2 / 2
	sqrt3over2 := math.Sqrt(3) / 2
	for _, tc := range []struct {
		turn     float64
		sin, cos float64
	}{
		{0, 0, 1},
		{0.25, 1, 0},
		{0.5, 0, -1},
		{0.75, -1, 0},
		{1.0 / 8, sqrt2over2, sqrt2over2},
		{1.0 / 12, 0.5, sqrt3over2},
		{5.0 / 6, -sqrt3over2, 0.5},
	} {
		tr := new(big.Rat).SetFloat64(tc.turn)
		require.NotNil(t, tr)
		sinIv, cosIv := turnSinCosInterval(tr)
		sinLo, _ := sinIv.lo.Float64()
		sinHi, _ := sinIv.hi.Float64()
		cosLo, _ := cosIv.lo.Float64()
		cosHi, _ := cosIv.hi.Float64()
		require.InDeltaf(t, tc.sin, (sinLo+sinHi)/2, 1e-12, "turn=%g sin", tc.turn)
		require.InDeltaf(t, tc.cos, (cosLo+cosHi)/2, 1e-12, "turn=%g cos", tc.turn)
		require.LessOrEqualf(t, sinLo, tc.sin+1e-12, "turn=%g sin below expected", tc.turn)
		require.GreaterOrEqualf(t, sinHi, tc.sin-1e-12, "turn=%g sin above expected", tc.turn)
	}
}

// BenchmarkTurnSinCosInterval is the design's own cost guard (A7 §5.6):
// ~34us/turn was the prototype's own figure with a 12-term series later
// found to under-charge the alternating-series remainder (trigSeriesTerms's
// own comment); this benchmark is this file's live measurement of the
// corrected 24-term series' real cost, in place of a number asserted in a
// comment. A fractional CircleSeg charges two calls (one per endpoint) per
// area or first-moment bracket.
func BenchmarkTurnSinCosInterval(b *testing.B) {
	turns := make([]*big.Rat, 64)
	for i := range turns {
		turns[i] = big.NewRat(int64(i), int64(len(turns)))
	}
	b.ResetTimer()
	for i := 0; b.Loop(); i++ {
		turnSinCosInterval(turns[i%len(turns)])
	}
}
