package decad

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestRatIntervalArithmeticOwnsFreshEndpoints(t *testing.T) {
	a := interval(big.NewRat(-7, 13), big.NewRat(11, 17))
	b := interval(big.NewRat(-5, 19), big.NewRat(23, 29))
	scale := big.NewRat(-31, 37)
	tests := []struct {
		name string
		got  ratInterval
		lo   *big.Rat
		hi   *big.Rat
	}{
		{name: "add", got: intervalAdd(a, b), lo: new(big.Rat).Add(a.lo, b.lo), hi: new(big.Rat).Add(a.hi, b.hi)},
		{name: "neg", got: intervalNeg(a), lo: new(big.Rat).Neg(a.hi), hi: new(big.Rat).Neg(a.lo)},
		{name: "sub", got: intervalSub(a, b), lo: new(big.Rat).Sub(a.lo, b.hi), hi: new(big.Rat).Sub(a.hi, b.lo)},
		{name: "scale", got: intervalScale(a, scale), lo: new(big.Rat).Mul(a.hi, scale), hi: new(big.Rat).Mul(a.lo, scale)},
		{name: "mul", got: intervalMul(a, b), lo: big.NewRat(-161, 377), hi: big.NewRat(253, 493)},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			require.Zero(t, tt.got.lo.Cmp(tt.lo))
			require.Zero(t, tt.got.hi.Cmp(tt.hi))
			require.NotSame(t, a.lo, tt.got.lo)
			require.NotSame(t, a.hi, tt.got.hi)
			require.NotSame(t, b.lo, tt.got.lo)
			require.NotSame(t, b.hi, tt.got.hi)

			wantLo := new(big.Rat).Set(tt.got.lo)
			wantHi := new(big.Rat).Set(tt.got.hi)
			a.lo.SetInt64(100)
			a.hi.SetInt64(101)
			b.lo.SetInt64(102)
			b.hi.SetInt64(103)
			require.Zero(t, tt.got.lo.Cmp(wantLo))
			require.Zero(t, tt.got.hi.Cmp(wantHi))

			tt.got.lo.SetInt64(200)
			tt.got.hi.SetInt64(201)
			require.Equal(t, int64(100), a.lo.Num().Int64())
			require.Equal(t, int64(101), a.hi.Num().Int64())
			require.Equal(t, int64(102), b.lo.Num().Int64())
			require.Equal(t, int64(103), b.hi.Num().Int64())
		})
	}
}

func TestRatIntervalMulSignCases(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name string
		iv   ratInterval
	}{
		{"positive", interval(big.NewRat(2, 3), big.NewRat(7, 5))},
		{"negative", interval(big.NewRat(-11, 7), big.NewRat(-3, 4))},
		{"mixed", interval(big.NewRat(-13, 9), big.NewRat(5, 6))},
		{"zero", interval(big.NewRat(0, 1), big.NewRat(0, 1))},
		{"zero-to-positive", interval(big.NewRat(0, 1), big.NewRat(17, 13))},
		{"negative-to-zero", interval(big.NewRat(-19, 11), big.NewRat(0, 1))},
		{"positive-point", interval(big.NewRat(23, 17), big.NewRat(23, 17))},
		{"negative-point", interval(big.NewRat(-29, 19), big.NewRat(-29, 19))},
	}
	check := func(t *testing.T, a, b ratInterval) {
		t.Helper()
		before := [4]*big.Rat{
			new(big.Rat).Set(a.lo), new(big.Rat).Set(a.hi),
			new(big.Rat).Set(b.lo), new(big.Rat).Set(b.hi),
		}
		corners := [4]*big.Rat{
			new(big.Rat).Mul(a.lo, b.lo), new(big.Rat).Mul(a.lo, b.hi),
			new(big.Rat).Mul(a.hi, b.lo), new(big.Rat).Mul(a.hi, b.hi),
		}
		wantLo, wantHi := corners[0], corners[0]
		for _, corner := range corners[1:] {
			if corner.Cmp(wantLo) < 0 {
				wantLo = corner
			}
			if corner.Cmp(wantHi) > 0 {
				wantHi = corner
			}
		}

		got := intervalMul(a, b)
		require.Zero(t, got.lo.Cmp(wantLo))
		require.Zero(t, got.hi.Cmp(wantHi))
		for i, input := range [4]*big.Rat{a.lo, a.hi, b.lo, b.hi} {
			require.Zero(t, input.Cmp(before[i]), "input endpoint %d changed", i)
			require.NotSame(t, input, got.lo)
			require.NotSame(t, input, got.hi)
		}
		require.NotSame(t, got.lo, got.hi)
		got.lo.SetInt64(101)
		got.hi.SetInt64(103)
		for i, input := range [4]*big.Rat{a.lo, a.hi, b.lo, b.hi} {
			require.Zero(t, input.Cmp(before[i]), "result mutation changed input endpoint %d", i)
		}
	}

	for _, a := range cases {
		for _, b := range cases {
			t.Run(a.name+"/"+b.name, func(t *testing.T) { check(t, a.iv, b.iv) })
		}
		t.Run(a.name+"/shared", func(t *testing.T) { check(t, a.iv, a.iv) })
	}
}

func TestRatIntervalConstructorsCopyBorrowedEndpoints(t *testing.T) {
	t.Parallel()
	value := big.NewRat(7, 11)
	got := pointInterval(value)
	value.SetInt64(13)
	require.Zero(t, got.lo.Cmp(big.NewRat(7, 11)))
	require.Zero(t, got.hi.Cmp(big.NewRat(7, 11)))

	got.lo.SetInt64(17)
	got.hi.SetInt64(19)
	require.Zero(t, value.Cmp(big.NewRat(13, 1)))
}

func TestRatIntervalCachedPiCopiesRemainIndependent(t *testing.T) {
	t.Parallel()
	wantLo := new(big.Rat).Set(quarterPiIv.lo)
	wantHi := new(big.Rat).Set(quarterPiIv.hi)

	got := quarterPiInterval()
	got.lo.SetInt64(0)
	got.hi.SetInt64(1)
	require.Zero(t, quarterPiIv.lo.Cmp(wantLo))
	require.Zero(t, quarterPiIv.hi.Cmp(wantHi))

	second := quarterPiInterval()
	require.Zero(t, second.lo.Cmp(wantLo))
	require.Zero(t, second.hi.Cmp(wantHi))
	require.NotSame(t, got.lo, second.lo)
	require.NotSame(t, got.hi, second.hi)
}

var ratIntervalBenchmarkSink ratInterval

func BenchmarkRatIntervalArithmetic(b *testing.B) {
	a := interval(big.NewRat(-7, 13), big.NewRat(11, 17))
	c := interval(big.NewRat(-5, 19), big.NewRat(23, 29))
	scale := big.NewRat(-31, 37)
	b.ReportAllocs()
	for b.Loop() {
		ratIntervalBenchmarkSink = intervalAdd(a, c)
		ratIntervalBenchmarkSink = intervalSub(a, c)
		ratIntervalBenchmarkSink = intervalScale(a, scale)
		ratIntervalBenchmarkSink = intervalMul(a, c)
	}
}
