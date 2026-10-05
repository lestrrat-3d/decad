package proof_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func TestRatIntervalArithmeticOwnsFreshEndpoints(t *testing.T) {
	a := proof.Interval(big.NewRat(-7, 13), big.NewRat(11, 17))
	b := proof.Interval(big.NewRat(-5, 19), big.NewRat(23, 29))
	scale := big.NewRat(-31, 37)
	tests := []struct {
		name string
		got  proof.RatInterval
		lo   *big.Rat
		hi   *big.Rat
	}{
		{name: "add", got: proof.AddInterval(a, b), lo: new(big.Rat).Add(a.Lo, b.Lo), hi: new(big.Rat).Add(a.Hi, b.Hi)},
		{name: "neg", got: proof.NegInterval(a), lo: new(big.Rat).Neg(a.Hi), hi: new(big.Rat).Neg(a.Lo)},
		{name: "sub", got: proof.SubInterval(a, b), lo: new(big.Rat).Sub(a.Lo, b.Hi), hi: new(big.Rat).Sub(a.Hi, b.Lo)},
		{name: "scale", got: proof.ScaleInterval(a, scale), lo: new(big.Rat).Mul(a.Hi, scale), hi: new(big.Rat).Mul(a.Lo, scale)},
		{name: "mul", got: proof.MulInterval(a, b), lo: big.NewRat(-161, 377), hi: big.NewRat(253, 493)},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			require.Zero(t, tt.got.Lo.Cmp(tt.lo))
			require.Zero(t, tt.got.Hi.Cmp(tt.hi))
			require.NotSame(t, a.Lo, tt.got.Lo)
			require.NotSame(t, a.Hi, tt.got.Hi)
			require.NotSame(t, b.Lo, tt.got.Lo)
			require.NotSame(t, b.Hi, tt.got.Hi)

			wantLo := new(big.Rat).Set(tt.got.Lo)
			wantHi := new(big.Rat).Set(tt.got.Hi)
			a.Lo.SetInt64(100)
			a.Hi.SetInt64(101)
			b.Lo.SetInt64(102)
			b.Hi.SetInt64(103)
			require.Zero(t, tt.got.Lo.Cmp(wantLo))
			require.Zero(t, tt.got.Hi.Cmp(wantHi))

			tt.got.Lo.SetInt64(200)
			tt.got.Hi.SetInt64(201)
			require.Equal(t, int64(100), a.Lo.Num().Int64())
			require.Equal(t, int64(101), a.Hi.Num().Int64())
			require.Equal(t, int64(102), b.Lo.Num().Int64())
			require.Equal(t, int64(103), b.Hi.Num().Int64())
		})
	}
}

func TestRatIntervalMulSignCases(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name string
		iv   proof.RatInterval
	}{
		{"positive", proof.Interval(big.NewRat(2, 3), big.NewRat(7, 5))},
		{"negative", proof.Interval(big.NewRat(-11, 7), big.NewRat(-3, 4))},
		{"mixed", proof.Interval(big.NewRat(-13, 9), big.NewRat(5, 6))},
		{"zero", proof.Interval(big.NewRat(0, 1), big.NewRat(0, 1))},
		{"zero-to-positive", proof.Interval(big.NewRat(0, 1), big.NewRat(17, 13))},
		{"negative-to-zero", proof.Interval(big.NewRat(-19, 11), big.NewRat(0, 1))},
		{"positive-point", proof.Interval(big.NewRat(23, 17), big.NewRat(23, 17))},
		{"negative-point", proof.Interval(big.NewRat(-29, 19), big.NewRat(-29, 19))},
	}
	check := func(t *testing.T, a, b proof.RatInterval) {
		t.Helper()
		before := [4]*big.Rat{
			new(big.Rat).Set(a.Lo), new(big.Rat).Set(a.Hi),
			new(big.Rat).Set(b.Lo), new(big.Rat).Set(b.Hi),
		}
		corners := [4]*big.Rat{
			new(big.Rat).Mul(a.Lo, b.Lo), new(big.Rat).Mul(a.Lo, b.Hi),
			new(big.Rat).Mul(a.Hi, b.Lo), new(big.Rat).Mul(a.Hi, b.Hi),
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

		got := proof.MulInterval(a, b)
		require.Zero(t, got.Lo.Cmp(wantLo))
		require.Zero(t, got.Hi.Cmp(wantHi))
		for i, input := range [4]*big.Rat{a.Lo, a.Hi, b.Lo, b.Hi} {
			require.Zero(t, input.Cmp(before[i]), "input endpoint %d changed", i)
			require.NotSame(t, input, got.Lo)
			require.NotSame(t, input, got.Hi)
		}
		require.NotSame(t, got.Lo, got.Hi)
		got.Lo.SetInt64(101)
		got.Hi.SetInt64(103)
		for i, input := range [4]*big.Rat{a.Lo, a.Hi, b.Lo, b.Hi} {
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
	got := proof.PointInterval(value)
	value.SetInt64(13)
	require.Zero(t, got.Lo.Cmp(big.NewRat(7, 11)))
	require.Zero(t, got.Hi.Cmp(big.NewRat(7, 11)))

	got.Lo.SetInt64(17)
	got.Hi.SetInt64(19)
	require.Zero(t, value.Cmp(big.NewRat(13, 1)))
}

var ratIntervalBenchmarkSink proof.RatInterval

func BenchmarkRatIntervalArithmetic(b *testing.B) {
	a := proof.Interval(big.NewRat(-7, 13), big.NewRat(11, 17))
	c := proof.Interval(big.NewRat(-5, 19), big.NewRat(23, 29))
	scale := big.NewRat(-31, 37)
	b.ReportAllocs()
	for b.Loop() {
		ratIntervalBenchmarkSink = proof.AddInterval(a, c)
		ratIntervalBenchmarkSink = proof.SubInterval(a, c)
		ratIntervalBenchmarkSink = proof.ScaleInterval(a, scale)
		ratIntervalBenchmarkSink = proof.MulInterval(a, c)
	}
}
