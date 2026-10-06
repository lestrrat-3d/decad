package proof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
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

// TestRatIntervalVectorProducts checks the three-component dot and cross
// products against every corner of the operand boxes: each corner's exact
// product must lie inside the enclosure, and the enclosure endpoints must be
// attained by some corner, since every component is a sum of products of
// independent intervals. Dropping any one component's term moves a corner
// outside the enclosure.
func TestRatIntervalVectorProducts(t *testing.T) {
	t.Parallel()
	a := [3]proof.RatInterval{
		proof.Interval(big.NewRat(-3, 2), big.NewRat(5, 4)),
		proof.Interval(big.NewRat(7, 8), big.NewRat(9, 4)),
		proof.Interval(big.NewRat(-11, 4), big.NewRat(-1, 8)),
	}
	b := [3]proof.RatInterval{
		proof.Interval(big.NewRat(1, 2), big.NewRat(3, 2)),
		proof.Interval(big.NewRat(-5, 2), big.NewRat(1, 4)),
		proof.Interval(big.NewRat(-1, 1), big.NewRat(13, 8)),
	}
	dot := proof.DotInterval3(a, b)
	cross := proof.CrossInterval3(a, b)
	pick := func(iv proof.RatInterval, hi bool) *big.Rat {
		if hi {
			return iv.Hi
		}
		return iv.Lo
	}
	// Componentwise extremes: for the dot product each product term is
	// independent, so the sum of each term's extreme corner is attained.
	wantDotLo, wantDotHi := new(big.Rat), new(big.Rat)
	for axis := range 3 {
		term := proof.MulInterval(a[axis], b[axis])
		wantDotLo.Add(wantDotLo, term.Lo)
		wantDotHi.Add(wantDotHi, term.Hi)
	}
	require.Zero(t, dot.Lo.Cmp(wantDotLo))
	require.Zero(t, dot.Hi.Cmp(wantDotHi))
	for corner := range 64 {
		var x, y [3]*big.Rat
		for axis := range 3 {
			x[axis] = pick(a[axis], corner&(1<<axis) != 0)
			y[axis] = pick(b[axis], corner&(1<<(axis+3)) != 0)
		}
		value := new(big.Rat)
		for axis := range 3 {
			value.Add(value, new(big.Rat).Mul(x[axis], y[axis]))
		}
		require.True(t, dot.Lo.Cmp(value) <= 0 && value.Cmp(dot.Hi) <= 0, "dot corner %d", corner)
		for axis := range 3 {
			j, k := (axis+1)%3, (axis+2)%3
			component := new(big.Rat).Sub(new(big.Rat).Mul(x[j], y[k]), new(big.Rat).Mul(x[k], y[j]))
			require.True(t, cross[axis].Lo.Cmp(component) <= 0 && component.Cmp(cross[axis].Hi) <= 0,
				"cross axis %d corner %d", axis, corner)
		}
	}
	// Point operands reduce both products to exact rational vector algebra.
	p := [3]proof.RatInterval{proof.PointInterval(big.NewRat(1, 1)), proof.PointInterval(big.NewRat(2, 1)),
		proof.PointInterval(big.NewRat(3, 1))}
	q := [3]proof.RatInterval{proof.PointInterval(big.NewRat(-4, 1)), proof.PointInterval(big.NewRat(5, 1)),
		proof.PointInterval(big.NewRat(1, 2))}
	pointDot := proof.DotInterval3(p, q)
	require.Zero(t, pointDot.Lo.Cmp(big.NewRat(15, 2)))
	require.Zero(t, pointDot.Hi.Cmp(big.NewRat(15, 2)))
	pointCross := proof.CrossInterval3(p, q)
	for axis, want := range []*big.Rat{big.NewRat(-14, 1), big.NewRat(-25, 2), big.NewRat(13, 1)} {
		require.Zero(t, pointCross[axis].Lo.Cmp(want), "axis %d", axis)
		require.Zero(t, pointCross[axis].Hi.Cmp(want), "axis %d", axis)
	}
}

// TestCommonDenominatorRatOpsMatchBigRat feeds AddRat, SubRat and MulRat and
// big.Rat's own Add, Sub and Mul the same operands and requires the same
// numerator and denominator, so a result in other than lowest terms fails
// even where it compares equal. The operands mix held floats across the
// whole exponent range, their sums and products (wide power-of-two
// denominators), integers with trailing zero bits, zero, and fractions over
// other denominators, which take big.Rat's path.
func TestCommonDenominatorRatOpsMatchBigRat(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x452821E638D01377, 0xBE5466CF34E90C6C))
	float := func() *big.Rat {
		for {
			f := math.Float64frombits(rng.Uint64())
			if math.IsNaN(f) || math.IsInf(f, 0) {
				continue
			}
			return new(big.Rat).SetFloat64(f)
		}
	}
	scaled := func() *big.Rat {
		f := (rng.Float64() - 0.5) * math.Ldexp(1, rng.IntN(80)-40)
		return new(big.Rat).SetFloat64(f)
	}
	operand := func() *big.Rat {
		switch rng.IntN(8) {
		case 0:
			return float()
		case 1:
			return new(big.Rat)
		case 2:
			return new(big.Rat).SetInt64(rng.Int64N(1<<20) << rng.IntN(12))
		case 3:
			return big.NewRat(rng.Int64N(2001)-1000, rng.Int64N(999)+1)
		case 4:
			return new(big.Rat).Add(scaled(), scaled())
		case 5:
			return new(big.Rat).Mul(scaled(), scaled())
		case 6:
			var zero big.Rat // an uninitialised denominator
			return &zero
		default:
			return scaled()
		}
	}
	ops := []struct {
		name string
		got  func(z, a, b *big.Rat) *big.Rat
		want func(z, a, b *big.Rat) *big.Rat
	}{
		{"add", proof.AddRat, (*big.Rat).Add},
		{"sub", proof.SubRat, (*big.Rat).Sub},
		{"mul", proof.MulRat, (*big.Rat).Mul},
	}
	requireSame := func(want, got *big.Rat, msgAndArgs ...any) {
		t.Helper()
		require.Zero(t, want.Num().Cmp(got.Num()), msgAndArgs...)
		require.Zero(t, want.Denom().Cmp(got.Denom()), msgAndArgs...)
	}
	for range 20000 {
		a, b := operand(), operand()
		heldA, heldB := new(big.Rat).Set(a), new(big.Rat).Set(b)
		for _, op := range ops {
			want := op.want(new(big.Rat), heldA, heldB)
			requireSame(want, op.got(new(big.Rat), a, b), "%s(%v, %v)", op.name, heldA, heldB)
			requireSame(heldA, a, "%s left operand unchanged", op.name)
			requireSame(heldB, b, "%s right operand unchanged", op.name)
			requireSame(want, op.got(new(big.Rat).Set(a), new(big.Rat).Set(a), b), "%s aliasing z and a", op.name)
			z := new(big.Rat).Set(b)
			requireSame(want, op.got(z, a, z), "%s aliasing z and b", op.name)
			z = new(big.Rat).Set(a)
			requireSame(op.want(new(big.Rat), heldA, heldA), op.got(z, z, z), "%s aliasing all three", op.name)
		}
	}
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
