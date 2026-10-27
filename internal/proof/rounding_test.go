package proof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// frexpFloatRat is FloatRat as it was written over math.Frexp.
func frexpFloatRat(value float64) *big.Rat {
	if value == 0 {
		return new(big.Rat)
	}
	frac, exp := math.Frexp(value)
	mant := int64(frac * (1 << 53))
	exp -= 53
	for mant&1 == 0 {
		mant >>= 1
		exp++
	}
	r := new(big.Rat).SetInt64(mant)
	if exp >= 0 {
		r.Num().Lsh(r.Num(), uint(exp))
	} else {
		r.Denom().Lsh(r.Denom(), uint(-exp))
	}
	return r
}

// bigRatFloatError is RationalFloatError as it was written over big.Rat.Sub.
func bigRatFloatError(exact *big.Rat, held float64) float64 {
	heldRat := frexpFloatRat(held)
	d := new(big.Rat).Sub(exact, heldRat)
	d.Abs(d)
	out, exactFloat := d.Float64()
	if d.Sign() > 0 && out == 0 {
		return math.SmallestNonzeroFloat64
	}
	if !exactFloat {
		out = proof.NextUp(out)
	}
	return out
}

func finiteBits(rng *rand.Rand) float64 {
	for {
		f := math.Float64frombits(rng.Uint64())
		if !math.IsNaN(f) && !math.IsInf(f, 0) {
			return f
		}
	}
}

func TestFloatRatMatchesFrexpBody(t *testing.T) {
	t.Parallel()
	special := []float64{
		0, math.Copysign(0, -1), 1, -1, 0.5, 3, -3, math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64,
		math.Float64frombits(1<<52 - 1), math.Float64frombits(1 << 52), -math.Float64frombits(1 << 52),
		math.MaxFloat64, -math.MaxFloat64, math.Nextafter(1, 2), math.Nextafter(1, 0), 1 << 60, 0.1,
	}
	check := func(f float64) {
		t.Helper()
		got, want := proof.FloatRat(f), frexpFloatRat(f)
		require.Zero(t, want.Cmp(got), "%v", f)
		require.Zero(t, want.Denom().Cmp(got.Denom()), "%v", f)
		require.Zero(t, want.Num().Cmp(got.Num()), "%v", f)
	}
	for _, f := range special {
		check(f)
	}
	rng := rand.New(rand.NewPCG(1, 2))
	for range 100000 {
		check(finiteBits(rng))
	}
	require.Nil(t, proof.FloatRat(math.NaN()))
	require.Nil(t, proof.FloatRat(math.Inf(1)))
}

func TestRationalFloatErrorMatchesBigRatSub(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(3, 4))
	for i := range 20000 {
		held := finiteBits(rng)
		if i%3 == 0 {
			held = (rng.Float64() - 0.5) * math.Ldexp(1, rng.IntN(60)-30)
		}
		var exact *big.Rat
		switch rng.IntN(6) {
		case 0:
			exact = frexpFloatRat(held)
		case 1:
			// a near-tie: held plus a sliver
			exact = new(big.Rat).Add(frexpFloatRat(held), big.NewRat(1, rng.Int64N(1<<40)+1))
		case 2:
			// a difference that flushes below the smallest subnormal
			exact = new(big.Rat).Add(frexpFloatRat(held), new(big.Rat).SetFrac(big.NewInt(1),
				new(big.Int).Lsh(big.NewInt(3), uint(1076+rng.IntN(200)))))
		case 3:
			exact = new(big.Rat).Add(frexpFloatRat(held), frexpFloatRat(finiteBits(rng)))
		case 4:
			exact = big.NewRat(rng.Int64N(2001)-1000, rng.Int64N(999)+1)
		default:
			exact = new(big.Rat).SetFrac(big.NewInt(rng.Int64N(1<<50)-1<<49),
				new(big.Int).Lsh(big.NewInt(rng.Int64N(1<<30)|1), uint(rng.IntN(70))))
		}
		held0 := new(big.Rat).Set(exact)
		got := proof.RationalFloatError(exact, held)
		want := bigRatFloatError(exact, held)
		require.Equal(t, math.Float64bits(want), math.Float64bits(got), "exact=%v held=%v", held0, held)
		require.Zero(t, held0.Cmp(exact))
	}
	require.True(t, math.IsInf(proof.RationalFloatError(nil, 1), 1))
	require.True(t, math.IsInf(proof.RationalFloatError(big.NewRat(1, 3), math.NaN()), 1))
}

func TestRationalFloatErrorRoundingBoundaries(t *testing.T) {
	t.Parallel()
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(3), 1200))
	huge := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 2048))
	for i, tc := range []struct {
		exact *big.Rat
		held  float64
	}{
		{new(big.Rat), 0},
		{new(big.Rat), math.Copysign(0, -1)},
		{big.NewRat(7, 3), 0},
		{big.NewRat(-7, 3), 0},
		{big.NewRat(7, 3), 7.0 / 3},
		{big.NewRat(-7, 3), -7.0 / 3},
		{proof.FloatRat(math.Pi), math.Pi},
		{proof.FloatRat(math.MaxFloat64), -math.MaxFloat64},
		{tiny, 0},
		{new(big.Rat).Neg(tiny), 0},
		{huge, 0},
		{new(big.Rat).Neg(huge), math.SmallestNonzeroFloat64},
		{big.NewRat(1<<53+1, 1<<53), 0},
		{big.NewRat(-(1<<53 + 1), 1<<53), 0},
	} {
		num, den := new(big.Int).Set(tc.exact.Num()), new(big.Int).Set(tc.exact.Denom())
		want := bigRatFloatError(tc.exact, tc.held)
		got := proof.RationalFloatError(tc.exact, tc.held)
		require.Equal(t, math.Float64bits(want), math.Float64bits(got), "case %d", i)
		require.Zero(t, num.Cmp(tc.exact.Num()), "case %d numerator changed", i)
		require.Zero(t, den.Cmp(tc.exact.Denom()), "case %d denominator changed", i)
	}
	for _, held := range []float64{math.Inf(-1), math.Inf(1), math.NaN()} {
		require.True(t, math.IsInf(proof.RationalFloatError(big.NewRat(1, 3), held), 1))
	}
}

func TestCmpFloatRatMatchesRatCmp(t *testing.T) {
	t.Parallel()
	// Each float is checked against many rationals. Its independent exact
	// reference is immutable and need only be converted once per bit pattern.
	references := make(map[uint64]*big.Rat)
	check := func(f float64, q *big.Rat) {
		t.Helper()
		before := new(big.Rat).Set(q)
		bits := math.Float64bits(f)
		ref := references[bits]
		if ref == nil {
			ref = new(big.Rat).SetFloat64(f)
			references[bits] = ref
		}
		want := ref.Cmp(q)
		require.Equal(t, want, proof.CmpFloatRat(f, q), "f=%v q=%v", f, q)
		require.Zero(t, before.Cmp(q), "CmpFloatRat modified q")
		require.Zero(t, before.Denom().Cmp(q.Denom()), "CmpFloatRat modified the denominator")
	}
	exactQ := func(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }
	floats := []float64{
		0, math.Copysign(0, -1), 1, -1, 0.1, -0.1, 3, 1 << 53, math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64,
		math.Float64frombits(1<<52 - 1), math.MaxFloat64, -math.MaxFloat64, math.Pi,
	}
	for e := -1074; e <= 1023; e++ {
		p := math.Ldexp(1, e)
		floats = append(floats, p, -p, math.Nextafter(p, 0), math.Nextafter(p, math.Inf(1)), 3*p/2)
	}
	rng := rand.New(rand.NewPCG(301, 303))
	for range 10000 {
		floats = append(floats, finiteBits(rng))
	}
	for _, f := range floats {
		if math.IsInf(f, 0) {
			continue
		}
		q := exactQ(f)
		check(f, q)
		check(f, new(big.Rat).Neg(q))
		check(f, new(big.Rat))
		for _, dir := range []float64{math.Inf(-1), math.Inf(1)} {
			n := math.Nextafter(f, dir)
			if math.IsInf(n, 0) {
				continue
			}
			nq := exactQ(n)
			check(f, nq)
			check(f, new(big.Rat).Quo(new(big.Rat).Add(q, nq), big.NewRat(2, 1)))
		}
		check(f, new(big.Rat).Mul(q, big.NewRat(1000001, 1000000)))
		check(f, new(big.Rat).Quo(q, big.NewRat(3, 1)))
		check(f, new(big.Rat).Add(q, big.NewRat(1, 3)))
		check(f, new(big.Rat).SetFrac(big.NewInt(int64(rng.IntN(2000)-1000)), big.NewInt(int64(rng.IntN(999)+1))))
		huge := new(big.Int).Lsh(big.NewInt(int64(rng.IntN(1<<30)+1)), uint(rng.IntN(1300)))
		check(f, new(big.Rat).SetFrac(huge, big.NewInt(int64(rng.IntN(1<<20)+1))))
		check(f, new(big.Rat).SetFrac(big.NewInt(int64(rng.IntN(1<<20)+1)), huge))
	}
	// A Rat whose denominator is unstored is read without being written.
	var zero big.Rat
	check(1, &zero)
	check(-1, &zero)
	unstored := new(big.Rat).SetInt(big.NewInt(1))
	unstored.SetFrac(big.NewInt(4), big.NewInt(2))
	check(2, unstored)
}

// TestFracFloatMatchesRatFloat checks FracFloatDown and FracFloatUp against
// proofbound.RatFloatDown and RatFloatUp of the reduced value, bit for bit, on
// every fraction of the same value the multipliers make: floats themselves,
// float neighbours and midpoints, slivers off a float, random rationals, and
// values that overflow or fall below the subnormals.
func TestFracFloatMatchesRatFloat(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(1101, 1103))
	multipliers := []*big.Int{
		big.NewInt(1), big.NewInt(2), big.NewInt(3), big.NewInt(1 << 40),
		new(big.Int).Lsh(big.NewInt(1), 300), new(big.Int).Mul(big.NewInt(1<<61-1), big.NewInt(1<<31-1)),
	}
	check := func(q *big.Rat) {
		t.Helper()
		wantDown, wantUp := proofbound.RatFloatDown(q), proofbound.RatFloatUp(q)
		for _, k := range multipliers {
			n := new(big.Int).Mul(q.Num(), k)
			d := new(big.Int).Mul(q.Denom(), k)
			n0, d0 := new(big.Int).Set(n), new(big.Int).Set(d)
			require.Equal(t, math.Float64bits(wantDown), math.Float64bits(proof.FracFloatDown(n, d)), "down q=%v k=%v", q, k)
			require.Equal(t, math.Float64bits(wantUp), math.Float64bits(proof.FracFloatUp(n, d)), "up q=%v k=%v", q, k)
			require.Zero(t, n0.Cmp(n), "numerator changed")
			require.Zero(t, d0.Cmp(d), "denominator changed")
		}
	}
	values := []*big.Rat{
		new(big.Rat), big.NewRat(1, 3), big.NewRat(-1, 3), big.NewRat(7, 5), big.NewRat(-7, 5),
		new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(3), 1100)),
		new(big.Rat).SetInt(new(big.Int).Neg(new(big.Int).Lsh(big.NewInt(3), 1100))),
		new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(3), 1100)),
		new(big.Rat).SetFrac(big.NewInt(-1), new(big.Int).Lsh(big.NewInt(3), 1100)),
	}
	// Between the largest float and 2^1024 the nearest float is an infinity,
	// which RatFloatDown and RatFloatUp return as it is.
	for _, below := range []uint{969, 900, 970} {
		near := new(big.Int).Lsh(big.NewInt(1), 1024)
		near.Sub(near, new(big.Int).Lsh(big.NewInt(1), below))
		values = append(values, new(big.Rat).SetInt(near), new(big.Rat).SetInt(new(big.Int).Neg(near)))
	}
	floats := []float64{1, -1, 0.1, math.Pi, math.MaxFloat64, -math.MaxFloat64, math.SmallestNonzeroFloat64,
		math.Float64frombits(1<<52 - 1), math.Float64frombits(1 << 52)}
	for e := -1074; e <= 1023; e += 7 {
		floats = append(floats, math.Ldexp(1, e), -math.Ldexp(3, e-1))
	}
	for range 2000 {
		floats = append(floats, finiteBits(rng), (rng.Float64()-0.5)*math.Ldexp(1, rng.IntN(80)-40))
	}
	sliver := big.NewRat(1, 1<<62)
	for _, f := range floats {
		q := new(big.Rat).SetFloat64(f)
		values = append(values, q, new(big.Rat).Mul(q, big.NewRat(1000001, 1000000)), new(big.Rat).Quo(q, big.NewRat(3, 1)))
		if n := math.Nextafter(f, math.Inf(1)); !math.IsInf(n, 0) {
			mid := new(big.Rat).Add(q, new(big.Rat).SetFloat64(n))
			values = append(values, mid.Quo(mid, big.NewRat(2, 1)))
		}
		lift := new(big.Rat).Mul(sliver, new(big.Rat).Abs(q))
		values = append(values, new(big.Rat).Add(q, lift), new(big.Rat).Sub(q, lift))
	}
	for range 2000 {
		values = append(values, new(big.Rat).SetFrac(big.NewInt(rng.Int64N(1<<62)-1<<61), big.NewInt(rng.Int64N(1<<40)+1)))
	}
	for _, q := range values {
		check(q)
	}
}
