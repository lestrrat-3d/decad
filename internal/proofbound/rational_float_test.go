package proofbound_test

import (
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// legacyRatOf, legacyRatFloatDown and legacyRatFloatUp are the bodies RatOf,
// RatFloatDown and RatFloatUp had before they used FloatRat and CmpFloatRat.
func legacyRatOf(f float64) (*big.Rat, bool) {
	r := new(big.Rat)
	if r.SetFloat64(f) == nil {
		return nil, false
	}
	return r, true
}

func legacyRatFloatDown(q *big.Rat) float64 {
	f, _ := q.Float64()
	if proofbound.IsNonFinite(f) {
		return f
	}
	if fr, ok := legacyRatOf(f); ok && fr.Cmp(q) <= 0 {
		return f
	}
	return proofarith.NextDown(f)
}

func legacyRatFloatUp(q *big.Rat) float64 {
	f, _ := q.Float64()
	if proofbound.IsNonFinite(f) {
		return f
	}
	if fr, ok := legacyRatOf(f); ok && fr.Cmp(q) >= 0 {
		return f
	}
	return proofarith.NextUp(f)
}

// adversarialFloats lists floats at the edges of the float64 encoding.
func adversarialFloats() []float64 {
	out := []float64{
		0, math.Copysign(0, -1), 1, -1, 0.5, -0.5, 3, -3, 0.1, -0.1, 1.0 / 3, 255, 256, 257, 1 << 53, 1<<53 + 2,
		math.SmallestNonzeroFloat64, 2 * math.SmallestNonzeroFloat64, 3 * math.SmallestNonzeroFloat64,
		math.Float64frombits(1<<52 - 1), math.Float64frombits(1 << 52), math.Float64frombits(1<<52 + 1),
		math.MaxFloat64, math.Nextafter(math.MaxFloat64, 0), math.Pi, math.E, 1e300, 1e-300, 1e-310,
	}
	for e := -1074; e <= 1023; e++ {
		p := math.Ldexp(1, e)
		out = append(out, p, math.Nextafter(p, 0))
		for _, f := range []float64{math.Nextafter(p, math.Inf(1)), 3 * p / 2} {
			if !proofbound.IsNonFinite(f) {
				out = append(out, f)
			}
		}
	}
	n := len(out)
	for i := range n {
		out = append(out, -out[i])
	}
	return out
}

// randomFinite draws a finite float64 from its bit pattern.
func randomFinite(rng *rand.Rand) float64 {
	for {
		f := math.Float64frombits(rng.Uint64())
		if !proofbound.IsNonFinite(f) {
			return f
		}
	}
}

// ratNear returns exact rationals around f: f itself, one ulp and half an ulp
// to either side, non-dyadic fractions just off f, and unrelated values, so
// that every sign of f - q occurs. f must be finite.
func ratNear(rng *rand.Rand, f float64) []*big.Rat {
	exact := new(big.Rat).SetFloat64(f)
	out := []*big.Rat{exact, new(big.Rat)}
	for _, dir := range []float64{math.Inf(-1), math.Inf(1)} {
		neighbour := math.Nextafter(f, dir)
		if proofbound.IsNonFinite(neighbour) {
			continue
		}
		nr := new(big.Rat).SetFloat64(neighbour)
		out = append(out, nr, new(big.Rat).Quo(new(big.Rat).Add(exact, nr), big.NewRat(2, 1)))
	}
	third := new(big.Rat).Mul(exact, big.NewRat(1000001, 1000000))
	out = append(out, third, new(big.Rat).Mul(exact, big.NewRat(999999, 1000000)),
		new(big.Rat).Add(exact, big.NewRat(1, 3)), new(big.Rat).Sub(exact, big.NewRat(1, 3)),
		new(big.Rat).Quo(exact, big.NewRat(3, 1)), new(big.Rat).Mul(exact, big.NewRat(7, 1)),
		new(big.Rat).Neg(exact))
	// One unit in the last place of the numerator of f/3, in either direction.
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(3), uint(rng.IntN(1200))))
	out = append(out, new(big.Rat).Add(exact, tiny), new(big.Rat).Sub(exact, tiny))
	return out
}

// extremeRats are values that are not near any float: beyond the finite range
// in both directions, below the smallest subnormal, huge and tiny fractions.
func extremeRats() []*big.Rat {
	huge := new(big.Int).Lsh(big.NewInt(1), 1100)
	tiny := new(big.Int).Lsh(big.NewInt(1), 1100)
	out := []*big.Rat{
		new(big.Rat).SetInt(huge), new(big.Rat).SetInt(new(big.Int).Neg(huge)),
		new(big.Rat).SetFrac(big.NewInt(1), tiny), new(big.Rat).SetFrac(big.NewInt(-1), tiny),
		new(big.Rat).SetFrac(big.NewInt(3), new(big.Int).Lsh(big.NewInt(1), 1075)),
		new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 1075)),
		new(big.Rat).SetFrac(big.NewInt(-1), new(big.Int).Lsh(big.NewInt(1), 1075)),
		new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 1074)),
		new(big.Rat).SetFrac(new(big.Int).Lsh(big.NewInt(1), 1024), big.NewInt(1)),
		new(big.Rat).SetFrac(new(big.Int).Sub(new(big.Int).Lsh(big.NewInt(1), 1024), big.NewInt(1)), big.NewInt(1)),
		new(big.Rat).SetFrac(new(big.Int).Lsh(big.NewInt(1), 1023), big.NewInt(1)),
		new(big.Rat).SetFrac(new(big.Int).Sub(new(big.Int).Lsh(big.NewInt(1), 1024), new(big.Int).Lsh(big.NewInt(1), 970)),
			big.NewInt(1)),
		new(big.Rat).SetFrac(new(big.Int).Sub(new(big.Int).Lsh(big.NewInt(1), 1024), new(big.Int).Lsh(big.NewInt(1), 970)),
			big.NewInt(-1)),
		big.NewRat(1, 3), big.NewRat(-1, 3), big.NewRat(5, 1), big.NewRat(-5, 1),
	}
	for _, shift := range []uint{0, 1, 52, 53, 54, 64, 200, 1000, 2000} {
		n := new(big.Int).Lsh(big.NewInt(0x1fffffffffffff), shift)
		out = append(out, new(big.Rat).SetFrac(n, big.NewInt(7)), new(big.Rat).SetFrac(big.NewInt(7), n))
	}
	return out
}

func samePanic(t *testing.T, want, got func(), msg string) {
	t.Helper()
	run := func(fn func()) (v any) {
		defer func() { v = recover() }()
		fn()
		return nil
	}
	wantVal, gotVal := run(want), run(got)
	require.NotNil(t, wantVal, "%s: the reference did not panic", msg)
	require.Equal(t, fmt.Sprint(wantVal), fmt.Sprint(gotVal), "%s: panic", msg)
}

func requireSameFloatBits(t *testing.T, want, got float64, msg string) {
	t.Helper()
	require.Equal(t, math.Float64bits(want), math.Float64bits(got), "%s: want %v got %v", msg, want, got)
}

func TestRatOfMatchesSetFloat64(t *testing.T) {
	check := func(f float64) {
		t.Helper()
		msg := fmt.Sprintf("RatOf(%v)", f)
		want, wantOK := legacyRatOf(f)
		got, gotOK := proofbound.RatOf(f)
		require.Equal(t, wantOK, gotOK, msg)
		if !wantOK {
			require.Nil(t, got, msg)
			return
		}
		requireSameResult(t, want, got, msg)
		require.Equal(t, want, got, "%s: representation", msg)
		// The result is used in place, so mutate both the same way.
		want.Num().Add(want.Num(), big.NewInt(2))
		got.Num().Add(got.Num(), big.NewInt(2))
		requireSameResult(t, want, got, msg+" after Num write")
		want.Denom().Lsh(want.Denom(), 3)
		got.Denom().Lsh(got.Denom(), 3)
		requireSameTerms(t, want, got, msg+" after Denom write")
		want.Add(want, big.NewRat(1, 3))
		got.Add(got, big.NewRat(1, 3))
		requireSameResult(t, want, got, msg+" after Add")
	}
	for _, f := range adversarialFloats() {
		check(f)
	}
	for _, f := range []float64{math.NaN(), math.Inf(1), math.Inf(-1), math.Float64frombits(0x7ff0000000000001)} {
		check(f)
	}
	rng := rand.New(rand.NewPCG(201, 203))
	for range 30000 {
		check(randomFinite(rng))
	}
}

// TestRatOfDenominatorStored pins the stored denominator of every result, zero
// included: Denom returns the same pointer on every call and writing through it
// changes the Rat.
func TestRatOfDenominatorStored(t *testing.T) {
	for _, f := range []float64{0, math.Copysign(0, -1), 1, -1, 2, 1 << 60, 0.5, 3, math.MaxFloat64, math.SmallestNonzeroFloat64} {
		want, _ := legacyRatOf(f)
		got, ok := proofbound.RatOf(f)
		require.True(t, ok)
		require.True(t, denomStored(got), "RatOf(%v)", f)
		require.Equal(t, denomStored(want), denomStored(got), "RatOf(%v)", f)
		got.Denom().SetInt64(5)
		require.Equal(t, int64(5), got.Denom().Int64(), "RatOf(%v) Denom write is lost", f)
	}
}

func TestRatFloatDownUpMatchLegacy(t *testing.T) {
	check := func(q *big.Rat) {
		t.Helper()
		before := new(big.Rat).Set(q)
		storedBefore := denomStored(q)
		wantDown, gotDown := legacyRatFloatDown(q), proofbound.RatFloatDown(q)
		if math.Float64bits(wantDown) != math.Float64bits(gotDown) {
			t.Fatalf("RatFloatDown mismatch for q=%s: want %v, got %v", q, wantDown, gotDown)
		}
		wantUp, gotUp := legacyRatFloatUp(q), proofbound.RatFloatUp(q)
		if math.Float64bits(wantUp) != math.Float64bits(gotUp) {
			t.Fatalf("RatFloatUp mismatch for q=%s: want %v, got %v", q, wantUp, gotUp)
		}
		if before.Num().Cmp(q.Num()) != 0 || before.Denom().Cmp(q.Denom()) != 0 ||
			before.IsInt() != q.IsInt() || storedBefore != denomStored(q) {
			t.Fatalf("RatFloat* modified q=%s", q)
		}
	}
	rng := rand.New(rand.NewPCG(211, 213))
	for _, q := range extremeRats() {
		check(q)
	}
	for _, f := range adversarialFloats() {
		for _, q := range ratNear(rng, f) {
			check(q)
		}
	}
	for range 40000 {
		switch rng.IntN(4) {
		case 0:
			check(ratNear(rng, randomFinite(rng))[rng.IntN(15)])
		case 1:
			check(proofarith.FloatRat(randomFinite(rng)))
		case 2:
			check(ratOperand(rng))
		default:
			a := new(big.Int).SetUint64(rng.Uint64())
			a.Lsh(a, uint(rng.IntN(1300)))
			b := new(big.Int).SetUint64(rng.Uint64() | 1)
			b.Lsh(b, uint(rng.IntN(1300)))
			q := new(big.Rat).SetFrac(a, b)
			if rng.IntN(2) == 0 {
				q.Neg(q)
			}
			check(q)
		}
	}
}

// TestRatFloatDownUpDirection pins the direction itself: an exact float comes
// back unchanged, any q off a float lands on the stated side by one ulp or
// less, never the other.
func TestRatFloatDownUpDirection(t *testing.T) {
	rng := rand.New(rand.NewPCG(223, 227))
	for range 20000 {
		f := randomFinite(rng)
		exact := new(big.Rat).SetFloat64(f)
		requireSameFloatBits(t, f, proofbound.RatFloatDown(exact), "RatFloatDown exact")
		requireSameFloatBits(t, f, proofbound.RatFloatUp(exact), "RatFloatUp exact")
		next := math.Nextafter(f, math.Inf(1))
		if proofbound.IsNonFinite(next) {
			continue
		}
		mid := new(big.Rat).Add(exact, new(big.Rat).SetFloat64(next))
		mid.Quo(mid, big.NewRat(2, 1))
		requireSameFloatBits(t, f, proofbound.RatFloatDown(mid), "RatFloatDown midpoint")
		requireSameFloatBits(t, next, proofbound.RatFloatUp(mid), "RatFloatUp midpoint")
		above := new(big.Rat).Add(exact, big.NewRat(0, 1))
		above.Add(above, new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(3), 1100)))
		if above.Cmp(new(big.Rat).SetFloat64(next)) < 0 {
			requireSameFloatBits(t, f, proofbound.RatFloatDown(above), "RatFloatDown just above f")
			requireSameFloatBits(t, next, proofbound.RatFloatUp(above), "RatFloatUp just above f")
		}
	}
}

func TestRatFloatDownUpNonFiniteAndNil(t *testing.T) {
	inf := new(big.Rat).SetFrac(new(big.Int).Lsh(big.NewInt(1), 1100), big.NewInt(1))
	ninf := new(big.Rat).Neg(inf)
	require.True(t, math.IsInf(proofbound.RatFloatUp(inf), 1))
	require.True(t, math.IsInf(proofbound.RatFloatDown(inf), 1))
	require.True(t, math.IsInf(proofbound.RatFloatUp(ninf), -1))
	require.True(t, math.IsInf(proofbound.RatFloatDown(ninf), -1))
	samePanic(t, func() { legacyRatFloatDown(nil) }, func() { proofbound.RatFloatDown(nil) }, "RatFloatDown(nil)")
	samePanic(t, func() { legacyRatFloatUp(nil) }, func() { proofbound.RatFloatUp(nil) }, "RatFloatUp(nil)")
}

// TestRatFloatDownUpMatchLegacyDyadic feeds rationals with a power-of-two
// denominator, which round by shifting: exact floats, halfway ties, a tie plus
// or minus one unit, integers, and values beyond either end of the float range.
func TestRatFloatDownUpMatchLegacyDyadic(t *testing.T) {
	check := func(q *big.Rat) {
		t.Helper()
		msg := fmt.Sprintf("q=%d bits / 2^%d", q.Num().BitLen(), q.Denom().BitLen()-1)
		requireSameFloatBits(t, legacyRatFloatDown(q), proofbound.RatFloatDown(q), "RatFloatDown "+msg)
		requireSameFloatBits(t, legacyRatFloatUp(q), proofbound.RatFloatUp(q), "RatFloatUp "+msg)
	}
	rng := rand.New(rand.NewPCG(231, 233))
	pow2 := func(k int) *big.Int { return new(big.Int).Lsh(big.NewInt(1), uint(k)) }
	for range 4000 {
		odd := new(big.Int).SetUint64(rng.Uint64()&(1<<54-1) | 1<<53 | 1)
		s := rng.IntN(1300)
		tie := new(big.Int).Lsh(odd, uint(s))
		for _, n := range []*big.Int{
			tie, new(big.Int).Add(tie, big.NewInt(1)), new(big.Int).Sub(tie, big.NewInt(1)),
			new(big.Int).Add(tie, pow2(s/2)),
		} {
			for _, k := range []int{0, 1, rng.IntN(1500), 1074, 1075, 1076} {
				q := new(big.Rat).SetFrac(n, pow2(k))
				check(q)
				check(new(big.Rat).Neg(q))
			}
		}
		f := randomFinite(rng)
		check(proofarith.FloatRat(f))
		check(new(big.Rat).Sub(proofarith.FloatRat(f), proofarith.FloatRat(randomFinite(rng))))
	}
	for _, k := range []int{1023, 1024, 1025, 1100} {
		check(new(big.Rat).SetInt(pow2(k)))
		check(new(big.Rat).SetInt(new(big.Int).Sub(pow2(k), big.NewInt(1))))
		check(new(big.Rat).SetFrac(big.NewInt(-1), pow2(k)))
	}
}
