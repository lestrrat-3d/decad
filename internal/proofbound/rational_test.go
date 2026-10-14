package proofbound_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// legacyRatAdd and legacyRatMul are the bodies RatAdd and RatMul had before
// they folded through proofarith.AddRat and MulRat.
func legacyRatAdd(values ...*big.Rat) *big.Rat {
	out := new(big.Rat)
	for _, value := range values {
		out.Add(out, value)
	}
	return out
}

func legacyRatMul(values ...*big.Rat) *big.Rat {
	out := big.NewRat(1, 1)
	for _, value := range values {
		out.Mul(out, value)
	}
	return out
}

func requireSameTerms(t *testing.T, want, got *big.Rat, msg string) {
	t.Helper()
	require.Zero(t, want.Cmp(got), "%s: value", msg)
	require.Equal(t, want.Num().String(), got.Num().String(), "%s: numerator", msg)
	require.Equal(t, want.Denom().String(), got.Denom().String(), "%s: denominator", msg)
	require.Equal(t, want.IsInt(), got.IsInt(), "%s: IsInt", msg)
}

// denomStored reports whether r holds its denominator, so that Denom returns
// the same pointer on every call, rather than a fresh detached Int.
func denomStored(r *big.Rat) bool {
	first, second := r.Denom(), r.Denom()
	return first == second
}

// requireSameResult adds the Denom pointer behaviour to requireSameTerms: a
// stored denominator returns one pointer per call, an unstored one a fresh
// detached Int.
func requireSameResult(t *testing.T, want, got *big.Rat, msg string) {
	t.Helper()
	requireSameTerms(t, want, got, msg)
	require.Equal(t, denomStored(want), denomStored(got), "%s: stored denominator", msg)
}

// ratOperand draws an operand from the classes the constructors see: zero,
// small integers, floats lifted exactly, huge and tiny dyadics, and
// non-dyadic fractions, each of either sign.
func ratOperand(rng *rand.Rand) *big.Rat {
	var r *big.Rat
	switch rng.IntN(8) {
	case 0:
		r = new(big.Rat)
	case 1:
		r = new(big.Rat).SetInt64(int64(rng.IntN(21) - 10))
	case 2:
		r = proofarith.FloatRat((rng.Float64() - 0.5) * 1e3)
		if r == nil {
			r = new(big.Rat)
		}
	case 3:
		r = new(big.Rat).SetFrac(new(big.Int).Lsh(big.NewInt(int64(rng.IntN(1<<20)|1)), uint(rng.IntN(900))), big.NewInt(1))
	case 4:
		r = proofarith.FloatRat(math.Ldexp(rng.Float64()+0.5, -rng.IntN(1000)))
	case 5:
		r = big.NewRat(int64(rng.IntN(2000)-1000), int64(rng.IntN(999)+1))
	case 6:
		n := new(big.Int).SetUint64(rng.Uint64())
		n.Lsh(n, 64).Add(n, new(big.Int).SetUint64(rng.Uint64()))
		d := new(big.Int).SetUint64(rng.Uint64() | 1)
		d.Mul(d, big.NewInt(3))
		r = new(big.Rat).SetFrac(n, d)
	default:
		r = new(big.Rat).SetFrac(big.NewInt(int64(rng.IntN(100)+1)), new(big.Int).Lsh(big.NewInt(1), uint(rng.IntN(300))))
	}
	if r.Sign() != 0 && rng.IntN(2) == 0 {
		r.Neg(r)
	}
	return r
}

func TestRatAddMulMatchBigRatFold(t *testing.T) {
	rng := rand.New(rand.NewPCG(101, 103))
	for range 6000 {
		values := make([]*big.Rat, rng.IntN(6))
		for i := range values {
			values[i] = ratOperand(rng)
		}
		// Equal operands, as one pointer and as an equal copy.
		if len(values) >= 2 && rng.IntN(4) == 0 {
			values[1] = values[0]
		} else if len(values) >= 2 && rng.IntN(4) == 0 {
			values[1] = new(big.Rat).Set(values[0])
		}
		before := make([]*big.Rat, len(values))
		for i, v := range values {
			before[i] = new(big.Rat).Set(v)
		}
		requireSameResult(t, legacyRatAdd(values...), proofbound.RatAdd(values...), "RatAdd")
		requireSameResult(t, legacyRatMul(values...), proofbound.RatMul(values...), "RatMul")
		for i, v := range values {
			requireSameTerms(t, before[i], v, "operand modified")
		}
	}
}

func TestRatAddMulEmptyAndSingle(t *testing.T) {
	requireSameResult(t, new(big.Rat), proofbound.RatAdd(), "RatAdd()")
	requireSameResult(t, big.NewRat(1, 1), proofbound.RatMul(), "RatMul()")
	for _, v := range []*big.Rat{new(big.Rat), big.NewRat(-3, 7), big.NewRat(5, 1), proofarith.FloatRat(-0.375)} {
		requireSameResult(t, legacyRatAdd(v), proofbound.RatAdd(v), "RatAdd(v)")
		requireSameResult(t, legacyRatMul(v), proofbound.RatMul(v), "RatMul(v)")
	}
}

// TestRatAddMulResultOwnsStorage checks the result shares no memory with an
// operand: changing the result leaves the operands alone, and changing an
// operand leaves the result alone.
func TestRatAddMulResultOwnsStorage(t *testing.T) {
	for name, fold := range map[string]func(...*big.Rat) *big.Rat{
		"RatAdd": proofbound.RatAdd,
		"RatMul": proofbound.RatMul,
	} {
		for _, single := range []bool{true, false} {
			a, b := big.NewRat(3, 8), big.NewRat(-5, 7)
			values := []*big.Rat{a, b}
			if single {
				values = values[:1]
			}
			want := new(big.Rat).Set(fold(values...))
			got := fold(values...)
			got.Add(got, big.NewRat(1000, 3))
			require.Zero(t, a.Cmp(big.NewRat(3, 8)), "%s: operand a changed by result", name)
			require.Zero(t, b.Cmp(big.NewRat(-5, 7)), "%s: operand b changed by result", name)
			got = fold(values...)
			a.Num().SetInt64(99)
			a.Denom().SetInt64(1)
			requireSameTerms(t, want, got, name+": result changed by operand")
		}
	}
}

func TestFixedToRatMatchesSetFrac(t *testing.T) {
	rng := rand.New(rand.NewPCG(107, 109))
	check := func(v *big.Int) {
		t.Helper()
		v0 := new(big.Int).Set(v)
		want := new(big.Rat).SetFrac(v, proofbound.TrigFixedOne)
		got := proofbound.FixedToRat(v)
		requireSameResult(t, want, got, "FixedToRat "+v.String())
		require.Zero(t, v.Cmp(v0), "FixedToRat modified its argument")
		v.Add(v, big.NewInt(1))
		requireSameTerms(t, new(big.Rat).SetFrac(v0, proofbound.TrigFixedOne), got, "FixedToRat aliases its argument")
	}
	check(new(big.Int))
	check(big.NewInt(1))
	check(big.NewInt(-1))
	check(new(big.Int).Set(proofbound.TrigFixedOne))
	check(new(big.Int).Neg(proofbound.TrigFixedOne))
	check(new(big.Int).Lsh(proofbound.TrigFixedOne, 3))
	check(new(big.Int).Rsh(proofbound.TrigFixedOne, 1))
	for range 3000 {
		v := new(big.Int)
		for range rng.IntN(5) + 1 {
			v.Lsh(v, 64).Add(v, new(big.Int).SetUint64(rng.Uint64()))
		}
		v.Lsh(v, uint(rng.IntN(210)))
		if rng.IntN(2) == 0 {
			v.Neg(v)
		}
		check(v)
	}
}

// TestRatAddMulIntegerDenominatorIsStored reports whether integer results,
// zero included, keep a stored denominator as the legacy folds do: Denom
// returns the same pointer on every call.
func TestRatAddMulIntegerDenominatorIsStored(t *testing.T) {
	stored := denomStored
	cases := [][]*big.Rat{
		{}, {new(big.Rat)}, {new(big.Rat), new(big.Rat)}, {big.NewRat(2, 1), big.NewRat(-2, 1)},
		{proofarith.FloatRat(0.5), proofarith.FloatRat(-0.5)}, {big.NewRat(3, 1)}, {big.NewRat(1, 3), big.NewRat(2, 3)},
		{big.NewRat(1, 3), new(big.Rat)},
	}
	for i, values := range cases {
		require.Equal(t, stored(legacyRatAdd(values...)), stored(proofbound.RatAdd(values...)), "RatAdd case %d", i)
		require.Equal(t, stored(legacyRatMul(values...)), stored(proofbound.RatMul(values...)), "RatMul case %d", i)
	}
}

// TestSetInt64MatchesNewRatInteger records the property the SetInt64 sites
// rely on: new(big.Rat).SetInt64(k) equals big.NewRat(k, 1) in value, terms
// and stored denominator, for zero, negatives and the int64 extremes.
func TestSetInt64MatchesNewRatInteger(t *testing.T) {
	for _, k := range []int64{0, 1, -1, 2, 8, -8, 1 << 40, math.MaxInt64, math.MinInt64} {
		requireSameResult(t, big.NewRat(k, 1), new(big.Rat).SetInt64(k), "SetInt64")
	}
}
