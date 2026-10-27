package meshbool

import (
	"math"
	"math/big"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// legacyXP2FromFloats is the body VertexProjection ran before
// newXP2FromFloats: lift each float with SetFloat64, then read the cache back
// out of the rational. It is the oracle for the equivalence tests.
func legacyXP2FromFloats(fu, fv float64) Xp2 {
	return NewXP2(polynomial.MustRatOf(fu), polynomial.MustRatOf(fv))
}

func liftAdversarialFloats() []float64 {
	base := []float64{
		0, math.SmallestNonzeroFloat64, 2 * math.SmallestNonzeroFloat64,
		math.Float64frombits(1<<52 - 1), math.Float64frombits(1 << 52),
		math.MaxFloat64, math.Nextafter(math.MaxFloat64, 0),
		0x1p-300, 0x1p250, 0x1p240, 0x1p-1022, 0x1p-1023, 0x1p1023, 0x1p-52,
		1, 3, 0.1, 1.5, 123456789.123456789, 0x1.fffffffffffffp52,
		math.Float64frombits(0x000fffffffffffff), math.Float64frombits(0x0010000000000001),
	}
	out := make([]float64, 0, 6*len(base))
	for _, f := range base {
		for _, g := range []float64{f, math.Nextafter(f, 0), math.Nextafter(f, math.Inf(1)),
			math.Nextafter(f, math.Inf(-1))} {
			if math.IsInf(g, 0) || math.IsNaN(g) {
				continue
			}
			out = append(out, g, -g)
		}
	}
	return out
}

func requireSameXP2(t *testing.T, want, got Xp2, msg string, args ...any) {
	t.Helper()
	require.Zero(t, want.U.Cmp(got.U), msg, args)
	require.Zero(t, want.V.Cmp(got.V), msg, args)
	require.Zero(t, want.U.Num().Cmp(got.U.Num()), msg, args)
	require.Zero(t, want.U.Denom().Cmp(got.U.Denom()), msg, args)
	require.Zero(t, want.V.Num().Cmp(got.V.Num()), msg, args)
	require.Zero(t, want.V.Denom().Cmp(got.V.Denom()), msg, args)
	require.Equal(t, math.Float64bits(want.Fu), math.Float64bits(got.Fu), msg, args)
	require.Equal(t, math.Float64bits(want.Fv), math.Float64bits(got.Fv), msg, args)
	require.Equal(t, want.FloatFinite, got.FloatFinite, msg, args)
	require.Equal(t, want.floatFromExact, got.floatFromExact, msg, args)
	require.Nil(t, got.Hu)
	require.Nil(t, got.Hv)
	require.Nil(t, got.Hw)
	require.True(t, proof.RatTextEqual(want.U, got.U) && proof.RatTextEqual(want.V, got.V), msg, args)
}

func TestXP2FromFloatsMatchesLegacy(t *testing.T) {
	adversarial := liftAdversarialFloats()
	for i, f := range adversarial {
		// Every value against a fixed partner of each kind, in both slots.
		for _, g := range []float64{0, math.Copysign(0, -1), 1, -0.1, 0x1p-300, math.MaxFloat64} {
			requireSameXP2(t, legacyXP2FromFloats(f, g), newXP2FromFloats(f, g), "case %d (%g, %g)", i, f, g)
			requireSameXP2(t, legacyXP2FromFloats(g, f), newXP2FromFloats(g, f), "case %d (%g, %g)", i, g, f)
		}
	}
	rng := rand.New(rand.NewSource(1))
	finite := func() float64 {
		for {
			f := math.Float64frombits(rng.Uint64())
			if !math.IsNaN(f) && !math.IsInf(f, 0) {
				return f
			}
		}
	}
	for i := range 100000 {
		f, g := finite(), finite()
		requireSameXP2(t, legacyXP2FromFloats(f, g), newXP2FromFloats(f, g), "random %d (%g, %g)", i, f, g)
	}
	// Plausible coordinates: odd and even mantissas at modest exponents.
	for i := range 50000 {
		f := float64(rng.Int63n(1<<53)-1<<52) * math.Ldexp(1, rng.Intn(120)-70)
		g := float64(rng.Int63n(1<<20)-1<<19) * math.Ldexp(1, rng.Intn(40)-20)
		requireSameXP2(t, legacyXP2FromFloats(f, g), newXP2FromFloats(f, g), "modest %d (%g, %g)", i, f, g)
	}
}

// TestXP2FromFloatsCacheIsTheInputFloat covers the guard: a coordinate the
// guard rejects (subnormal, below 2^-300, above 2^250) still caches the input
// float, and FloatFinite is false, exactly as NewXP2 reports it.
func TestXP2FromFloatsCacheIsTheInputFloat(t *testing.T) {
	type guardCase struct {
		f    float64
		safe bool
	}
	cases := []guardCase{
		{0, true}, {math.Copysign(0, -1), true},
		{math.SmallestNonzeroFloat64, false}, {-math.SmallestNonzeroFloat64, false},
		{math.Nextafter(0x1p-300, 0), false}, {0x1p-300, true}, {math.Nextafter(0x1p-300, 1), true},
		{0x1p250, true}, {math.Nextafter(0x1p250, math.Inf(1)), false}, {-0x1p250, true},
		{math.MaxFloat64, false}, {-math.MaxFloat64, false}, {1, true},
	}
	for _, c := range cases {
		got := newXP2FromFloats(c.f, 1)
		want := legacyXP2FromFloats(c.f, 1)
		require.Equal(t, c.safe, got.FloatFinite, "%g", c.f)
		require.Equal(t, want.FloatFinite, got.FloatFinite, "%g", c.f)
		if c.f == 0 {
			require.Equal(t, uint64(0), math.Float64bits(got.Fu), "zero caches +0")
			continue
		}
		require.Equal(t, math.Float64bits(c.f), math.Float64bits(got.Fu), "%g", c.f)
		back, exact := got.U.Float64()
		require.True(t, exact)
		require.Equal(t, math.Float64bits(c.f), math.Float64bits(back), "%g", c.f)
	}
}

// TestXP2FromFloatsPanicsLikeMustRatOf requires the same panic as the legacy
// body for NaN and the infinities, in either slot.
func TestXP2FromFloatsPanicsLikeMustRatOf(t *testing.T) {
	for _, bad := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
		for _, args := range [][2]float64{{bad, 1}, {1, bad}} {
			var want, got any
			func() {
				defer func() { want = recover() }()
				legacyXP2FromFloats(args[0], args[1])
			}()
			func() {
				defer func() { got = recover() }()
				newXP2FromFloats(args[0], args[1])
			}()
			require.NotNil(t, want)
			require.Equal(t, want, got)
		}
	}
}

// TestFloatRatLiftEqualsSetFloat64 checks the argument newXP2FromFloats rests
// on: for every float tried, the lift is SetFloat64's value in lowest terms.
func TestFloatRatLiftEqualsSetFloat64(t *testing.T) {
	for _, f := range liftAdversarialFloats() {
		got := liftProjectionFloat(f)
		want := new(big.Rat).SetFloat64(f)
		require.Zero(t, want.Cmp(got), "%g", f)
		require.Zero(t, want.Num().Cmp(got.Num()), "%g", f)
		require.Zero(t, want.Denom().Cmp(got.Denom()), "%g", f)
	}
}

// TestXP2FromFloatsZeroKeepsStoredDenominator requires a zero projection to
// hold its denominator as SetFloat64 does: Denom returns the same pointer on
// repeated calls. A zero Rat from proof.FloatRat returns a fresh detached 1
// each call, so lifting zero through it would fail this test.
func TestXP2FromFloatsZeroKeepsStoredDenominator(t *testing.T) {
	for _, z := range []float64{0, math.Copysign(0, -1)} {
		for name, xp := range map[string]Xp2{
			"legacy": legacyXP2FromFloats(z, z),
			"new":    newXP2FromFloats(z, z),
		} {
			for _, r := range []*big.Rat{xp.U, xp.V} {
				require.Same(t, r.Denom(), r.Denom(), "%s zero %v", name, z)
				require.Zero(t, r.Num().Sign(), "%s zero %v", name, z)
				require.Zero(t, r.Denom().Cmp(big.NewInt(1)), "%s zero %v", name, z)
			}
		}
	}
}
