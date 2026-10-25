package proof

import (
	"encoding/binary"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"
)

// legacyDyadic is the big.Int-only Dyadic this file's inline representation
// replaced, copied verbatim apart from its names. It is the oracle of
// TestDyadicMatchesLegacyForm: the inline form must reproduce every mantissa,
// exponent, float conversion, square-root seed and key it produced.
type legacyDyadic struct {
	mant *big.Int
	exp  int
}

func (d legacyDyadic) sign() int {
	if d.mant == nil {
		return 0
	}
	return d.mant.Sign()
}

func (d legacyDyadic) isZero() bool { return d.sign() == 0 }

func (d legacyDyadic) norm() legacyDyadic {
	if d.mant == nil || d.mant.Sign() == 0 {
		return legacyDyadic{}
	}
	if shift := d.mant.TrailingZeroBits(); shift > 0 {
		d.mant.Rsh(d.mant, shift)
		d.exp += int(shift)
	}
	return d
}

func legacyOf(f float64) legacyDyadic {
	if f == 0 {
		return legacyDyadic{}
	}
	frac, exp := math.Frexp(f)
	return legacyDyadic{mant: new(big.Int).SetInt64(int64(frac * (1 << 53))), exp: exp - 53}.norm()
}

func legacyAdd(a, b legacyDyadic) legacyDyadic {
	switch {
	case a.isZero() && b.isZero():
		return legacyDyadic{}
	case a.isZero():
		return legacyDyadic{mant: new(big.Int).Set(b.mant), exp: b.exp}
	case b.isZero():
		return legacyDyadic{mant: new(big.Int).Set(a.mant), exp: a.exp}
	case a.exp == b.exp:
		return legacyDyadic{mant: new(big.Int).Add(a.mant, b.mant), exp: a.exp}.norm()
	case a.exp > b.exp:
		out := new(big.Int).Lsh(a.mant, uint(a.exp-b.exp))
		return legacyDyadic{mant: out.Add(out, b.mant), exp: b.exp}.norm()
	default:
		out := new(big.Int).Lsh(b.mant, uint(b.exp-a.exp))
		return legacyDyadic{mant: out.Add(a.mant, out), exp: a.exp}.norm()
	}
}

func legacySub(a, b legacyDyadic) legacyDyadic {
	switch {
	case a.isZero() && b.isZero():
		return legacyDyadic{}
	case a.isZero():
		return legacyDyadic{mant: new(big.Int).Neg(b.mant), exp: b.exp}
	case b.isZero():
		return legacyDyadic{mant: new(big.Int).Set(a.mant), exp: a.exp}
	case a.exp == b.exp:
		return legacyDyadic{mant: new(big.Int).Sub(a.mant, b.mant), exp: a.exp}.norm()
	case a.exp > b.exp:
		out := new(big.Int).Lsh(a.mant, uint(a.exp-b.exp))
		return legacyDyadic{mant: out.Sub(out, b.mant), exp: b.exp}.norm()
	default:
		out := new(big.Int).Lsh(b.mant, uint(b.exp-a.exp))
		return legacyDyadic{mant: out.Sub(a.mant, out), exp: a.exp}.norm()
	}
}

func legacyMul(a, b legacyDyadic) legacyDyadic {
	if a.isZero() || b.isZero() {
		return legacyDyadic{}
	}
	return legacyDyadic{mant: new(big.Int).Mul(a.mant, b.mant), exp: a.exp + b.exp}
}

func legacyCmp(a, b legacyDyadic) int {
	switch {
	case a.isZero():
		return -b.sign()
	case b.isZero():
		return a.sign()
	case a.exp == b.exp:
		return a.mant.Cmp(b.mant)
	}
	sign := a.mant.Sign()
	if sign != b.mant.Sign() {
		return sign
	}
	if top, other := a.mant.BitLen()+a.exp, b.mant.BitLen()+b.exp; top != other {
		if top > other {
			return sign
		}
		return -sign
	}
	switch {
	case a.exp > b.exp:
		return new(big.Int).Lsh(a.mant, uint(a.exp-b.exp)).Cmp(b.mant)
	default:
		return a.mant.Cmp(new(big.Int).Lsh(b.mant, uint(b.exp-a.exp)))
	}
}

func legacyNeg(d legacyDyadic) legacyDyadic {
	if d.mant == nil {
		return legacyDyadic{}
	}
	return legacyDyadic{mant: new(big.Int).Neg(d.mant), exp: d.exp}
}

func legacyAbs(d legacyDyadic) legacyDyadic {
	if d.mant == nil {
		return legacyDyadic{}
	}
	return legacyDyadic{mant: new(big.Int).Abs(d.mant), exp: d.exp}
}

func legacyShift(d legacyDyadic, n int) legacyDyadic {
	if d.mant == nil || d.mant.Sign() == 0 {
		return legacyDyadic{}
	}
	return legacyDyadic{mant: new(big.Int).Set(d.mant), exp: d.exp + n}
}

func (d legacyDyadic) float64() (float64, bool) {
	if d.mant == nil || d.mant.Sign() == 0 {
		return 0, true
	}
	prec := max(uint(d.mant.BitLen()), 53)
	f := new(big.Float).SetPrec(prec).SetInt(d.mant)
	f.SetMantExp(f, d.exp)
	out, acc := f.Float64()
	return out, acc == big.Exact
}

func legacySqrtSeed(d legacyDyadic) float64 {
	mant := new(big.Float).SetPrec(64)
	exp := d.exp + new(big.Float).SetPrec(64).SetInt(d.mant).MantExp(mant)
	if exp%2 != 0 {
		exp--
		mant.SetMantExp(mant, 1)
	}
	m, _ := mant.Float64()
	return math.Ldexp(math.Sqrt(m), exp/2)
}

func (d legacyDyadic) appendKey(b []byte) []byte {
	if d.isZero() {
		return append(b, 0)
	}
	mant, exp := d.mant, d.exp
	if shift := mant.TrailingZeroBits(); shift > 0 {
		mant = new(big.Int).Rsh(mant, shift)
		exp += int(shift)
	}
	sign := byte(1)
	if mant.Sign() < 0 {
		sign = 2
	}
	b = append(b, sign)
	b = binary.AppendVarint(b, int64(exp))
	words := mant.Bytes()
	b = binary.AppendUvarint(b, uint64(len(words)))
	return append(b, words...)
}

// requireMatchesLegacy checks one inline result against the legacy result of
// the same operation, and checks the inline form's canonical invariant: a
// mantissa that fits the inline width is held inline and odd, a wider one in
// big.Int, and zero is the zero value.
func requireMatchesLegacy(t *testing.T, want legacyDyadic, got Dyadic, op string) {
	t.Helper()
	switch {
	case got.big != nil:
		require.Greater(t, got.big.BitLen(), dyBits, "%s: a big mantissa that fits inline", op)
	case got.lo == 0 && got.hi&^dySign == 0:
		require.Equal(t, Dyadic{}, got, "%s: zero is the zero value", op)
	default:
		require.Equal(t, uint64(1), got.lo&1, "%s: an even inline mantissa", op)
	}
	if want.isZero() {
		require.True(t, got.IsZero(), op)
	} else {
		require.Zero(t, want.mant.Cmp(got.Mant()), "%s: mantissa", op)
		require.Equal(t, want.exp, got.Exp(), "%s: exponent", op)
	}
	require.Equal(t, want.sign(), got.Sign(), "%s: sign", op)
	wantF, wantExact := want.float64()
	gotF, gotExact := got.Float64()
	require.Equal(t, math.Float64bits(wantF), math.Float64bits(gotF), "%s: Float64", op)
	require.Equal(t, wantExact, gotExact, "%s: Float64 exactness", op)
	require.Equal(t, want.appendKey(nil), got.AppendKey(nil), "%s: key", op)
	if want.sign() > 0 {
		require.Equal(t, math.Float64bits(legacySqrtSeed(want)), math.Float64bits(DySqrtSeed(got)), "%s: sqrt seed", op)
	}
}

// TestDyadicMatchesLegacyForm runs random chains of operations — the
// products, differences, sums and comparisons the contact proofs chain — in
// the inline form and in the legacy big.Int form side by side, from random
// float bit patterns, so mantissas grow from 53 bits across the inline width
// and back down through cancellation. After every operation the two forms
// must hold the same mantissa and exponent and give the same float
// conversion, key and square-root seed.
//
// Legs shown to fail: see TestDyadicInlineBoundariesMatchBigRat, whose broken
// paths each also fail here, and additionally keeping a big mantissa that a
// cancellation brought back within the inline width (fromBig's demotion).
func TestDyadicMatchesLegacyForm(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x452821E638D01377, 0xBE5466CF34E90C6C))
	randomFloat := func() float64 {
		switch rng.IntN(6) {
		case 0:
			return 0
		case 1:
			// Small integers and binary fractions, whose mantissas are short.
			return float64(rng.IntN(2001)-1000) / float64(int(1)<<rng.IntN(12))
		}
		for {
			bits := rng.Uint64()
			if rng.IntN(4) != 0 {
				// Keep most exponents near 1 so products stay in range.
				bits = bits&^(uint64(0x7ff)<<52) | uint64(1023+rng.IntN(41)-20)<<52
			}
			f := math.Float64frombits(bits)
			if !math.IsNaN(f) && !math.IsInf(f, 0) {
				return f
			}
		}
	}
	for range 3000 {
		pool := make([]Dyadic, 0, 32)
		legacy := make([]legacyDyadic, 0, 32)
		for range 6 {
			f := randomFloat()
			pool = append(pool, MustDyOf(f))
			legacy = append(legacy, legacyOf(f))
			requireMatchesLegacy(t, legacy[len(legacy)-1], pool[len(pool)-1], "lift")
		}
		for range 24 {
			i, j := rng.IntN(len(pool)), rng.IntN(len(pool))
			var got Dyadic
			var want legacyDyadic
			var op string
			switch rng.IntN(7) {
			case 0:
				got, want, op = DyAdd(pool[i], pool[j]), legacyAdd(legacy[i], legacy[j]), "add"
			case 1:
				got, want, op = DySubScalar(pool[i], pool[j]), legacySub(legacy[i], legacy[j]), "sub"
			case 2, 3:
				got, want, op = DyMul(pool[i], pool[j]), legacyMul(legacy[i], legacy[j]), "mul"
			case 4:
				got, want, op = DyNeg(pool[i]), legacyNeg(legacy[i]), "neg"
			case 5:
				got, want, op = DyAbs(pool[i]), legacyAbs(legacy[i]), "abs"
			default:
				n := rng.IntN(129) - 64
				got, want, op = DyShift(pool[i], n), legacyShift(legacy[i], n), "shift"
			}
			requireMatchesLegacy(t, want, got, op)
			k := rng.IntN(len(pool))
			require.Equal(t, legacyCmp(want, legacy[k]), DyCmp(got, pool[k]), "cmp after %s", op)
			require.Equal(t, legacyCmp(legacy[k], want), DyCmp(pool[k], got), "reverse cmp after %s", op)
			pool, legacy = append(pool, got), append(legacy, want)
		}
	}
}

// legacyDyOfFinite is DyOfFinite as it was written over math.Frexp.
func legacyDyOfFinite(f float64) Dyadic {
	frac, exp := math.Frexp(f)
	return dyInt64(int64(frac*(1<<53)), exp-53)
}

// TestDyOfFiniteMatchesFrexpForm checks the bit-cast DyOfFinite against the
// Frexp form field for field.
func TestDyOfFiniteMatchesFrexpForm(t *testing.T) {
	check := func(f float64) {
		require.Equal(t, legacyDyOfFinite(f), DyOfFinite(f), "%x", math.Float64bits(f))
	}
	for _, x := range []float64{
		0, math.Copysign(0, -1), math.MaxFloat64, -math.MaxFloat64, math.SmallestNonzeroFloat64,
		-math.SmallestNonzeroFloat64, 0x1p-1022, -0x1p-1022, 1, -1,
	} {
		check(x)
		check(math.Nextafter(x, 0))
	}
	for e := -1074; e <= 1023; e++ {
		p := math.Ldexp(1, e)
		check(p)
		check(-p)
		check(math.Nextafter(p, 0))
		check(math.Nextafter(p, math.Inf(1)))
	}
	r := rand.New(rand.NewPCG(5, 6))
	for range 100_000 {
		f := math.Float64frombits(r.Uint64())
		if math.IsNaN(f) || math.IsInf(f, 0) {
			continue
		}
		check(f)
	}
}

// bigFloatSqrtSeed is DySqrtSeed's big.Float path, kept as the reference for
// the machine-word path.
func bigFloatSqrtSeed(d Dyadic) float64 {
	var t big.Int
	var buf dyBuf
	mant := new(big.Float).SetPrec(64).SetInt(d.view(&t, &buf))
	exp := d.exp + mant.MantExp(mant)
	if exp%2 != 0 {
		exp--
		mant.SetMantExp(mant, 1)
	}
	m, _ := mant.Float64()
	return math.Ldexp(math.Sqrt(m), exp/2)
}

// randomInlineDyadic builds a positive inline Dyadic of 1-127 mantissa bits;
// a quarter of the wide ones sit on an exact tie at the 64-bit cut and a
// quarter carry all ones below it.
func randomInlineDyadic(r *rand.Rand, exp int) Dyadic {
	nb := 1 + r.IntN(127)
	var lo, hi uint64
	if nb <= 64 {
		lo = r.Uint64()>>(64-uint(nb)) | 1<<uint(nb-1)
		return fromMag(lo, 0, false, exp)
	}
	lo = r.Uint64()
	hi = r.Uint64()>>(128-uint(nb)) | 1<<uint(nb-65)
	switch r.IntN(4) {
	case 0:
		lo = lo&^(1<<uint(nb-64)-1) | 1<<uint(nb-65)
	case 1:
		lo |= 1<<uint(nb-64) - 1
	}
	return fromMag(lo, hi, false, exp)
}

// TestDySqrtSeedInlineMatchesBigFloat compares the machine-word seed with the
// big.Float one by exact bits, over exponents that run the power-of-two table
// and the Ldexp fallback.
func TestDySqrtSeedInlineMatchesBigFloat(t *testing.T) {
	r := rand.New(rand.NewPCG(3, 4))
	for range 50_000 {
		d := randomInlineDyadic(r, r.IntN(2201)-1100)
		got, want := DySqrtSeed(d), bigFloatSqrtSeed(d)
		require.Equal(t, math.Float64bits(want), math.Float64bits(got), "%+v", d)
	}
}

func TestDySqrtSeedBigMatchesBigFloat(t *testing.T) {
	r := rand.New(rand.NewPCG(0x495f31, 0x72b40a))
	check := func(d Dyadic) {
		t.Helper()
		before := new(big.Int).Set(d.big)
		got, want := DySqrtSeed(d), bigFloatSqrtSeed(d)
		require.Equal(t, math.Float64bits(want), math.Float64bits(got), "mantissa bits %d, exponent %d", d.bitLen(), d.exp)
		require.Zero(t, d.big.Cmp(before), "seed changed its immutable mantissa")
	}
	for range 6000 {
		width := 128 + r.IntN(3969)
		raw := make([]byte, (width+7)/8)
		for i := range raw {
			raw[i] = byte(r.Uint64())
		}
		mant := new(big.Int).SetBytes(raw)
		mant.Rsh(mant, uint(len(raw)*8-width))
		mant.SetBit(mant, width-1, 1)
		mant.SetBit(mant, 0, 1)
		for _, exp := range []int{r.IntN(8193) - 4096, -width + r.IntN(8) - 4} {
			check(fromBig(new(big.Int).Set(mant), exp))
		}
	}
	// Keep wide odd mantissas on either side of a binary64 midpoint, where
	// the prefix interval may require the original full conversion.
	for _, width := range []uint{128, 129, 191, 256, 513, 2048} {
		top := new(big.Int).Add(new(big.Int).Lsh(big.NewInt(1), 63), big.NewInt(1024))
		top.Lsh(top, width-64)
		for _, low := range []int64{-9, -7, -1, 1, 7, 9} {
			for _, exp := range []int{-4096, -2148, -1, 0, 1, 1024, 4096} {
				check(fromBig(new(big.Int).Add(top, big.NewInt(low)), exp))
			}
		}
	}
}

func TestDySqrtSeedBigDoesNotAllocate(t *testing.T) {
	d := fromBig(new(big.Int).Add(new(big.Int).Lsh(big.NewInt(1), 512), big.NewInt(1)), -512)
	require.Equal(t, math.Float64bits(bigFloatSqrtSeed(d)), math.Float64bits(DySqrtSeed(d)))
	require.Zero(t, testing.AllocsPerRun(100, func() { _ = DySqrtSeed(d) }))
}

// TestDyadicFloat64MatchesLdexp checks the table-scaled inline conversion
// against Ldexp over inline Dyadics whose value is a normal float.
func TestDyadicFloat64MatchesLdexp(t *testing.T) {
	r := rand.New(rand.NewPCG(7, 8))
	for range 50_000 {
		d := randomInlineDyadic(r, r.IntN(2300)-1150)
		n := d.bitLen()
		if n-1+d.exp < -1022 || n+d.exp > 1024 {
			continue
		}
		top, shift := d.lo, 0
		if hi := d.hi &^ dySign; hi != 0 {
			shift = n - 64
			top = hi<<(64-shift) | d.lo>>shift
			if d.lo<<(64-shift) != 0 {
				top |= 1
			}
		}
		want := math.Ldexp(float64(top), shift+d.exp)
		got, exact := d.Float64()
		require.Equal(t, math.Float64bits(want), math.Float64bits(got), "%+v", d)
		require.Equal(t, n <= 53, exact)
	}
}
