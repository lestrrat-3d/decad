package proof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// TestSplitGCDRatOpsMatchBigRat feeds AddRat, SubRat, MulRat and QuoRat and
// big.Rat's own operations the same operands, many of which share large
// factors, and requires the same numerator and denominator (so a result not
// in lowest terms fails), unchanged operands, and the same result when z is a,
// b or both.
func TestSplitGCDRatOpsMatchBigRat(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	pow := func(base int64, n int) *big.Int {
		return new(big.Int).Exp(big.NewInt(base), big.NewInt(int64(n)), nil)
	}
	smooth := func() *big.Int {
		d := pow(2, rng.IntN(65))
		d.Mul(d, pow(3, rng.IntN(81)))
		d.Mul(d, pow(5, rng.IntN(81)))
		d.Mul(d, pow(7, rng.IntN(11)))
		u := randBits(rng, 1+rng.IntN(40))
		return d.Mul(d, u.SetBit(u, 0, 1))
	}
	num := func() *big.Int {
		n := randBits(rng, 1+rng.IntN(600))
		if rng.IntN(2) == 0 {
			n.Neg(n)
		}
		return n
	}
	piUpper := new(big.Rat).SetFrac(
		new(big.Int).Lsh(big.NewInt(0x3243F6A8885A3), 4), new(big.Int).Lsh(big.NewInt(1), 52))
	gap := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), piUpper), big.NewRat(4096, 1))
	operand := func() *big.Rat {
		switch rng.IntN(10) {
		case 0:
			return new(big.Rat)
		case 1:
			return new(big.Rat).SetInt64(rng.Int64N(2001) - 1000)
		case 2:
			x := new(big.Rat).SetFloat64(rng.NormFloat64())
			if rng.IntN(2) == 0 {
				return x.Add(x, gap)
			}
			return x.Sub(x, gap)
		default:
			return new(big.Rat).SetFrac(num(), smooth())
		}
	}
	type op struct {
		name string
		got  func(z, a, b *big.Rat) *big.Rat
		want func(z, a, b *big.Rat) *big.Rat
	}
	ops := []op{
		{"AddRat", proof.AddRat, (*big.Rat).Add},
		{"SubRat", proof.SubRat, (*big.Rat).Sub},
		{"MulRat", proof.MulRat, (*big.Rat).Mul},
		{"QuoRat", proof.QuoRat, (*big.Rat).Quo},
	}
	same := func(x, y *big.Rat) bool {
		return x.Num().Cmp(y.Num()) == 0 && x.Denom().Cmp(y.Denom()) == 0
	}
	for range 20000 {
		a, b := operand(), operand()
		for _, o := range ops {
			if o.name == "QuoRat" && b.Sign() == 0 {
				continue
			}
			want := o.want(new(big.Rat), a, b)
			a0, b0 := new(big.Rat).Set(a), new(big.Rat).Set(b)

			got := o.got(new(big.Rat), a, b)
			require.True(t, same(got, want), "%s(%v, %v) = %v, want %v", o.name, a, b, got, want)
			require.True(t, same(a, a0) && same(b, b0), "%s changed an operand", o.name)

			za := new(big.Rat).Set(a)
			require.True(t, same(o.got(za, za, b), want), "%s z=a (%v, %v)", o.name, a, b)
			zb := new(big.Rat).Set(b)
			require.True(t, same(o.got(zb, a, zb), want), "%s z=b (%v, %v)", o.name, a, b)
		}
		// z aliasing both operands: a∘a. MulRat keeps big.Rat's squaring path.
		for _, o := range ops {
			if o.name == "QuoRat" && a.Sign() == 0 {
				continue
			}
			want := o.want(new(big.Rat), a, a)
			za := new(big.Rat).Set(a)
			require.True(t, same(o.got(za, za, za), want), "%s z=a=b (%v)", o.name, a)
		}
	}

	// Operands that are not in lowest terms (see AddRat): the value must equal
	// big.Rat's by Cmp and the operands stay as they were. The representation
	// of the result is NOT compared: AddRat(3/9, 0) returns 3/9 where big.Rat
	// returns 1/3, and no production path passes such an operand.
	for range 20000 {
		a, b := nonReducedRat(rng), operand()
		require.False(t, isReduced(a), "generator made the reduced %s", rawString(a))
		switch rng.IntN(3) {
		case 0:
			a, b = b, a
		case 1:
			b = nonReducedRat(rng)
		}
		for _, o := range ops {
			if o.name == "QuoRat" && b.Sign() == 0 {
				continue
			}
			ar, br := rawCopy(a), rawCopy(b)
			want := o.want(new(big.Rat), reduceCopy(a), reduceCopy(b))
			got := o.got(new(big.Rat), a, b)
			require.Zero(t, got.Cmp(want), "%s(%s, %s) = %v, want %v", o.name, rawString(ar), rawString(br), got, want)
			require.Equal(t, rawString(ar), rawString(a), "%s changed operand a", o.name)
			require.Equal(t, rawString(br), rawString(b), "%s changed operand b", o.name)

			za := rawCopy(a)
			require.Zero(t, o.got(za, za, b).Cmp(want), "%s z=a (%s, %s)", o.name, rawString(ar), rawString(br))
			zb := rawCopy(b)
			require.Zero(t, o.got(zb, a, zb).Cmp(want), "%s z=b (%s, %s)", o.name, rawString(ar), rawString(br))
		}
	}
}

// TestNonReducedWitness is the 3/9 operand the review used: the value is
// exact, the representation is not lowest terms. Keys built from value text
// would see 3/18 where big.Rat gives 1/6; no production path builds such an
// operand (see the precondition on AddRat).
func TestNonReducedWitness(t *testing.T) {
	t.Parallel()
	third := big.NewRat(1, 9)
	third.Num().SetInt64(3) // 3/9, not reduced
	require.Equal(t, "3/9", rawString(third))
	zero := new(big.Rat)
	half := proof.MulRat(new(big.Rat), big.NewRat(1, 2), proof.AddRat(new(big.Rat), third, zero))
	require.Zero(t, half.Cmp(big.NewRat(1, 6)))
	require.Zero(t, proof.CmpRat(half, big.NewRat(1, 6)))
	require.Equal(t, 1, proof.CmpRat(third, big.NewRat(1, 4)))
	require.Equal(t, -1, proof.CmpRat(third, big.NewRat(1, 2)))
}

// rawString is a Rat's stored numerator and denominator, with no reduction.
func rawString(r *big.Rat) string { return r.Num().String() + "/" + r.Denom().String() }

// rawCopy copies r's stored numerator and denominator without reducing them.
func rawCopy(r *big.Rat) *big.Rat {
	c := new(big.Rat).SetInt(r.Num())
	c.Denom().Set(r.Denom())
	return c
}

// reduceCopy is r's value in lowest terms.
func reduceCopy(r *big.Rat) *big.Rat { return new(big.Rat).SetFrac(r.Num(), r.Denom()) }

// nonReducedRat returns a value whose stored numerator and denominator share
// a factor, made by scaling both in place by 2, 3, 2^k (small and large k) or a
// random odd number, or by giving zero a denominator other than 1, or the
// witness 3/9.
func nonReducedRat(rng *rand.Rand) *big.Rat {
	var r *big.Rat
	switch rng.IntN(6) {
	case 0:
		r = big.NewRat(1, 9)
		r.Num().SetInt64(3)
		return r
	case 1:
		r = new(big.Rat).SetFrac(big.NewInt(0), big.NewInt(1))
		r.Denom().SetInt64(1 + rng.Int64N(100)) // 0/d
		if r.Denom().Cmp(big.NewInt(1)) == 0 {
			r.Denom().SetInt64(2)
		}
		return r
	}
	n := randBits(rng, 1+rng.IntN(300))
	if rng.IntN(2) == 0 {
		n.Neg(n)
	}
	d := randBits(rng, 2+rng.IntN(100))
	d.SetBit(d, 0, 1)
	d.Add(d, big.NewInt(2))
	r = new(big.Rat).SetFrac(n, d)
	if r.IsInt() {
		r = big.NewRat(5, 7)
	}
	var k *big.Int
	switch rng.IntN(5) {
	case 0:
		k = big.NewInt(2)
	case 1:
		k = big.NewInt(3)
	case 2:
		k = new(big.Int).Lsh(big.NewInt(1), uint(1+rng.IntN(8)))
	case 3:
		k = new(big.Int).Lsh(big.NewInt(1), uint(64+rng.IntN(400)))
	default:
		k = randBits(rng, 2+rng.IntN(120))
		k.SetBit(k, 0, 1)
		k.Add(k, big.NewInt(2))
	}
	r.Num().Mul(r.Num(), k)
	r.Denom().Mul(r.Denom(), k)
	return r
}

// isReduced reports whether r's stored numerator and denominator are coprime
// (zero is reduced only over 1).
func isReduced(r *big.Rat) bool {
	n, d := r.Num(), r.Denom()
	if n.Sign() == 0 {
		return d.Cmp(big.NewInt(1)) == 0
	}
	return new(big.Int).GCD(nil, nil, new(big.Int).Abs(n), d).Cmp(big.NewInt(1)) == 0
}

func TestSplitGCDQuoRatZeroDivisorPanics(t *testing.T) {
	t.Parallel()
	require.Panics(t, func() { new(big.Rat).Quo(big.NewRat(1, 3), new(big.Rat)) })
	require.Panics(t, func() { proof.QuoRat(new(big.Rat), big.NewRat(1, 3), new(big.Rat)) })
}

// randBits returns a non-negative integer of at most bits bits.
func randBits(rng *rand.Rand, bits int) *big.Int {
	n := new(big.Int)
	for done := 0; done < bits; done += 64 {
		n.Lsh(n, 64)
		n.Or(n, new(big.Int).SetUint64(rng.Uint64()))
	}
	return n.Rsh(n, uint((bits+63)/64*64-bits))
}

// TestCmpRatMatchesRatCmp compares CmpRat with big.Rat.Cmp over 50,000 random
// pairs that exercise each branch: two dyadics with exponents 0 to 1100, a
// dyadic against a fraction, two fractions over one denominator, integers,
// zero, opposite signs, equal values in different spellings, and Rats built
// by FloatRat or by Num alone whose integer denominator is unstored. CmpRat
// must leave every input untouched: the same numerator pointer, value and
// sign, the same denominator value, and an unstored denominator still unstored.
func TestCmpRatMatchesRatCmp(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(13, 17))
	signed := func(bits int) *big.Int {
		n := randBits(rng, bits)
		if rng.IntN(2) == 0 {
			n.Neg(n)
		}
		return n
	}
	pow2 := func(e int) *big.Int { return new(big.Int).Lsh(big.NewInt(1), uint(e)) }
	// odd denominator times an optional power of two, never 1.
	oddDenom := func() *big.Int {
		d := randBits(rng, 2+rng.IntN(120))
		d.SetBit(d, 0, 1)
		if d.IsInt64() && d.Int64() == 1 {
			d.SetInt64(3)
		}
		if rng.IntN(3) == 0 {
			d.Lsh(d, uint(rng.IntN(70)))
		}
		return d
	}
	dyadic := func() *big.Rat {
		e := rng.IntN(1101)
		if rng.IntN(4) == 0 {
			e = rng.IntN(6)
		}
		return new(big.Rat).SetFrac(signed(1+rng.IntN(200)), pow2(e))
	}
	fraction := func() *big.Rat { return new(big.Rat).SetFrac(signed(1+rng.IntN(200)), oddDenom()) }
	unstoredInt := func(n *big.Int) *big.Rat {
		r := new(big.Rat)
		r.Num().Set(n)
		return r
	}
	integer := func() *big.Rat {
		n := signed(rng.IntN(130))
		if rng.IntN(2) == 0 {
			return unstoredInt(n)
		}
		return new(big.Rat).SetInt(n)
	}
	floatRat := func() *big.Rat {
		switch rng.IntN(4) {
		case 0:
			return proof.FloatRat(float64(rng.Int64N(1<<20) - 1<<19))
		case 1:
			return proof.FloatRat(rng.NormFloat64() * 1e-3)
		case 2:
			return proof.FloatRat(math.Ldexp(rng.NormFloat64(), rng.IntN(2001)-1000))
		}
		return proof.FloatRat(0)
	}
	anyRat := func() *big.Rat {
		switch rng.IntN(5) {
		case 0:
			return dyadic()
		case 1:
			return fraction()
		case 2:
			return integer()
		case 3:
			return floatRat()
		}
		return new(big.Rat)
	}
	pair := func() (*big.Rat, *big.Rat) {
		switch rng.IntN(11) {
		case 9: // a Rat not in lowest terms against any Rat
			return nonReducedRat(rng), anyRat()
		case 10: // the same value in lowest terms and not
			a := fraction()
			c := rawCopy(a)
			c.Num().Lsh(c.Num(), 3)
			c.Denom().Lsh(c.Denom(), 3)
			return a, c
		case 0: // dyadic against dyadic
			return dyadic(), dyadic()
		case 1: // dyadic against fraction
			return dyadic(), fraction()
		case 2: // equal denominators
			d := oddDenom()
			if rng.IntN(2) == 0 {
				d = pow2(1 + rng.IntN(300))
			}
			return new(big.Rat).SetFrac(signed(1+rng.IntN(150)), d), new(big.Rat).SetFrac(signed(1+rng.IntN(150)), d)
		case 3: // integers and zero
			return integer(), integer()
		case 4: // opposite signs, equal magnitude
			a := anyRat()
			return a, new(big.Rat).Neg(a)
		case 5: // equal values in different spellings
			f := rng.NormFloat64() * math.Ldexp(1, rng.IntN(200)-100)
			a := new(big.Rat).SetFloat64(f)
			return a, proof.FloatRat(f)
		case 6: // a value against itself, shared or copied
			a := anyRat()
			if rng.IntN(2) == 0 {
				return a, a
			}
			return a, new(big.Rat).Set(a)
		case 7: // one integer against a fraction or dyadic
			if rng.IntN(2) == 0 {
				return integer(), fraction()
			}
			return dyadic(), integer()
		}
		return anyRat(), anyRat()
	}
	type snapshot struct {
		numPtr *big.Int
		num    string
		denom  string
		sign   int
		stored bool
	}
	snap := func(r *big.Rat) snapshot {
		// An unstored denominator makes Denom return a fresh Int on each call.
		d1, d2 := r.Denom(), r.Denom()
		return snapshot{r.Num(), r.Num().String(), d1.String(), r.Sign(), d1 == d2}
	}
	for range 50000 {
		a, b := pair()
		sa, sb := snap(a), snap(b)
		want := a.Cmp(b)
		require.Equal(t, want, proof.CmpRat(a, b), "CmpRat(%v, %v)", a, b)
		require.Equal(t, -want, proof.CmpRat(b, a), "CmpRat(%v, %v)", b, a)
		require.Equal(t, 0, proof.CmpRat(a, a), "CmpRat(%v, %v)", a, a)
		require.Same(t, sa.numPtr, a.Num())
		require.Same(t, sb.numPtr, b.Num())
		require.Equal(t, sa, snap(a), "CmpRat changed %v", a)
		require.Equal(t, sb, snap(b), "CmpRat changed %v", b)
	}
}

// TestHelperOutputsStayReduced chains AddRat, SubRat, MulRat and QuoRat over
// operands from FloatRat, Dyadic.Rat and RatFrac: each result feeds the next
// call, and every result must carry the same numerator and denominator
// big.Rat's own chain carries. A reduced operand therefore yields a reduced
// result, which is why the helpers need no reduction step of their own.
// 20,000 chains of 4 steps.
func TestHelperOutputsStayReduced(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(19, 23))
	family := func() *big.Rat {
		switch rng.IntN(4) {
		case 0:
			return proof.FloatRat(math.Ldexp(rng.NormFloat64(), rng.IntN(121)-60))
		case 1:
			return proof.DyOfFinite(math.Ldexp(rng.NormFloat64(), rng.IntN(121)-60)).Rat()
		case 2:
			n := randBits(rng, 1+rng.IntN(120))
			if rng.IntN(2) == 0 {
				n.Neg(n)
			}
			d := randBits(rng, 1+rng.IntN(80))
			d.Add(d, big.NewInt(1))
			return proof.RatFrac(new(big.Rat), n, d)
		}
		n := randBits(rng, 1+rng.IntN(120))
		return proof.RatFrac(new(big.Rat), n, new(big.Int).Lsh(big.NewInt(1), uint(rng.IntN(100))))
	}
	type op struct {
		got  func(z, a, b *big.Rat) *big.Rat
		want func(z, a, b *big.Rat) *big.Rat
	}
	ops := []op{
		{proof.AddRat, (*big.Rat).Add},
		{proof.SubRat, (*big.Rat).Sub},
		{proof.MulRat, (*big.Rat).Mul},
		{proof.QuoRat, (*big.Rat).Quo},
	}
	for range 20000 {
		x := family()
		xw := rawCopy(x)
		for step := range 4 {
			y := family()
			o := ops[rng.IntN(len(ops))]
			if y.Sign() == 0 {
				o = ops[rng.IntN(3)]
			}
			yw := rawCopy(y)
			x = o.got(new(big.Rat), x, y)
			xw = o.want(new(big.Rat), xw, yw)
			require.Equal(t, rawString(xw), rawString(x), "step %d", step)
			require.True(t, isReduced(x), "step %d produced %s", step, rawString(x))
		}
	}
}
