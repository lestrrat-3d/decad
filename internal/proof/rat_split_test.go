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
		switch rng.IntN(9) {
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
