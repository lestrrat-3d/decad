package proof_test

import (
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
