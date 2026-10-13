package proof

import "math/big"

// RatFrac sets z = n/d in lowest terms, as z.SetFrac(n, d) does, and returns
// z. A positive power-of-two d cancels against n by a shift of n's trailing
// zero bits instead of a GCD: gcd(n, 2^k) = 2^min(tz(n), k), so the shifted
// pair is big.Rat's lowest terms. Any other d takes z.SetFrac. z must not
// alias n or d; n and d are not modified.
func RatFrac(z *big.Rat, n, d *big.Int) *big.Rat {
	if d.Sign() > 0 {
		k := d.TrailingZeroBits()
		if uint(d.BitLen()) == k+1 {
			z.Num().Set(n)
			return setPowerOfTwoFrac(z, k)
		}
	}
	return z.SetFrac(n, d)
}
