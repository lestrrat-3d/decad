package proof

import "math/big"

// DyAcc is an exact running sum of Dyadic values kept in one mutable mantissa
// at the smallest exponent seen so far. Its value is always the exact sum, so
// Dyadic returns what a DyAdd chain over the same terms returns. The zero
// value is an empty (zero) sum, ready to use.
type DyAcc struct {
	mant big.Int // the sum is mant × 2^exp; mant is zero exactly when the sum is
	exp  int
	t    big.Int // scratch
	x, y big.Int // owned input mantissas reused across additions and products
}

// Add adds x to the sum.
func (a *DyAcc) Add(x Dyadic) {
	if x.IsZero() {
		return
	}
	if x.big != nil {
		a.addMant(x.big, x.exp)
		return
	}
	x.MantInto(&a.t)
	a.addScratch(x.exp)
}

// AddProduct adds x·y to the sum.
func (a *DyAcc) AddProduct(x, y Dyadic) {
	if x.IsZero() || y.IsZero() {
		return
	}
	xm, ym := x.big, y.big
	if xm == nil {
		xm = x.MantInto(&a.x)
	}
	if ym == nil {
		ym = y.MantInto(&a.y)
	}
	a.t.Mul(xm, ym)
	a.addScratch(x.exp + y.exp)
}

// AddDot adds v·w to the sum, reusing the product scratch for each term
// instead of constructing a separate Dyadic for the dot product.
func (a *DyAcc) AddDot(v, w DyV3) {
	for axis := range 3 {
		x, y := v[axis], w[axis]
		a.AddProduct(x, y)
	}
}

// addMant adds m × 2^e, with m a read-only non-zero mantissa. It never keeps
// a reference to m, since a Dyadic's big mantissa is shared and immutable.
func (a *DyAcc) addMant(m *big.Int, e int) {
	if a.mant.Sign() == 0 {
		a.mant.Set(m)
		a.exp = e
		return
	}
	switch {
	case e == a.exp:
		a.mant.Add(&a.mant, m)
	case e > a.exp:
		a.t.Lsh(m, uint(e-a.exp))
		a.mant.Add(&a.mant, &a.t)
	default:
		a.mant.Lsh(&a.mant, uint(a.exp-e))
		a.exp = e
		a.mant.Add(&a.mant, m)
	}
}

// addScratch adds a.t × 2^e, consuming a.t.
func (a *DyAcc) addScratch(e int) {
	if a.mant.Sign() == 0 {
		a.mant.Set(&a.t)
		a.exp = e
		return
	}
	switch {
	case e == a.exp:
	case e > a.exp:
		a.t.Lsh(&a.t, uint(e-a.exp))
	default:
		a.mant.Lsh(&a.mant, uint(a.exp-e))
		a.exp = e
	}
	a.mant.Add(&a.mant, &a.t)
}

// Dyadic returns the sum as a canonical Dyadic. The result does not share
// storage with the accumulator.
func (a *DyAcc) Dyadic() Dyadic {
	return fromBig(new(big.Int).Set(&a.mant), a.exp)
}

// Rat returns the sum as a reduced rational.
func (a *DyAcc) Rat() *big.Rat { return a.Dyadic().Rat() }
