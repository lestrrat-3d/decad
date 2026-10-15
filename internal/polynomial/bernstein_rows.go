package polynomial

import "math/big"

// BernsteinRowsPositive reports whether every control of the three rows is
// strictly positive.
func BernsteinRowsPositive(rows [3][]*big.Rat) bool {
	for _, row := range rows {
		for _, value := range row {
			if value.Sign() <= 0 {
				return false
			}
		}
	}
	return true
}

// BernsteinRowsCross reports whether every row runs from a negative first
// control to a positive last control and has a strictly positive derivative.
func BernsteinRowsCross(q [3][]*big.Rat) bool {
	for _, row := range q {
		if row[0].Sign() >= 0 || row[len(row)-1].Sign() <= 0 {
			return false
		}
	}
	var derivative [3][]*big.Rat
	for i := range q {
		derivative[i] = BernsteinDeriv(q[i])
	}
	return BernsteinRowsPositive(derivative)
}

// BernsteinRowsAt evaluates the quadratic Bernstein combination of the three
// rows at band time t, control by control.
func BernsteinRowsAt(q [3][]*big.Rat, t *big.Rat) []*big.Rat {
	oneMinus := ratSub(ratOne(), t)
	a := ratMul(oneMinus, oneMinus)
	b := ratMul(big.NewRat(2, 1), ratMul(t, oneMinus))
	c := ratMul(t, t)
	out := make([]*big.Rat, len(q[0]))
	for i := range out {
		out[i] = ratAdd(ratAdd(ratMul(a, q[0][i]), ratMul(b, q[1][i])), ratMul(c, q[2][i]))
	}
	return out
}

// BernsteinRowsSignAt reports whether every Bernstein control of the band-time
// curve at source s, restricted to [tLo, tHi], has the given sign.
func BernsteinRowsSignAt(q [3][]*big.Rat, s, tLo, tHi *big.Rat, sign int) bool {
	curve := []*big.Rat{BernsteinValue(q[0], s), BernsteinValue(q[1], s),
		BernsteinValue(q[2], s)}
	for _, coefficient := range BernsteinRestrict(curve, tLo, tHi) {
		if coefficient.Sign() != sign {
			return false
		}
	}
	return true
}

// BernsteinRectOpenSide returns 1 or -1 for the side on which the rectangle is
// strictly signed, and 0 otherwise.
// A Bernstein rectangle with nonnegative controls is strictly positive for
// every interior band time when each source column has a positive control.
// Closed time endpoints additionally require every source control in that
// endpoint row to be strict. The root bracket owns excluded endpoints.
func BernsteinRectOpenSide(q [3][]*big.Rat,
	tLo, tHi *big.Rat, loOpen, hiOpen bool) int {
	if tLo == nil || tHi == nil || tLo.Cmp(tHi) >= 0 || len(q[0]) == 0 ||
		len(q[1]) != len(q[0]) || len(q[2]) != len(q[0]) {
		return 0
	}
	// The source interval is [0, 1], and the time restriction already owns fresh rows.
	box := BernsteinRectTimeRestrict(q, tLo, tHi)
	for _, side := range [...]int{1, -1} {
		valid := true
		for col := range box[0] {
			positive := false
			for row := range box {
				sign := box[row][col].Sign() * side
				if sign < 0 || row == 0 && !loOpen && sign == 0 ||
					row == 2 && !hiOpen && sign == 0 {
					valid = false
					break
				}
				positive = positive || sign > 0
			}
			if !valid || !positive {
				valid = false
				break
			}
		}
		if valid {
			return side
		}
	}
	return 0
}

// BernsteinRowsPointSign returns -1 or 1 when the three rows are all strictly
// negative or all strictly positive at fixed source s, and 0 otherwise.
// A strict sign at fixed source s for all times follows from the exact
// quadratic Bernstein coefficients in band time.
func BernsteinRowsPointSign(q [3][]*big.Rat, s *big.Rat) int {
	negative, positive := true, true
	for _, row := range q {
		sign := BernsteinSign(row, s)
		negative = negative && sign < 0
		positive = positive && sign > 0
	}
	if negative {
		return -1
	}
	if positive {
		return 1
	}
	return 0
}

// BernsteinRectRestrict restricts the rectangle of three rows to the source
// range [sLo, sHi] and the band time range [tLo, tHi].
func BernsteinRectRestrict(rows [3][]*big.Rat,
	sLo, sHi, tLo, tHi *big.Rat) [3][]*big.Rat {
	return BernsteinRectSourceRestrict(BernsteinRectTimeRestrict(rows, tLo, tHi), sLo, sHi)
}

// BernsteinRectTimeRestrict restricts every source column of the three rows
// to band time [tLo, tHi]. The result is fresh; rows is not changed.
func BernsteinRectTimeRestrict(rows [3][]*big.Rat, tLo, tHi *big.Rat) [3][]*big.Rat {
	var restricted [3][]*big.Rat
	for row := range restricted {
		restricted[row] = make([]*big.Rat, len(rows[row]))
	}
	for column := range rows[0] {
		coeff := BernsteinRestrict([]*big.Rat{rows[0][column], rows[1][column],
			rows[2][column]}, tLo, tHi)
		for row := range restricted {
			restricted[row][column] = coeff[row]
		}
	}
	return restricted
}

// BernsteinRectSourceRestrict restricts each of the three rows to source
// [sLo, sHi]. The result is fresh; rows is not changed, so one time
// restriction can serve several source ranges.
func BernsteinRectSourceRestrict(rows [3][]*big.Rat, sLo, sHi *big.Rat) [3][]*big.Rat {
	var restricted [3][]*big.Rat
	for row := range restricted {
		restricted[row] = BernsteinRestrict(rows[row], sLo, sHi)
	}
	return restricted
}
