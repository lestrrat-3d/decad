package proof

// LazyIntervalMulLo is LazyIntervalMul(a, b).Lo, computing only that end:
// the same zero shortcut and sign branches, and both candidates plus
// LazyCmp where both intervals straddle zero.
func LazyIntervalMulLo(a, b LazyInterval) LazyRat {
	if lazyZeroInterval(a) || lazyZeroInterval(b) {
		return lazyZero()
	}
	m := LazyMul
	if a.Lo.Sign() >= 0 {
		if b.Lo.Sign() >= 0 {
			return m(a.Lo, b.Lo)
		}
		return m(a.Hi, b.Lo)
	}
	if a.Hi.Sign() <= 0 {
		switch {
		case b.Lo.Sign() >= 0:
			return m(a.Lo, b.Hi)
		case b.Hi.Sign() <= 0:
			return m(a.Hi, b.Hi)
		default:
			return m(a.Lo, b.Hi)
		}
	}
	if b.Lo.Sign() >= 0 {
		return m(a.Lo, b.Hi)
	}
	if b.Hi.Sign() <= 0 {
		return m(a.Hi, b.Lo)
	}
	loA, loB := m(a.Lo, b.Hi), m(a.Hi, b.Lo)
	if LazyCmp(loB, loA) < 0 {
		return loB
	}
	return loA
}

// LazyIntervalMulHi is LazyIntervalMul(a, b).Hi, computing only that end.
func LazyIntervalMulHi(a, b LazyInterval) LazyRat {
	if lazyZeroInterval(a) || lazyZeroInterval(b) {
		return lazyZero()
	}
	m := LazyMul
	if a.Lo.Sign() >= 0 {
		switch {
		case b.Lo.Sign() >= 0:
			return m(a.Hi, b.Hi)
		case b.Hi.Sign() <= 0:
			return m(a.Lo, b.Hi)
		default:
			return m(a.Hi, b.Hi)
		}
	}
	if a.Hi.Sign() <= 0 {
		if b.Lo.Sign() >= 0 {
			return m(a.Hi, b.Lo)
		}
		return m(a.Lo, b.Lo)
	}
	if b.Lo.Sign() >= 0 {
		return m(a.Hi, b.Hi)
	}
	if b.Hi.Sign() <= 0 {
		return m(a.Lo, b.Lo)
	}
	hiA, hiB := m(a.Lo, b.Lo), m(a.Hi, b.Hi)
	if LazyCmp(hiB, hiA) > 0 {
		return hiB
	}
	return hiA
}

// LazyIntervalSquareLo is LazyIntervalSquare(a).Lo, computing only that end.
func LazyIntervalSquareLo(a LazyInterval) LazyRat {
	if a.Lo.Sign() >= 0 {
		return LazyMul(a.Lo, a.Lo)
	}
	if a.Hi.Sign() <= 0 {
		return LazyMul(a.Hi, a.Hi)
	}
	return lazyZero()
}

// LazyIntervalSquareHi is LazyIntervalSquare(a).Hi, computing only that end.
func LazyIntervalSquareHi(a LazyInterval) LazyRat {
	lo2, hi2 := LazyMul(a.Lo, a.Lo), LazyMul(a.Hi, a.Hi)
	if a.Lo.Sign() >= 0 {
		return hi2
	}
	if a.Hi.Sign() <= 0 {
		return lo2
	}
	if LazyCmp(hi2, lo2) > 0 {
		return hi2
	}
	return lo2
}
