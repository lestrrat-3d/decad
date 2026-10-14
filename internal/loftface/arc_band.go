package loftface

import "math/big"

// Mapped arc band outcomes.
const (
	ArcMiss    = 0
	ArcInside  = -1
	ArcOutside = 1
)

// RatMin returns the smaller of a and b.
func RatMin(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) < 0 {
		return a
	}
	return b
}

// RatMax returns the larger of a and b.
func RatMax(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) > 0 {
		return a
	}
	return b
}

// ArcCapRanges returns the X, Y and Z ranges of the cap map over the source
// corners.
func ArcCapRanges(mapped Map, corners [4][2]*big.Rat) [3]Interval {
	return cornerAxisRanges(mapped, corners)
}

// cornerAxisRanges evaluates the map once at each corner and returns the
// X, Y and Z ranges of the four images. Each interval holds fresh copies.
func cornerAxisRanges(m Map, corners [4][2]*big.Rat) [3]Interval {
	var images [4]RatVec
	for i, corner := range corners {
		images[i] = m.At(corner[0], corner[1])
	}
	var out [3]Interval
	for axis := range out {
		for i := range images {
			value := new(big.Rat).Set(images[i][axis])
			if i == 0 {
				out[axis] = Interval{value, value}
				continue
			}
			if value.Cmp(out[axis].Lo) < 0 {
				out[axis].Lo = value
			}
			if value.Cmp(out[axis].Hi) > 0 {
				out[axis].Hi = value
			}
		}
	}
	return out
}

// ArcBlend returns the interval blend of first and second at time t.
func ArcBlend(first, second Interval, t *big.Rat) Interval {
	a, z := ratSub(ratOne(), t), t
	return Interval{
		Lo: ratAdd(ratMul(a, first.Lo), ratMul(z, second.Lo)),
		Hi: ratAdd(ratMul(a, first.Hi), ratMul(z, second.Hi)),
	}
}

// ArcBandSide classifies the mapped arc band over band time [tLo, tHi] and
// X in [x0, x1] against the radial limit m*x+b: ArcMiss, ArcInside or
// ArcOutside, or false when depth bisections cannot decide.
func ArcBandSide(first, second [3]Interval, x0, x1, m, b *big.Rat,
	tLo, tHi *big.Rat, depth int) (int, bool) {
	var box [3]Interval
	for axis := range box {
		box[axis] = IntervalHull(ArcBlend(first[axis], second[axis], tLo),
			ArcBlend(first[axis], second[axis], tHi))
	}
	if box[0].Hi.Cmp(x0) < 0 || box[0].Lo.Cmp(x1) > 0 {
		return ArcMiss, true
	}
	x := Interval{Lo: RatMax(box[0].Lo, x0), Hi: RatMin(box[0].Hi, x1)}
	radius := Interval{Lo: ratAdd(ratMul(m, x.Lo), b), Hi: ratAdd(ratMul(m, x.Hi), b)}
	if radius.Lo.Cmp(radius.Hi) > 0 {
		radius.Lo, radius.Hi = radius.Hi, radius.Lo
	}
	low := ratSub(ratAdd(IntervalSquareMin(box[1]), IntervalSquareMin(box[2])),
		IntervalSquareMax(radius))
	high := ratSub(ratAdd(IntervalSquareMax(box[1]), IntervalSquareMax(box[2])),
		IntervalSquareMin(radius))
	if low.Sign() > 0 {
		return ArcOutside, true
	}
	if high.Sign() < 0 {
		return ArcInside, true
	}
	if depth == 0 {
		return 0, false
	}
	mid := ratMul(ratAdd(tLo, tHi), big.NewRat(1, 2))
	left, ok := ArcBandSide(first, second, x0, x1, m, b, tLo, mid, depth-1)
	if !ok {
		return 0, false
	}
	right, ok := ArcBandSide(first, second, x0, x1, m, b, mid, tHi, depth-1)
	if !ok || left != 0 && right != 0 && left != right {
		return 0, false
	}
	if left != 0 {
		return left, true
	}
	return right, true
}

// RadialCap is the exact X and the source U/V of one radial cap.
type RadialCap struct {
	X    *big.Rat
	U, V [2]*big.Rat
}

// RadiusSquare returns the squared distance from (u, v) to the center.
func RadiusSquare(u, v, centerU, centerV *big.Rat) *big.Rat {
	du, dv := ratSub(u, centerU), ratSub(v, centerV)
	return ratAdd(ratMul(du, du), ratMul(dv, dv))
}

// A quadratic polynomial in band time vanishes identically when its exact
// values at 0, 1/2, and 1 vanish. These checks prove angular independence
// for every intermediate map, not merely for the two cap endpoints.
func RadialScale(mappedCaps [2]RadialCap, t *big.Rat) (*big.Rat, bool) {
	var u, v [2]*big.Rat
	for axis := range u {
		u[axis] = ratAdd(ratMul(ratSub(ratOne(), t), mappedCaps[0].U[axis]),
			ratMul(t, mappedCaps[1].U[axis]))
		v[axis] = ratAdd(ratMul(ratSub(ratOne(), t), mappedCaps[0].V[axis]),
			ratMul(t, mappedCaps[1].V[axis]))
	}
	u2 := ratAdd(ratMul(u[0], u[0]), ratMul(u[1], u[1]))
	v2 := ratAdd(ratMul(v[0], v[0]), ratMul(v[1], v[1]))
	dot := ratAdd(ratMul(u[0], v[0]), ratMul(u[1], v[1]))
	return u2, u2.Cmp(v2) == 0 && dot.Sign() == 0
}
