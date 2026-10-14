package loftface

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Vec returns v as an exact rational vector.
func Vec(v r3.Vec) RatVec { return RatVec{ratF(v.X), ratF(v.Y), ratF(v.Z)} }

// VAdd returns a + b exactly.
func VAdd(a, b RatVec) RatVec {
	return RatVec{ratAdd(a[0], b[0]), ratAdd(a[1], b[1]), ratAdd(a[2], b[2])}
}

// VSub returns a - b exactly.
func VSub(a, b RatVec) RatVec {
	return RatVec{ratSub(a[0], b[0]), ratSub(a[1], b[1]), ratSub(a[2], b[2])}
}

// VScale returns a scaled by s exactly.
func VScale(a RatVec, s *big.Rat) RatVec {
	return RatVec{ratMul(a[0], s), ratMul(a[1], s), ratMul(a[2], s)}
}

// VDot returns the exact dot product of a and b.
func VDot(a, b RatVec) *big.Rat {
	return ratAdd(ratAdd(ratMul(a[0], b[0]), ratMul(a[1], b[1])), ratMul(a[2], b[2]))
}

// VCross returns the exact cross product a x b.
func VCross(a, b RatVec) RatVec {
	return RatVec{
		ratSub(ratMul(a[1], b[2]), ratMul(a[2], b[1])),
		ratSub(ratMul(a[2], b[0]), ratMul(a[0], b[2])),
		ratSub(ratMul(a[0], b[1]), ratMul(a[1], b[0])),
	}
}

// MatVec returns the exact product m * v.
func MatVec(m RatMat, v RatVec) RatVec {
	var out RatVec
	for i := range out {
		out[i] = ratZero()
		for j := range v {
			out[i] = ratAdd(out[i], ratMul(m[i][j], v[j]))
		}
	}
	return out
}

// MatMul returns the exact product a * b.
func MatMul(a, b RatMat) RatMat {
	var out RatMat
	for i := range out {
		for j := range out[i] {
			out[i][j] = ratZero()
			for k := range a[i] {
				out[i][j] = ratAdd(out[i][j], ratMul(a[i][k], b[k][j]))
			}
		}
	}
	return out
}

// Identity returns the exact identity map.
func Identity() Affine {
	var out Affine
	for i := range out.A {
		out.B[i] = ratZero()
		for j := range out.A[i] {
			out.A[i][j] = ratZero()
		}
		out.A[i][i] = ratOne()
	}
	return out
}

// Motion returns the exact affine map of the rigid transform t.
func Motion(t r3.Transform) Affine {
	basis := t.Basis()
	cols := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	out := Affine{B: Vec(t.Translation())}
	for j, col := range cols {
		v := Vec(col)
		for i := range v {
			out.A[i][j] = v[i]
		}
	}
	return out
}

// Scale returns the exact uniform scaling of factor about anchor.
func Scale(anchor r3.Vec, factor float64) Affine {
	out := Identity()
	for i, v := range Vec(anchor) {
		out.A[i][i] = ratF(factor)
		out.B[i] = ratMul(ratSub(ratOne(), ratF(factor)), v)
	}
	return out
}

// Compose returns the exact affine map that applies prior and then next.
func Compose(prior, next Affine) Affine {
	return Affine{A: MatMul(next.A, prior.A),
		B: VAdd(MatVec(next.A, prior.B), next.B)}
}

// AffineStretch returns a float upper bound on the Frobenius norm of a.
func AffineStretch(a RatMat) float64 {
	squares := 0.0
	for i := range a {
		for j := range a[i] {
			v := RatUp(ratAbs(a[i][j]))
			squares = proofbound.AbsSumUpper(squares, proofbound.ProductUpper(v, v))
		}
	}
	return math.Nextafter(math.Sqrt(squares), math.Inf(1))
}

// RatUp returns the smallest float64 that is not below q (Inf when q overflows).
func RatUp(q *big.Rat) float64 {
	f, _ := q.Float64()
	if !proofbound.IsNonFinite(f) && ratF(f).Cmp(q) < 0 {
		f = math.Nextafter(f, math.Inf(1))
	}
	return f
}

// IntervalHull returns the smallest interval holding a and b.
func IntervalHull(a, b Interval) Interval {
	out := a
	if b.Lo.Cmp(out.Lo) < 0 {
		out.Lo = b.Lo
	}
	if b.Hi.Cmp(out.Hi) > 0 {
		out.Hi = b.Hi
	}
	return out
}

// IntervalSub returns a - b as an exact interval.
func IntervalSub(a, b Interval) Interval {
	return Interval{ratSub(a.Lo, b.Hi), ratSub(a.Hi, b.Lo)}
}

// IntervalAbs returns the largest |x - center| over the interval a.
func IntervalAbs(a Interval, center *big.Rat) *big.Rat {
	lo, hi := ratAbs(ratSub(a.Lo, center)), ratAbs(ratSub(a.Hi, center))
	if lo.Cmp(hi) > 0 {
		return lo
	}
	return hi
}

// TileCorners returns the four corners of the tile [u0, u1] x [v0, v1] in the order (u0,v0), (u0,v1), (u1,v0), (u1,v1).
func TileCorners(u0, u1, v0, v1 *big.Rat) [4][2]*big.Rat {
	return [4][2]*big.Rat{{u0, v0}, {u0, v1}, {u1, v0}, {u1, v1}}
}

// ScalarRange returns the interval of row . m(u, v) over the corners of a tile.
//
// Over the rationals row . F(u, v) = row . Constant + (row . Q0 + u row . QU +
// v row . QV) / G(u, v): the dot product is linear, and F(u, v) is Constant
// plus Q(u, v) scaled by 1/G(u, v). The four row dot products do not depend on
// the corner, so each is taken once. Every operation is exact, so the value is
// the same lowest-terms rational the corner-by-corner form gives. A zero
// denominator panics in the division, as it does in Map.At.
func ScalarRange(m Map, row RatVec, corners [4][2]*big.Rat) Interval {
	c, q0, qu, qv := VDot(row, m.Constant), VDot(row, m.Q0), VDot(row, m.QU), VDot(row, m.QV)
	var out Interval
	for i, corner := range corners {
		u, v := corner[0], corner[1]
		num := ratAdd(ratAdd(q0, ratMul(qu, u)), ratMul(qv, v))
		value := ratAdd(c, ratDiv(num, m.Denominator(u, v)))
		if i == 0 {
			out = Interval{value, value}
			continue
		}
		if value.Cmp(out.Lo) < 0 {
			out.Lo = value
		}
		if value.Cmp(out.Hi) > 0 {
			out.Hi = value
		}
	}
	return out
}

// InverseRows returns the rows of the inverse of the matrix with columns c0, c1, c2 and its determinant, or an error when the matrix is singular.
func InverseRows(c0, c1, c2 RatVec) ([3]RatVec, *big.Rat, error) {
	det := VDot(c0, VCross(c1, c2))
	if det.Sign() == 0 {
		return [3]RatVec{}, nil,
			fmt.Errorf("%w: loft band source map is singular", decaderr.ErrUnsupported)
	}
	return [3]RatVec{
		VScale(VCross(c1, c2), ratDiv(ratOne(), det)),
		VScale(VCross(c2, c0), ratDiv(ratOne(), det)),
		VScale(VCross(c0, c1), ratDiv(ratOne(), det)),
	}, det, nil
}

// BoxCorners returns the four corners of the source box (u0, u1, v0, v1) in TileCorners order.
func BoxCorners(box [4]*big.Rat) [4][2]*big.Rat {
	return TileCorners(box[0], box[1], box[2], box[3])
}

// PositiveDenominator reports an error unless the map denominator is positive at every corner of the box.
func PositiveDenominator(m Map, box [4]*big.Rat) error {
	for _, corner := range BoxCorners(box) {
		if m.Denominator(corner[0], corner[1]).Sign() <= 0 {
			return fmt.Errorf("%w: loft cap ray denominator crosses its source region", decaderr.ErrUnsupported)
		}
	}
	return nil
}

// PlaneOrder proves that an exact intermediate cap separates every earlier
// and later cap. Since each band is a convex interpolation of its end caps,
// this also separates nonadjacent bands without a sampled crossing claim.
func PlaneOrder(maps []Map, box [4]*big.Rat) error {
	corners := BoxCorners(box)
	for middle := 1; middle+1 < len(maps); middle++ {
		p0 := maps[middle].At(corners[0][0], corners[0][1])
		p1 := maps[middle].At(corners[2][0], corners[2][1])
		p2 := maps[middle].At(corners[1][0], corners[1][1])
		normal := VCross(VSub(p1, p0), VSub(p2, p0))
		if normal[0].Sign() == 0 && normal[1].Sign() == 0 && normal[2].Sign() == 0 {
			return fmt.Errorf("%w: loft cap %d has no exact supporting plane", decaderr.ErrUnsupported, middle)
		}
		previousSign := 0
		for section, capMap := range maps {
			if section == middle {
				continue
			}
			for _, corner := range corners {
				sign := VDot(normal, VSub(capMap.At(corner[0], corner[1]), p0)).Sign()
				if sign == 0 {
					return fmt.Errorf("%w: loft cap %d meets section %d", decaderr.ErrUnsupported, middle, section)
				}
				if section < middle {
					if previousSign == 0 {
						previousSign = sign
					}
					if sign != previousSign {
						return fmt.Errorf("%w: loft cap %d does not separate earlier sections", decaderr.ErrUnsupported, middle)
					}
				} else if sign == previousSign {
					return fmt.Errorf("%w: loft cap %d does not separate later sections", decaderr.ErrUnsupported, middle)
				}
			}
		}
	}
	return nil
}

// IntervalSquareMin returns the smallest x*x over the interval i.
func IntervalSquareMin(i Interval) *big.Rat {
	if i.Lo.Sign() <= 0 && i.Hi.Sign() >= 0 {
		return ratZero()
	}
	a, b := ratMul(i.Lo, i.Lo), ratMul(i.Hi, i.Hi)
	if a.Cmp(b) < 0 {
		return a
	}
	return b
}

// IntervalSquareMax returns the largest x*x over the interval i.
func IntervalSquareMax(i Interval) *big.Rat {
	a, b := ratMul(i.Lo, i.Lo), ratMul(i.Hi, i.Hi)
	if a.Cmp(b) > 0 {
		return a
	}
	return b
}
