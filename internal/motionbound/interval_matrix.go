package motionbound

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// RatVec is an exact rational vector.
type RatVec [3]*big.Rat

func RatVecOf(v r3.Vec) (RatVec, bool) {
	x, y, z := proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)
	if x == nil || y == nil || z == nil {
		return RatVec{}, false
	}
	return RatVec{x, y, z}, true
}

// IvVec and IvMat are interval vectors and 3×3 interval matrices, the
// matrix stored by rows.
type IvVec [3]proofbound.RatInterval

type IvMat [3][3]proofbound.RatInterval

func PointVec(v RatVec) IvVec {
	return IvVec{proofbound.PointInterval(v[0]), proofbound.PointInterval(v[1]), proofbound.PointInterval(v[2])}
}

func (m IvMat) Apply(v IvVec) IvVec {
	var out IvVec
	for i := range 3 {
		sum := proofbound.IntervalMul(m[i][0], v[0])
		sum = proofbound.IntervalAdd(sum, proofbound.IntervalMul(m[i][1], v[1]))
		out[i] = proofbound.IntervalAdd(sum, proofbound.IntervalMul(m[i][2], v[2]))
	}
	return out
}

// ScaledIvMat is an interval matrix with its 18 endpoints written as integer
// numerators over one shared positive denominator: entry (i, j) is
// [lo[i][j]/den, hi[i][j]/den] exactly.
type ScaledIvMat struct {
	Den    *big.Int
	Lo, Hi [3][3]*big.Int
}

// entry is entry (i, j) as its exact rational interval.
func (s ScaledIvMat) Entry(i, j int) proofbound.RatInterval {
	return proofbound.IntervalOwned(new(big.Rat).SetFrac(s.Lo[i][j], s.Den), new(big.Rat).SetFrac(s.Hi[i][j], s.Den))
}

func NewScaledIvMat(m IvMat) ScaledIvMat {
	den := big.NewInt(1)
	for i := range 3 {
		for j := range 3 {
			den = proofarith.LcmInt(proofarith.LcmInt(den, m[i][j].Lo.Denom()), m[i][j].Hi.Denom())
		}
	}
	s := ScaledIvMat{Den: den}
	for i := range 3 {
		for j := range 3 {
			s.Lo[i][j], s.Hi[i][j] = proofarith.ScaledNum(m[i][j].Lo, den), proofarith.ScaledNum(m[i][j].Hi, den)
		}
	}
	return s
}

// applyScaled is IvMat.apply on the exact point whose coordinates are n/q,
// for integer numerators n over a positive q: the endpoint numerators of each
// row over den·q. The interval product of an entry with a point coordinate v
// is [lo·v, hi·v] for v ≥ 0 and [hi·v, lo·v] for v < 0 (MulInterval), and a
// row sums them, so each endpoint is one integer dot product.
func (s ScaledIvMat) ApplyScaled(n [3]*big.Int) ([3]*big.Int, [3]*big.Int) {
	var lo, hi [3]*big.Int
	term := new(big.Int)
	for i := range 3 {
		lo[i], hi[i] = new(big.Int), new(big.Int)
		for j := range 3 {
			low, high := s.Lo[i][j], s.Hi[i][j]
			if n[j].Sign() < 0 {
				low, high = high, low
			}
			lo[i].Add(lo[i], term.Mul(low, n[j]))
			hi[i].Add(hi[i], term.Mul(high, n[j]))
		}
	}
	return lo, hi
}

// mulPoints is IvMat.mul with a right factor o whose entries are all points,
// as ExactTransform's are: the interval product of an entry with a point v is
// [lo·v, hi·v] for v ≥ 0 and [hi·v, lo·v] for v < 0 (MulInterval), so each
// result endpoint is one integer dot product over den times o's own shared
// denominator. ok is false when an entry of o is not a point.
func (s ScaledIvMat) MulPoints(o IvMat) (ScaledIvMat, bool) {
	values := make([]*big.Rat, 0, 9)
	for k := range 3 {
		for j := range 3 {
			if o[k][j].Lo.Cmp(o[k][j].Hi) != 0 {
				return ScaledIvMat{}, false
			}
			values = append(values, o[k][j].Lo)
		}
	}
	pointDen := proofarith.CommonDenom(values...)
	var point [3][3]*big.Int
	for k := range 3 {
		for j := range 3 {
			point[k][j] = proofarith.ScaledNum(values[3*k+j], pointDen)
		}
	}
	out := ScaledIvMat{Den: new(big.Int).Mul(s.Den, pointDen)}
	term := new(big.Int)
	for i := range 3 {
		for j := range 3 {
			lo, hi := new(big.Int), new(big.Int)
			for k := range 3 {
				low, high := s.Lo[i][k], s.Hi[i][k]
				if point[k][j].Sign() < 0 {
					low, high = high, low
				}
				lo.Add(lo, term.Mul(low, point[k][j]))
				hi.Add(hi, term.Mul(high, point[k][j]))
			}
			out.Lo[i][j], out.Hi[i][j] = lo, hi
		}
	}
	return out, true
}

func IvVecAdd(a, b IvVec) IvVec {
	return IvVec{proofbound.IntervalAdd(a[0], b[0]), proofbound.IntervalAdd(a[1], b[1]), proofbound.IntervalAdd(a[2], b[2])}
}

func IvVecSub(a, b IvVec) IvVec {
	return IvVec{proofbound.IntervalSub(a[0], b[0]), proofbound.IntervalSub(a[1], b[1]), proofbound.IntervalSub(a[2], b[2])}
}

// MagnitudeSquaredUpper is Σ max(|lo|, |hi|)² over the entries: an exact
// upper bound on the squared Euclidean (or, over a matrix's entries,
// Frobenius) norm of every member of the enclosure.
func MagnitudeSquaredUpper(entries ...proofbound.RatInterval) *big.Rat {
	sum := new(big.Rat)
	for _, e := range entries {
		m := new(big.Rat).Abs(e.Lo)
		if hi := new(big.Rat).Abs(e.Hi); hi.Cmp(m) > 0 {
			m = hi
		}
		sum.Add(sum, m.Mul(m, m))
	}
	return sum
}

// UnitScaleInterval encloses 1/|a| for a nonzero exact vector a: the inverse
// of proofbound.RatSqrtUp/proofbound.RatSqrtDown's directed roots of |a|², each proven by exact
// comparison. When |a|² is the exact square of a float both roots agree and
// the enclosure is a point, which is what keeps an axis-aligned direction
// exact.
func UnitScaleInterval(a RatVec) (proofbound.RatInterval, bool) {
	sq := proofbound.RatAdd(proofbound.RatMul(a[0], a[0]), proofbound.RatMul(a[1], a[1]), proofbound.RatMul(a[2], a[2]))
	up, down := proofbound.RatSqrtUp(sq), proofbound.RatSqrtDown(sq)
	if !(down > 0) || proofbound.IsNonFinite(up) {
		return proofbound.RatInterval{}, false
	}
	lo, hi := proofarith.FloatRat(up), proofarith.FloatRat(down)
	return proofbound.IntervalOwned(new(big.Rat).Inv(lo), new(big.Rat).Inv(hi)), true
}

func (m IvMat) Mul(o IvMat) IvMat {
	var out IvMat
	for i := range 3 {
		for j := range 3 {
			sum := proofbound.IntervalMul(m[i][0], o[0][j])
			sum = proofbound.IntervalAdd(sum, proofbound.IntervalMul(m[i][1], o[1][j]))
			out[i][j] = proofbound.IntervalAdd(sum, proofbound.IntervalMul(m[i][2], o[2][j]))
		}
	}
	return out
}

// ExactTransform reads a float transform's linear part (by rows) and its
// translation as the exact rationals its entries denote.
func ExactTransform(t r3.Transform) (IvMat, RatVec, bool) {
	b := t.Basis()
	var rot IvMat
	for j, col := range []r3.Vec{b.EX, b.EY, b.EZ} {
		c, ok := RatVecOf(col)
		if !ok {
			return IvMat{}, RatVec{}, false
		}
		for i := range 3 {
			rot[i][j] = proofbound.PointInterval(c[i])
		}
	}
	shift, ok := RatVecOf(t.Translation())
	if !ok {
		return IvMat{}, RatVec{}, false
	}
	return rot, shift, true
}
