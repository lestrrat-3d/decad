package linkagebound

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// The common-denominator reading. readExact evaluates every point's position
// and velocity in big.Rat interval arithmetic, and big.Rat reduces each sum
// and product to lowest terms with a GCD; RoundCorners then keeps only the
// floats around each endpoint. readScaled evaluates the same interval
// expressions over integer numerators that share one positive denominator
// per point and per velocity, and rounds each endpoint straight from its
// numerator and denominator (proofarith.FracFloatDown, FracFloatUp), so no
// intermediate value is ever reduced.
//
// Why the Bounds are the same, bit for bit:
//
//   - Every interval operation readExact uses is a function of its operands'
//     values alone: AddInterval and SubInterval add and subtract endpoints;
//     MulInterval picks its two products by the endpoints' signs and, where
//     both operands straddle zero, by comparing products; IvMat.Apply sums
//     three such products. readScaled takes the same branches on the
//     numerators, whose signs are the values' signs and whose comparisons
//     over one positive denominator are the values' comparisons, and forms
//     the same sums and products of the numerators, so each endpoint it
//     holds is n/d for the very value readExact holds.
//   - ApplyIdeal is rot·(x − pivot) + pivot + shift. For a point x and a point
//     pivot, x − pivot is a point, and MulInterval of an entry [lo, hi] with a
//     point v is [lo·v, hi·v] for v ≥ 0 and [hi·v, lo·v] for v < 0 (zero for a
//     zero entry or v), the products ScaledIvMat.ApplyScaled sums. Interval
//     addition of exact rationals is associative, so pivot + shift adds as one
//     interval k, computed once per pose.
//   - RoundCorners keeps RatFloatDown of each low end and RatFloatUp of each
//     high end, read back by FloatRat. Both depend on the value alone, and
//     FracFloatDown and FracFloatUp return them from any numerator and
//     positive denominator of that value; an endpoint whose float is not
//     finite fails the reading as RoundCorners fails it.
//
// ReadPointBounds takes readScaled only for exact points under a pose whose
// pivot is a point, which every linkage pose has (a revolute's pivot is its
// centre, a prismatic's the origin, a composed pose's zero); any other reading
// goes through readExact.

// scaledIv is an interval whose endpoints are integer numerators over a
// positive denominator kept beside it.
type scaledIv struct{ lo, hi *big.Int }

func (a scaledIv) zero() bool { return a.lo.Sign() == 0 && a.hi.Sign() == 0 }

// pointVec reports whether every entry of v is a point.
func pointVec(v motionbound.IvVec) bool {
	for d := range 3 {
		if proofarith.CmpRat(v[d].Lo, v[d].Hi) != 0 {
			return false
		}
	}
	return true
}

// pointReading reports whether every position of r is a point.
func pointReading(r Reading) bool {
	for _, p := range r.Pos {
		if !pointVec(p) {
			return false
		}
	}
	return true
}

// scaleVec writes v's six endpoints as numerators over their least common
// denominator.
func scaleVec(v motionbound.IvVec) (*big.Int, [3]scaledIv) {
	den := proofarith.CommonDenom(v[0].Lo, v[0].Hi, v[1].Lo, v[1].Hi, v[2].Lo, v[2].Hi)
	var out [3]scaledIv
	for d := range 3 {
		out[d] = scaledIv{proofarith.ScaledNum(v[d].Lo, den), proofarith.ScaledNum(v[d].Hi, den)}
	}
	return den, out
}

// mulScaled is proofarith.MulInterval on numerators: for a over da and b over
// db it returns the product's endpoints over da·db, taking MulInterval's
// branches in the same order.
func mulScaled(a, b scaledIv) scaledIv {
	if a.zero() || b.zero() {
		return scaledIv{new(big.Int), new(big.Int)}
	}
	m := func(x, y *big.Int) *big.Int { return new(big.Int).Mul(x, y) }
	if a.lo.Sign() >= 0 {
		switch {
		case b.lo.Sign() >= 0:
			return scaledIv{m(a.lo, b.lo), m(a.hi, b.hi)}
		case b.hi.Sign() <= 0:
			return scaledIv{m(a.hi, b.lo), m(a.lo, b.hi)}
		default:
			return scaledIv{m(a.hi, b.lo), m(a.hi, b.hi)}
		}
	}
	if a.hi.Sign() <= 0 {
		switch {
		case b.lo.Sign() >= 0:
			return scaledIv{m(a.lo, b.hi), m(a.hi, b.lo)}
		case b.hi.Sign() <= 0:
			return scaledIv{m(a.hi, b.hi), m(a.lo, b.lo)}
		default:
			return scaledIv{m(a.lo, b.hi), m(a.lo, b.lo)}
		}
	}
	if b.lo.Sign() >= 0 {
		return scaledIv{m(a.lo, b.hi), m(a.hi, b.hi)}
	}
	if b.hi.Sign() <= 0 {
		return scaledIv{m(a.hi, b.lo), m(a.lo, b.lo)}
	}
	loA, loB := m(a.lo, b.hi), m(a.hi, b.lo)
	hiA, hiB := m(a.lo, b.lo), m(a.hi, b.hi)
	if loB.Cmp(loA) < 0 {
		loA = loB
	}
	if hiB.Cmp(hiA) > 0 {
		hiA = hiB
	}
	return scaledIv{loA, hiA}
}

// roundScaled is RoundOut of the interval a over den, with the two floats it
// read the enclosure back from; ok is false when an end overflows a float.
func roundScaled(a scaledIv, den *big.Int) (proofbound.RatInterval, float64, float64, bool) {
	lof, hif := proofarith.FracFloatDown(a.lo, den), proofarith.FracFloatUp(a.hi, den)
	lo, hi := proofarith.FloatRat(lof), proofarith.FloatRat(hif)
	if lo == nil || hi == nil {
		return proofbound.RatInterval{}, 0, 0, false
	}
	return proofbound.IntervalOwned(lo, hi), lof, hif, true
}

// scaledJoint is a jointAt in numerator form: a revolute's unit axis and
// pivot each over its own denominator, a prismatic's direction as it is.
type scaledJoint struct {
	revolute          bool
	direction         motionbound.IvVec
	unitDen, pivotDen *big.Int
	unit, pivot       [3]scaledIv
}

// readScaled is RoundCorners(readExact(joints, &pose, out)) for a reading of
// exact points under a pose whose pivot is a point (see the file comment). It
// reads out and writes none of its slices.
func readScaled(joints []jointAt, pose motionbound.IdealPose, out Reading) (Bounds, bool) {
	count := len(out.Pos)
	bounds := Bounds{
		Lo: make([][3]*big.Rat, count), Hi: make([][3]*big.Rat, count),
		Vel: make([][][3]proofbound.RatInterval, count), Pad: out.Pad, PrismK: out.PrismK,
		LoF: make([][3]float64, count), HiF: make([][3]float64, count),
	}
	rot := motionbound.NewScaledIvMat(pose.Rot)
	var k motionbound.IvVec
	for d := range 3 {
		k[d] = proofbound.IntervalAdd(pose.Pivot[d], pose.Shift[d])
	}
	kDen, kNum := scaleVec(k)
	scaled := make([]scaledJoint, len(joints))
	for n, j := range joints {
		scaled[n].revolute = j.revolute
		if !j.revolute {
			scaled[n].direction = j.unit
			continue
		}
		scaled[n].unitDen, scaled[n].unit = scaleVec(j.unit)
		scaled[n].pivotDen, scaled[n].pivot = scaleVec(j.pivot)
	}
	for c, x := range out.Pos {
		var rel [3]*big.Rat
		for d := range 3 {
			rel[d] = proofarith.SubRat(new(big.Rat), x[d].Lo, pose.Pivot[d].Lo)
		}
		q := proofarith.CommonDenom(rel[:]...)
		var num [3]*big.Int
		for d := range 3 {
			num[d] = proofarith.ScaledNum(rel[d], q)
		}
		// rot·(x − pivot) over rot.Den·q, then k over kDen: the position over
		// rot.Den·q·kDen.
		lo, hi := rot.ApplyScaled(num)
		rq := new(big.Int).Mul(rot.Den, q)
		den := new(big.Int).Mul(rq, kDen)
		var pos [3]scaledIv
		for d := range 3 {
			lo[d].Mul(lo[d], kDen)
			lo[d].Add(lo[d], new(big.Int).Mul(kNum[d].lo, rq))
			hi[d].Mul(hi[d], kDen)
			hi[d].Add(hi[d], new(big.Int).Mul(kNum[d].hi, rq))
			pos[d] = scaledIv{lo[d], hi[d]}
			iv, lof, hif, ok := roundScaled(pos[d], den)
			if !ok {
				return Bounds{}, false
			}
			bounds.Lo[c][d], bounds.Hi[c][d] = iv.Lo, iv.Hi
			bounds.LoF[c][d], bounds.HiF[c][d] = lof, hif
		}
		bounds.Vel[c] = make([][3]proofbound.RatInterval, len(scaled))
		for n, j := range scaled {
			if !j.revolute {
				for d := range 3 {
					iv, ok := RoundOut(j.direction[d])
					if !ok {
						return Bounds{}, false
					}
					bounds.Vel[c][n][d] = iv
				}
				continue
			}
			// The offset from the joint's pivot, position − pivot, over
			// den·pivotDen; its cross product with the unit axis over
			// unitDen·den·pivotDen.
			var r [3]scaledIv
			for d := range 3 {
				rLo := new(big.Int).Mul(pos[d].lo, j.pivotDen)
				rLo.Sub(rLo, new(big.Int).Mul(j.pivot[d].hi, den))
				rHi := new(big.Int).Mul(pos[d].hi, j.pivotDen)
				rHi.Sub(rHi, new(big.Int).Mul(j.pivot[d].lo, den))
				r[d] = scaledIv{rLo, rHi}
			}
			velDen := new(big.Int).Mul(j.unitDen, den)
			velDen.Mul(velDen, j.pivotDen)
			for d := range 3 {
				a, b := (d+1)%3, (d+2)%3
				first, second := mulScaled(j.unit[a], r[b]), mulScaled(j.unit[b], r[a])
				v := scaledIv{first.lo.Sub(first.lo, second.hi), first.hi.Sub(first.hi, second.lo)}
				iv, _, _, ok := roundScaled(v, velDen)
				if !ok {
					return Bounds{}, false
				}
				bounds.Vel[c][n][d] = iv
			}
		}
	}
	return bounds, true
}
