package decad

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// These exact distances only decide whether a held Loft wall can meet a
// proposed ring after its authenticated, two-sided wall displacement.
type loftCylinderRingVec [3]*big.Rat

func loftCylinderRingPoint(p proofarith.Xpt) loftCylinderRingVec {
	return loftCylinderRingVec{
		new(big.Rat).SetFrac(p.X, p.W), new(big.Rat).SetFrac(p.Y, p.W),
		new(big.Rat).SetFrac(p.Z, p.W),
	}
}

func loftCylinderRingVertex(p r3.Vec) loftCylinderRingVec {
	return loftCylinderRingVec{proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y), proofarith.FloatRat(p.Z)}
}

func loftCylinderRingSub(a, b loftCylinderRingVec) loftCylinderRingVec {
	return loftCylinderRingVec{
		new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1]),
		new(big.Rat).Sub(a[2], b[2]),
	}
}

func loftCylinderRingDot(a, b loftCylinderRingVec) *big.Rat {
	q := new(big.Rat)
	for i := range a {
		q.Add(q, new(big.Rat).Mul(a[i], b[i]))
	}
	return q
}

func loftCylinderRingCross(a, b loftCylinderRingVec) loftCylinderRingVec {
	return loftCylinderRingVec{
		new(big.Rat).Sub(new(big.Rat).Mul(a[1], b[2]), new(big.Rat).Mul(a[2], b[1])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[2], b[0]), new(big.Rat).Mul(a[0], b[2])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0])),
	}
}

func loftCylinderRingPointSegment2(p, a, b loftCylinderRingVec) *big.Rat {
	d := loftCylinderRingSub(b, a)
	length2 := loftCylinderRingDot(d, d)
	if length2.Sign() == 0 {
		v := loftCylinderRingSub(p, a)
		return loftCylinderRingDot(v, v)
	}
	v := loftCylinderRingSub(p, a)
	along := loftCylinderRingDot(v, d)
	if along.Sign() <= 0 {
		return loftCylinderRingDot(v, v)
	}
	if along.Cmp(length2) >= 0 {
		v = loftCylinderRingSub(p, b)
		return loftCylinderRingDot(v, v)
	}
	return new(big.Rat).Sub(loftCylinderRingDot(v, v),
		new(big.Rat).Quo(new(big.Rat).Mul(along, along), length2))
}

func loftCylinderRingPointTriangle2(p loftCylinderRingVec, tri [3]loftCylinderRingVec) *big.Rat {
	n := loftCylinderRingCross(loftCylinderRingSub(tri[1], tri[0]),
		loftCylinderRingSub(tri[2], tri[0]))
	n2 := loftCylinderRingDot(n, n)
	if n2.Sign() == 0 {
		return nil
	}
	inside := true
	for i := range tri {
		e := loftCylinderRingSub(tri[(i+1)%3], tri[i])
		v := loftCylinderRingSub(p, tri[i])
		if loftCylinderRingDot(loftCylinderRingCross(e, v), n).Sign() < 0 {
			inside = false
			break
		}
	}
	if inside {
		q := loftCylinderRingDot(loftCylinderRingSub(p, tri[0]), n)
		return new(big.Rat).Quo(new(big.Rat).Mul(q, q), n2)
	}
	var minimum *big.Rat
	for i := range tri {
		q := loftCylinderRingPointSegment2(p, tri[i], tri[(i+1)%3])
		if minimum == nil || q.Cmp(minimum) < 0 {
			minimum = q
		}
	}
	return minimum
}

// The endpoint and edge cases cover the constrained minimum of two segments.
// An interior stationary pair is added when their directions are independent.
func loftCylinderRingSegments2(a, b, c, d loftCylinderRingVec) *big.Rat {
	minimum := loftCylinderRingPointSegment2(a, c, d)
	for _, q := range []*big.Rat{
		loftCylinderRingPointSegment2(b, c, d),
		loftCylinderRingPointSegment2(c, a, b),
		loftCylinderRingPointSegment2(d, a, b),
	} {
		if q.Cmp(minimum) < 0 {
			minimum = q
		}
	}
	u, v, w := loftCylinderRingSub(b, a), loftCylinderRingSub(d, c), loftCylinderRingSub(a, c)
	uu, vv, uv := loftCylinderRingDot(u, u), loftCylinderRingDot(v, v), loftCylinderRingDot(u, v)
	det := new(big.Rat).Sub(new(big.Rat).Mul(uu, vv), new(big.Rat).Mul(uv, uv))
	if det.Sign() == 0 {
		return minimum
	}
	uw, vw := loftCylinderRingDot(u, w), loftCylinderRingDot(v, w)
	s := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(uv, vw),
		new(big.Rat).Mul(vv, uw)), det)
	t := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(uu, vw),
		new(big.Rat).Mul(uv, uw)), det)
	if s.Sign() <= 0 || s.Cmp(big.NewRat(1, 1)) >= 0 ||
		t.Sign() <= 0 || t.Cmp(big.NewRat(1, 1)) >= 0 {
		return minimum
	}
	for i := range w {
		w[i] = new(big.Rat).Sub(new(big.Rat).Add(w[i], new(big.Rat).Mul(s, u[i])),
			new(big.Rat).Mul(t, v[i]))
	}
	q := loftCylinderRingDot(w, w)
	if q.Cmp(minimum) < 0 {
		return q
	}
	return minimum
}

// A segment/triangle minimum occurs at a segment endpoint, an edge pair,
// or a direct crossing. The zero-distance crossing is checked by the two
// segment endpoints' signed plane values and the triangle's edge planes.
func loftCylinderRingSegmentTriangle2(a, b loftCylinderRingVec,
	tri [3]loftCylinderRingVec) *big.Rat {
	n := loftCylinderRingCross(loftCylinderRingSub(tri[1], tri[0]),
		loftCylinderRingSub(tri[2], tri[0]))
	if loftCylinderRingDot(n, n).Sign() == 0 {
		return nil
	}
	da := loftCylinderRingDot(loftCylinderRingSub(a, tri[0]), n)
	db := loftCylinderRingDot(loftCylinderRingSub(b, tri[0]), n)
	if da.Sign() == 0 || db.Sign() == 0 || da.Sign() != db.Sign() {
		den := new(big.Rat).Sub(da, db)
		if den.Sign() != 0 {
			t := new(big.Rat).Quo(da, den)
			if t.Sign() >= 0 && t.Cmp(big.NewRat(1, 1)) <= 0 {
				p := loftCylinderRingVec{}
				for i := range p {
					p[i] = new(big.Rat).Add(a[i],
						new(big.Rat).Mul(t, new(big.Rat).Sub(b[i], a[i])))
				}
				if q := loftCylinderRingPointTriangle2(p, tri); q != nil && q.Sign() == 0 {
					return q
				}
			}
		}
	}
	minimum := loftCylinderRingPointTriangle2(a, tri)
	if minimum == nil {
		return nil
	}
	if q := loftCylinderRingPointTriangle2(b, tri); q.Cmp(minimum) < 0 {
		minimum = q
	}
	for i := range tri {
		q := loftCylinderRingSegments2(a, b, tri[i], tri[(i+1)%3])
		if q.Cmp(minimum) < 0 {
			minimum = q
		}
	}
	return minimum
}

type loftCylinderRingBox [3][2]float64

func loftCylinderRingTriangleBox(tri [3]r3.Vec, displacement float64) loftCylinderRingBox {
	var box loftCylinderRingBox
	for axis := range box {
		value := [3]float64{}
		for j, p := range tri {
			switch axis {
			case 0:
				value[j] = p.X
			case 1:
				value[j] = p.Y
			default:
				value[j] = p.Z
			}
		}
		lo, hi := value[0], value[0]
		for _, x := range value[1:] {
			if x < lo {
				lo = x
			}
			if x > hi {
				hi = x
			}
		}
		box[axis] = [2]float64{
			proofbound.RatFloatDown(new(big.Rat).Sub(proofarith.FloatRat(lo), proofarith.FloatRat(displacement))),
			proofbound.RatFloatUp(new(big.Rat).Add(proofarith.FloatRat(hi), proofarith.FloatRat(displacement))),
		}
	}
	return box
}

func loftCylinderRingEdgeBox(a, b loftCylinderRingVec) loftCylinderRingBox {
	var box loftCylinderRingBox
	for i := range box {
		lo, hi := a[i], b[i]
		if lo.Cmp(hi) > 0 {
			lo, hi = hi, lo
		}
		box[i] = [2]float64{proofbound.RatFloatDown(lo), proofbound.RatFloatUp(hi)}
	}
	return box
}

func loftCylinderRingBoxesMeet(a, b loftCylinderRingBox) bool {
	for i := range a {
		if a[i][1] < b[i][0] || b[i][1] < a[i][0] {
			return false
		}
	}
	return true
}
