package meshbool

import (
	"math"
	"math/big"
	"math/bits"
	"sync"

	"github.com/lestrrat-3d/r3"
)

// The exact triangle-distance reference the TriTriDistanceExceeds tests read.
// It shares no code with the production distance or its proofs. Every pair
// is lifted to integers by one power of two, so its arithmetic is big.Int
// products and sums with no GCD, and a squared distance is a fraction
// compared by cross multiplication.

// refInt is a corner scaled by 2^k to integers.
type refInt [3]*big.Int

// refDist is the exact squared distance n/(d·4^k), with d > 0.
type refDist struct {
	n, d *big.Int
	k    int
}

// rat is the squared distance as a rational.
func (r refDist) rat() *big.Rat {
	return new(big.Rat).SetFrac(r.n, new(big.Int).Lsh(r.d, uint(2*r.k)))
}

// refFloatParts splits a finite nonzero x into an odd mantissa and a binary
// exponent: x = m·2^e.
func refFloatParts(x float64) (int64, int) {
	f, e := math.Frexp(x)
	m := int64(f * (1 << 53))
	e -= 53
	tz := bits.TrailingZeros64(uint64(m))
	return m >> tz, e + tz
}

// refFinite reports whether every corner coordinate is finite.
func refFinite(ta, tb [3]r3.Vec) bool {
	for _, t := range [2][3]r3.Vec{ta, tb} {
		for _, v := range t {
			for _, c := range [3]float64{v.X, v.Y, v.Z} {
				if math.IsNaN(c) || math.IsInf(c, 0) {
					return false
				}
			}
		}
	}
	return true
}

// refScale lifts the finite corners of ta and tb to integers by the
// smallest power of two 2^k, k >= 0, that makes every coordinate one.
func refScale(ta, tb [3]r3.Vec) ([3]refInt, [3]refInt, int) {
	k := 0
	for _, t := range [2][3]r3.Vec{ta, tb} {
		for _, v := range t {
			for _, c := range [3]float64{v.X, v.Y, v.Z} {
				if c != 0 {
					if _, e := refFloatParts(c); -e > k {
						k = -e
					}
				}
			}
		}
	}
	lift := func(c float64) *big.Int {
		if c == 0 {
			return new(big.Int)
		}
		m, e := refFloatParts(c)
		return new(big.Int).Lsh(big.NewInt(m), uint(e+k))
	}
	var a, b [3]refInt
	for i := range 3 {
		a[i] = refInt{lift(ta[i].X), lift(ta[i].Y), lift(ta[i].Z)}
		b[i] = refInt{lift(tb[i].X), lift(tb[i].Y), lift(tb[i].Z)}
	}
	return a, b, k
}

func refSub(a, b refInt) refInt {
	return refInt{new(big.Int).Sub(a[0], b[0]), new(big.Int).Sub(a[1], b[1]), new(big.Int).Sub(a[2], b[2])}
}

func refDot(a, b refInt) *big.Int {
	r := new(big.Int).Mul(a[0], b[0])
	r.Add(r, new(big.Int).Mul(a[1], b[1]))
	return r.Add(r, new(big.Int).Mul(a[2], b[2]))
}

func refCross(a, b refInt) refInt {
	m := func(x, y, z, w *big.Int) *big.Int {
		return new(big.Int).Sub(new(big.Int).Mul(x, y), new(big.Int).Mul(z, w))
	}
	return refInt{m(a[1], b[2], a[2], b[1]), m(a[2], b[0], a[0], b[2]), m(a[0], b[1], a[1], b[0])}
}

// refFrac is n/d with d > 0.
type refFrac struct{ n, d *big.Int }

func (f refFrac) less(g refFrac) bool {
	return new(big.Int).Mul(f.n, g.d).Cmp(new(big.Int).Mul(g.n, f.d)) < 0
}

func refMin(fs ...refFrac) refFrac {
	best := fs[0]
	for _, f := range fs[1:] {
		if f.less(best) {
			best = f
		}
	}
	return best
}

// refPointSeg2 is the exact squared distance from p to the closed segment ab.
func refPointSeg2(p, a, b refInt) refFrac {
	u, w := refSub(b, a), refSub(p, a)
	uu, wu := refDot(u, u), refDot(w, u)
	if uu.Sign() == 0 || wu.Sign() <= 0 {
		return refFrac{refDot(w, w), big.NewInt(1)}
	}
	if wu.Cmp(uu) >= 0 {
		v := refSub(p, b)
		return refFrac{refDot(v, v), big.NewInt(1)}
	}
	n := new(big.Int).Mul(refDot(w, w), uu)
	return refFrac{n.Sub(n, new(big.Int).Mul(wu, wu)), uu}
}

// refPointTri2 is the exact squared distance from p to the closed triangle t.
func refPointTri2(p refInt, t [3]refInt) refFrac {
	u, v, w := refSub(t[1], t[0]), refSub(t[2], t[0]), refSub(p, t[0])
	uu, uv, vv, wu, wv := refDot(u, u), refDot(u, v), refDot(v, v), refDot(w, u), refDot(w, v)
	det := new(big.Int).Sub(new(big.Int).Mul(uu, vv), new(big.Int).Mul(uv, uv))
	if det.Sign() > 0 {
		s := new(big.Int).Sub(new(big.Int).Mul(wu, vv), new(big.Int).Mul(wv, uv))
		r := new(big.Int).Sub(new(big.Int).Mul(wv, uu), new(big.Int).Mul(wu, uv))
		if s.Sign() >= 0 && r.Sign() >= 0 && new(big.Int).Add(s, r).Cmp(det) <= 0 {
			n := refCross(u, v)
			wn := refDot(w, n)
			return refFrac{new(big.Int).Mul(wn, wn), refDot(n, n)}
		}
	}
	return refMin(refPointSeg2(p, t[0], t[1]), refPointSeg2(p, t[1], t[2]), refPointSeg2(p, t[2], t[0]))
}

// refSegSeg2 is the exact squared distance between two closed segments.
func refSegSeg2(p0, p1, q0, q1 refInt) refFrac {
	best := refMin(refPointSeg2(p0, q0, q1), refPointSeg2(p1, q0, q1), refPointSeg2(q0, p0, p1),
		refPointSeg2(q1, p0, p1))
	u, v, w := refSub(p1, p0), refSub(q1, q0), refSub(p0, q0)
	a, b, c := refDot(u, u), refDot(u, v), refDot(v, v)
	d, e := refDot(u, w), refDot(v, w)
	den := new(big.Int).Sub(new(big.Int).Mul(a, c), new(big.Int).Mul(b, b))
	if den.Sign() <= 0 {
		return best
	}
	sn := new(big.Int).Sub(new(big.Int).Mul(b, e), new(big.Int).Mul(c, d))
	tn := new(big.Int).Sub(new(big.Int).Mul(a, e), new(big.Int).Mul(b, d))
	if sn.Sign() < 0 || sn.Cmp(den) > 0 || tn.Sign() < 0 || tn.Cmp(den) > 0 {
		return best
	}
	var diff refInt
	for k := range 3 {
		x := new(big.Int).Mul(w[k], den)
		x.Add(x, new(big.Int).Mul(u[k], sn))
		diff[k] = x.Sub(x, new(big.Int).Mul(v[k], tn))
	}
	return refMin(best, refFrac{refDot(diff, diff), new(big.Int).Mul(den, den)})
}

// refSeparated reports whether some axis strictly separates the exact
// projections of a and b: the two normals, the nine edge-edge cross products,
// and each edge crossed with either normal, which covers coplanar pairs and a
// collapsed triangle against a proper one.
func refSeparated(a, b [3]refInt) bool {
	na := refCross(refSub(a[1], a[0]), refSub(a[2], a[0]))
	nb := refCross(refSub(b[1], b[0]), refSub(b[2], b[0]))
	axes := []refInt{na, nb}
	for i := range 3 {
		ea, eb := refSub(a[(i+1)%3], a[i]), refSub(b[(i+1)%3], b[i])
		axes = append(axes, refCross(ea, na), refCross(ea, nb), refCross(eb, na), refCross(eb, nb))
		for j := range 3 {
			axes = append(axes, refCross(ea, refSub(b[(j+1)%3], b[j])))
		}
	}
	for _, n := range axes {
		if n[0].Sign() == 0 && n[1].Sign() == 0 && n[2].Sign() == 0 {
			continue
		}
		var loA, hiA, loB, hiB *big.Int
		for k := range 3 {
			pa, pb := refDot(n, a[k]), refDot(n, b[k])
			if loA == nil || pa.Cmp(loA) < 0 {
				loA = pa
			}
			if hiA == nil || pa.Cmp(hiA) > 0 {
				hiA = pa
			}
			if loB == nil || pb.Cmp(loB) < 0 {
				loB = pb
			}
			if hiB == nil || pb.Cmp(hiB) > 0 {
				hiB = pb
			}
		}
		if loB.Cmp(hiA) > 0 || loA.Cmp(hiB) > 0 {
			return true
		}
	}
	return false
}

// refFlat reports whether the lifted triangle t has a zero exact normal.
func refFlat(t [3]refInt) bool {
	n := refCross(refSub(t[1], t[0]), refSub(t[2], t[0]))
	return n[0].Sign() == 0 && n[1].Sign() == 0 && n[2].Sign() == 0
}

// refProper reports whether both float triangles have finite corners and a
// nonzero exact normal.
func refProper(ta, tb [3]r3.Vec) bool {
	if !refFinite(ta, tb) {
		return false
	}
	a, b, _ := refScale(ta, tb)
	return !refFlat(a) && !refFlat(b)
}

// refTriDistance2 is the exact squared distance between the closed float
// triangles ta and tb, and whether this reference decides it. A separated
// pair takes the smallest of its fifteen candidates; a pair of proper
// triangles that no axis separates intersects (the axes are complete for
// two triangles), at distance zero. A non-finite corner, or a collapsed
// triangle that no axis separates, is left undecided.
func refTriDistance2(ta, tb [3]r3.Vec) (refDist, bool) {
	if !refFinite(ta, tb) {
		return refDist{}, false
	}
	a, b, k := refScale(ta, tb)
	if !refSeparated(a, b) {
		if refFlat(a) || refFlat(b) {
			return refDist{}, false
		}
		return refDist{n: new(big.Int), d: big.NewInt(1), k: k}, true
	}
	var cands []refFrac
	for i := range 3 {
		for j := range 3 {
			cands = append(cands, refSegSeg2(a[i], a[(i+1)%3], b[j], b[(j+1)%3]))
		}
		cands = append(cands, refPointTri2(a[i], b), refPointTri2(b[i], a))
	}
	best := refMin(cands...)
	return refDist{n: best.n, d: best.d, k: k}, true
}

// refAbove reports whether the exact squared distance r exceeds slack. A
// negative slack is below every distance; a NaN or infinite one is above.
func refAbove(r refDist, slack float64) bool {
	switch {
	case math.IsNaN(slack) || math.IsInf(slack, 1):
		return false
	case slack < 0:
		return true
	case slack == 0:
		return r.n.Sign() > 0
	}
	// r.n/(r.d·4^k) > (m·2^e)² ⇔ r.n > r.d·m²·4^(k+e).
	m, e := refFloatParts(slack)
	lhs := new(big.Int).Set(r.n)
	rhs := new(big.Int).Mul(r.d, new(big.Int).Mul(big.NewInt(m), big.NewInt(m)))
	if sh := 2 * (r.k + e); sh >= 0 {
		rhs.Lsh(rhs, uint(sh))
	} else {
		lhs.Lsh(lhs, uint(-sh))
	}
	return lhs.Cmp(rhs) > 0
}

// refDistanceFloat is the exact distance read as the float64 at or above it.
func refDistanceFloat(r refDist) float64 {
	f := new(big.Float).SetPrec(256).Sqrt(new(big.Float).SetPrec(256).SetRat(r.rat()))
	v, acc := f.Float64()
	if acc == big.Below {
		v = math.Nextafter(v, math.Inf(1))
	}
	return v
}

// referenceExceeds is the far answer TriTriDistanceExceeds owes a pair of
// proper float triangles: the float reading exceeds slack and so does the
// exact distance, read from refTriDistance2. A pair the reference leaves
// undecided is near. A gap wider than slack between the two triangles'
// coordinate ranges on one axis settles the exact distance without the
// candidates. Answers are memoized per corners and slack: the gather tests
// ask for the same pairs many times.
func referenceExceeds(ta, tb [3]r3.Vec, slack float64) bool {
	if !(TriTriDistance(ta, tb) > slack) {
		return false
	}
	key := referenceKey{ta, tb, slack}
	if far, ok := referenceMemo.Load(key); ok {
		return far.(bool)
	}
	far := refFinite(ta, tb) && slack > 0 && slack <= math.MaxFloat64 && refCoordGapAbove(ta, tb, slack)
	if !far {
		d2, ok := refTriDistance2(ta, tb)
		far = ok && refAbove(d2, slack)
	}
	referenceMemo.Store(key, far)
	return far
}

// referenceKey is one referenceExceeds question. A NaN in it never matches,
// so such a question is answered afresh.
type referenceKey struct {
	ta, tb [3]r3.Vec
	slack  float64
}

var referenceMemo sync.Map

// refCoordGapAbove reports whether the exact coordinate ranges of the finite
// triangles ta and tb leave a gap wider than slack > 0 along x, y or z.
func refCoordGapAbove(ta, tb [3]r3.Vec, slack float64) bool {
	a, b, k := refScale(ta, tb)
	m, e := refFloatParts(slack)
	for c := range 3 {
		loA, hiA := a[0][c], a[0][c]
		loB, hiB := b[0][c], b[0][c]
		for i := 1; i < 3; i++ {
			if a[i][c].Cmp(loA) < 0 {
				loA = a[i][c]
			}
			if a[i][c].Cmp(hiA) > 0 {
				hiA = a[i][c]
			}
			if b[i][c].Cmp(loB) < 0 {
				loB = b[i][c]
			}
			if b[i][c].Cmp(hiB) > 0 {
				hiB = b[i][c]
			}
		}
		for _, gap := range [2]*big.Int{new(big.Int).Sub(loB, hiA), new(big.Int).Sub(loA, hiB)} {
			// gap/2^k > m·2^e ⇔ gap > m·2^(k+e).
			lhs, rhs := gap, big.NewInt(m)
			if sh := k + e; sh >= 0 {
				rhs.Lsh(rhs, uint(sh))
			} else {
				lhs = new(big.Int).Lsh(gap, uint(-sh))
			}
			if lhs.Cmp(rhs) > 0 {
				return true
			}
		}
	}
	return false
}
