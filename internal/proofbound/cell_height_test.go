package proofbound_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// Every expectation below is worked by hand from the corners. Both bounds are
// exact rational readings of float corners, so the values do not depend on
// the platform's float arithmetic.
func TestCellHeightOffsetUpper(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name               string
		vLo, vHi, wLo, wHi r3.Vec
		// twist is |T|/4 with T = vLo − vHi − wLo + wHi, or −1 where that
		// is not a float and goes unchecked.
		twist  float64
		height float64
		ok     bool
	}{
		// A trapezoid in the plane z = y: rings k and k+1 are parallel
		// chords of lengths 4 and 2. T = (−2, 0, 0), so |T|/4 = 1/2, all of
		// it movement within the plane; the fourth corner lies in the plane
		// of the other three, so the height is 0.
		{"planar trapezoid", r3.NewVec(0, 0, 0), r3.NewVec(4, 0, 0), r3.NewVec(1, 1, 1), r3.NewVec(3, 1, 1), 0.5, 0, true},
		// The unit square with its fourth corner lifted by 1/2: T = (0, 0,
		// 1/2), so |T|/4 = 1/8, and the fourth corner stands 1/2 above the
		// plane z = 0 of the other three. The height bound is the larger.
		{"lifted corner", r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(1, 1, 0.5), 0.125, 0.5, true},
		// Boundary (0,0) → (1,0) → (0,1) → (1,1) crosses itself. T = (−2, 0, 0).
		{"bowtie", r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(1, 1, 0), r3.NewVec(0, 1, 0), 0.5, 0, false},
		// Boundary (0,0) → (4,0) → (1,1) → (0,4) turns clockwise at (1,1):
		// in = (−3, 1), out = (−1, 3), in × out = −8.
		{"dart", r3.NewVec(0, 0, 0), r3.NewVec(4, 0, 0), r3.NewVec(0, 4, 0), r3.NewVec(1, 1, 0), -1, 0, false},
		// vHi − vLo and wLo − vLo are parallel, so the corners name no plane.
		// T = (0, 1, 0).
		{"collinear first three", r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(2, 0, 0), r3.NewVec(3, 1, 0), 0.25, 0, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			if tc.twist >= 0 {
				require.Equal(t, tc.twist, proofbound.CellTwistOffsetUpper(tc.vLo, tc.vHi, tc.wLo, tc.wHi))
			}
			height, ok := proofbound.CellHeightOffsetUpper(tc.vLo, tc.vHi, tc.wLo, tc.wHi)
			require.Equal(t, tc.ok, ok)
			if ok {
				require.Equal(t, tc.height, height)
			}
		})
	}
	t.Run("non-finite corner", func(t *testing.T) {
		t.Parallel()
		height, ok := proofbound.CellHeightOffsetUpper(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(math.NaN(), 1, 0))
		require.False(t, ok)
		require.True(t, math.IsInf(height, 1))
	})
}

// ratVec is an exact rational point.
type ratVec [3]*big.Rat

func ratVecOf(v r3.Vec) ratVec {
	return ratVec{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
}

func (a ratVec) sub(b ratVec) ratVec {
	return ratVec{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1]), new(big.Rat).Sub(a[2], b[2])}
}

func (a ratVec) add(b ratVec) ratVec {
	return ratVec{new(big.Rat).Add(a[0], b[0]), new(big.Rat).Add(a[1], b[1]), new(big.Rat).Add(a[2], b[2])}
}

func (a ratVec) scale(s *big.Rat) ratVec {
	return ratVec{new(big.Rat).Mul(a[0], s), new(big.Rat).Mul(a[1], s), new(big.Rat).Mul(a[2], s)}
}

func (a ratVec) dot(b ratVec) *big.Rat {
	out := new(big.Rat).Mul(a[0], b[0])
	out.Add(out, new(big.Rat).Mul(a[1], b[1]))
	return out.Add(out, new(big.Rat).Mul(a[2], b[2]))
}

// closestOnTriangle is the point of triangle abc nearest p, by the region
// tests of Ericson's Real-Time Collision Detection §5.1.5. Every step is a
// dot product or a quotient, so over rationals the answer is exact.
func closestOnTriangle(p, a, b, c ratVec) ratVec {
	ab, ac, ap := b.sub(a), c.sub(a), p.sub(a)
	d1, d2 := ab.dot(ap), ac.dot(ap)
	if d1.Sign() <= 0 && d2.Sign() <= 0 {
		return a
	}
	bp := p.sub(b)
	d3, d4 := ab.dot(bp), ac.dot(bp)
	if d3.Sign() >= 0 && d4.Cmp(d3) <= 0 {
		return b
	}
	vc := new(big.Rat).Sub(new(big.Rat).Mul(d1, d4), new(big.Rat).Mul(d3, d2))
	if vc.Sign() <= 0 && d1.Sign() >= 0 && d3.Sign() <= 0 {
		return a.add(ab.scale(new(big.Rat).Quo(d1, new(big.Rat).Sub(d1, d3))))
	}
	cp := p.sub(c)
	d5, d6 := ab.dot(cp), ac.dot(cp)
	if d6.Sign() >= 0 && d5.Cmp(d6) <= 0 {
		return c
	}
	vb := new(big.Rat).Sub(new(big.Rat).Mul(d5, d2), new(big.Rat).Mul(d1, d6))
	if vb.Sign() <= 0 && d2.Sign() >= 0 && d6.Sign() <= 0 {
		return a.add(ac.scale(new(big.Rat).Quo(d2, new(big.Rat).Sub(d2, d6))))
	}
	va := new(big.Rat).Sub(new(big.Rat).Mul(d3, d6), new(big.Rat).Mul(d5, d4))
	e43, e56 := new(big.Rat).Sub(d4, d3), new(big.Rat).Sub(d5, d6)
	if va.Sign() <= 0 && e43.Sign() >= 0 && e56.Sign() >= 0 {
		return b.add(c.sub(b).scale(new(big.Rat).Quo(e43, new(big.Rat).Add(e43, e56))))
	}
	sum := new(big.Rat).Add(va, new(big.Rat).Add(vb, vc))
	v, w := new(big.Rat).Quo(vb, sum), new(big.Rat).Quo(vc, sum)
	return a.add(ab.scale(v)).add(ac.scale(w))
}

// pairDistSq is the exact squared distance from p to the nearer of two
// triangles.
func pairDistSq(p ratVec, t0, t1 [3]ratVec) *big.Rat {
	d0 := p.sub(closestOnTriangle(p, t0[0], t0[1], t0[2]))
	d1 := p.sub(closestOnTriangle(p, t1[0], t1[1], t1[2]))
	return ratMinOf(d0.dot(d0), d1.dot(d1))
}

func ratMinOf(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) <= 0 {
		return a
	}
	return b
}

// The height bound is checked against the surfaces it claims to separate,
// on cells the two ring chords of a fillet band produce and on cells with no
// such structure. Every bilinear point Q(s, u) on a dyadic grid must lie
// within the bound of each triangle pair the cell can be split into, and the
// distance is exact rational arithmetic, so no tolerance enters the check: it
// can only find the bound too small.
func TestCellHeightOffsetUpperCoversTheBilinearPatch(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	dyadic := func(scale float64) float64 { return math.Ldexp(float64(rng.IntN(1<<12)-1<<11), -11) * scale }
	checked := 0
	for i := range 120 {
		var vLo, vHi, wLo, wHi r3.Vec
		if i%2 == 0 {
			// Two parallel chords in a tilted plane, then one corner lifted
			// off it by a small dyadic amount.
			u := r3.NewVec(1, dyadic(0.5), dyadic(0.5))
			v := r3.NewVec(dyadic(0.5), 1, dyadic(0.5))
			s0, s1, lift := 2+dyadic(1), 1+dyadic(0.5), dyadic(0.01)
			off := dyadic(0.5)
			vLo = r3.Vec{}
			vHi = u.Scale(s0)
			wLo = v.Add(u.Scale(off))
			wHi = wLo.Add(u.Scale(s1)).Add(u.Cross(v).Scale(lift))
		} else {
			vLo = r3.NewVec(dyadic(1), dyadic(1), dyadic(1))
			vHi = r3.NewVec(2+dyadic(1), dyadic(1), dyadic(1))
			wLo = r3.NewVec(dyadic(1), 2+dyadic(1), dyadic(1))
			wHi = r3.NewVec(2+dyadic(1), 2+dyadic(1), dyadic(1))
		}
		height, ok := proofbound.CellHeightOffsetUpper(vLo, vHi, wLo, wHi)
		if !ok {
			continue
		}
		checked++
		bound := new(big.Rat).SetFloat64(height)
		boundSq := new(big.Rat).Mul(bound, bound)
		a, b, A, B := ratVecOf(vLo), ratVecOf(vHi), ratVecOf(wLo), ratVecOf(wHi)
		splits := [][2][3]ratVec{
			{{a, b, B}, {a, B, A}},
			{{A, B, b}, {A, b, a}},
		}
		for si := range 5 {
			for ui := range 5 {
				s, u := big.NewRat(int64(si), 4), big.NewRat(int64(ui), 4)
				one := big.NewRat(1, 1)
				ms, mu := new(big.Rat).Sub(one, s), new(big.Rat).Sub(one, u)
				q := a.scale(new(big.Rat).Mul(ms, mu)).
					add(b.scale(new(big.Rat).Mul(s, mu))).
					add(A.scale(new(big.Rat).Mul(ms, u))).
					add(B.scale(new(big.Rat).Mul(s, u)))
				for _, split := range splits {
					d := pairDistSq(q, split[0], split[1])
					require.LessOrEqual(t, d.Cmp(boundSq), 0,
						"cell %d at s = %v, u = %v: the bilinear point is %s² from the triangles, beyond the bound %v",
						i, s, u, d.FloatString(20), height)
				}
			}
		}
	}
	require.Greater(t, checked, 90, "most sampled cells meet the convexity precondition")
}
