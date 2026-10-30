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

// cellCorners names a cell's corners as Rings.Emit passes them: vLo, vHi on
// ring k, wLo, wHi on ring k+1.
type cellCorners struct{ vLo, vHi, wLo, wHi r3.Vec }

func (c cellCorners) rat() [4]ratVec {
	return [4]ratVec{ratVecOf(c.vLo), ratVecOf(c.vHi), ratVecOf(c.wLo), ratVecOf(c.wHi)}
}

// foldedCell projects onto the plane of vLo, vHi and wLo as a folded
// quadrilateral. Its fourth corner stands about 0.0346 from that plane, while
// the bilinear patch strays about 0.07 from the triangle pair.
var foldedCell = cellCorners{
	r3.NewVec(-0.9134, -0.8767, -0.1183), r3.NewVec(0.8705, 0.5755, 0.1542),
	r3.NewVec(-0.1163, -0.1102, 0.1935), r3.NewVec(-0.0309, -0.5004, -0.6137),
}

// planarDart is the dart of TestCellHeightOffsetUpper: all four corners in
// z = 0, so its height is 0, and its boundary turns clockwise at wHi.
var planarDart = cellCorners{r3.NewVec(0, 0, 0), r3.NewVec(4, 0, 0), r3.NewVec(0, 4, 0), r3.NewVec(1, 1, 0)}

// bilinearAt is Q(s, u) = (1−s)(1−u)·vLo + s(1−u)·vHi + (1−s)u·wLo + s·u·wHi.
func bilinearAt(c [4]ratVec, s, u *big.Rat) ratVec {
	one := big.NewRat(1, 1)
	ms, mu := new(big.Rat).Sub(one, s), new(big.Rat).Sub(one, u)
	return c[0].scale(new(big.Rat).Mul(ms, mu)).
		add(c[1].scale(new(big.Rat).Mul(s, mu))).
		add(c[2].scale(new(big.Rat).Mul(ms, u))).
		add(c[3].scale(new(big.Rat).Mul(s, u)))
}

// cellSplits is the cell's two triangle pairs, one per diagonal; Rings.Emit
// writes one or the other by the band's start side.
func cellSplits(c [4]ratVec) [2][2][3]ratVec {
	a, b, A, B := c[0], c[1], c[2], c[3]
	return [2][2][3]ratVec{
		{{a, b, B}, {a, B, A}},
		{{A, B, b}, {A, b, a}},
	}
}

func (a ratVec) cross(b ratVec) ratVec {
	term := func(i, j int) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(a[i], b[j]), new(big.Rat).Mul(a[j], b[i]))
	}
	return ratVec{term(1, 2), term(2, 0), term(0, 1)}
}

// exactHeightSq is H² = (n·(wHi − vLo))² / (n·n) with n = (vHi − vLo) ×
// (wLo − vLo), computed here from the corners alone.
func exactHeightSq(c [4]ratVec) *big.Rat {
	n := c[1].sub(c[0]).cross(c[2].sub(c[0]))
	d := n.dot(c[3].sub(c[0]))
	return new(big.Rat).Quo(new(big.Rat).Mul(d, d), n.dot(n))
}

// gridQuarters is the dyadic grid {0, 1/4, …, 1}.
func gridQuarters() []*big.Rat {
	out := make([]*big.Rat, 5)
	for i := range out {
		out[i] = big.NewRat(int64(i), 4)
	}
	return out
}

// bilinearWitness is a point of the bilinear patch near y: the point above
// y's projection onto the plane of vLo, vHi and wLo, found by Newton's method
// in float64 and then refined in 200-bit floats, with its parameters clamped
// to the unit square and then read as exact rationals. Any point of the patch bounds y's distance to the
// patch from above, so the witness only has to be on the patch, which it is.
func bilinearWitness(c [4]ratVec, y ratVec) ratVec {
	const prec = 200
	fl := func(r *big.Rat) *big.Float { return new(big.Float).SetPrec(prec).SetRat(r) }
	type fvec [3]*big.Float
	of := func(v ratVec) fvec { return fvec{fl(v[0]), fl(v[1]), fl(v[2])} }
	sub := func(a, b fvec) fvec {
		var out fvec
		for i := range out {
			out[i] = new(big.Float).SetPrec(prec).Sub(a[i], b[i])
		}
		return out
	}
	dot := func(a, b fvec) *big.Float {
		out := new(big.Float).SetPrec(prec)
		for i := range a {
			out.Add(out, new(big.Float).SetPrec(prec).Mul(a[i], b[i]))
		}
		return out
	}
	mul := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Mul(a, b) }
	add := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Add(a, b) }
	scale := func(v fvec, k *big.Float) fvec { return fvec{mul(v[0], k), mul(v[1], k), mul(v[2], k)} }
	vadd := func(a, b fvec) fvec { return fvec{add(a[0], b[0]), add(a[1], b[1]), add(a[2], b[2])} }
	a, b, A, B := of(c[0]), of(c[1]), of(c[2]), of(c[3])
	t1 := sub(b, a)
	t2 := of(c[1].sub(c[0]).cross(c[2].sub(c[0])).cross(c[1].sub(c[0])))
	e1, e2, tw := sub(b, a), sub(A, a), sub(sub(B, b), sub(A, a))
	py := of(y)
	s0, u0 := float64Witness(c, y)
	s := new(big.Float).SetPrec(prec).SetFloat64(s0)
	u := new(big.Float).SetPrec(prec).SetFloat64(u0)
	for range 4 {
		q := vadd(vadd(a, scale(e1, s)), vadd(scale(e2, u), scale(tw, mul(s, u))))
		r := sub(q, py)
		f1, f2 := dot(r, t1), dot(r, t2)
		qs, qu := vadd(e1, scale(tw, u)), vadd(e2, scale(tw, s))
		j11, j12, j21, j22 := dot(qs, t1), dot(qu, t1), dot(qs, t2), dot(qu, t2)
		det := new(big.Float).SetPrec(prec).Sub(mul(j11, j22), mul(j12, j21))
		if det.Sign() == 0 {
			break
		}
		ds := new(big.Float).SetPrec(prec).Quo(new(big.Float).SetPrec(prec).Sub(mul(j22, f1), mul(j12, f2)), det)
		du := new(big.Float).SetPrec(prec).Quo(new(big.Float).SetPrec(prec).Sub(mul(j11, f2), mul(j21, f1)), det)
		s.Sub(s, ds)
		u.Sub(u, du)
	}
	clamp := func(x *big.Float) *big.Rat {
		r, _ := x.Rat(nil)
		if r.Sign() < 0 {
			return new(big.Rat)
		}
		if r.Cmp(big.NewRat(1, 1)) > 0 {
			return big.NewRat(1, 1)
		}
		return r
	}
	return bilinearAt(c, clamp(s), clamp(u))
}

// float64Witness is bilinearWitness's Newton iteration in float64, the
// starting parameters of its refinement.
func float64Witness(c [4]ratVec, y ratVec) (float64, float64) {
	vec := func(v ratVec) r3.Vec {
		x, _ := v[0].Float64()
		yy, _ := v[1].Float64()
		z, _ := v[2].Float64()
		return r3.NewVec(x, yy, z)
	}
	a, b, A, B, p := vec(c[0]), vec(c[1]), vec(c[2]), vec(c[3]), vec(y)
	e1, e2 := b.Sub(a), A.Sub(a)
	tw := B.Sub(b).Sub(e2)
	t1, t2 := e1, e1.Cross(e2).Cross(e1)
	s, u := 0.5, 0.5
	for range 40 {
		r := a.Add(e1.Scale(s)).Add(e2.Scale(u)).Add(tw.Scale(s * u)).Sub(p)
		qs, qu := e1.Add(tw.Scale(u)), e2.Add(tw.Scale(s))
		j11, j12, j21, j22 := qs.Dot(t1), qu.Dot(t1), qs.Dot(t2), qu.Dot(t2)
		det := j11*j22 - j12*j21
		if det == 0 {
			break
		}
		f1, f2 := r.Dot(t1), r.Dot(t2)
		s -= (j22*f1 - j12*f2) / det
		u -= (j11*f2 - j21*f1) / det
	}
	return s, u
}

// A folded cell fails the height bound's precondition, and the bound it would
// give is too small: a bilinear point lies farther from the triangle pair
// than the fourth corner's height. The breach is read in exact rational
// arithmetic, so it shows why the precondition gates the bound.
func TestCellHeightOffsetUpperRefusesFoldedCells(t *testing.T) {
	t.Parallel()
	t.Run("planar dart", func(t *testing.T) {
		t.Parallel()
		c := planarDart.rat()
		_, ok := proofbound.CellHeightOffsetUpper(planarDart.vLo, planarDart.vHi, planarDart.wLo, planarDart.wHi)
		require.False(t, ok)
		require.Zero(t, exactHeightSq(c).Sign(), "every corner lies in z = 0")
		// Q(1/2, 1/2) is the corners' mean (5/4, 5/4, 0), beyond the reflex
		// corner (1, 1). Its offset (1/4, 1/4) projects on the edge toward
		// (4, 0), direction (3, −1)/√10, at 1/(2√10) inside the edge, so
		// its squared distance is 1/8 − 1/40 = 1/10.
		q := bilinearAt(c, big.NewRat(1, 2), big.NewRat(1, 2))
		split := cellSplits(c)[0]
		require.Zero(t, pairDistSq(q, split[0], split[1]).Cmp(big.NewRat(1, 10)))
	})
	t.Run("folded cell", func(t *testing.T) {
		t.Parallel()
		c := foldedCell.rat()
		_, ok := proofbound.CellHeightOffsetUpper(foldedCell.vLo, foldedCell.vHi, foldedCell.wLo, foldedCell.wHi)
		require.False(t, ok)
		// On the split along the vHi–wLo diagonal, which Rings.Emit writes
		// on a band's start side, the farthest point of the patch on the
		// 1/32 grid is Q(3/8, 19/32), about 0.0715 from both triangles,
		// against H ≈ 0.0348.
		hSq := exactHeightSq(c)
		split := cellSplits(c)[1]
		d := pairDistSq(bilinearAt(c, big.NewRat(3, 8), big.NewRat(19, 32)), split[0], split[1])
		require.Positive(t, d.Cmp(hSq), "Q(3/8, 19/32) is %s² from the triangles, within H² = %s",
			d.FloatString(6), hSq.FloatString(6))
	})
}

// The height bound is checked against the surfaces it claims to separate,
// in both directions, on cells the two ring chords of a fillet band produce,
// on cells with no such structure, and on the two folded cells above, which
// the precondition must refuse. For a cell the precondition admits, every
// bilinear point Q(s, u) on a dyadic grid must lie within the bound of each
// triangle pair the cell can be split into, and every point of each triangle
// on a barycentric grid must lie within the bound of a witness point of the
// patch. Every distance is exact rational arithmetic, so no tolerance enters
// the check: it can only find the bound too small.
func TestCellHeightOffsetUpperCoversTheBilinearPatch(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	dyadic := func(scale float64) float64 { return math.Ldexp(float64(rng.IntN(1<<12)-1<<11), -11) * scale }
	cells := []cellCorners{foldedCell, planarDart}
	for i := range 64 {
		if i%2 == 0 {
			// Two parallel chords in a tilted plane, then one corner lifted
			// off it by a small dyadic amount.
			u := r3.NewVec(1, dyadic(0.5), dyadic(0.5))
			v := r3.NewVec(dyadic(0.5), 1, dyadic(0.5))
			s0, s1, lift := 2+dyadic(1), 1+dyadic(0.5), dyadic(0.01)
			off := dyadic(0.5)
			wLo := v.Add(u.Scale(off))
			cells = append(cells, cellCorners{r3.Vec{}, u.Scale(s0), wLo, wLo.Add(u.Scale(s1)).Add(u.Cross(v).Scale(lift))})
			continue
		}
		cells = append(cells, cellCorners{
			r3.NewVec(dyadic(1), dyadic(1), dyadic(1)), r3.NewVec(2+dyadic(1), dyadic(1), dyadic(1)),
			r3.NewVec(dyadic(1), 2+dyadic(1), dyadic(1)), r3.NewVec(2+dyadic(1), 2+dyadic(1), dyadic(1)),
		})
	}
	thirds := []*big.Rat{big.NewRat(0, 1), big.NewRat(1, 3), big.NewRat(2, 3), big.NewRat(1, 1)}
	checked, refused := 0, 0
	for i, cell := range cells {
		height, ok := proofbound.CellHeightOffsetUpper(cell.vLo, cell.vHi, cell.wLo, cell.wHi)
		if !ok {
			refused++
			continue
		}
		checked++
		bound := new(big.Rat).SetFloat64(height)
		boundSq := new(big.Rat).Mul(bound, bound)
		c := cell.rat()
		for si, split := range cellSplits(c) {
			for _, s := range gridQuarters() {
				for _, u := range gridQuarters() {
					d := pairDistSq(bilinearAt(c, s, u), split[0], split[1])
					require.LessOrEqual(t, d.Cmp(boundSq), 0,
						"cell %d, split %d: Q(%v, %v) is %s² from the triangles, beyond the bound %v",
						i, si, s, u, d.FloatString(20), height)
				}
			}
			if height == 0 {
				// A zero bound needs a witness on the patch exactly, which
				// the Newton step does not give.
				continue
			}
			for _, tri := range split {
				for _, x := range thirds {
					for _, y := range thirds {
						if new(big.Rat).Add(x, y).Cmp(big.NewRat(1, 1)) > 0 {
							continue
						}
						p := tri[0].add(tri[1].sub(tri[0]).scale(x)).add(tri[2].sub(tri[0]).scale(y))
						w := p.sub(bilinearWitness(c, p))
						d := w.dot(w)
						require.LessOrEqual(t, d.Cmp(boundSq), 0,
							"cell %d, split %d: a triangle point is %s² from its patch witness, beyond the bound %v",
							i, si, d.FloatString(20), height)
					}
				}
			}
		}
	}
	require.Greater(t, checked, 45, "most sampled cells meet the convexity precondition")
	require.Greater(t, refused, 2, "the folded cells and some random ones are refused")
}
