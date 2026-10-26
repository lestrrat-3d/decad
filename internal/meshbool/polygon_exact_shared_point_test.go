package meshbool

import (
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/hwfault"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proof"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The oracle functions below are frozen copies of Cross2xSign, PlaneSide and
// SegTriOverlap2 as they were before shared-pointer points returned 0.
// SegTriOverlap2 itself is unchanged; its oracle pins it against the changed
// Cross2xSign. A later production edit must NOT be
// mirrored into them: a disagreement means production changed a verdict.

func oracleCross2xSign(a, b, c Xp2) int {
	au, av, bu, bv, cu, cv := a.Fu, a.Fv, b.Fu, b.Fv, c.Fu, c.Fv
	if a.FloatFinite && b.FloatFinite && c.FloatFinite {
		det := (bu-au)*(cv-av) - (bv-av)*(cu-au)
		scale := (proof.Abs(bu)+proof.Abs(au))*(proof.Abs(cv)+proof.Abs(av)) +
			(proof.Abs(bv)+proof.Abs(av))*(proof.Abs(cu)+proof.Abs(au))
		err := 1e-12 * scale
		if det == det && det <= math.MaxFloat64 && det >= -math.MaxFloat64 && err <= math.MaxFloat64 && err > 0 {
			if det > err {
				return 1
			}
			if det < -err {
				return -1
			}
		}
	}
	if a.Hu != nil && b.Hu != nil && c.Hu != nil {
		return cross2xHomNumerator(a, b, c).Sign()
	}
	if au == bu && a.U.Cmp(b.U) == 0 {
		return -b.V.Cmp(a.V) * c.U.Cmp(a.U)
	}
	if av == bv && a.V.Cmp(b.V) == 0 {
		return b.U.Cmp(a.U) * c.V.Cmp(a.V)
	}
	if au == cu && a.U.Cmp(c.U) == 0 {
		return b.U.Cmp(a.U) * c.V.Cmp(a.V)
	}
	if av == cv && a.V.Cmp(c.V) == 0 {
		return -b.V.Cmp(a.V) * c.U.Cmp(a.U)
	}
	return cross2xRatSign(a, b, c)
}

func oraclePlaneSide(a, b, p, n proof.Xpt) int {
	return proof.XdotSign(proof.Xcross(proof.Xsub(b, a), proof.Xsub(p, a)), n)
}

func oracleSegTriOverlap2(a, b, ta, tb, tc Xp2) bool {
	ccw := oracleCross2xSign(ta, tb, tc)
	if ccw == 0 {
		return false
	}
	edges := [3][2]Xp2{{ta, tb}, {tb, tc}, {tc, ta}}
	lo := ClipFrac{Num: big.NewInt(0), Den: big.NewInt(1)}
	hi := ClipFrac{Num: big.NewInt(1), Den: big.NewInt(1)}
	for _, e := range edges {
		sa := oracleCross2xSign(e[0], e[1], a)
		sb := oracleCross2xSign(e[0], e[1], b)
		if ccw < 0 {
			sa = -sa
			sb = -sb
		}
		switch {
		case sa >= 0 && sb >= 0:
			continue
		case sa < 0 && sb < 0:
			return false
		}
		fa, fb := EdgeCross2Fracs(e[0], e[1], a, b)
		naWb := new(big.Int).Mul(fa.Num, fb.Den)
		t := ClipFrac{
			Num: new(big.Int).Neg(naWb),
			Den: new(big.Int).Sub(new(big.Int).Mul(fb.Num, fa.Den), naWb),
		}
		if t.Den.Sign() < 0 {
			t.Num.Neg(t.Num)
			t.Den.Neg(t.Den)
		}
		if sa < 0 {
			if CmpClipFrac(t, lo) > 0 {
				lo = t
			}
			continue
		}
		if CmpClipFrac(t, hi) < 0 {
			hi = t
		}
	}
	return CmpClipFrac(lo, hi) < 0
}

// sharedPointFloats mixes zeros of both signs, a subnormal and values at the
// edge of the float filter's admitted range with plain dyadics.
var sharedPointFloats = []float64{
	0, math.Copysign(0, -1), 1, -1, 0.5, -3, 7, 0.1,
	0x1p-1074, 0x1p-300, -0x1p-300, 0x1p250, -0x1p250, 1e300, 1e-300,
}

// sharedPointLifts builds one Xp2 per supported representation for the float
// pair, so a case reaches the rational, homogeneous and float-constructed
// routes of Cross2xSign.
func sharedPointLifts(x, y float64) []Xp2 {
	return []Xp2{
		NewXP2(polynomial.MustRatOf(x), polynomial.MustRatOf(y)),
		newXP2FromFloats(x, y),
		NewXP2FromXpt(proof.XptOf(r3.Vec{X: x, Y: y}), 0, 1),
	}
}

func TestCross2xSignSharedPointersMatchOracle(t *testing.T) {
	t.Parallel()
	type pair struct{ x, y float64 }
	var pool []pair
	for _, x := range sharedPointFloats {
		for _, y := range sharedPointFloats[:9] {
			pool = append(pool, pair{x, y})
		}
	}
	rng := rand.New(rand.NewPCG(7, 11))
	for range 400 {
		p, q, r := pool[rng.IntN(len(pool))], pool[rng.IntN(len(pool))], pool[rng.IntN(len(pool))]
		for kind := range 3 {
			a := sharedPointLifts(p.x, p.y)[kind]
			b := sharedPointLifts(q.x, q.y)[kind]
			c := sharedPointLifts(r.x, r.y)[kind]
			// Every arrangement of one lift copied into two argument slots
			// shares pointers; a second lift of the same value does not.
			again := sharedPointLifts(p.x, p.y)[kind]
			for _, tri := range [][3]Xp2{
				{a, b, c}, {a, a, c}, {a, b, a}, {a, b, b}, {a, a, a}, {a, again, c}, {a, b, again},
			} {
				require.Equal(t, oracleCross2xSign(tri[0], tri[1], tri[2]),
					Cross2xSign(tri[0], tri[1], tri[2]), "kind %d", kind)
			}
		}
	}
}

// TestCross2xSignPartiallySharedPointersTakeExactPath pins the shortcut to
// ALL of a point's coordinate pointers. Two points that share U but not V (or
// Hu and Hw but not Hv) are different points, and the sign must stay exact.
func TestCross2xSignPartiallySharedPointersTakeExactPath(t *testing.T) {
	t.Parallel()
	// c lies 2^-40 off the line through a and b: exactly nonzero, but inside
	// the float filter's error band, so the exact path decides.
	u := polynomial.MustRatOf(1)
	a := NewXP2(u, polynomial.MustRatOf(0))
	b := NewXP2(u, polynomial.MustRatOf(2))
	c := NewXP2(polynomial.MustRatOf(1+0x1p-40), polynomial.MustRatOf(1))
	require.Equal(t, oracleCross2xSign(a, b, c), Cross2xSign(a, b, c))
	require.Equal(t, -1, Cross2xSign(a, b, c))

	v := polynomial.MustRatOf(1)
	d := NewXP2(polynomial.MustRatOf(0), v)
	e := NewXP2(polynomial.MustRatOf(2), v)
	f := NewXP2(polynomial.MustRatOf(1), polynomial.MustRatOf(1+0x1p-40))
	require.Equal(t, oracleCross2xSign(d, e, f), Cross2xSign(d, e, f))
	require.NotZero(t, Cross2xSign(d, e, f))

	// Same Xpt, two projection planes: Hu and Hw are shared, Hv is not.
	p := proof.XptOf(r3.Vec{X: 1, Y: 2, Z: 4})
	pxy := NewXP2FromXpt(p, 0, 1)
	pxz := NewXP2FromXpt(p, 0, 2)
	q := NewXP2FromXpt(proof.XptOf(r3.Vec{X: 1 + 0x1p-40, Y: 3, Z: 3}), 0, 1)
	require.Same(t, pxy.Hu, pxz.Hu)
	require.Same(t, pxy.Hw, pxz.Hw)
	require.NotSame(t, pxy.Hv, pxz.Hv)
	require.Equal(t, oracleCross2xSign(pxy, pxz, q), Cross2xSign(pxy, pxz, q))
	require.NotZero(t, Cross2xSign(pxy, pxz, q))
	// Same Hu and Hv, another Hw: a rescaled, different point.
	scaled := NewXP2FromXpt(proof.Xpt{X: p.X, Y: p.Y, Z: p.Z, W: new(big.Int).Add(p.W, p.W)}, 0, 1)
	require.Same(t, pxy.Hu, scaled.Hu)
	require.Same(t, pxy.Hv, scaled.Hv)
	require.NotSame(t, pxy.Hw, scaled.Hw)
	// pxy=(1,2) and scaled=(1/2,1) lie on y=2x; q2 sits 2^-40 above that line.
	q2 := NewXP2FromXpt(proof.XptOf(r3.Vec{X: 2, Y: 4 + 0x1p-40}), 0, 1)
	require.Equal(t, oracleCross2xSign(pxy, scaled, q2), Cross2xSign(pxy, scaled, q2))
	require.Equal(t, oracleCross2xSign(q2, pxy, scaled), Cross2xSign(q2, pxy, scaled))
	require.NotZero(t, Cross2xSign(pxy, scaled, q2))
	// The wrong pair order exercises b==c and a==c slots too.
	require.Equal(t, oracleCross2xSign(q, pxy, pxz), Cross2xSign(q, pxy, pxz))
	require.Equal(t, oracleCross2xSign(pxy, q, pxz), Cross2xSign(pxy, q, pxz))
}

// TestCross2xSignSharedPointerZeroNeedsNoBigInt shows the shortcut fires:
// a shared pair returns 0 without allocating, where the exact path allocates.
func TestCross2xSignSharedPointerZeroNeedsNoBigInt(t *testing.T) {
	// AllocsPerRun cannot run in a parallel test.
	for kind := range 3 {
		a := sharedPointLifts(1, 1)[kind]
		b := sharedPointLifts(1, 3)[kind]
		c := sharedPointLifts(1, 7)[kind] // collinear, so the float filter abstains
		var got int
		require.Zero(t, testing.AllocsPerRun(20, func() { got = Cross2xSign(a, a, c) }), "kind %d", kind)
		require.Zero(t, got)
		require.Zero(t, testing.AllocsPerRun(20, func() { got = Cross2xSign(a, b, b) }), "kind %d", kind)
		require.Zero(t, got)
		require.Zero(t, testing.AllocsPerRun(20, func() { got = Cross2xSign(c, b, c) }), "kind %d", kind)
		require.Zero(t, got)
	}
}

func TestPlaneSideSharedPointersMatchOracle(t *testing.T) {
	t.Parallel()
	vals := []float64{0, math.Copysign(0, -1), 1, -2, 0.5, 3, 0x1p-60, 0x1p60, 0.1}
	rng := rand.New(rand.NewPCG(3, 5))
	pick := func() r3.Vec {
		return r3.Vec{X: vals[rng.IntN(len(vals))], Y: vals[rng.IntN(len(vals))], Z: vals[rng.IntN(len(vals))]}
	}
	clone := func(p proof.Xpt) proof.Xpt {
		return proof.Xpt{X: new(big.Int).Set(p.X), Y: new(big.Int).Set(p.Y),
			Z: new(big.Int).Set(p.Z), W: new(big.Int).Set(p.W)}
	}
	for range 1500 {
		a, b, p, n := proof.XptOf(pick()), proof.XptOf(pick()), proof.XptOf(pick()), proof.XptOf(pick())
		for _, args := range [][4]proof.Xpt{
			{a, b, p, n}, {a, b, a, n}, {a, b, b, n}, {a, a, a, n}, {a, b, clone(a), n},
			{a, b, clone(b), n}, {a, a, p, n}, {a, b, n, n},
		} {
			require.Equal(t, oraclePlaneSide(args[0], args[1], args[2], args[3]),
				PlaneSide(args[0], args[1], args[2], args[3]))
		}
	}
}

// TestPlaneSideSharedCoordinatesWithDifferentWeight pins the shortcut to all
// four pointers: the same X, Y and Z pointers with another W is another point.
func TestPlaneSideSharedCoordinatesWithDifferentWeight(t *testing.T) {
	t.Parallel()
	a := proof.XptOf(r3.Vec{X: 1, Y: 0, Z: 0})
	b := proof.XptOf(r3.Vec{X: 4, Y: 1, Z: 0})
	p := proof.XptOf(r3.Vec{X: 1, Y: 1, Z: 0})
	n := proof.XptOf(r3.Vec{X: 0, Y: 0, Z: 1})
	moved := proof.Xpt{X: p.X, Y: p.Y, Z: p.Z, W: new(big.Int).Add(p.W, big.NewInt(1))}
	require.NotZero(t, oraclePlaneSide(a, b, p, n))
	require.Equal(t, oraclePlaneSide(a, b, moved, n), PlaneSide(a, b, moved, n))
	moved = proof.Xpt{X: b.X, Y: b.Y, Z: b.Z, W: new(big.Int).Add(b.W, b.W)}
	require.NotZero(t, oraclePlaneSide(a, b, moved, n))
	require.Equal(t, oraclePlaneSide(a, b, moved, n), PlaneSide(a, b, moved, n))
	moved = proof.Xpt{X: a.X, Y: a.Y, Z: a.Z, W: new(big.Int).Add(a.W, a.W)}
	require.NotZero(t, oraclePlaneSide(a, b, moved, n))
	require.Equal(t, oraclePlaneSide(a, b, moved, n), PlaneSide(a, b, moved, n))
}

func TestPlaneSideSharedPointNeedsNoExactConstruction(t *testing.T) {
	a := proof.XptOf(r3.Vec{X: 0.1, Y: 0.2, Z: 0.3})
	b := proof.XptOf(r3.Vec{X: 1.1, Y: 2.2, Z: 0.7})
	n := proof.XptOf(r3.Vec{X: 0, Y: 0, Z: 1})
	var got int
	require.Zero(t, testing.AllocsPerRun(20, func() { got = PlaneSide(a, b, a, n) }))
	require.Zero(t, got)
	require.Zero(t, testing.AllocsPerRun(20, func() { got = PlaneSide(a, b, b, n) }))
	require.Zero(t, got)
}

func TestSegTriOverlap2SharedVerticesMatchOracle(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(13, 17))
	// A 4x4 lattice: coplanar triangles and segments reuse each other's
	// vertices, equal-valued vertices appear at distinct pointers, and
	// segments end exactly on triangle edges.
	const side = 4
	for kind := range 3 {
		lifts := make([]Xp2, 0, side*side)
		for y := range side {
			for x := range side {
				lifts = append(lifts, sharedPointLifts(float64(x)/2, float64(y)/2)[kind])
			}
		}
		fresh := func(i int) Xp2 {
			return sharedPointLifts(float64(i%side)/2, float64(i/side)/2)[kind]
		}
		for range 6000 {
			var idx [5]int
			for k := range idx {
				idx[k] = rng.IntN(len(lifts))
			}
			pts := [5]Xp2{}
			for k, i := range idx {
				// Half the points are re-lifted: equal value, new pointers.
				if rng.IntN(2) == 0 {
					pts[k] = lifts[i]
					continue
				}
				pts[k] = fresh(i)
			}
			require.Equal(t,
				oracleSegTriOverlap2(pts[0], pts[1], pts[2], pts[3], pts[4]),
				SegTriOverlap2(pts[0], pts[1], pts[2], pts[3], pts[4]), "kind %d idx %v", kind, idx)
		}
	}
}

// TestSegTriOverlap2ZeroEndpointSignCases covers the clip-loop exit by name:
// a segment end on a triangle edge with the other end outside it, for both
// windings, which is the shape the sign shortcut returns on.
func TestSegTriOverlap2ZeroEndpointSignCases(t *testing.T) {
	t.Parallel()
	tris := [][3]pt2{{{0, 0}, {4, 0}, {0, 4}}, {{0, 0}, {0, 4}, {4, 0}}}
	segs := [][2]pt2{
		{{2, 0}, {2, -1}}, {{2, -1}, {2, 0}}, // ends on the bottom edge, outside
		{{2, 0}, {2, 1}}, {{2, 1}, {2, 0}}, // ends on the bottom edge, inside
		{{0, 0}, {-1, -1}}, {{-1, -1}, {0, 0}}, // ends on a vertex, outside
		{{4, 0}, {5, 0}}, {{2, 2}, {3, 3}}, {{2, 2}, {1, 1}}, // on the hypotenuse
	}
	for _, tri := range tris {
		for _, seg := range segs {
			for _, flavour := range xp2Flavours {
				a, b := flavour.lift(seg[0]), flavour.lift(seg[1])
				ta, tb, tc := flavour.lift(tri[0]), flavour.lift(tri[1]), flavour.lift(tri[2])
				require.Equal(t, oracleSegTriOverlap2(a, b, ta, tb, tc), SegTriOverlap2(a, b, ta, tb, tc),
					"%v %v %s", tri, seg, flavour.name)
			}
		}
	}
	// One pinned verdict, so a shared break of oracle and production shows.
	f := xp2Flavours[1]
	require.False(t, SegTriOverlap2(f.lift(pt2{2, 0}), f.lift(pt2{2, -1}),
		f.lift(pt2{0, 0}), f.lift(pt2{4, 0}), f.lift(pt2{0, 4})))
	require.True(t, SegTriOverlap2(f.lift(pt2{2, 0}), f.lift(pt2{2, 1}),
		f.lift(pt2{0, 0}), f.lift(pt2{4, 0}), f.lift(pt2{0, 4})))
}

// TestPlaneSideOneDifferentCoordinatePointer changes one of p's four
// coordinates to a new integer and keeps the other three pointers shared with
// a or b. Each such p is a different point, so the sign stays exact.
func TestPlaneSideOneDifferentCoordinatePointer(t *testing.T) {
	t.Parallel()
	a := proof.XptOf(r3.Vec{X: 1, Y: 2, Z: 0.5})
	b := proof.XptOf(r3.Vec{X: 4, Y: 1, Z: 2})
	n := proof.XptOf(r3.Vec{X: 1, Y: 2, Z: 3})
	nonzero := 0
	for _, base := range []proof.Xpt{a, b} {
		bump := func(v *big.Int) *big.Int { return new(big.Int).Add(v, big.NewInt(1)) }
		for _, p := range []proof.Xpt{
			{X: bump(base.X), Y: base.Y, Z: base.Z, W: base.W},
			{X: base.X, Y: bump(base.Y), Z: base.Z, W: base.W},
			{X: base.X, Y: base.Y, Z: bump(base.Z), W: base.W},
			{X: base.X, Y: base.Y, Z: base.Z, W: bump(base.W)},
		} {
			want := oraclePlaneSide(a, b, p, n)
			require.Equal(t, want, PlaneSide(a, b, p, n))
			if want != 0 {
				nonzero++
			}
		}
	}
	require.Equal(t, 8, nonzero, "every variant must be a point off the edge's plane test")
}

// outcome is what a call did: the value it returned, or the value it panicked
// with. The differential tests below compare it between the frozen old body
// and the production function, so a panic that disappears, appears or changes
// is a difference like a changed result.
func outcome(call func() any) string {
	var out string
	func() {
		defer func() {
			if r := recover(); r != nil {
				out = fmt.Sprintf("panic: %v", r)
			}
		}()
		out = fmt.Sprintf("value: %v", call())
	}()
	return out
}

// xp2Masked copies p keeping only the fields whose bit is set in mask (bit 0
// U, 1 V, 2 Hu, 3 Hv, 4 Hw). The float cache stays as it was.
func xp2Masked(p Xp2, mask int) Xp2 {
	if mask&1 == 0 {
		p.U = nil
	}
	if mask&2 == 0 {
		p.V = nil
	}
	if mask&4 == 0 {
		p.Hu = nil
	}
	if mask&8 == 0 {
		p.Hv = nil
	}
	if mask&16 == 0 {
		p.Hw = nil
	}
	return p
}

// xp2Weighted respells a homogeneous point with weight w (zero and negative
// included): numerator and weight scale together, the rational and float
// fields stay as the constructor made them.
func xp2Weighted(p Xp2, w int64) Xp2 {
	bw := big.NewInt(w)
	p.Hu = new(big.Int).Mul(p.Hu, bw)
	p.Hv = new(big.Int).Mul(p.Hv, bw)
	p.Hw = new(big.Int).Mul(p.Hw, bw)
	return p
}

// TestCross2xSignMalformedPointsMatchOracle enumerates every nil-field mask of
// a shared pair and of the third point. Shared pointers must never turn a
// panic of the old exact path into a 0.
func TestCross2xSignMalformedPointsMatchOracle(t *testing.T) {
	hwfault.SkipOnWindows(t)
	t.Parallel()
	for _, finite := range []bool{false, true} {
		for kind := range 3 {
			base := sharedPointLifts(1, 1)[kind]
			other := sharedPointLifts(3, 5)[kind]
			base.FloatFinite, other.FloatFinite = finite, finite
			for ma := range 32 {
				for mc := range 32 {
					a, c := xp2Masked(base, ma), xp2Masked(other, mc)
					relifted := xp2Masked(sharedPointLifts(1, 1)[kind], ma)
					relifted.FloatFinite = finite
					for _, tri := range [][3]Xp2{
						{a, a, c}, {a, c, a}, {c, a, a}, {a, relifted, c}, {a, a, a}, {a, c, c},
					} {
						want := outcome(func() any { return oracleCross2xSign(tri[0], tri[1], tri[2]) })
						got := outcome(func() any { return Cross2xSign(tri[0], tri[1], tri[2]) })
						require.Equal(t, want, got, "kind %d finite %v ma %d mc %d", kind, finite, ma, mc)
					}
				}
			}
		}
	}
}

func TestCross2xSignMalformedSharedPairPanicsLikeTheOldPath(t *testing.T) {
	hwfault.SkipOnWindows(t)
	t.Parallel()
	a := NewXP2(big.NewRat(0, 1), big.NewRat(0, 1))
	require.Panics(t, func() { oracleCross2xSign(a, a, Xp2{}) })
	require.Panics(t, func() { Cross2xSign(a, a, Xp2{}) })

	h := NewXP2FromXpt(proof.XptOf(r3.Vec{X: 1, Y: 1}), 0, 1)
	c := NewXP2FromXpt(proof.XptOf(r3.Vec{X: 3, Y: 5}), 0, 1)
	c.Hv = nil
	require.Panics(t, func() { oracleCross2xSign(h, h, c) })
	require.Panics(t, func() { Cross2xSign(h, h, c) })
}

func xptMasked(p proof.Xpt, mask int) proof.Xpt {
	if mask&1 == 0 {
		p.X = nil
	}
	if mask&2 == 0 {
		p.Y = nil
	}
	if mask&4 == 0 {
		p.Z = nil
	}
	if mask&8 == 0 {
		p.W = nil
	}
	return p
}

func TestPlaneSideMalformedPointsMatchOracle(t *testing.T) {
	hwfault.SkipOnWindows(t)
	t.Parallel()
	a := proof.XptOf(r3.Vec{X: 1, Y: 2, Z: 0.5})
	b := proof.XptOf(r3.Vec{X: 4, Y: 1, Z: 2})
	n := proof.XptOf(r3.Vec{X: 1, Y: 2, Z: 3})
	for ma := range 16 {
		for mb := range 16 {
			for mn := range 16 {
				ma2, mb2, mn2 := xptMasked(a, ma), xptMasked(b, mb), xptMasked(n, mn)
				for _, args := range [][4]proof.Xpt{{ma2, mb2, ma2, mn2}, {ma2, mb2, mb2, mn2}} {
					want := outcome(func() any { return oraclePlaneSide(args[0], args[1], args[2], args[3]) })
					got := outcome(func() any { return PlaneSide(args[0], args[1], args[2], args[3]) })
					require.Equal(t, want, got, "ma %d mb %d mn %d", ma, mb, mn)
				}
			}
		}
	}
}

func TestPlaneSideSharedPointWithNilNormalPanics(t *testing.T) {
	hwfault.SkipOnWindows(t)
	t.Parallel()
	a := proof.XptOf(r3.Vec{X: 1, Y: 2, Z: 0.5})
	b := proof.XptOf(r3.Vec{X: 4, Y: 1, Z: 2})
	require.Panics(t, func() { oraclePlaneSide(a, b, a, proof.Xpt{}) })
	require.Panics(t, func() { PlaneSide(a, b, a, proof.Xpt{}) })
}

// TestPlaneSideSharedPointWithOddWeightsMatchesOracle runs the shortcut with
// zero and negative weights on every point, which the old exact path accepts.
func TestPlaneSideSharedPointWithOddWeightsMatchesOracle(t *testing.T) {
	t.Parallel()
	mk := func(x, y, z, w int64) proof.Xpt {
		return proof.Xpt{X: big.NewInt(x), Y: big.NewInt(y), Z: big.NewInt(z), W: big.NewInt(w)}
	}
	for _, wa := range []int64{1, 0, -2} {
		for _, wb := range []int64{1, 0, -3} {
			for _, wn := range []int64{1, 0, -1} {
				a, b, n := mk(1, 2, 0, wa), mk(4, 1, 2, wb), mk(1, 2, 3, wn)
				for _, args := range [][4]proof.Xpt{{a, b, a, n}, {a, b, b, n}} {
					want := outcome(func() any { return oraclePlaneSide(args[0], args[1], args[2], args[3]) })
					got := outcome(func() any { return PlaneSide(args[0], args[1], args[2], args[3]) })
					require.Equal(t, want, got, "weights %d %d %d", wa, wb, wn)
				}
			}
		}
	}
}

// homPoint2 builds a homogeneous Xp2 from raw integers with no constructor,
// so a zero or negative weight survives. FloatFinite stays false.
func homPoint2(hu, hv, hw int64) Xp2 {
	return Xp2{Hu: big.NewInt(hu), Hv: big.NewInt(hv), Hw: big.NewInt(hw)}
}

// TestSegTriOverlap2ZeroWeightWitness is the review's witness: a zero weight
// on b makes the old clip form t = 0/0 and report an overlap.
func TestSegTriOverlap2ZeroWeightWitness(t *testing.T) {
	t.Parallel()
	ta, tb, tc := homPoint2(0, 0, 1), homPoint2(2, 0, 1), homPoint2(0, 2, 1)
	a, b := homPoint2(1, -1, 1), homPoint2(0, 0, 0)
	require.True(t, oracleSegTriOverlap2(a, b, ta, tb, tc))
	require.True(t, SegTriOverlap2(a, b, ta, tb, tc))
}

// TestSegTriOverlap2OddWeightsMatchOracle gives each of the five points every
// weight in {2, 0, -1} over configurations whose clip loop reaches the zero
// endpoint exit.
func TestSegTriOverlap2OddWeightsMatchOracle(t *testing.T) {
	t.Parallel()
	tris := [][3]pt2{{{0, 0}, {4, 0}, {0, 4}}, {{0, 0}, {0, 4}, {4, 0}}}
	segs := [][2]pt2{
		{{2, 0}, {2, -1}}, {{2, -1}, {2, 0}}, {{2, 0}, {2, 1}}, {{0, 0}, {-1, -1}},
		{{1, -1}, {0, 0}}, {{4, 0}, {5, 0}}, {{2, 2}, {3, 3}}, {{-1, 2}, {1, 2}},
	}
	weights := []int64{2, 0, -1}
	lift := func(p pt2, w int64) Xp2 {
		return xp2Weighted(NewXP2FromXpt(proof.XptOf(r3.Vec{X: p.x, Y: p.y}), 0, 1), w)
	}
	for _, tri := range tris {
		for _, seg := range segs {
			pts := [5]pt2{seg[0], seg[1], tri[0], tri[1], tri[2]}
			for code := range 243 {
				var xs [5]Xp2
				c := code
				for k := range xs {
					xs[k] = lift(pts[k], weights[c%3])
					c /= 3
				}
				want := outcome(func() any { return oracleSegTriOverlap2(xs[0], xs[1], xs[2], xs[3], xs[4]) })
				got := outcome(func() any { return SegTriOverlap2(xs[0], xs[1], xs[2], xs[3], xs[4]) })
				require.Equal(t, want, got, "tri %v seg %v weights code %d", tri, seg, code)
			}
		}
	}
}

// TestSegTriOverlap2MalformedPointsMatchOracle nils every field mask of every
// single point and every pair of points, for each flavour.
func TestSegTriOverlap2MalformedPointsMatchOracle(t *testing.T) {
	hwfault.SkipOnWindows(t)
	t.Parallel()
	tri := [3]pt2{{0, 0}, {4, 0}, {0, 4}}
	segs := [][2]pt2{{{2, 0}, {2, -1}}, {{2, -1}, {2, 0}}, {{2, 0}, {2, 1}}}
	for _, flavour := range xp2Flavours {
		for _, seg := range segs {
			base := [5]Xp2{
				flavour.lift(seg[0]), flavour.lift(seg[1]),
				flavour.lift(tri[0]), flavour.lift(tri[1]), flavour.lift(tri[2]),
			}
			check := func(xs [5]Xp2, what string) {
				want := outcome(func() any { return oracleSegTriOverlap2(xs[0], xs[1], xs[2], xs[3], xs[4]) })
				got := outcome(func() any { return SegTriOverlap2(xs[0], xs[1], xs[2], xs[3], xs[4]) })
				require.Equal(t, want, got, "%s %s %v", flavour.name, what, seg)
			}
			for i := range 5 {
				for mi := range 32 {
					xs := base
					xs[i] = xp2Masked(xs[i], mi)
					check(xs, fmt.Sprintf("point %d mask %d", i, mi))
					for j := i + 1; j < 5; j++ {
						for mj := range 32 {
							xs2 := xs
							xs2[j] = xp2Masked(xs2[j], mj)
							check(xs2, fmt.Sprintf("points %d,%d masks %d,%d", i, j, mi, mj))
						}
					}
				}
			}
		}
	}
}

// TestSegTriOverlap2UntrustedFloatCacheMatchesOracle corrupts the float caches
// of literal-built points (FloatFinite set, no constructor proof). The filter
// then decides signs from floats that disagree with the exact coordinates, and
// the zero-endpoint exit must still match the old clip.
func TestSegTriOverlap2UntrustedFloatCacheMatchesOracle(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(21, 23))
	const side = 4
	for kind := range 3 {
		for range 30000 {
			var xs [5]Xp2
			for k := range xs {
				x, y := float64(rng.IntN(side))/2, float64(rng.IntN(side))/2
				p := sharedPointLifts(x, y)[kind]
				if rng.IntN(2) == 0 {
					p.floatFromExact = false
					p.FloatFinite = true
					p.Fu, p.Fv = float64(rng.IntN(side))/2-0.5, float64(rng.IntN(side))/2-0.5
				}
				xs[k] = p
			}
			want := outcome(func() any { return oracleSegTriOverlap2(xs[0], xs[1], xs[2], xs[3], xs[4]) })
			got := outcome(func() any { return SegTriOverlap2(xs[0], xs[1], xs[2], xs[3], xs[4]) })
			require.Equal(t, want, got, "kind %d points %v", kind, xs)
		}
	}
}

// TestSegTriOverlap2StaleFloatCacheWitness is the re-review's witness: a
// caller rewrites the public Fv of a constructor-built point. The filter then
// reads a false float, but the old clip still runs on exact fractions (here
// 0/0) and reports an overlap.
func TestSegTriOverlap2StaleFloatCacheWitness(t *testing.T) {
	t.Parallel()
	for _, flavour := range xp2Flavours {
		ta, tb, tc := flavour.lift(pt2{0, 0}), flavour.lift(pt2{2, 0}), flavour.lift(pt2{0, 2})
		a, b := flavour.lift(pt2{0.5, 0}), flavour.lift(pt2{1, 0})
		a.Fv = -1
		require.True(t, oracleSegTriOverlap2(a, b, ta, tb, tc), flavour.name)
		require.True(t, SegTriOverlap2(a, b, ta, tb, tc), flavour.name)
	}
}

// floatCorruptions are the replacement values the sweeps write into a float
// cache field.
var floatCorruptions = []float64{0, math.Copysign(0, -1), -1, 0.25, 3, 1e300, -1e300, math.NaN(), math.Inf(1)}

// corruptXp2Floats yields p with each public float field rewritten in turn,
// leaving floatFromExact as the constructor set it.
func corruptXp2Floats(p Xp2) []Xp2 {
	var out []Xp2
	for _, v := range floatCorruptions {
		q := p
		q.Fu = v
		out = append(out, q)
		q = p
		q.Fv = v
		out = append(out, q)
	}
	q := p
	q.FloatFinite = !p.FloatFinite
	return append(out, q)
}

func TestSegTriOverlap2FloatFieldSweepMatchesOracle(t *testing.T) {
	t.Parallel()
	tri := [3]pt2{{0, 0}, {2, 0}, {0, 2}}
	segs := [][2]pt2{
		{{0.5, 0}, {1, 0}}, {{1, 0}, {0.5, 0}}, {{1, 0}, {1, -1}}, {{1, -1}, {1, 0}},
		{{1, 0}, {1, 0.5}}, {{0, 0}, {-1, -1}}, {{0.5, 0.5}, {1, 1}},
	}
	for _, flavour := range xp2Flavours {
		for _, seg := range segs {
			base := [5]Xp2{
				flavour.lift(seg[0]), flavour.lift(seg[1]),
				flavour.lift(tri[0]), flavour.lift(tri[1]), flavour.lift(tri[2]),
			}
			for i := range 5 {
				for _, q := range corruptXp2Floats(base[i]) {
					xs := base
					xs[i] = q
					want := outcome(func() any { return oracleSegTriOverlap2(xs[0], xs[1], xs[2], xs[3], xs[4]) })
					got := outcome(func() any { return SegTriOverlap2(xs[0], xs[1], xs[2], xs[3], xs[4]) })
					require.Equal(t, want, got, "%s seg %v point %d", flavour.name, seg, i)
				}
			}
		}
	}
}

func TestCross2xSignFloatFieldSweepMatchesOracle(t *testing.T) {
	t.Parallel()
	for kind := range 3 {
		for _, pts := range [][3]pt2{{{1, 1}, {3, 3}, {2, 2}}, {{1, 1}, {3, 3}, {2, 2.5}}, {{0, 0}, {1, 0}, {0, 1}}} {
			base := [3]Xp2{
				sharedPointLifts(pts[0].x, pts[0].y)[kind],
				sharedPointLifts(pts[1].x, pts[1].y)[kind],
				sharedPointLifts(pts[2].x, pts[2].y)[kind],
			}
			for i := range 3 {
				for _, q := range corruptXp2Floats(base[i]) {
					for _, shape := range [][3]int{{0, 1, 2}, {0, 0, 2}, {0, 1, 0}, {0, 1, 1}} {
						xs := base
						xs[i] = q
						args := [3]Xp2{xs[shape[0]], xs[shape[1]], xs[shape[2]]}
						// A shared slot must hold one point, floats included.
						if shape[0] == shape[1] {
							args[1] = args[0]
						}
						if shape[0] == shape[2] {
							args[2] = args[0]
						}
						if shape[1] == shape[2] {
							args[2] = args[1]
						}
						want := outcome(func() any { return oracleCross2xSign(args[0], args[1], args[2]) })
						got := outcome(func() any { return Cross2xSign(args[0], args[1], args[2]) })
						require.Equal(t, want, got, "kind %d pts %v point %d shape %v", kind, pts, i, shape)
					}
				}
			}
		}
	}
}
