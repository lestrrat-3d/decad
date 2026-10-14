package meshbool

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proof"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The oracle functions below are frozen copies of Cross2xSign, PlaneSide and
// SegTriOverlap2 as they were before shared-pointer points returned 0 and a
// zero endpoint sign ended the clip loop. A later production edit must NOT be
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

// TestSegTriOverlap2ZeroEndpointSignSkipsClip shows the early return fires:
// both orders of an endpoint on a triangle edge and the other outside the
// edge's half-plane allocate less than the frozen clip loop, which builds the
// clip fractions first.
func TestSegTriOverlap2ZeroEndpointSignSkipsClip(t *testing.T) {
	// AllocsPerRun cannot run in a parallel test.
	f := xp2Flavours[1]
	ta, tb, tc := f.lift(pt2{0, 0}), f.lift(pt2{4, 0}), f.lift(pt2{0, 4})
	on, out := f.lift(pt2{2, 0}), f.lift(pt2{2, -1})
	for _, seg := range [][2]Xp2{{on, out}, {out, on}} {
		var got bool
		fast := testing.AllocsPerRun(20, func() { got = SegTriOverlap2(seg[0], seg[1], ta, tb, tc) })
		require.False(t, got)
		slow := testing.AllocsPerRun(20, func() { got = oracleSegTriOverlap2(seg[0], seg[1], ta, tb, tc) })
		require.False(t, got)
		require.Less(t, fast, slow)
	}
}
