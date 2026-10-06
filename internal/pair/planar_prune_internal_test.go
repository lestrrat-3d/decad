package pair

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// floatBoxOf is the outward float copy of the exact box [lo, hi].
func floatBoxOf(lo, hi [3]proof.Dyadic) floatBox {
	var out floatBox
	for axis := range 3 {
		out.lo[axis], _ = floatBounds(lo[axis])
		_, out.hi[axis] = floatBounds(hi[axis])
	}
	return out
}

// pruneDyadic draws a dyadic whose mantissa usually needs more than 53 bits:
// a product of two or three random floats, sometimes scaled past the float
// range, so its float conversion rounds or overflows.
func pruneDyadic(rng *rand.Rand) proof.Dyadic {
	d := proof.MustDyOf(rng.Float64()*16 - 8)
	for range rng.IntN(3) {
		d = proof.DyMul(d, proof.MustDyOf(rng.Float64()*4-2))
	}
	switch rng.IntN(16) {
	case 0:
		d = proof.DyShift(d, 1100)
	case 1:
		d = proof.DyShift(d, -1100)
	}
	return d
}

// atMost reports whether the float f is at or below the exact d. An infinity
// compares by its sign.
func atMost(t *testing.T, f float64, d proof.Dyadic) bool {
	t.Helper()
	if math.IsInf(f, 0) {
		return f < 0
	}
	return proof.DyCmp(proof.MustDyOf(f), d) <= 0
}

// TestFloatBoundsBracketDyadic holds floatBounds to its claim lo ≤ d ≤ hi.
//
// Leg shown to fail: either Nextafter step removed, the nearest float lies on
// the wrong side of d for about half the inexact draws.
func TestFloatBoundsBracketDyadic(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(29, 31))
	inexact := 0
	for range 20000 {
		d := pruneDyadic(rng)
		lo, hi := floatBounds(d)
		require.True(t, atMost(t, lo, d), "lo %v above %v", lo, d.Rat())
		require.True(t, atMost(t, -hi, proof.DyNeg(d)), "hi %v below %v", hi, d.Rat())
		if lo != hi {
			inexact++
		}
	}
	require.Positive(t, inexact, "premise: some conversions round")
}

// TestGapSquaredBelowBoundsExactGap holds gapSquaredBelow at or below the
// exact squared gap of the exact boxes its float boxes enclose. Each rounding
// step in it is a directed operation that TestDirectedOpsBracketExact holds.
func TestGapSquaredBelowBoundsExactGap(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(37, 41))
	box := func() ([3]proof.Dyadic, [3]proof.Dyadic) {
		var lo, hi [3]proof.Dyadic
		for axis := range 3 {
			a, b := pruneDyadic(rng), pruneDyadic(rng)
			if proof.DyCmp(a, b) > 0 {
				a, b = b, a
			}
			lo[axis], hi[axis] = a, b
		}
		return lo, hi
	}
	positive := 0
	for range 20000 {
		alo, ahi := box()
		blo, bhi := box()
		below := gapSquaredBelow(floatBoxOf(alo, ahi), floatBoxOf(blo, bhi))
		require.False(t, math.IsNaN(below))
		require.True(t, atMost(t, below, boxGapSquared(alo, ahi, blo, bhi)))
		if below > 0 {
			positive++
		}
	}
	require.Positive(t, positive, "premise: some float gaps are positive")
}

// TestFracAboveBoundsFraction holds fracAbove at or above num/den.
//
// Leg shown to fail: the final Nextafter removed, the rounded quotient lies
// below the exact fraction on some draws.
func TestFracAboveBoundsFraction(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(43, 47))
	finite := 0
	for range 20000 {
		num, den := proof.DyAbs(pruneDyadic(rng)), proof.DyAbs(pruneDyadic(rng))
		if den.Sign() == 0 {
			continue
		}
		above := fracAbove(frac{num: num, den: den})
		if math.IsInf(above, 1) {
			continue
		}
		finite++
		// above ≥ num/den exactly when above·den ≥ num.
		require.GreaterOrEqual(t, proof.DyCmp(proof.DyMul(proof.MustDyOf(above), den), num), 0)
	}
	require.Positive(t, finite, "premise: some bounds are finite")
}

// TestPrunedFloatPretestKeepsAnswers holds the prune with its float pretest to
// the exact comparison it shortens, at minima placed on, just above and just
// below each pair's exact squared gap, where the floats cannot tell them
// apart, and at minima far from it, where the pretest decides.
func TestPrunedFloatPretestKeepsAnswers(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(53, 59))
	box := func() ([3]proof.Dyadic, [3]proof.Dyadic) {
		var lo, hi [3]proof.Dyadic
		for axis := range 3 {
			a := proof.DyMul(proof.MustDyOf(rng.Float64()*8-4), proof.MustDyOf(rng.Float64()+.5))
			lo[axis] = a
			hi[axis] = proof.DyAdd(a, proof.MustDyOf(rng.Float64()))
		}
		return lo, hi
	}
	tiny := proof.DyShift(proof.DyInt(1), -90)
	pretest, exact := 0, 0
	for range 4000 {
		alo, ahi := box()
		blo, bhi := box()
		gap := boxGapSquared(alo, ahi, blo, bhi)
		if gap.Sign() == 0 {
			continue
		}
		den := proof.DyAbs(pruneDyadic(rng))
		if den.Sign() == 0 {
			continue
		}
		// Minima num/den with num = gap·den·(1 + s) for each offset s.
		offsets := []proof.Dyadic{proof.DyZero(), tiny, proof.DyNeg(tiny), proof.DyShift(proof.DyInt(-1), -1),
			proof.DyInt(3)}
		for _, s := range offsets {
			num := proof.DyMul(proof.DyMul(gap, den), proof.DyAdd(proof.DyInt(1), s))
			best := frac{num: num, den: den}
			k := planarKernel{best: best, bestUp: fracAbove(best), hasBest: true}
			fa, fb := floatBoxOf(alo, ahi), floatBoxOf(blo, bhi)
			want := fracCmp(frac{num: gap, den: proof.DyInt(1)}, best) > 0
			require.Equal(t, want, k.pruned(alo, ahi, blo, bhi, fa, fb))
			if gapSquaredBelow(fa, fb) > k.bestUp {
				pretest++
			} else {
				exact++
			}
		}
	}
	require.Positive(t, pretest, "premise: the float pretest prunes some pairs")
	require.Positive(t, exact, "premise: the exact comparison decides some pairs")
}

// TestSidePlanesMatchEdgeSide holds the vertex-facet side test through
// sidePlanes to edgeSide, the test it rewrites by the scalar triple product,
// over points inside, on the edges of, and outside random triangles.
//
// Leg shown to fail: the side plane built as (w−u)×n, the reversed cross
// product, flips every nonzero sign.
func TestSidePlanesMatchEdgeSide(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(61, 67))
	coordinate := func() proof.Dyadic { return proof.MustDyOf(float64(rng.IntN(33)-16) / 4) }
	signs := map[int]int{}
	for range 400 {
		var s PlanarSolid
		for range 3 {
			s.Verts = append(s.Verts, proof.DyV3{coordinate(), coordinate(), coordinate()})
		}
		s.Tris = [][3]int{{0, 1, 2}}
		prep := preparePlanar(&s)
		planes, _ := prep.sidePlanes(0)
		// A free point, the exact midpoint of edge 0-1, and corner 2.
		sum := proof.DvAdd(s.Verts[0], s.Verts[1])
		mid := proof.DyV3{proof.DyShift(sum[0], -1), proof.DyShift(sum[1], -1), proof.DyShift(sum[2], -1)}
		for _, x := range []proof.DyV3{{coordinate(), coordinate(), coordinate()}, mid, s.Verts[2]} {
			for i := range 3 {
				u, w := s.Verts[i], s.Verts[(i+1)%3]
				want := edgeSide(prep.normal[0], u, w, dyPoint(x))
				require.Equal(t, want, proof.DvDot(proof.DvSub(x, u), planes[i]).Sign())
				signs[want]++
			}
		}
	}
	require.Positive(t, signs[1], "premise: some points lie inside an edge")
	require.Positive(t, signs[-1], "premise: some points lie outside an edge")
	require.Positive(t, signs[0], "premise: some points lie on an edge line")
}

// exactPointBox is the float box of the exact point v.
func exactPointBox(v proof.DyV3) floatBox { return floatBoxOf(v, v) }

// enclosureDyadic draws a float, or a product of floats that usually needs
// more than 53 bits, with a zero now and then.
func enclosureDyadic(rng *rand.Rand) proof.Dyadic {
	if rng.IntN(20) == 0 {
		return proof.DyZero()
	}
	d := proof.MustDyOf(rng.Float64()*8 - 4)
	if rng.IntN(2) == 0 {
		d = proof.DyMul(d, proof.MustDyOf(rng.Float64()*8-4))
	}
	return d
}

// TestDotEnclosureHoldsExactDot holds dotEnclosure's interval around the exact
// (x−u)·m, and floatDotSign's decided signs to the exact sign. Each rounding
// step in it is a directed operation that TestDirectedOpsBracketExact holds.
//
// Leg shown to fail: mulEnclosure taking its low end from mulUp and its high
// end from mulDown, the interval misses the exact dot on some draws.
func TestDotEnclosureHoldsExactDot(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(71, 73))
	vec := func() proof.DyV3 {
		return proof.DyV3{enclosureDyadic(rng), enclosureDyadic(rng), enclosureDyadic(rng)}
	}
	decided, undecided := 0, 0
	for range 20000 {
		x, u, m := vec(), vec(), vec()
		exact := proof.DvDot(proof.DvSub(x, u), m)
		lo, hi := dotEnclosure(exactPointBox(x), exactPointBox(u), exactPointBox(m))
		require.True(t, atMost(t, lo, exact), "lo %v above %v", lo, exact.Rat())
		require.True(t, atMost(t, -hi, proof.DyNeg(exact)), "hi %v below %v", hi, exact.Rat())
		sign, ok := floatDotSign(exactPointBox(x), exactPointBox(u), exactPointBox(m))
		if !ok {
			undecided++
			continue
		}
		decided++
		require.Equal(t, exact.Sign(), sign)
	}
	require.Positive(t, decided, "premise: the float pre-test decides some signs")
	require.Positive(t, undecided, "premise: some signs defer to the exact dot")
}

// TestFloatDotSignDefersAtZero holds floatDotSign undecided wherever the exact
// dot is zero, with inputs whose products round: x−u = (−b, a, c) against
// m = (a, b, 0) with a and b 53-bit floats.
func TestFloatDotSignDefersAtZero(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(79, 83))
	for range 5000 {
		a, b, c := proof.MustDyOf(rng.Float64()*8-4), proof.MustDyOf(rng.Float64()*8-4), proof.MustDyOf(rng.Float64())
		u := proof.DyV3{proof.MustDyOf(rng.Float64()), proof.MustDyOf(rng.Float64()), proof.MustDyOf(rng.Float64())}
		x := proof.DvAdd(u, proof.DyV3{proof.DyNeg(b), a, c})
		m := proof.DyV3{a, b, proof.DyZero()}
		require.Zero(t, proof.DvDot(proof.DvSub(x, u), m).Sign())
		_, ok := floatDotSign(exactPointBox(x), exactPointBox(u), exactPointBox(m))
		require.False(t, ok)
	}
}

// TestFloatDotSignUndecidedOnNaN holds an enclosure that meets an infinity
// times zero undecided rather than signed.
func TestFloatDotSignUndecidedOnNaN(t *testing.T) {
	t.Parallel()
	huge := proof.DyShift(proof.DyInt(1), 1100)
	x := proof.DyV3{huge, proof.DyInt(1), proof.DyInt(1)}
	u := proof.DyV3{}
	m := proof.DyV3{proof.DyZero(), proof.DyInt(1), proof.DyInt(1)}
	lo, hi := dotEnclosure(exactPointBox(x), exactPointBox(u), exactPointBox(m))
	require.True(t, math.IsNaN(lo) && math.IsNaN(hi), "premise: the enclosure is NaN, got [%v, %v]", lo, hi)
	_, ok := floatDotSign(exactPointBox(x), exactPointBox(u), exactPointBox(m))
	require.False(t, ok)
}

// directedOperand draws a float across the whole range: ordinary values,
// values whose products overflow or fall into the subnormals, and both signs.
func directedOperand(rng *rand.Rand) float64 {
	f := rng.Float64() + .5
	switch rng.IntN(8) {
	case 0:
		f = math.Ldexp(f, 1000+rng.IntN(23))
	case 1:
		f = math.Ldexp(f, -1000-rng.IntN(70))
	default:
		f = math.Ldexp(f, rng.IntN(40)-20)
	}
	if rng.IntN(2) == 0 {
		f = -f
	}
	return f
}

// TestDirectedOpsBracketExact holds every directed operation to its side of
// the exact result: a down result at or below it and an up result at or above
// it, across rounding, overflow and subnormal results.
//
// Legs shown to fail (each removed in turn): the outward step of addDown,
// addUp, subDown, subUp, mulDown, mulUp and divUp each lands on the wrong side
// of the exact result on some draws.
func TestDirectedOpsBracketExact(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(89, 97))
	type directed struct {
		name   string
		op     func(a, b float64) float64
		upward bool
		exact  func(a, b proof.Dyadic) proof.Dyadic
	}
	ops := []directed{
		{"addDown", addDown, false, proof.DyAdd},
		{"addUp", addUp, true, proof.DyAdd},
		{"subDown", subDown, false, proof.DySubScalar},
		{"subUp", subUp, true, proof.DySubScalar},
		{"mulDown", mulDown, false, proof.DyMul},
		{"mulUp", mulUp, true, proof.DyMul},
	}
	for range 20000 {
		a, b := directedOperand(rng), directedOperand(rng)
		da, db := proof.MustDyOf(a), proof.MustDyOf(b)
		for _, op := range ops {
			got, exact := op.op(a, b), op.exact(da, db)
			if op.upward {
				require.True(t, atMost(t, -got, proof.DyNeg(exact)), "%s(%v, %v) = %v", op.name, a, b, got)
			} else {
				require.True(t, atMost(t, got, exact), "%s(%v, %v) = %v", op.name, a, b, got)
			}
		}
		// divUp(a, b) ≥ a/b over a positive divisor exactly when
		// divUp(a, b)·b ≥ a.
		b = math.Abs(b)
		got := divUp(a, b)
		if math.IsInf(got, 1) {
			continue
		}
		require.GreaterOrEqual(t, proof.DyCmp(proof.DyMul(proof.MustDyOf(got), proof.MustDyOf(b)), da), 0,
			"divUp(%v, %v) = %v", a, b, got)
	}
}
