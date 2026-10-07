package planar

import (
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// floatBoxOf is the outward float copy of the exact box [lo, hi].
func floatBoxOf(lo, hi [3]proof.Dyadic) floatBox {
	var out floatBox
	for axis := range 3 {
		out.lo[axis], _ = proof.FloatBounds(lo[axis])
		_, out.hi[axis] = proof.FloatBounds(hi[axis])
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

// touchingBox draws an exact box whose corners usually round, and now and
// then shares a corner coordinate with other, so exactly touching faces occur.
func touchingBox(rng *rand.Rand, other [2][3]proof.Dyadic) ([3]proof.Dyadic, [3]proof.Dyadic) {
	var lo, hi [3]proof.Dyadic
	for axis := range 3 {
		a, b := pruneDyadic(rng), pruneDyadic(rng)
		switch rng.IntN(4) {
		case 0:
			a = other[1][axis]
		case 1:
			b = other[0][axis]
		}
		if proof.DyCmp(a, b) > 0 {
			a, b = b, a
		}
		lo[axis], hi[axis] = a, b
	}
	return lo, hi
}

// TestFloatApartProvesBoxesApart holds floatApart and floatApartOff to the
// exact box tests they stand in for: float boxes apart on an axis prove the
// exact boxes apart on that axis.
//
// Leg shown to fail: floatApart comparing with <= reports exactly touching
// boxes apart.
func TestFloatApartProvesBoxesApart(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(101, 103))
	apartOn := func(alo, ahi, blo, bhi [3]proof.Dyadic, axis int) bool {
		return proof.DyCmp(ahi[axis], blo[axis]) < 0 || proof.DyCmp(bhi[axis], alo[axis]) < 0
	}
	floatSaw, exactOnly := 0, 0
	for range 20000 {
		alo, ahi := touchingBox(rng, [2][3]proof.Dyadic{})
		blo, bhi := touchingBox(rng, [2][3]proof.Dyadic{alo, ahi})
		fa, fb := floatBoxOf(alo, ahi), floatBoxOf(blo, bhi)
		exact := boxesApart(alo, ahi, blo, bhi)
		if floatApart(fa, fb) {
			require.True(t, exact)
			floatSaw++
		} else if exact {
			exactOnly++
		}
		skip := rng.IntN(3)
		if floatApartOff(fa, fb, skip) {
			off := false
			for axis := range 3 {
				off = off || (axis != skip && apartOn(alo, ahi, blo, bhi, axis))
			}
			require.True(t, off, "skip %d", skip)
		}
	}
	require.Positive(t, floatSaw, "premise: the float test sees some boxes apart")
	require.Positive(t, exactOnly, "premise: the exact test alone sees some boxes apart")
}

// ratAtMost reports whether the float f is at or below the exact rational r.
// An infinity compares by its sign.
func ratAtMost(f float64, r *big.Rat) bool {
	if math.IsInf(f, 0) {
		return f < 0
	}
	return new(big.Rat).SetFloat64(f).Cmp(r) <= 0
}

// TestHpointFloatBoxHoldsPoint holds hpoint.floatBox around the exact x/w on
// every axis, over weights of both signs whose floats round, overflow or
// underflow.
//
// Legs shown to fail: divDown and divUp without their outward step each
// leave x/w outside the box on some draws, and divEnclosure built from the
// corners (a, c) and (b, d) alone misses x/w under a negative weight.
func TestHpointFloatBoxHoldsPoint(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(107, 109))
	boxed, refused := 0, 0
	for range 20000 {
		w := pruneDyadic(rng)
		if w.Sign() == 0 {
			continue
		}
		h := hpoint{x: proof.DyV3{pruneDyadic(rng), pruneDyadic(rng), pruneDyadic(rng)}, w: w}
		box, ok := h.floatBox()
		if !ok {
			refused++
			continue
		}
		boxed++
		for axis := range 3 {
			exact := new(big.Rat).Quo(h.x[axis].Rat(), h.w.Rat())
			require.True(t, ratAtMost(box.lo[axis], exact), "lo %v above %v", box.lo[axis], exact)
			require.True(t, ratAtMost(-box.hi[axis], new(big.Rat).Neg(exact)), "hi %v below %v", box.hi[axis], exact)
		}
	}
	require.Positive(t, boxed, "premise: most points get a box")
	require.Positive(t, refused, "premise: some weights' floats hold zero or overflow")
}

// TestPointInFacetFloatPretestKeepsAnswers holds pointInFacetBoxed to
// pointInFacet over rational points x/w at a triangle's corners, on its edges,
// inside it, just past its box, and away from it. Points at the corners lie
// on the float boxes' edges, where the float test cannot decide.
//
// Leg shown to fail: hpoint.floatBox taking the nearest float of each
// quotient, without the outward steps, rejects some corners.
func TestPointInFacetFloatPretestKeepsAnswers(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(113, 127))
	counts := map[string]int{}
	for trial := range 400 {
		// Odd trials hold float corners, whose float boxes have no slack, so
		// a quotient's rounding alone decides whether a corner is kept.
		coordinate := func() proof.Dyadic {
			d := proof.MustDyOf(rng.Float64()*8 - 4)
			if trial%2 == 0 {
				d = proof.DyMul(d, proof.MustDyOf(rng.Float64()+.5))
			}
			return d
		}
		var s PlanarSolid
		for range 3 {
			s.Verts = append(s.Verts, proof.DyV3{coordinate(), coordinate(), coordinate()})
		}
		s.Tris = [][3]int{{0, 1, 2}}
		prep := preparePlanar(&s)
		prep.floatBoxes()
		a, b, c := s.Verts[0], s.Verts[1], s.Verts[2]
		tiny := proof.DyShift(proof.DyInt(1), -70)
		weights := [][3]int64{{4, 0, 0}, {0, 4, 0}, {0, 0, 4}, {2, 2, 0}, {0, 2, 2}, {1, 1, 2},
			{-1, 2, 3}, {6, -1, -1}, {40, -20, -16}}
		for _, weight := range weights {
			var sum proof.DyV3
			for k := range 3 {
				sum[k] = proof.DyAdd(proof.DyAdd(proof.DyMul(proof.DyInt(weight[0]), a[k]),
					proof.DyMul(proof.DyInt(weight[1]), b[k])), proof.DyMul(proof.DyInt(weight[2]), c[k]))
			}
			past := proof.DvAdd(sum, proof.DyV3{tiny, proof.DyNeg(tiny), tiny})
			for _, point := range []proof.DyV3{sum, past} {
				// x/w = point/4 over a weight whose float rounds.
				w := proof.DyMul(proof.MustDyOf(rng.Float64()+.5), proof.MustDyOf(rng.Float64()+.5))
				if rng.IntN(2) == 0 {
					w = proof.DyNeg(w)
				}
				x := hpoint{x: dvScale(point, proof.DyShift(w, -2)), w: w}
				xb, ok := x.floatBox()
				want := pointInFacet(prep, 0, x)
				require.Equal(t, want, pointInFacetBoxed(prep, 0, x, xb, ok))
				switch {
				case ok && floatApart(xb, prep.triBox[0]):
					counts["float"]++
				case want:
					counts["inside"]++
				default:
					counts["exact"]++
				}
			}
		}
	}
	require.Positive(t, counts["float"], "premise: the float test rejects some points")
	require.Positive(t, counts["exact"], "premise: the exact tests reject some points")
	require.Positive(t, counts["inside"], "premise: some points lie in their triangle")
}

// castAlongExact is castAlong before its float pre-test: every triangle's
// exact edge signs, then its plane.
func castAlongExact(p proof.DyV3, dir [3]int64, solid *planarPrep) castResult {
	d := proof.DyV3{proof.DyInt(dir[0]), proof.DyInt(dir[1]), proof.DyInt(dir[2])}
	crossings := 0
	for t, tri := range solid.s.Tris {
		var signs [3]int
		for i := range 3 {
			u, w := solid.s.Verts[tri[i]], solid.s.Verts[tri[(i+1)%3]]
			signs[i] = proof.DvDot(d, proof.DvCross(proof.DvSub(u, p), proof.DvSub(w, p))).Sign()
		}
		if hasSign(signs, 1) && hasSign(signs, -1) {
			continue
		}
		normal := solid.normal[t]
		along := proof.DvDot(normal, d).Sign()
		ahead := proof.DvDot(normal, proof.DvSub(solid.s.Verts[tri[0]], p)).Sign()
		if along == 0 {
			if ahead != 0 {
				continue
			}
			if pointInFacet(solid, t, dyPoint(p)) {
				return castOnBoundary
			}
			return castAmbiguous
		}
		if ahead == 0 {
			return castOnBoundary
		}
		if ahead != along {
			continue
		}
		if signs[0] == 0 || signs[1] == 0 || signs[2] == 0 {
			return castAmbiguous
		}
		crossings++
	}
	if crossings%2 == 1 {
		return castInside
	}
	return castOutside
}

// TestCastAxisPretestKeepsAnswers holds castAlong with its float pre-test to
// the exact ray it shortens, on every ladder direction, over triangle sets on
// a coarse grid. The grid makes axis-aligned triangles, triangles seen
// edge-on, and points in their planes common, so every outcome of the ray
// occurs.
//
// Legs shown to fail: the pre-test without its nonzero-normal condition skips
// an edge-on triangle whose plane holds p, which the exact ray reports
// ambiguous; and the pre-test comparing along the ray's own axis too skips
// triangles the ray crosses.
func TestCastAxisPretestKeepsAnswers(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(131, 137))
	coordinate := func() proof.Dyadic {
		d := proof.DyInt(int64(rng.IntN(7) - 3))
		if rng.IntN(4) == 0 {
			d = proof.DyAdd(d, proof.DyMul(proof.MustDyOf(rng.Float64()), proof.MustDyOf(rng.Float64())))
		}
		return d
	}
	outcomes := map[castResult]int{}
	skipped := 0
	k := planarKernel{poll: func() error { return nil }}
	for range 300 {
		var s PlanarSolid
		for range 4 {
			v := len(s.Verts)
			for range 3 {
				s.Verts = append(s.Verts, proof.DyV3{coordinate(), coordinate(), coordinate()})
			}
			if rng.IntN(2) == 0 {
				// An axis-aligned triangle: one coordinate shared by all
				// three corners.
				axis := rng.IntN(3)
				s.Verts[v+1][axis], s.Verts[v+2][axis] = s.Verts[v][axis], s.Verts[v][axis]
			}
			if proof.DvIsZero(proof.DvCross(proof.DvSub(s.Verts[v+1], s.Verts[v]), proof.DvSub(s.Verts[v+2], s.Verts[v]))) {
				s.Verts = s.Verts[:v]
				continue
			}
			s.Tris = append(s.Tris, [3]int{v, v + 1, v + 2})
		}
		if len(s.Tris) == 0 {
			continue
		}
		prep := preparePlanar(&s)
		prep.floatBoxes()
		for range 8 {
			p := proof.DyV3{coordinate(), coordinate(), coordinate()}
			if rng.IntN(2) == 0 {
				// p in the plane of an axis-aligned triangle, often.
				v := s.Verts[rng.IntN(len(s.Verts))]
				axis := rng.IntN(3)
				p[axis] = v[axis]
			}
			pb := pointFloatBox(p)
			for _, dir := range rayLadder {
				want := castAlongExact(p, dir, prep)
				got, err := k.castAlong(p, pb, true, dir, prep)
				require.NoError(t, err)
				require.Equal(t, want, got, "p %v dir %v", p, dir)
				outcomes[want]++
				if axis := rayAxis(dir); axis >= 0 {
					for tri := range s.Tris {
						if prep.normal[tri][axis].Sign() != 0 && floatApartOff(pb, prep.triBox[tri], axis) {
							skipped++
						}
					}
				}
			}
		}
	}
	require.Positive(t, skipped, "premise: the pre-test skips some triangles")
	for _, outcome := range []castResult{castInside, castOutside, castOnBoundary, castAmbiguous} {
		require.Positive(t, outcomes[outcome], "premise: outcome %d occurs", outcome)
	}
}

// TestRayAxisNamesAxisRays holds rayAxis to the ladder: the six axis rays
// name their axis and every other ray none.
func TestRayAxisNamesAxisRays(t *testing.T) {
	t.Parallel()
	want := []int{0, 1, 2, 0, 1, 2}
	for i, dir := range rayLadder {
		if i < len(want) {
			require.Equal(t, want[i], rayAxis(dir), "dir %v", dir)
			continue
		}
		require.Equal(t, -1, rayAxis(dir), "dir %v", dir)
	}
}

// prunedPlain is pruned before the cap: the running minimum alone.
func (k *planarKernel) prunedPlain(alo, ahi, blo, bhi [3]proof.Dyadic, fa, fb floatBox) bool {
	if !k.hasBest {
		return false
	}
	if gapSquaredBelow(fa, fb) > k.bestUp {
		return true
	}
	if !boxesApart(alo, ahi, blo, bhi) {
		return false
	}
	if k.best.num.Sign() == 0 && k.best.den.Sign() > 0 {
		return true
	}
	return fracCmp(frac{num: boxGapSquared(alo, ahi, blo, bhi), den: proof.DyInt(1)}, k.best) > 0
}

// scanPlain is the distance scan before its hint and block boxes: every
// vertex against every facet, then every edge pair, each behind prunedPlain.
func (k *planarKernel) scanPlain() error {
	for _, side := range [][2]*planarPrep{{k.a, k.b}, {k.b, k.a}} {
		verts, tris := side[0], side[1]
		for v, point := range verts.s.Verts {
			for t := range tris.s.Tris {
				if err := k.poll(); err != nil {
					return err
				}
				if k.prunedPlain(point, point, tris.triLo[t], tris.triHi[t], verts.vertBox[v], tris.triBox[t]) {
					continue
				}
				k.cur = vertexFacetHint(verts == k.b, v, t)
				k.vertexFacet(verts, v, tris, t)
			}
		}
	}
	for ea, edgeA := range k.a.edges {
		for eb, edgeB := range k.b.edges {
			if err := k.poll(); err != nil {
				return err
			}
			if k.prunedPlain(k.a.edgeLo[ea], k.a.edgeHi[ea], k.b.edgeLo[eb], k.b.edgeHi[eb], k.a.edgeBox[ea], k.b.edgeBox[eb]) {
				continue
			}
			k.cur = PlanarHint{kind: hintEdgeEdge, i: ea, j: eb}
			k.edgeEdge(edgeA, edgeB)
		}
	}
	return nil
}

// requireSameScan holds two finished distance scans to the same minimum, as
// the same fraction, the same nearest pair and the same sites in order.
func requireSameScan(t *testing.T, want, got *planarKernel, msg string) {
	t.Helper()
	require.Equal(t, want.hasBest, got.hasBest, msg)
	if want.hasBest {
		require.Zero(t, proof.DyCmp(want.best.num, got.best.num), "%s: numerator", msg)
		require.Zero(t, proof.DyCmp(want.best.den, got.best.den), "%s: denominator", msg)
	}
	require.Equal(t, want.nearest, got.nearest, msg)
	require.Len(t, got.sites, len(want.sites), msg)
	for i := range want.sites {
		w, g := want.sites[i], got.sites[i]
		require.Equal(t, w.contact, g.contact, "%s: site %d", msg, i)
		require.Equal(t, w.cell, g.cell, "%s: site %d", msg, i)
		require.Equal(t, w.skipLocal, g.skipLocal, "%s: site %d", msg, i)
		require.Zero(t, proof.DyCmp(w.at.w, g.at.w), "%s: site %d", msg, i)
		for axis := range 3 {
			require.Zero(t, proof.DyCmp(w.at.x[axis], g.at.x[axis]), "%s: site %d", msg, i)
			require.Zero(t, proof.DyCmp(w.dir[axis], g.dir[axis]), "%s: site %d", msg, i)
		}
	}
}

// scanSoups draws two triangle sets for the distance scan: rngTris triangles
// each, corners on a half grid with some axis-aligned triangles, the second
// set lifted by lift, so equal distances, exact box gaps at the minimum and
// zero-distance sites are common. Corners that round appear now and then.
func scanSoups(rng *rand.Rand, tris int, lift proof.Dyadic) (*planarPrep, *planarPrep) {
	coordinate := func() proof.Dyadic {
		d := proof.DyShift(proof.DyInt(int64(rng.IntN(17)-8)), -1)
		if rng.IntN(16) == 0 {
			d = proof.DyAdd(d, proof.DyMul(proof.MustDyOf(rng.Float64()), proof.MustDyOf(rng.Float64())))
		}
		return d
	}
	soup := func(lift proof.Dyadic) *planarPrep {
		var s PlanarSolid
		for len(s.Tris) < tris {
			v := len(s.Verts)
			for range 3 {
				s.Verts = append(s.Verts, proof.DyV3{coordinate(), coordinate(), proof.DyAdd(coordinate(), lift)})
			}
			if rng.IntN(2) == 0 {
				axis := rng.IntN(3)
				s.Verts[v+1][axis], s.Verts[v+2][axis] = s.Verts[v][axis], s.Verts[v][axis]
			}
			if proof.DvIsZero(proof.DvCross(proof.DvSub(s.Verts[v+1], s.Verts[v]), proof.DvSub(s.Verts[v+2], s.Verts[v]))) {
				s.Verts = s.Verts[:v]
				continue
			}
			s.Tris = append(s.Tris, [3]int{v, v + 1, v + 2})
		}
		prep := preparePlanar(&s)
		prep.floatBoxes()
		return prep
	}
	return soup(proof.DyZero()), soup(lift)
}

// countingPoll returns a poll that counts its calls in count.
func countingPoll(count *int) func() error {
	return func() error {
		*count++
		return nil
	}
}

// TestDistanceScanBlocksKeepAnswers holds the distance scan with its block
// boxes to the scan without them, over triangle sets several blocks long:
// the same minimum as the same fraction, the same nearest pair, the same
// sites in order, and the same polls.
//
// Legs shown to fail: blockBoxes keeping only each block's first box skips
// blocks that hold nearer pairs, and blockPruned skipping a block without
// its polls charges fewer polls.
func TestDistanceScanBlocksKeepAnswers(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(139, 149))
	skippable, sited := 0, 0
	for trial := range 30 {
		lift := proof.DyShift(proof.DyInt(int64(rng.IntN(13))), -1)
		a, b := scanSoups(rng, 17+rng.IntN(32), lift)
		var wantPolls, gotPolls int
		want := &planarKernel{a: a, b: b, poll: countingPoll(&wantPolls)}
		require.NoError(t, want.scanPlain())
		got := &planarKernel{a: a, b: b, poll: countingPoll(&gotPolls)}
		require.NoError(t, got.scan())
		msg := fmt.Sprintf("trial %d", trial)
		requireSameScan(t, want, got, msg)
		require.Equal(t, wantPolls, gotPolls, "%s: polls", msg)
		if len(want.sites) > 0 {
			sited++
		}
		// The final bound skips some block, so the scan met skippable blocks.
		for _, vb := range a.vertBox {
			for _, block := range b.triBlock {
				if skip, _ := got.blockPruned(vb, block, 0); skip {
					skippable++
				}
			}
		}
	}
	require.Positive(t, skippable, "premise: some blocks lie past the minimum")
	require.Positive(t, sited, "premise: some scans record sites")
}

// lastAtMinimum returns the last candidate pair in scan order whose own
// squared distance equals best, and the number of such pairs.
func lastAtMinimum(a, b *planarPrep, best frac) (PlanarHint, int) {
	var last PlanarHint
	count := 0
	try := func(h PlanarHint) {
		s := &planarKernel{a: a, b: b, poll: func() error { return nil }, hint: h}
		s.applyHint()
		if s.hasCap && fracCmp(s.cap, best) == 0 {
			last = h
			count++
		}
	}
	for _, side := range [][2]*planarPrep{{a, b}, {b, a}} {
		for v := range side[0].s.Verts {
			for t := range side[1].s.Tris {
				try(vertexFacetHint(side[0] == b, v, t))
			}
		}
	}
	for ea := range a.edges {
		for eb := range b.edges {
			try(PlanarHint{kind: hintEdgeEdge, i: ea, j: eb})
		}
	}
	return last, count
}

// TestDistanceScanHintKeepsAnswers holds the distance scan under every kind
// of hint to the scan without one: no hint, a vertex-facet pair each way, an
// edge pair, the plain scan's own nearest pair, on some draws the last pair
// at the minimum, and indices out of range.
//
// Leg shown to fail: applyHint offering the hinted distance as the minimum
// records the last pair at the minimum instead of the first.
func TestDistanceScanHintKeepsAnswers(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(151, 157))
	capped, tied := 0, 0
	for trial := range 60 {
		lift := proof.DyShift(proof.DyInt(int64(rng.IntN(13))), -1)
		a, b := scanSoups(rng, 4+rng.IntN(20), lift)
		var wantPolls int
		want := &planarKernel{a: a, b: b, poll: countingPoll(&wantPolls)}
		require.NoError(t, want.scanPlain())
		hints := []PlanarHint{{},
			{kind: hintVertexFacet, i: rng.IntN(len(a.s.Verts)), j: rng.IntN(len(b.s.Tris))},
			{kind: hintFacetVertex, i: rng.IntN(len(b.s.Verts)), j: rng.IntN(len(a.s.Tris))},
			{kind: hintEdgeEdge, i: rng.IntN(len(a.edges)), j: rng.IntN(len(b.edges))},
			want.nearest,
			{kind: hintVertexFacet, i: len(a.s.Verts), j: 0},
			{kind: hintFacetVertex, i: 0, j: -1},
			{kind: hintEdgeEdge, i: 0, j: len(b.edges)},
		}
		if trial%3 == 0 && want.hasBest {
			last, count := lastAtMinimum(a, b, want.best)
			hints = append(hints, last)
			if count > 1 {
				tied++
			}
		}
		for h, hint := range hints {
			var gotPolls int
			got := &planarKernel{a: a, b: b, poll: countingPoll(&gotPolls), hint: hint}
			got.applyHint()
			if got.hasCap {
				capped++
			}
			require.NoError(t, got.scan())
			msg := fmt.Sprintf("trial %d hint %d", trial, h)
			requireSameScan(t, want, got, msg)
			require.Equal(t, wantPolls, gotPolls, "%s: polls", msg)
		}
	}
	require.Positive(t, capped, "premise: some hints set a cap")
	require.Positive(t, tied, "premise: some minima are reached by more than one pair")
}

// fineBox is the closed box [lo, lo+size] with each face cut into an n×n grid
// of squares, two triangles each, wound outward, sheared by x += sx·z.
func fineBox(lo, size [3]proof.Dyadic, n int, sx proof.Dyadic) PlanarSolid {
	var s PlanarSolid
	index := map[[3]int]int{}
	vertex := func(g [3]int) int {
		if v, ok := index[g]; ok {
			return v
		}
		var p proof.DyV3
		for axis := range 3 {
			p[axis] = proof.DyAdd(lo[axis], proof.DyMul(size[axis], ratioDyadic(g[axis], n)))
		}
		p[0] = proof.DyAdd(p[0], proof.DyMul(sx, p[2]))
		index[g] = len(s.Verts)
		s.Verts = append(s.Verts, p)
		return index[g]
	}
	for axis := range 3 {
		u, w := (axis+1)%3, (axis+2)%3
		for _, side := range []int{0, n} {
			for i := range n {
				for j := range n {
					var q [4]int
					for c, d := range [4][2]int{{0, 0}, {1, 0}, {1, 1}, {0, 1}} {
						var g [3]int
						g[axis], g[u], g[w] = side, i+d[0], j+d[1]
						q[c] = vertex(g)
					}
					// (u, w, axis) is right-handed, so q runs counterclockwise
					// seen from +axis: outward on the far side.
					if side == 0 {
						q[1], q[3] = q[3], q[1]
					}
					s.Tris = append(s.Tris, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
				}
			}
		}
	}
	return s
}

// ratioDyadic is i/n for n a power of two.
func ratioDyadic(i, n int) proof.Dyadic {
	shift := 0
	for 1<<shift < n {
		shift++
	}
	return proof.DyShift(proof.DyInt(int64(i)), -shift)
}

// TestClassifyPlanarHintKeepsAnswers holds ClassifyPlanarHinted to
// ClassifyPlanar on finely cut boxes resting on, hovering over and sinking
// into a finely cut floor, some sheared, under hints of every kind, the
// result's own Nearest among them: the same relation, reason, gap, contacts,
// crossings, nearest pair and polls.
//
// Leg shown to fail: the cap pruning a box pair whose squared gap equals it
// drops the first pair at the minimum of a box resting flat.
func TestClassifyPlanarHintKeepsAnswers(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(163, 167))
	quarter := func(lo, hi int) proof.Dyadic {
		return proof.DyShift(proof.DyInt(int64(lo+rng.IntN(hi-lo+1))), -2)
	}
	relations := map[pair.Relation]int{}
	for trial := range 40 {
		floor := fineBox([3]proof.Dyadic{proof.DyZero(), proof.DyZero(), proof.DyInt(-2)},
			[3]proof.Dyadic{proof.DyInt(4), proof.DyInt(4), proof.DyInt(2)}, 4, proof.DyZero())
		sx := proof.DyZero()
		if trial%3 == 1 {
			sx = quarter(-1, 1)
		}
		guest := fineBox([3]proof.Dyadic{quarter(-2, 12), quarter(-2, 12), quarter(-1, 2)},
			[3]proof.Dyadic{quarter(2, 6), quarter(2, 6), quarter(2, 6)}, 2, sx)
		a, b := &guest, &floor
		if trial%2 == 1 {
			a, b = b, a
		}
		for _, s := range []*PlanarSolid{a, b} {
			ok, err := CheckPlanarSolid(s, noPollInternal)
			require.NoError(t, err)
			require.True(t, ok, "trial %d: premise: an audited solid", trial)
		}
		var wantPolls int
		want, err := ClassifyPlanar(a, b, countingPoll(&wantPolls))
		require.NoError(t, err)
		relations[want.Relation]++
		edgesA, edgesB := len(want.preps[0].edges), len(want.preps[1].edges)
		hints := []PlanarHint{want.Nearest,
			{kind: hintVertexFacet, i: rng.IntN(len(a.Verts)), j: rng.IntN(len(b.Tris))},
			{kind: hintFacetVertex, i: rng.IntN(len(b.Verts)), j: rng.IntN(len(a.Tris))},
			{kind: hintEdgeEdge, i: rng.IntN(edgesA), j: rng.IntN(edgesB)},
			{kind: hintEdgeEdge, i: edgesA, j: 0},
		}
		for h, hint := range hints {
			var gotPolls int
			got, err := ClassifyPlanarHinted(a, b, hint, countingPoll(&gotPolls))
			require.NoError(t, err)
			msg := fmt.Sprintf("trial %d hint %d", trial, h)
			require.Equal(t, want.Relation, got.Relation, msg)
			require.Equal(t, want.Reason, got.Reason, msg)
			require.Equal(t, want.Gap, got.Gap, msg)
			require.Equal(t, want.Contacts, got.Contacts, msg)
			require.Equal(t, want.Crossings, got.Crossings, msg)
			require.Equal(t, want.Nearest, got.Nearest, msg)
			require.Equal(t, wantPolls, gotPolls, "%s: polls", msg)
		}
	}
	for _, relation := range []pair.Relation{pair.Separated, pair.Touching, pair.Overlapping} {
		require.Positive(t, relations[relation], "premise: relation %v occurs", relation)
	}
}
