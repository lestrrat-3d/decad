package pair

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

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
