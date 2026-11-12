package planar

import (
	"errors"
	"fmt"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// planarDeepVertexUnboxed is PlanarDeepVertex as it ran before it filled the
// float boxes: the same loop over the preps, with no pre-test in any cast and
// the exact box test on every triangle.
func planarDeepVertexUnboxed(a, b *PlanarSolid, margin proof.Dyadic, poll func() error) (bool, error) {
	return planarDeepVertexExactScan(preparePlanar(a), preparePlanar(b), margin, poll)
}

// planarDeepVertexExactScan runs the deep-vertex loop over the given preps
// with the exact box test on every triangle, whether or not the preps hold
// float boxes.
func planarDeepVertexExactScan(pa, pb *planarPrep, margin proof.Dyadic, poll func() error) (bool, error) {
	marginSquared := proof.DyMul(margin, margin)
	k := &planarKernel{a: pa, b: pb, poll: poll}
	for _, side := range [][2]*planarPrep{{pa, pb}, {pb, pa}} {
		from, to := side[0], side[1]
		for _, point := range from.s.Verts {
			deep, err := exactDeepInside(k, point, to, marginSquared)
			if err != nil || deep {
				return deep, err
			}
		}
	}
	return false, nil
}

// exactDeepInside is deepInside as it ran before the float skip.
func exactDeepInside(k *planarKernel, p proof.DyV3, solid *planarPrep, marginSquared proof.Dyadic) (bool, error) {
	for t := range solid.s.Tris {
		if err := k.poll(); err != nil {
			return false, err
		}
		if proof.DyCmp(boxGapSquared(p, p, solid.triLo[t], solid.triHi[t]), marginSquared) > 0 {
			continue
		}
		d := pointTriangle(solid, t, p)
		if proof.DyCmp(d.num, proof.DyMul(marginSquared, d.den)) <= 0 {
			return false, nil
		}
	}
	result, err := k.cast(p, solid)
	return result == castInside, err
}

var cubeTriangles = [][3]int{
	{0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5}, {0, 1, 5}, {0, 5, 4},
	{2, 6, 7}, {2, 7, 3}, {0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6},
}

// cubeSolid is the cube from lo to hi on every axis, each coordinate scaled
// by 2^shift.
func cubeSolid(lo, hi proof.Dyadic, shift int) PlanarSolid {
	var s PlanarSolid
	for index := range 8 {
		var v proof.DyV3
		for axis := range 3 {
			v[axis] = proof.DyShift(lo, shift)
			if index>>axis&1 == 1 {
				v[axis] = proof.DyShift(hi, shift)
			}
		}
		s.Verts = append(s.Verts, v)
	}
	s.Tris = append(s.Tris, cubeTriangles...)
	return s
}

// depthShapes draws the two solids of one configuration. Corners lie on a
// small integer grid, so faces touch, vertices sit on the other solid's
// faces and boxes touch exactly. Some corners carry a tiny dyadic offset that
// float boxes round, and the whole configuration is scaled by a power of two
// that reaches the float range's edges: tiny, subnormal, and past MaxFloat64,
// where the float bounds widen to infinities.
func depthShapes(rng *rand.Rand) (PlanarSolid, PlanarSolid) {
	scales := []int{0, 0, 0, 0, -60, 40, -1060, -1080, 1030, 1100}
	shift := scales[rng.IntN(len(scales))]
	coordinate := func() proof.Dyadic {
		d := proof.DyInt(int64(rng.IntN(9) - 2))
		if rng.IntN(8) == 0 {
			d = proof.DyAdd(d, proof.DyShift(proof.DyInt(int64(rng.IntN(5)-2)), -50-rng.IntN(20)))
		}
		return proof.DyShift(d, shift)
	}
	solid := func() PlanarSolid {
		var s PlanarSolid
		if rng.IntN(2) == 0 {
			var lo, hi [3]proof.Dyadic
			for axis := range 3 {
				lo[axis] = coordinate()
				hi[axis] = proof.DyAdd(lo[axis], proof.DyShift(proof.DyInt(int64(1+rng.IntN(5))), shift))
			}
			for index := range 8 {
				var v proof.DyV3
				for axis := range 3 {
					v[axis] = lo[axis]
					if index>>axis&1 == 1 {
						v[axis] = hi[axis]
					}
				}
				s.Verts = append(s.Verts, v)
			}
			s.Tris = append(s.Tris, cubeTriangles...)
			return s
		}
		for len(s.Tris) < 3+rng.IntN(4) {
			appendRandomTriangle(&s, rng, coordinate)
		}
		return s
	}
	return solid(), solid()
}

// appendRandomTriangle adds one non-degenerate random triangle using the
// caller's coordinate source, preserving the order of its random draws.
func appendRandomTriangle(s *PlanarSolid, rng *rand.Rand, coordinate func() proof.Dyadic) {
	v := len(s.Verts)
	for range 3 {
		s.Verts = append(s.Verts, proof.DyV3{coordinate(), coordinate(), coordinate()})
	}
	if rng.IntN(2) == 0 {
		axis := rng.IntN(3)
		s.Verts[v+1][axis], s.Verts[v+2][axis] = s.Verts[v][axis], s.Verts[v][axis]
	}
	if proof.DvIsZero(proof.DvCross(proof.DvSub(s.Verts[v+1], s.Verts[v]), proof.DvSub(s.Verts[v+2], s.Verts[v]))) {
		s.Verts = s.Verts[:v]
		return
	}
	s.Tris = append(s.Tris, [3]int{v, v + 1, v + 2})
}

// failAfter returns a poll that errors on call n and counts every call.
func failAfter(n int, count *int, stop error) func() error {
	return func() error {
		*count++
		if *count == n {
			return stop
		}
		return nil
	}
}

// TestPlanarDeepVertexFloatBoxesKeepAnswers holds PlanarDeepVertex, whose
// casts run the float pre-test, to the same loop without it: the same answer,
// the same error, and the same poll count, including when the budget ends the
// call one poll short and at the exact limit. A skipped triangle still polls,
// so the count never moves.
//
// Legs shown to fail: a float box shrunk by one ulp on its low side in
// floatBoxes, and a pre-test that skips every triangle regardless of its box,
// each fail this test.
func TestPlanarDeepVertexFloatBoxesKeepAnswers(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(2411, 3076))
	stop := errors.New("budget")
	margins := []proof.Dyadic{proof.DyZero(), proof.MustDyOf(0.25), proof.DyInt(1)}
	var deepCount, notDeepCount int
	for config := range 350 {
		a, b := depthShapes(rng)
		margin := margins[rng.IntN(len(margins))]
		var wantPolls, gotPolls int
		want, wantErr := planarDeepVertexUnboxed(&a, &b, margin, countingPoll(&wantPolls))
		got, gotErr := PlanarDeepVertex(&a, &b, margin, countingPoll(&gotPolls))
		if want != got || wantErr != nil || gotErr != nil || wantPolls != gotPolls {
			t.Fatalf("config %d: deep %v/%v, errors %v/%v, polls %v/%v (old/new)",
				config, want, got, wantErr, gotErr, wantPolls, gotPolls)
		}
		if want {
			deepCount++
		} else {
			notDeepCount++
		}
		// The budget ends the call at its last poll and one poll before.
		for _, limit := range []int{wantPolls, wantPolls - 1} {
			if limit < 1 {
				continue
			}
			var oldCount, newCount int
			oldDeep, oldErr := planarDeepVertexUnboxed(&a, &b, margin, failAfter(limit, &oldCount, stop))
			newDeep, newErr := PlanarDeepVertex(&a, &b, margin, failAfter(limit, &newCount, stop))
			if oldDeep != newDeep || !errors.Is(newErr, oldErr) || (oldErr == nil) != (newErr == nil) || oldCount != newCount {
				t.Fatalf("config %d limit %d: deep %v/%v, error %v/%v, polls %d/%d (old/new)",
					config, limit, oldDeep, newDeep, oldErr != nil, newErr != nil, oldCount, newCount)
			}
		}
	}
	require.Positive(t, deepCount, "premise: some configurations hold a deep vertex")
	require.Positive(t, notDeepCount, "premise: some configurations hold none")
}

// nearMargins returns three margins: a vertex's box gap g to a triangle, and
// g plus and minus g·2^-52. The pair is the first vertex and triangle whose
// boxes are apart on exactly one axis, so g squared is the squared box gap.
func nearMargins(a, b *PlanarSolid) []proof.Dyadic {
	pb := preparePlanar(b)
	for _, v := range a.Verts {
		for t := range b.Tris {
			var gaps []proof.Dyadic
			for axis := range 3 {
				switch {
				case proof.DyCmp(v[axis], pb.triLo[t][axis]) < 0:
					gaps = append(gaps, proof.DySubScalar(pb.triLo[t][axis], v[axis]))
				case proof.DyCmp(pb.triHi[t][axis], v[axis]) < 0:
					gaps = append(gaps, proof.DySubScalar(v[axis], pb.triHi[t][axis]))
				}
			}
			if len(gaps) == 1 {
				g := gaps[0]
				ulp := proof.DyShift(g, -52)
				return []proof.Dyadic{g, proof.DyAdd(g, ulp), proof.DySubScalar(g, ulp)}
			}
		}
	}
	return nil
}

// TestPlanarDeepVertexGapSkipKeepsAnswers holds the float skip of deepInside
// to the exact box test on every triangle: the same answer, error and poll
// count, including at the exact budget limit and one poll short, with margins
// at, just above and just below a vertex's box gap.
//
// Legs shown to fail: a skip on a bound at or above the margin (>=), and the
// lower float bound of the squared margin in place of the upper one, each
// fail this test. A limit one ulp below the upper bound fails nothing, and is
// sound: every rounding step of a positive bound is strictly downward, so the
// bound lies strictly below the exact squared gap, hence strictly below a
// squared margin the gap does not exceed.
func TestPlanarDeepVertexGapSkipKeepsAnswers(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(3081, 4242))
	stop := errors.New("budget")
	var near, deepCount int
	for config := range 600 {
		a, b := depthShapes(rng)
		margins := []proof.Dyadic{proof.DyZero(), proof.MustDyOf(0.25), proof.DyInt(1), proof.DyShift(proof.DyInt(1), -52)}
		extra := nearMargins(&a, &b)
		near += len(extra)
		margins = append(margins, extra...)
		for mi, margin := range margins {
			var wantPolls, gotPolls int
			want, wantErr := planarDeepVertexUnboxed(&a, &b, margin, countingPoll(&wantPolls))
			got, gotErr := PlanarDeepVertex(&a, &b, margin, countingPoll(&gotPolls))
			if want != got || wantErr != nil || gotErr != nil || wantPolls != gotPolls {
				t.Fatalf("config %d margin %d: deep %v/%v, errors %v/%v, polls %v/%v (old/new)",
					config, mi, want, got, wantErr, gotErr, wantPolls, gotPolls)
			}
			if want {
				deepCount++
			}
			for _, limit := range []int{wantPolls, wantPolls - 1} {
				if limit < 1 {
					continue
				}
				var oldCount, newCount int
				oldDeep, oldErr := planarDeepVertexUnboxed(&a, &b, margin, failAfter(limit, &oldCount, stop))
				newDeep, newErr := PlanarDeepVertex(&a, &b, margin, failAfter(limit, &newCount, stop))
				if oldDeep != newDeep || (oldErr == nil) != (newErr == nil) || oldCount != newCount {
					t.Fatalf("config %d margin %d limit %d: deep %v/%v, error %v/%v, polls %d/%d (old/new)",
						config, mi, limit, oldDeep, newDeep, oldErr != nil, newErr != nil, oldCount, newCount)
				}
			}
		}
	}
	require.Positive(t, near, "premise: some configurations give near-margin cases")
	require.Positive(t, deepCount, "premise: some configurations hold a deep vertex")
}

// TestPlanarDeepVertexGapSkipAtMargin holds the float skip of deepInside to
// the exact box test where the two meet: a cube nested in another, each inner
// vertex exactly one margin from three outer faces. At a margin equal to that
// gap the exact test keeps the face triangles, the vertex is not deep, and the
// float skip must keep them too; just below it the vertex is deep. A margin
// whose square is no float and the float range's ends take the same cases.
//
// Legs shown to fail: a skip on a bound at or above the margin (>=), which
// skips the touching triangles at margin 0, and the lower float bound of the
// squared margin in place of the upper one, which skips every triangle once
// the squared margin underflows, each fail this test.
func TestPlanarDeepVertexGapSkipAtMargin(t *testing.T) {
	t.Parallel()
	one, three, four := proof.DyInt(1), proof.DyInt(3), proof.DyInt(4)
	ulp := proof.DyShift(one, -52)
	odd := proof.DyAdd(one, proof.DyShift(one, -30))
	type nested struct {
		inner  [2]proof.Dyadic
		margin proof.Dyadic
		deep   bool
	}
	cases := []nested{
		{inner: [2]proof.Dyadic{one, three}, margin: one, deep: false},
		{inner: [2]proof.Dyadic{one, three}, margin: proof.DySubScalar(one, ulp), deep: true},
		{inner: [2]proof.Dyadic{one, three}, margin: proof.DyAdd(one, ulp), deep: false},
		{inner: [2]proof.Dyadic{odd, proof.DySubScalar(four, odd)}, margin: odd, deep: false},
		{inner: [2]proof.Dyadic{odd, proof.DySubScalar(four, odd)}, margin: proof.DySubScalar(odd, ulp), deep: true},
		{inner: [2]proof.Dyadic{one, three}, margin: proof.DyZero(), deep: true},
		{inner: [2]proof.Dyadic{proof.DyZero(), four}, margin: proof.DyZero(), deep: false},
	}
	stop := errors.New("budget")
	for _, shift := range []int{0, -60, -1060, 1030} {
		for ci, c := range cases {
			outer := cubeSolid(proof.DyZero(), four, shift)
			inner := cubeSolid(c.inner[0], c.inner[1], shift)
			margin := proof.DyShift(c.margin, shift)
			var wantPolls, gotPolls int
			want, wantErr := planarDeepVertexUnboxed(&inner, &outer, margin, countingPoll(&wantPolls))
			got, gotErr := PlanarDeepVertex(&inner, &outer, margin, countingPoll(&gotPolls))
			require.NoError(t, wantErr)
			require.NoError(t, gotErr)
			require.Equal(t, c.deep, want, "shift %d case %d: exact scan", shift, ci)
			require.Equal(t, c.deep, got, "shift %d case %d: float skip", shift, ci)
			require.Equal(t, wantPolls, gotPolls, "shift %d case %d: polls", shift, ci)
			for _, limit := range []int{wantPolls, wantPolls - 1} {
				var oldCount, newCount int
				oldDeep, oldErr := planarDeepVertexUnboxed(&inner, &outer, margin, failAfter(limit, &oldCount, stop))
				newDeep, newErr := PlanarDeepVertex(&inner, &outer, margin, failAfter(limit, &newCount, stop))
				require.Equal(t, oldDeep, newDeep, "shift %d case %d limit %d: deep", shift, ci, limit)
				require.Equal(t, oldErr == nil, newErr == nil, "shift %d case %d limit %d: error", shift, ci, limit)
				require.Equal(t, oldCount, newCount, "shift %d case %d limit %d: polls", shift, ci, limit)
			}
		}
	}

	// The predicate itself, for the inner corner (1, 1, 1) and an outer face
	// triangle at x = 0: an exact squared gap of 1 equal to the squared margin
	// is never skipped, and one above it by a wide step is.
	outer := cubeSolid(proof.DyZero(), four, 0)
	prep := preparePlanar(&outer)
	prep.floatBoxes()
	corner := proof.DyV3{one, one, one}
	face := 0
	require.Zero(t, proof.DyCmp(boxGapSquared(corner, corner, prep.triLo[face], prep.triHi[face]), one),
		"premise: the corner's squared box gap to the face triangle is 1")
	for _, c := range []struct {
		margin proof.Dyadic
		skip   bool
	}{{one, false}, {proof.DyAdd(one, ulp), false}, {proof.DyShift(one, -1), true}} {
		_, marginUp := proof.FloatBounds(proof.DyMul(c.margin, c.margin))
		skip := gapSquaredBelowLimit(pointFloatBox(corner), prep.triBox[face], marginUp) > marginUp
		require.Equal(t, c.skip, skip, "margin %v", c.margin)
	}
}

func TestDeepInsideExpandedBoxPrefilter(t *testing.T) {
	t.Parallel()
	for _, shift := range []int{0, -1080, 1030} {
		outer := cubeSolid(proof.DyZero(), proof.DyInt(4), shift)
		prep := preparePlanar(&outer)
		prep.floatBoxes()
		corner := proof.DyV3{
			proof.DyShift(proof.DyInt(1), shift),
			proof.DyShift(proof.DyInt(1), shift),
			proof.DyShift(proof.DyInt(1), shift),
		}
		face := 0
		gapSquared := boxGapSquared(corner, corner, prep.triLo[face], prep.triHi[face])
		require.Equal(t, 0, proof.DyCmp(gapSquared, proof.DyShift(proof.DyInt(1), 2*shift)),
			"shift %d: the exact box gap is the square of the x offset", shift)

		cases := []struct {
			name      string
			margin    proof.Dyadic
			wantApart bool
		}{
			{name: "zero", margin: proof.DyZero(), wantApart: shift != -1080},
			{name: "half gap", margin: proof.DyShift(proof.DyInt(1), shift-1), wantApart: shift == 0},
			{name: "equal gap", margin: proof.DyShift(proof.DyInt(1), shift)},
			{name: "over gap", margin: proof.DyShift(proof.DyInt(1), shift+1)},
		}
		for _, tc := range cases {
			t.Run(fmt.Sprintf("shift_%d/%s", shift, tc.name), func(t *testing.T) {
				_, radiusUp := proof.FloatBounds(tc.margin)
				expanded := expandFloatBox(pointFloatBox(corner), radiusUp)
				apart := floatApart(expanded, prep.triBox[face])
				require.Equal(t, tc.wantApart, apart)
				if apart {
					require.Greater(t, proof.DyCmp(gapSquared, proof.DyMul(tc.margin, tc.margin)), 0,
						"a box reject must prove exact squared distance exceeds margin squared")
				}
			})
		}
	}
}

func TestDeepInsideExpandedBoxPollsBeforeReject(t *testing.T) {
	t.Parallel()
	solid := cubeSolid(proof.DyZero(), proof.DyInt(1), 0)
	prep := preparePlanar(&solid)
	prep.floatBoxes()
	p := proof.DyV3{proof.DyInt(4), proof.DyInt(4), proof.DyInt(4)}
	stop := errors.New("poll before box reject")
	k := planarKernel{poll: func() error { return stop }}
	marginSquared := proof.DyZero()
	deep, err := k.deepInside(p, pointFloatBox(p), prep, marginSquared, 0, 0)
	require.ErrorIs(t, err, stop)
	require.False(t, deep)
}
