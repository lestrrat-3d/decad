package planar

import (
	"errors"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// planarDeepVertexUnboxed is PlanarDeepVertex as it ran before it filled the
// float boxes: the same loop over the preps, with no pre-test in any cast.
func planarDeepVertexUnboxed(a, b *PlanarSolid, margin proof.Dyadic, poll func() error) (bool, error) {
	marginSquared := proof.DyMul(margin, margin)
	pa, pb := preparePlanar(a), preparePlanar(b)
	k := &planarKernel{a: pa, b: pb, poll: poll}
	for _, side := range [][2]*planarPrep{{pa, pb}, {pb, pa}} {
		from, to := side[0], side[1]
		for _, point := range from.s.Verts {
			deep, err := k.deepInside(point, to, marginSquared)
			if err != nil || deep {
				return deep, err
			}
		}
	}
	return false, nil
}

var cubeTriangles = [][3]int{
	{0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5}, {0, 1, 5}, {0, 5, 4},
	{2, 6, 7}, {2, 7, 3}, {0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6},
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
				continue
			}
			s.Tris = append(s.Tris, [3]int{v, v + 1, v + 2})
		}
		return s
	}
	return solid(), solid()
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
