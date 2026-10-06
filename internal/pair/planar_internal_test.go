package pair

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// TestBoxesApartMatchesBoxGap holds the comparison-only prune to the exact gap
// it stands in for: boxes are apart exactly when their squared gap is
// positive. Corners on a coarse dyadic grid make touching faces and shared
// corners common.
func TestBoxesApartMatchesBoxGap(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	box := func() ([3]proof.Dyadic, [3]proof.Dyadic) {
		var lo, hi [3]proof.Dyadic
		for axis := range 3 {
			a, b := rng.IntN(9)-4, rng.IntN(9)-4
			lo[axis] = proof.DyShift(proof.DyInt(int64(min(a, b))), -rng.IntN(3))
			hi[axis] = proof.DyShift(proof.DyInt(int64(max(a, b))), 0)
			if proof.DyCmp(lo[axis], hi[axis]) > 0 {
				lo[axis], hi[axis] = hi[axis], lo[axis]
			}
		}
		return lo, hi
	}
	apart, touching := 0, 0
	for range 4000 {
		alo, ahi := box()
		blo, bhi := box()
		want := boxGapSquared(alo, ahi, blo, bhi).Sign() > 0
		require.Equal(t, want, boxesApart(alo, ahi, blo, bhi))
		if want {
			apart++
		} else {
			touching++
		}
	}
	require.Positive(t, apart, "premise: some boxes are apart")
	require.Positive(t, touching, "premise: some boxes meet")
}

// TestPrunedMatchesGapComparison holds the prune to the comparison it
// shortens, the squared box gap against the running minimum, for minima of
// every kind an offer makes: none yet, zero over a positive denominator, zero
// over a zero denominator (a degenerate facet's height), and positive.
func TestPrunedMatchesGapComparison(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(19, 23))
	box := func() ([3]proof.Dyadic, [3]proof.Dyadic) {
		var lo, hi [3]proof.Dyadic
		for axis := range 3 {
			a, b := rng.IntN(9)-4, rng.IntN(9)-4
			lo[axis], hi[axis] = proof.DyInt(int64(min(a, b))), proof.DyInt(int64(max(a, b)))
		}
		return lo, hi
	}
	minima := []*frac{nil, {num: proof.DyZero(), den: proof.DyInt(3)}, {num: proof.DyZero(), den: proof.DyZero()},
		{num: proof.DyInt(5), den: proof.DyInt(2)}, {num: proof.DyInt(1), den: proof.DyInt(64)}}
	pruned, kept := 0, 0
	for _, best := range minima {
		k := planarKernel{}
		if best != nil {
			k.best, k.hasBest = *best, true
		}
		for range 2000 {
			alo, ahi := box()
			blo, bhi := box()
			want := k.hasBest &&
				fracCmp(frac{num: boxGapSquared(alo, ahi, blo, bhi), den: proof.DyInt(1)}, k.best) > 0
			require.Equal(t, want, k.pruned(alo, ahi, blo, bhi), "minimum %+v", best)
			if want {
				pruned++
			} else {
				kept++
			}
		}
	}
	require.Positive(t, pruned, "premise: some boxes are pruned")
	require.Positive(t, kept, "premise: some boxes are kept")
}

// pointInFacetPlane is pointInFacet without its box prefilter: the plane test
// and the three edge sides alone.
func pointInFacetPlane(p *planarPrep, t int, x hpoint) bool {
	tri := p.s.Tris[t]
	if proof.DvDot(p.normal[t], x.from(p.s.Verts[tri[0]])).Sign() != 0 {
		return false
	}
	for i := range 3 {
		if edgeSide(p.normal[t], p.s.Verts[tri[i]], p.s.Verts[tri[(i+1)%3]], x) < 0 {
			return false
		}
	}
	return true
}

// TestPointInFacetBoxPrefilterKeepsAnswers holds pointInFacet to the plane
// and edge tests alone over homogeneous points of every weight sign: points in
// each triangle (its corners, edge midpoints and an interior point), points in
// its plane outside it, and points off it. Every tenth triangle is collinear,
// whose zero normal accepts every point.
func TestPointInFacetBoxPrefilterKeepsAnswers(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(13, 17))
	coordinate := func() proof.Dyadic { return proof.DyInt(int64(rng.IntN(17) - 8)) }
	inside, outside := 0, 0
	for trial := range 300 {
		var s PlanarSolid
		for range 3 {
			s.Verts = append(s.Verts, proof.DyV3{coordinate(), coordinate(), coordinate()})
		}
		if trial%10 == 0 {
			// The third corner on the line through the first two.
			s.Verts[2] = proof.DvAdd(s.Verts[1], proof.DvSub(s.Verts[1], s.Verts[0]))
		}
		s.Tris = [][3]int{{0, 1, 2}}
		prep := preparePlanar(&s)
		a, b, c := s.Verts[0], s.Verts[1], s.Verts[2]
		// Barycentric weights over 4: corners, edge midpoints, interior, and
		// in-plane points outside the triangle.
		weights := [][3]int64{{4, 0, 0}, {0, 4, 0}, {0, 0, 4}, {2, 2, 0}, {0, 2, 2}, {1, 1, 2},
			{-1, 2, 3}, {6, -1, -1}, {2, 3, -1}}
		for _, weight := range weights {
			var sum proof.DyV3
			for k := range 3 {
				sum[k] = proof.DyAdd(proof.DyAdd(proof.DyMul(proof.DyInt(weight[0]), a[k]),
					proof.DyMul(proof.DyInt(weight[1]), b[k])), proof.DyMul(proof.DyInt(weight[2]), c[k]))
			}
			off := proof.DvAdd(sum, proof.DyV3{proof.DyInt(1), proof.DyInt(-2), proof.DyInt(3)})
			for _, w := range []int64{1, 2, -4, 8} {
				// sum/4 written over the weight w: x = sum·w/4, so x/w = sum/4.
				scale := proof.DyShift(proof.DyInt(w), -2)
				for _, point := range []proof.DyV3{sum, off} {
					x := hpoint{x: dvScale(point, scale), w: proof.DyInt(w)}
					want := pointInFacetPlane(prep, 0, x)
					require.Equal(t, want, pointInFacet(prep, 0, x))
					if want {
						inside++
					} else {
						outside++
					}
				}
			}
		}
	}
	require.Positive(t, inside, "premise: some points lie in their triangle")
	require.Positive(t, outside, "premise: some points miss their triangle")
}
