package loftmesh

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

const patchBoundaryAllowance = uint64(100_000_000)

type patchRim struct {
	name string
	pts  [][2]float64
	// want is the condition both checks must return.
	want ProjectedPatchCondition
}

// rimPoints places a planar rim on two coordinate axes of 3D space and
// projects it back, so the points carry the same held floats the proof reads.
func rimPoints(rim [][2]float64, axes [2]int) []meshbool.Xp2 {
	points := make([]meshbool.Xp2, len(rim))
	for i, p := range rim {
		var v [3]float64
		v[axes[0]], v[axes[1]] = p[0], p[1]
		points[i] = patchProjectedPoint(r3.Vec{X: v[0], Y: v[1], Z: v[2]}, axes)
	}
	return points
}

func circleRim(n int, rng *rand.Rand, jitter float64) [][2]float64 {
	rim := make([][2]float64, n)
	for i := range rim {
		angle := 2 * math.Pi * float64(i) / float64(n)
		radius := 1.0
		if jitter > 0 {
			radius += jitter * (rng.Float64() - 0.5)
		}
		rim[i] = [2]float64{radius * math.Cos(angle), radius * math.Sin(angle)}
	}
	return rim
}

func patchRims() []patchRim {
	rng := rand.New(rand.NewPCG(83, 89))
	rims := []patchRim{
		{"convex 3", circleRim(3, rng, 0), ProjectedPatchHolds},
		{"convex 4", circleRim(4, rng, 0), ProjectedPatchHolds},
		{"convex 17", circleRim(17, rng, 0), ProjectedPatchHolds},
		{"convex 400", circleRim(400, rng, 0), ProjectedPatchHolds},
		{"star 17", circleRim(17, rng, 1), ProjectedPatchHolds},
		{"star 400", circleRim(400, rng, 1), ProjectedPatchHolds},
		{"bow tie", [][2]float64{{0, 0}, {2, 2}, {2, 0}, {0, 2}}, ProjectedPatchSelfContact},
		// Vertex (1,1) lies inside the vertical edge from (1,0) to (1,2). The
		// edges that meet there have low u end 1 (and 0.5), and the vertical
		// edge's high u end is 1: the touch sits exactly on the sweep's stop.
		{"T contact on vertical edge",
			[][2]float64{{1, 0}, {1, 2}, {3, 3}, {1, 1}, {0.5, 0.5}}, ProjectedPatchSelfContact},
		// Non-adjacent edges (0,0)-(1,1) and (3,1)-(1,1) meet at one point
		// that is one edge's high u end and the other's low u end.
		{"touch at u ends",
			[][2]float64{{0, 0}, {1, 1}, {2, 3}, {3, 1}, {1, 1}, {0, 2}}, ProjectedPatchSelfContact},
		// Collinear fold-back: the two adjacent edges overlap, and every pair
		// of edges of a triangle is adjacent.
		{"adjacent fold back", [][2]float64{{0, 0}, {2, 0}, {1, 0}}, ProjectedPatchSelfContact},
		{"repeated point", [][2]float64{{0, 0}, {2, 0}, {1, 1}, {2, 0}}, ProjectedPatchSelfContact},
		{"comb with vertical edges",
			[][2]float64{{0, 0}, {1, 0}, {1, 2}, {2, 2}, {2, 0}, {3, 0}, {3, 3}, {0, 3}}, ProjectedPatchHolds},
		{"many edges on one u",
			[][2]float64{{0, 0}, {4, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4}, {0, 4}, {0, 3}, {0, 2}, {0, 1}},
			ProjectedPatchHolds},
	}
	return rims
}

func runPatchBoundary(t *testing.T, check func(*proofbound.WorkBudget, []meshbool.Xp2, *uint64) ProjectedPatchCondition,
	points []meshbool.Xp2, allowance uint64) (ProjectedPatchCondition, uint64) {
	t.Helper()
	remaining := allowance
	got := check(proofbound.NewWorkBudget(t.Context()), points, &remaining)
	return got, allowance - remaining
}

func TestPatchBoundarySweepMatchesQuadratic(t *testing.T) {
	axesList := [][2]int{{0, 1}, {1, 2}, {0, 2}}
	for _, axes := range axesList {
		for _, rim := range patchRims() {
			t.Run(rim.name, func(t *testing.T) {
				points := rimPoints(rim.pts, axes)
				want, _ := runPatchBoundary(t, patchBoundarySimpleQuadratic, points, patchBoundaryAllowance)
				require.Equal(t, rim.want, want, "quadratic reference on axes %v", axes)
				got, _ := runPatchBoundary(t, patchBoundarySimple, points, patchBoundaryAllowance)
				require.Equal(t, want, got, "axes %v", axes)
			})
		}
	}
	t.Run("random rims", func(t *testing.T) {
		rng := rand.New(rand.NewPCG(83, 89))
		seen := map[ProjectedPatchCondition]int{}
		for trial := range 200 {
			n := 3 + rng.IntN(10)
			rim := make([][2]float64, n)
			for i := range rim {
				// A coarse grid forces equal u values and touching edges.
				rim[i] = [2]float64{float64(rng.IntN(6)), float64(rng.IntN(6))}
			}
			axes := axesList[trial%len(axesList)]
			points := rimPoints(rim, axes)
			want, _ := runPatchBoundary(t, patchBoundarySimpleQuadratic, points, patchBoundaryAllowance)
			got, _ := runPatchBoundary(t, patchBoundarySimple, points, patchBoundaryAllowance)
			require.Equal(t, want, got, "trial %d rim %v axes %v", trial, rim, axes)
			seen[want]++
		}
		require.Positive(t, seen[ProjectedPatchSelfContact])
	})
}

func TestPatchBoundarySweepChargesTestedPairs(t *testing.T) {
	points := rimPoints(circleRim(400, rand.New(rand.NewPCG(83, 89)), 0), [2]int{0, 1})
	got, used := runPatchBoundary(t, patchBoundarySimple, points, patchBoundaryAllowance)
	require.Equal(t, ProjectedPatchHolds, got)
	_, quadratic := runPatchBoundary(t, patchBoundarySimpleQuadratic, points, patchBoundaryAllowance)
	require.Equal(t, uint64(400*399/2), quadratic)
	require.Less(t, used, quadratic)
	require.Positive(t, used)

	got, _ = runPatchBoundary(t, patchBoundarySimple, points, used)
	require.Equal(t, ProjectedPatchHolds, got)
	got, _ = runPatchBoundary(t, patchBoundarySimple, points, used-1)
	require.Equal(t, ProjectedPatchStructure, got)
}

func TestPatchBoundarySweepNonFiniteFallsBack(t *testing.T) {
	rng := rand.New(rand.NewPCG(83, 89))
	for _, bad := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
		for _, onV := range []bool{false, true} {
			points := rimPoints(circleRim(17, rng, 0), [2]int{0, 1})
			if onV {
				points[5].Fv = bad
			} else {
				points[5].Fu = bad
			}
			want, wantUsed := runPatchBoundary(t, patchBoundarySimpleQuadratic, points, patchBoundaryAllowance)
			got, used := runPatchBoundary(t, patchBoundarySimple, points, patchBoundaryAllowance)
			require.Equal(t, want, got)
			// Only the quadratic check charges every pair.
			require.Equal(t, wantUsed, used, "value %v on v=%v", bad, onV)
		}
	}
}

// intersectingPairs is the exact all-pairs oracle: every pair of edges that
// meets other than adjacent edges at their shared vertex.
func intersectingPairs(points []meshbool.Xp2) map[[2]int]struct{} {
	n := len(points)
	out := map[[2]int]struct{}{}
	for i := range n {
		a, b := points[i], points[(i+1)%n]
		for j := i + 1; j < n; j++ {
			c, d := points[j], points[(j+1)%n]
			bad := false
			switch {
			case j == i+1:
				bad = !patchAdjacentEdgesMeetOnlyAtVertex(a, b, d)
			case i == 0 && j == n-1:
				bad = !patchAdjacentEdgesMeetOnlyAtVertex(c, d, b)
			default:
				bad = patchSegmentsMeet(a, b, c, d)
			}
			if bad {
				out[[2]int{i, j}] = struct{}{}
			}
		}
	}
	return out
}

// ulpRims returns random and adversarial rims whose u coordinates sit on,
// one ulp above and one ulp below shared values, so edge u ranges touch or
// overlap by exactly one ulp. Duplicate points and vertical or zero-length
// edges occur naturally on the coarse grid.
func ulpRims(rng *rand.Rand, count int) [][][2]float64 {
	bases := []float64{0, 1, 1.5, 3}
	us := make([]float64, 0, 3*len(bases)+1)
	us = append(us, math.Copysign(0, -1))
	for _, b := range bases {
		us = append(us, b, math.Nextafter(b, math.Inf(1)), math.Nextafter(b, math.Inf(-1)))
	}
	rims := make([][][2]float64, count)
	for k := range rims {
		n := 3 + rng.IntN(8)
		rim := make([][2]float64, n)
		for i := range rim {
			rim[i] = [2]float64{us[rng.IntN(len(us))], float64(rng.IntN(4))}
		}
		rims[k] = rim
	}
	return rims
}

func TestPatchBoundarySweepChecksEveryIntersectingPair(t *testing.T) {
	rng := rand.New(rand.NewPCG(83, 89))
	axesList := [][2]int{{0, 1}, {1, 2}, {0, 2}}
	var intersecting int
	for trial, rim := range ulpRims(rng, 3000) {
		axes := axesList[trial%len(axesList)]
		points := rimPoints(rim, axes)
		want := intersectingPairs(points)
		lo, hi, order, ok := patchUSweepOrder(points)
		require.True(t, ok)
		visited := map[[2]int]struct{}{}
		for x, y := range patchUOverlapPairs(lo, hi, order) {
			visited[[2]int{int(min(x, y)), int(max(x, y))}] = struct{}{}
		}
		for pair := range want {
			require.Contains(t, visited, pair, "rim %v axes %v: intersecting pair %v is never tested", rim, axes, pair)
		}
		got, _ := runPatchBoundary(t, patchBoundarySimple, points, patchBoundaryAllowance)
		if len(want) > 0 {
			intersecting++
			require.Equal(t, ProjectedPatchSelfContact, got, "rim %v axes %v", rim, axes)
			continue
		}
		require.Equal(t, ProjectedPatchHolds, got, "rim %v axes %v", rim, axes)
	}
	require.Positive(t, intersecting)
}

func TestPatchBoundarySweepTouchingAndOneUlpRanges(t *testing.T) {
	up := math.Nextafter(1, 2)
	// Edge (0,0)-(1,0) and edge (a,1)-(a,3), non adjacent: their v ranges are
	// disjoint, so they never meet. Their u ranges touch (a == 1), are apart
	// by one ulp, or overlap by one ulp; the verdict is Holds in every case.
	for _, tc := range []struct {
		a    float64
		want ProjectedPatchCondition
	}{
		{1, ProjectedPatchHolds},
		{up, ProjectedPatchHolds},
		{math.Nextafter(1, 0), ProjectedPatchHolds},
	} {
		rim := [][2]float64{{0, 0}, {1, 0}, {2, 5}, {tc.a, 3}, {tc.a, 1}, {-1, 2}}
		points := rimPoints(rim, [2]int{0, 1})
		want := intersectingPairs(points)
		got, _ := runPatchBoundary(t, patchBoundarySimple, points, patchBoundaryAllowance)
		ref, _ := runPatchBoundary(t, patchBoundarySimpleQuadratic, points, patchBoundaryAllowance)
		require.Equal(t, ref, got, "a=%v", tc.a)
		require.Equal(t, len(want) > 0, got == ProjectedPatchSelfContact, "a=%v", tc.a)
	}
	// Edges meeting exactly at the shared u value, one of them vertical: the
	// vertex (1,1) is the low u end of one edge and the high u end of the other.
	points := rimPoints([][2]float64{{0, 0}, {1, 1}, {2, 3}, {3, 1}, {1, 1}, {0, 2}}, [2]int{0, 1})
	got, _ := runPatchBoundary(t, patchBoundarySimple, points, patchBoundaryAllowance)
	require.Equal(t, ProjectedPatchSelfContact, got)
}

func TestPatchBoundarySweepInexactFloatFallsBack(t *testing.T) {
	// A point whose held float is not its exact rational must not be pruned on
	// the float: the check takes the quadratic path and charges every pair.
	rim := circleRim(17, rand.New(rand.NewPCG(83, 89)), 0)
	points := rimPoints(rim, [2]int{0, 1})
	points[4].U = big.NewRat(1, 3)
	lo, hi, order, ok := patchUSweepOrder(points)
	require.Nil(t, lo)
	require.Nil(t, hi)
	require.Nil(t, order)
	require.False(t, ok)
	want, wantUsed := runPatchBoundary(t, patchBoundarySimpleQuadratic, points, patchBoundaryAllowance)
	got, used := runPatchBoundary(t, patchBoundarySimple, points, patchBoundaryAllowance)
	require.Equal(t, want, got)
	require.Equal(t, wantUsed, used)
}

// TestPatchUOverlapPairsYieldsExactlyOverlappingPairs compares the sweep's
// pairs with a plain O(n^2) loop over closed u ranges: the sweep must yield
// every pair whose ranges overlap or touch, and no other pair.
func TestPatchUOverlapPairsYieldsExactlyOverlappingPairs(t *testing.T) {
	rng := rand.New(rand.NewPCG(83, 89))
	rims := ulpRims(rng, 2000)
	for _, rim := range patchRims() {
		rims = append(rims, rim.pts)
	}
	for k, rim := range rims {
		points := rimPoints(rim, [2]int{0, 1})
		lo, hi, order, ok := patchUSweepOrder(points)
		require.True(t, ok)
		want := map[[2]int]struct{}{}
		for i := range lo {
			for j := i + 1; j < len(lo); j++ {
				if lo[i] <= hi[j] && lo[j] <= hi[i] {
					want[[2]int{i, j}] = struct{}{}
				}
			}
		}
		got := map[[2]int]struct{}{}
		for x, y := range patchUOverlapPairs(lo, hi, order) {
			pair := [2]int{int(min(x, y)), int(max(x, y))}
			// A map lookup, not require.NotContains: NotContains walks every key
			// by reflection, which is quadratic in the pairs of the star 400 rim.
			_, dup := got[pair]
			require.False(t, dup, "rim %d yields pair %v twice", k, pair)
			got[pair] = struct{}{}
		}
		require.Equal(t, want, got, "rim %d %v", k, rim)
	}
}
