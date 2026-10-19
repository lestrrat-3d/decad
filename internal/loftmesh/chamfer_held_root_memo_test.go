package loftmesh

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// heldRootWalls returns triangles whose vertices are the given triples.
func heldRootWalls(tris ...[3]r3.Vec) ([]r3.Vec, [][3]int) {
	var verts []r3.Vec
	var idx [][3]int
	for _, t := range tris {
		n := len(verts)
		verts = append(verts, t[0], t[1], t[2])
		idx = append(idx, [3]int{n, n + 1, n + 2})
	}
	return verts, idx
}

type heldRootStep struct {
	lo, hi *big.Rat
	want   bool
}

func runHeldRootSteps(t *testing.T, m *ChamferHeldRootMemo, verts []r3.Vec, tris [][3]int,
	radius float64, steps []heldRootStep) {
	t.Helper()
	for i, s := range steps {
		direct := ChamferMiddleHeldRootInside(verts, tris, len(tris), 0, s.lo, s.hi, radius)
		require.Equal(t, s.want, direct, "step %d: direct answer", i)
		require.Equal(t, direct, m.Inside(verts, tris, len(tris), 0, s.lo, s.hi, radius), "step %d", i)
	}
}

func TestChamferHeldRootMemoNeverBridgesAGap(t *testing.T) {
	verts, tris := heldRootWalls(
		[3]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 0.5, Y: 0, Z: 0}, {X: 0, Y: 0.5, Z: 1}},
		[3]r3.Vec{{X: 3, Y: 0, Z: 1.4}, {X: 3.5, Y: 0, Z: 1.4}, {X: 3, Y: 0.5, Z: 1.6}},
		[3]r3.Vec{{X: 0, Y: 0, Z: 3}, {X: 0.5, Y: 0, Z: 3}, {X: 0, Y: 0.5, Z: 4}},
	)
	r := big.NewRat
	var m ChamferHeldRootMemo
	runHeldRootSteps(t, &m, verts, tris, 1, []heldRootStep{
		{r(0, 1), r(1, 1), true},
		{r(3, 1), r(4, 1), true},
		{r(1, 2), r(7, 2), false}, // spans the gap, tri 1 is inside the slab
		{r(1, 4), r(3, 4), true},
		{r(1, 2), r(2, 1), false},
		{r(4, 5), r(6, 5), true}, // tri 1 starts at 1.4
	})
	require.Zero(t, m.lo.Cmp(r(0, 1)))
	require.Zero(t, m.hi.Cmp(r(6, 5)))
	runHeldRootSteps(t, &m, verts, tris, 1, []heldRootStep{
		{r(1, 10), r(11, 10), true},
		{r(1, 1), r(1, 1), false},
		{r(3, 4), r(1, 2), false},
		{nil, r(1, 2), false},
	})
	// A smaller radius for the same arrays must not use the interval.
	require.False(t, m.Inside(verts, tris, len(tris), 0, r(1, 4), r(3, 4), 0.35))
}

// Two touching accepted slabs cover [0, 2], but a wall triangle at z 2.2..2.4
// lies outside the disk. A query spanning both slabs is true; one that
// reaches past the interval runs the exact check and is refused.
func TestChamferHeldRootMemoStopsAtTheRegionBoundary(t *testing.T) {
	verts, tris := heldRootWalls(
		[3]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 0.5, Y: 0, Z: 0}, {X: 0, Y: 0.5, Z: 1}},
		[3]r3.Vec{{X: 0, Y: 0, Z: 1}, {X: 0.5, Y: 0, Z: 1}, {X: 0, Y: 0.9, Z: 2}},
		[3]r3.Vec{{X: 3, Y: 0, Z: 2.2}, {X: 3.5, Y: 0, Z: 2.2}, {X: 3, Y: 0.5, Z: 2.4}},
	)
	r := big.NewRat
	var m ChamferHeldRootMemo
	runHeldRootSteps(t, &m, verts, tris, 1, []heldRootStep{
		{r(0, 1), r(1, 1), true},
		{r(1, 1), r(2, 1), true},
		{r(1, 2), r(3, 2), true},
		{r(0, 1), r(2, 1), true},
		{r(0, 1), r(21, 10), true},
		{r(0, 1), r(11, 5), true},  // float 2.2 is just above 11/5, so tri 2 stays out
		{r(0, 1), r(5, 2), false},  // beyond the interval and the disk
		{r(3, 2), r(5, 2), false},  // starts inside the interval
		{r(2, 1), r(12, 5), false}, // overlaps the interval at its end only
	})
}

func TestChamferHeldRootMemoMatchesDirect(t *testing.T) {
	rng := rand.New(rand.NewPCG(83, 89))
	quarter := func(top int) float64 { return float64(rng.IntN(top*4+1)) / 4 }
	eighth := func() *big.Rat { return big.NewRat(int64(rng.IntN(33)), 8) }
	hits, trues, falses := 0, 0, 0
	for range 200 {
		var ts [][3]r3.Vec
		for range 6 {
			var tri [3]r3.Vec
			for k := range tri {
				tri[k] = r3.Vec{
					X: rng.Float64()*3 - 1.5, Y: rng.Float64()*3 - 1.5, Z: quarter(4),
				}
			}
			ts = append(ts, tri)
		}
		verts, tris := heldRootWalls(ts...)
		radius := 1 + rng.Float64()
		var m ChamferHeldRootMemo
		for range 25 {
			lo, hi := eighth(), eighth()
			switch rng.IntN(10) {
			case 0:
				hi = new(big.Rat).Set(lo)
			case 1:
				lo, hi = hi, lo
			}
			if m.lo != nil && lo.Cmp(m.lo) >= 0 && hi.Cmp(m.hi) <= 0 {
				hits++
			}
			want := ChamferMiddleHeldRootInside(verts, tris, len(tris), 0, lo, hi, radius)
			require.Equal(t, want, m.Inside(verts, tris, len(tris), 0, lo, hi, radius))
			if want {
				trues++
			} else {
				falses++
			}
		}
	}
	require.GreaterOrEqual(t, hits, 200)
	require.GreaterOrEqual(t, trues, 200)
	require.GreaterOrEqual(t, falses, 200)
}
