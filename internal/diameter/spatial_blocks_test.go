package diameter

import (
	"math"
	"math/rand/v2"
	"sort"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// spatialPointBlocksBySort is the full-sort partition spatialPointBlocks
// replaces: every node sorts its whole segment on its widest axis. The
// differential test holds the selection partition to this arrangement.
func spatialPointBlocksBySort(points []r3.Vec) ([]spatialPoint, []spatialBlock) {
	ordered := make([]spatialPoint, len(points))
	for i, point := range points {
		ordered[i] = spatialPoint{value: point, index: i}
	}
	blocks := make([]spatialBlock, 0, (len(points)+pointBlockSize-1)/pointBlockSize)
	var partition func(int, int)
	partition = func(begin, end int) {
		lo, hi := ordered[begin].value, ordered[begin].value
		for _, entry := range ordered[begin+1 : end] {
			p := entry.value
			lo.X, lo.Y, lo.Z = math.Min(lo.X, p.X), math.Min(lo.Y, p.Y), math.Min(lo.Z, p.Z)
			hi.X, hi.Y, hi.Z = math.Max(hi.X, p.X), math.Max(hi.Y, p.Y), math.Max(hi.Z, p.Z)
		}
		if end-begin <= pointBlockSize {
			blocks = append(blocks, spatialBlock{box: pointBlock{min: lo, max: hi}, begin: begin, end: end})
			return
		}
		axis := 0
		span := hi.X - lo.X
		if hi.Y-lo.Y > span {
			axis, span = 1, hi.Y-lo.Y
		}
		if hi.Z-lo.Z > span {
			axis = 2
		}
		segment := ordered[begin:end]
		sort.Slice(segment, func(i, j int) bool {
			a, b := coordinate(segment[i].value, axis), coordinate(segment[j].value, axis)
			if a == b {
				return segment[i].index < segment[j].index
			}
			return a < b
		})
		mid := begin + (end-begin)/2
		partition(begin, mid)
		partition(mid, end)
	}
	partition(0, len(ordered))
	return ordered, blocks
}

// spatialFixtures are point sets with the shapes faceted meshes hold: rings
// of a revolve, a grid full of equal coordinates, a sorted run, a reversed
// run, a cloud, signed zeros and one repeated point.
func spatialFixtures() map[string][]r3.Vec {
	rng := rand.New(rand.NewPCG(0x5eed, 0xb10c))
	fixtures := map[string][]r3.Vec{}
	ring := func(rings, around int) []r3.Vec {
		out := make([]r3.Vec, 0, rings*around)
		for z := range rings {
			for k := range around {
				angle := 2 * math.Pi * float64(k) / float64(around)
				out = append(out, r3.NewVec(10*math.Cos(angle), 10*math.Sin(angle), float64(z)/4))
			}
		}
		return out
	}
	fixtures["rings"] = ring(48, 96)
	fixtures["rings-small"] = ring(9, 64)
	grid := make([]r3.Vec, 0, 20*20*20)
	for x := range 20 {
		for y := range 20 {
			for z := range 20 {
				grid = append(grid, r3.NewVec(float64(x), float64(y%5), float64(z)))
			}
		}
	}
	fixtures["grid"] = grid
	run := make([]r3.Vec, 3000)
	reversed := make([]r3.Vec, 3000)
	for i := range run {
		run[i] = r3.NewVec(float64(i), float64(i%3), 0)
		reversed[i] = r3.NewVec(float64(-i), 0, float64(i%11))
	}
	fixtures["run"] = run
	fixtures["reversed"] = reversed
	cloud := make([]r3.Vec, 5000)
	for i := range cloud {
		cloud[i] = r3.NewVec(rng.NormFloat64(), 3*rng.Float64(), rng.Float64()-0.5)
	}
	fixtures["cloud"] = cloud
	zeros := make([]r3.Vec, 700)
	for i := range zeros {
		zero := 0.0
		if i%2 == 0 {
			zero = math.Copysign(0, -1)
		}
		zeros[i] = r3.NewVec(zero, float64(i%4), zero)
	}
	fixtures["signed-zeros"] = zeros
	same := make([]r3.Vec, 600)
	for i := range same {
		same[i] = r3.NewVec(1, 2, 3)
	}
	fixtures["repeated"] = same
	odd := make([]r3.Vec, 1031)
	for i := range odd {
		odd[i] = r3.NewVec(float64(rng.IntN(7)), float64(rng.IntN(5)), float64(rng.IntN(3)))
	}
	fixtures["odd-count-ties"] = odd
	// One leaf at the root: no node above it chose an axis to sort it on.
	fixtures["single-leaf"] = cloud[:40]
	return fixtures
}

func TestSpatialPointBlocksMatchesFullSort(t *testing.T) {
	t.Parallel()
	for name, points := range spatialFixtures() {
		wantOrdered, wantBlocks := spatialPointBlocksBySort(points)
		gotOrdered, gotBlocks := spatialPointBlocks(points)
		require.Len(t, gotOrdered, len(wantOrdered), name)
		require.Len(t, gotBlocks, len(wantBlocks), name)
		mismatch := -1
		for i := range wantOrdered {
			if !sameSpatialPoint(gotOrdered[i], wantOrdered[i]) {
				mismatch = i
				break
			}
		}
		require.Equal(t, -1, mismatch, "%s: first differing position", name)
		mismatch = -1
		for i := range wantBlocks {
			w, g := wantBlocks[i], gotBlocks[i]
			if w.begin != g.begin || w.end != g.end || !sameBitsVec(w.box.min, g.box.min) ||
				!sameBitsVec(w.box.max, g.box.max) {
				mismatch = i
				break
			}
		}
		require.Equal(t, -1, mismatch, "%s: first differing block", name)
	}
}

// TestSelectSpatialSplitsAtK checks every k of several small tie-heavy sets,
// with the normal round limit and with limits that force the sort fallback
// at once or partway: s[k] is the full sort's s[k], and every point sits on
// its own side of it.
func TestSelectSpatialSplitsAtK(t *testing.T) {
	t.Parallel()
	odd := spatialFixtures()["odd-count-ties"]
	for _, n := range []int{1, 2, 3, 5, 64, 65, 200} {
		points := odd[:n]
		sorted := spatialPointsOf(points)
		sortSpatial(sorted, 1)
		for _, limit := range []int{-1, 0, 1, 2} {
			bad := 0
			for k := range n {
				got := spatialPointsOf(points)
				if limit < 0 {
					selectSpatial(got, k, 1)
				} else {
					selectSpatialRounds(got, k, 1, limit)
				}
				if !sameSpatialPoint(sorted[k], got[k]) {
					bad++
				}
				for i := range got {
					c := spatialCompare(got[i], got[k], 1)
					if i < k && c >= 0 || i > k && c <= 0 {
						bad++
					}
				}
			}
			require.Zero(t, bad, "n=%d limit=%d: points misplaced around k", n, limit)
		}
	}
}

func spatialPointsOf(points []r3.Vec) []spatialPoint {
	out := make([]spatialPoint, len(points))
	for i, point := range points {
		out[i] = spatialPoint{value: point, index: i}
	}
	return out
}

func sameSpatialPoint(a, b spatialPoint) bool {
	return a.index == b.index && sameBitsVec(a.value, b.value)
}

func sameBitsVec(a, b r3.Vec) bool {
	return math.Float64bits(a.X) == math.Float64bits(b.X) && math.Float64bits(a.Y) == math.Float64bits(b.Y) &&
		math.Float64bits(a.Z) == math.Float64bits(b.Z)
}

func BenchmarkSpatialPointBlocks(b *testing.B) {
	points := spatialFixtures()["rings"]
	b.ReportAllocs()
	for b.Loop() {
		spatialPointBlocks(points)
	}
}

func BenchmarkPointsContextRings(b *testing.B) {
	points := spatialFixtures()["rings"]
	ctx := b.Context()
	b.ReportAllocs()
	for b.Loop() {
		if _, ok, err := PointsContext(ctx, points); err != nil || !ok {
			b.Fatal("no diameter")
		}
	}
}
