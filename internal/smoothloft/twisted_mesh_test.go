package smoothloft

import (
	"context"
	"math/big"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

type cancelDuringBounds struct {
	checks int
	done   chan struct{}
}

func (c *cancelDuringBounds) Deadline() (time.Time, bool) { return time.Time{}, false }
func (c *cancelDuringBounds) Done() <-chan struct{}       { return c.done }
func (c *cancelDuringBounds) Value(any) any               { return nil }

func (c *cancelDuringBounds) Err() error {
	c.checks++
	if c.checks > 1 {
		if c.checks == 2 {
			close(c.done)
		}
		return context.Canceled
	}
	return nil
}

func TestCircularMeshCancelsBetweenBounds(t *testing.T) {
	center := twistTestPoint(0, 0)
	curve := circularChart(rint(1), center, 0, rint(0), rint(1))
	transition := rationalTransition{a: curve, b: curve}
	ctx := &cancelDuringBounds{done: make(chan struct{})}
	_, err := buildSectionMesh(ctx, nil, [][]rationalTransition{{transition, transition}}, nil, 0, nil, nil)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, 2, ctx.checks)
}

func TestTwistedCountsZeroMixedBound(t *testing.T) {
	line := twistTestLinePatch(twistTestPoint(4, -2), twistTestPoint(4, 2)).bounds()
	require.Zero(t, line.mut.Sign())
	// A second band requires subdivision, so the zero bound must coexist with
	// the other band's constraints on the shared station count.
	other := line
	other.mut = big.NewRat(1, 10)
	target := big.NewRat(3, 100)
	nu, nt, err := twistCounts(t.Context(), [][]twistBounds{{line, other}}, target)
	require.NoError(t, err)
	require.Equal(t, []int{3}, nu)
	require.Equal(t, []int{1, 1}, nt)
	for i, b := range []twistBounds{line, other} {
		_, _, mixed := twistTerms(b, nu[0], nt[i])
		require.LessOrEqual(t, mixed.Cmp(div(target, rint(3))), 0)
	}
	_, _, previous := twistTerms(other, nu[0]-1, nt[1])
	require.Greater(t, previous.Cmp(div(target, rint(3))), 0)
}

func TestTwistedCountsRejectsBoundsBeyondCeilings(t *testing.T) {
	target := big.NewRat(3, 100)
	third := big.NewRat(1, 100)
	huge := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 2048))
	delta := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Exp(big.NewInt(10), big.NewInt(30), nil))
	heightMax := mul(mul(rint(8), third), rint(twistMaxT*twistMaxT))
	stationMax := mul(mul(rint(8), third), rint(twistMaxU*twistMaxU))
	mixedMax := mul(mul(rint(4), third), rint(twistMaxU))
	for _, tc := range []struct {
		name          string
		muu, mtt, mut *big.Rat
		want          string
	}{
		{"height", rint(0), huge, rint(0), "height-band ceiling"},
		{"station", huge, rint(0), rint(0), "station ceiling"},
		{"mixed", rint(0), rint(0), huge, "station ceiling"},
		{"height just over", rint(0), add(heightMax, delta), rint(0), "height-band ceiling"},
		{"station just over", add(stationMax, delta), rint(0), rint(0), "station ceiling"},
		{"mixed just over", rint(0), rint(0), add(mixedMax, delta), "station ceiling"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, _, err := twistCounts(t.Context(), [][]twistBounds{{{muu: tc.muu, mtt: tc.mtt, mut: tc.mut}}}, target)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, tc.want)
		})
	}
	for _, tc := range []struct {
		name          string
		muu, mtt, mut *big.Rat
		wantU, wantT  int
	}{
		{"height at ceiling", rint(0), heightMax, rint(0), 1, twistMaxT},
		{"station at ceiling", stationMax, rint(0), rint(0), twistMaxU, 1},
		{"mixed at ceiling", rint(0), rint(0), mixedMax, twistMaxU, 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			nu, nt, err := twistCounts(t.Context(), [][]twistBounds{{{muu: tc.muu, mtt: tc.mtt, mut: tc.mut}}}, target)
			require.NoError(t, err)
			require.Equal(t, []int{tc.wantU}, nu)
			require.Equal(t, []int{tc.wantT}, nt)
		})
	}
}

func TestTwistedMeshStraightSquare(t *testing.T) {
	points := [][2]int64{{-2, -2}, {2, -2}, {2, 2}, {-2, 2}}
	patches := make([][]twistPatch, len(points))
	for i, p := range points {
		q := points[(i+1)%len(points)]
		patches[i] = []twistPatch{twistTestLinePatch(twistTestPoint(p[0], p[1]), twistTestPoint(q[0], q[1]))}
		require.Zero(t, patches[i][0].overCommon().bounds().mut.Sign())
	}
	mesh, err := buildTwistedMesh(t.Context(), patches, []int{0, 1, 2, 3}, 4,
		[]sectionrecord.PlaneRecord{{}}, []*big.Rat{rint(0), rint(3)})
	require.NoError(t, err)
	require.Len(t, mesh.Vertices, 8)
	require.Len(t, mesh.Triangles, 12)
	require.Zero(t, mesh.MeshBound)
	for i, p := range points {
		require.Equal(t, r3.Vec{X: float64(p[0]), Y: float64(p[1])}, mesh.Vertices[i])
		require.Equal(t, r3.Vec{X: float64(p[0]), Y: float64(p[1]), Z: 3}, mesh.Vertices[i+4])
	}
}

func TestTwistedMeshTriangulatesEachEndRing(t *testing.T) {
	bottom := [][2]int64{{-2, -2}, {2, -2}, {2, 2}, {-2, 2}}
	top := [][2]int64{{-2, -2}, {2, -2}, {0, -1}, {-2, 2}}
	point := func(p [2]int64) freeform.RatPoint {
		return freeform.RatPoint{U: big.NewRat(p[0], 10), V: big.NewRat(p[1], 10)}
	}
	patches := make([][]twistPatch, len(bottom))
	for i := range bottom {
		next := (i + 1) % len(bottom)
		low := twistLineSpan(point(bottom[i]), point(bottom[next]))
		high := twistLineSpan(point(top[i]), point(top[next]))
		var slope twistSpan
		for j := range slope {
			slope[j] = twistScale(twistSub(high[j], low[j]), big.NewRat(1, 3))
		}
		patches[i] = []twistPatch{twistPatchOf(low, high, slope, slope, rint(3))}
	}
	mesh, err := buildTwistedMesh(t.Context(), patches, []int{0, 1, 2, 3}, 4,
		[]sectionrecord.PlaneRecord{{}}, []*big.Rat{rint(0), rint(3)})
	require.NoError(t, err)
	require.Len(t, mesh.Sources, len(mesh.Triangles))
	var signedArea [2]float64
	var capCount [2]int
	for i, tri := range mesh.Triangles {
		source := mesh.Sources[i] - 4
		if source < 0 {
			continue
		}
		require.Less(t, source, 2)
		a, b, c := mesh.Vertices[tri[0]], mesh.Vertices[tri[1]], mesh.Vertices[tri[2]]
		cross := (b.X-a.X)*(c.Y-a.Y) - (b.Y-a.Y)*(c.X-a.X)
		if source == 0 {
			require.Negative(t, cross)
			require.Zero(t, a.Z)
			require.Zero(t, b.Z)
			require.Zero(t, c.Z)
		} else {
			require.Positive(t, cross)
			require.Equal(t, 3.0, a.Z)
			require.Equal(t, 3.0, b.Z)
			require.Equal(t, 3.0, c.Z)
		}
		signedArea[source] += cross / 2
		capCount[source]++
	}
	require.Positive(t, capCount[0])
	require.Positive(t, capCount[1])
	require.InDelta(t, -0.16, signedArea[0], 1e-12)
	require.InDelta(t, 0.06, signedArea[1], 1e-12)
}

// TestTwistedSelectCountsFallsBackPastTheCeiling checks that the finest
// target is used while its table fits the triangle ceiling, that each coarser
// target is tried in turn when the finer table does not, and that the loft is
// refused when none fits.
func TestTwistedSelectCountsFallsBackPastTheCeiling(t *testing.T) {
	patches := twistBenchPatches(8)
	row := make([]twistBounds, len(patches))
	for i := range patches {
		row[i] = patches[i].overCommon().bounds()
	}
	bounds := [][]twistBounds{row}

	var nus, nts [][]int
	var triangles []int
	for _, target := range twistTargets {
		nu, nt, err := twistCounts(t.Context(), bounds, target)
		require.NoError(t, err)
		nus, nts = append(nus, nu), append(nts, nt)
		triangles = append(triangles, twistTriangleCount(nu, nt))
	}
	require.Greater(t, triangles[0], triangles[1], "premise: the 0.02 mm table is larger than the 0.03 mm table")
	require.Greater(t, triangles[1], triangles[2], "premise: the 0.03 mm table is larger than the 0.049 mm table")

	for _, tc := range []struct {
		name    string
		ceiling int
		want    int // index of the selected target; -1 for a refusal
	}{
		{"finest fits", triangles[0], 0},
		{"finest one over", triangles[0] - 1, 1},
		{"middle fits", triangles[1], 1},
		{"middle one over", triangles[1] - 1, 2},
		{"coarsest fits", triangles[2], 2},
		{"coarsest one over", triangles[2] - 1, -1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			nu, nt, err := twistSelectCounts(t.Context(), bounds, tc.ceiling)
			if tc.want < 0 {
				require.ErrorIs(t, err, decaderr.ErrUnsupported)
				require.ErrorContains(t, err, "needs more than")
				return
			}
			require.NoError(t, err)
			require.Equal(t, nus[tc.want], nu)
			require.Equal(t, nts[tc.want], nt)
		})
	}

	t.Run("cancelled", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, _, err := twistSelectCounts(ctx, bounds, triangles[0])
		require.ErrorIs(t, err, context.Canceled)
	})
}
