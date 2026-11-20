package smoothloft

import (
	"context"
	"math/big"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
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

// circularMeshCall holds the arguments the circular fitted route hands to
// the mesh builder.
type circularMeshCall struct {
	rational [][]rationalTransition
	edgeOf   []int
	edges    int
	planes   []sectionrecord.PlaneRecord
	zs       []*big.Rat
	emerging []bool
	firstFit int
}

func (c circularMeshCall) build(ctx context.Context, planes []sectionrecord.PlaneRecord) (Build, error) {
	return buildSectionMeshMode(ctx, nil, c.rational, c.edgeOf, c.edges, planes, c.zs, c.emerging, c.firstFit)
}

// circularFitRecords records a circular fitted loft: two circles about
// (0.25, 0.125) followed by two quarter-spline squares, one section per
// millimetre of height.
func circularFitRecords(t *testing.T) ([]momentinput.Profile, []sectionrecord.PlaneRecord, []*freeform.FreeformWork) {
	t.Helper()
	const circles = 2
	offsets := [2]float64{0.05, 0.1}
	w := sketch.NewWorld()
	profiles := make([]momentinput.Profile, circles+len(offsets))
	planes := make([]sectionrecord.PlaneRecord, len(profiles))
	work := make([]*freeform.FreeformWork, len(profiles))
	for i := range profiles {
		plane := w.XY()
		if i > 0 {
			var err error
			plane, err = w.CreateOffsetPlane(w.XY(), float64(i))
			require.NoError(t, err)
		}
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		point := func(x, y float64) *sketch.Point {
			p := s.CreatePoint(x, y)
			s.Fix(p)
			return p
		}
		if i < circles {
			origin := point(0.25, 0.125)
			v := []*sketch.Point{point(1.25, 0.125), point(0.25, 1.125), point(-0.75, 0.125), point(0.25, -0.875)}
			for j := range v {
				s.CreateArc(origin, v[j], v[(j+1)%len(v)])
			}
		} else {
			off := offsets[i-circles]
			v := []*sketch.Point{point(1, 0), point(0, 1), point(-1, 0), point(0, -1)}
			_, err = s.CreateSpline(v[0], point(0.75+off, 0.25+off), point(0.5+off, 0.5+off),
				point(0.25+off, 0.75+off), v[1])
			require.NoError(t, err)
			for j := 1; j < len(v); j++ {
				s.CreateLine(v[j], v[(j+1)%len(v)])
			}
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		var profile *sketch.Profile
		for _, p := range s.Profiles() {
			if p.Valid && len(p.Holes) == 0 && len(p.Outer) == 4 {
				profile = p
			}
		}
		require.NotNil(t, profile, "section %d has no four-edge profile", i)
		var area float64
		profiles[i], planes[i], area, err = sectioncert.RecordProfile(s, profile)
		require.NoError(t, err)
		work[i] = freeform.NewFreeformWork()
		_, err = profiles[i].FalsifyLoftRecordedArea(area, work[i])
		require.NoError(t, err)
	}
	return profiles, planes, work
}

// circularStationReference evaluates every ring vertex of c through the
// per-call rationalTransition.at, shifted by origin, in mesh order.
func circularStationReference(t *testing.T, c circularMeshCall, origin r3.Vec) ([]*big.Rat, []*big.Rat, []r3.Vec) {
	t.Helper()
	bounds := make([][]twistBounds, len(c.rational))
	for k := range c.rational {
		for _, p := range c.rational[k] {
			b, err := p.bounds()
			require.NoError(t, err)
			bounds[k] = append(bounds[k], b)
		}
	}
	nu, nt, err := twistSelectCounts(t.Context(), bounds, twistMaxTriangles)
	require.NoError(t, err)
	var xs, ys []*big.Rat
	var verts []r3.Vec
	for i := range nt {
		for b := range nt[i] + 1 {
			if i > 0 && b == 0 {
				continue
			}
			tt := big.NewRat(int64(b), int64(nt[i]))
			z := add(c.zs[i], mul(sub(c.zs[i+1], c.zs[i]), tt))
			zf, _ := z.Float64()
			for k := range c.rational {
				for a := range nu[k] {
					p := c.rational[k][i].at(big.NewRat(int64(a), int64(nu[k])), tt)
					x, y := add(p.U, rf(origin.X)), add(p.V, rf(origin.Y))
					xs, ys = append(xs, x), append(ys, y)
					verts = append(verts, r3.Vec{X: ratFloat(x), Y: ratFloat(y), Z: zf})
				}
			}
		}
	}
	return xs, ys, verts
}

// requireSameElements compares slices element by element and reports only
// the mismatch count and the first mismatching index, never the slices.
func requireSameElements[T comparable](t *testing.T, want, got []T, label string) {
	t.Helper()
	require.Len(t, got, len(want), label)
	mismatches, first := 0, -1
	for i := range want {
		if want[i] != got[i] {
			if first < 0 {
				first = i
			}
			mismatches++
		}
	}
	require.Zero(t, mismatches, "%s: first mismatch at index %d of %d", label, first, len(want))
}

// TestCircularMeshStationTableMatchesAt checks that the station table
// reproduces the per-call rationalTransition.at evaluation exactly, on the
// transitions a recorded circular fitted loft builds.
func TestCircularMeshStationTableMatchesAt(t *testing.T) {
	profiles, planes, work := circularFitRecords(t)
	var calls []circularMeshCall
	circularMesh = func(ctx context.Context, patches [][]twistPatch, rational [][]rationalTransition,
		edgeOf []int, edges int, planes []sectionrecord.PlaneRecord, zs []*big.Rat,
		emerging []bool, firstFit int) (Build, error) {
		calls = append(calls, circularMeshCall{rational, edgeOf, edges, planes, zs, emerging, firstFit})
		return buildSectionMeshMode(ctx, patches, rational, edgeOf, edges, planes, zs, emerging, firstFit)
	}
	t.Cleanup(func() { circularMesh = buildSectionMeshMode })
	built, err := BuildCircularFits(t.Context(), profiles, planes, work)
	require.NoError(t, err)
	require.Len(t, calls, 1)
	c := calls[0]
	require.Len(t, c.rational[0], 3)
	require.True(t, c.rational[0][0].linear, "the circle-to-circle span blends linearly")
	require.False(t, c.rational[0][1].linear, "the circle-to-fit span blends by smoothstep")
	require.NotContains(t, c.emerging, true)

	_, _, want := circularStationReference(t, c, c.planes[0].Origin)
	requireSameElements(t, want, built.Vertices, "recorded loft")

	// A shifted, non-dyadic origin checks the origin folded into the table.
	shifted := append([]sectionrecord.PlaneRecord(nil), c.planes...)
	shifted[0].Origin.X += 0.1
	shifted[0].Origin.Y -= 0.3
	xs, ys, want := circularStationReference(t, c, shifted[0].Origin)
	got, err := c.build(t.Context(), shifted)
	require.NoError(t, err)
	requireSameElements(t, want, got.Vertices, "shifted origin")

	t.Run("exact station values", func(t *testing.T) {
		ox, oy := rf(shifted[0].Origin.X), rf(shifted[0].Origin.Y)
		bounds := make([][]twistBounds, len(c.rational))
		for k := range c.rational {
			for _, p := range c.rational[k] {
				b, err := p.bounds()
				require.NoError(t, err)
				bounds[k] = append(bounds[k], b)
			}
		}
		nu, nt, err := twistSelectCounts(t.Context(), bounds, twistMaxTriangles)
		require.NoError(t, err)
		t.Logf("nu=%v nt=%v vertices=%d", nu, nt, len(xs))
		n, mismatches, first := 0, 0, -1
		for i := range nt {
			for b := range nt[i] + 1 {
				if i > 0 && b == 0 {
					continue
				}
				tt := big.NewRat(int64(b), int64(nt[i]))
				for k := range c.rational {
					s := smoothstep(tt)
					if c.rational[k][i].linear {
						s = tt
					}
					for a := range nu[k] {
						x, y := c.rational[k][i].station(big.NewRat(int64(a), int64(nu[k])), ox, oy).at(s)
						if x.Cmp(xs[n]) != 0 || y.Cmp(ys[n]) != 0 {
							if first < 0 {
								first = n
							}
							mismatches++
						}
						n++
					}
				}
			}
		}
		require.Equal(t, len(xs), n)
		require.Zero(t, mismatches, "first exact mismatch at vertex %d of %d", first, n)
	})

	t.Run("one span per table group", func(t *testing.T) {
		saved := twistStationTable
		twistStationTable = 1
		t.Cleanup(func() { twistStationTable = saved })
		split, err := c.build(t.Context(), shifted)
		require.NoError(t, err)
		requireSameElements(t, got.Vertices, split.Vertices, "one span per group")
		requireSameElements(t, got.Triangles, split.Triangles, "triangles")
		requireSameElements(t, got.Sources, split.Sources, "sources")
		require.Equal(t, [4]float64{got.MeshBound, got.VolSymDiff, got.AreaSlack, got.Diameter},
			[4]float64{split.MeshBound, split.VolSymDiff, split.AreaSlack, split.Diameter})
	})
}
