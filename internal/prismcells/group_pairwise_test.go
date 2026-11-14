package prismcells_test

import (
	"errors"
	"fmt"
	"math/rand/v2"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// disc is the whole circle of radius r about (u, v). Each costs 256 arranger
// segments against MaxArrangementSegments.
func disc(u, v, r float64) momentinput.Profile {
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: u, V: v}, Radius: units.Millimeters(r), CCW: true, TEnd: 1},
	}}}
}

// discRow is count unit discs whose centres are step apart on U.
func discRow(count int, step float64) []momentinput.Profile {
	out := make([]momentinput.Profile, count)
	for i := range out {
		out[i] = disc(float64(i)*step, 0, 1)
	}
	return out
}

func pairwise(t *testing.T, regions []momentinput.Profile, inflate float64) (bool, float64, error) {
	t.Helper()
	return prismcells.ProveGroupApartPairwise(t.Context(), proofbound.NewWorkBudget(t.Context()), regions, inflate)
}

// TestGroupApartPairwiseOverCap is a row of 20 discs, 5120 arranger segments,
// which no single scene admits. Apart discs are proven; any pair that
// overlaps, touches at one tangent point, or nests is not.
//
// Shown red: dropping a pair scene (or reading box overlap as separation)
// admits the overlap, tangent and nested rows.
func TestGroupApartPairwiseOverCap(t *testing.T) {
	t.Parallel()
	_, within, err := prismcells.RegionsWithinWorkCap(proofbound.NewWorkBudget(t.Context()), discRow(20, 1)...)
	require.NoError(t, err)
	require.False(t, within, "the premise: one scene of the row is over the cap")
	_, _, err = prismcells.ProveGroupApart(t.Context(), proofbound.NewWorkBudget(t.Context()), discRow(20, 10))
	require.Error(t, err, "the premise: the single scene refuses")

	tangent := discRow(20, 10)
	tangent[8] = disc(72, 0, 1) // touches the disc at 70 at (71, 0)
	nested := discRow(20, 10)
	nested[5] = disc(50, 0, 3) // holds the small disc below
	nested = append(nested, disc(50, 0, 0.5))
	crossing := discRow(20, 10)
	crossing[12] = disc(111.5, 0, 1) // crosses the disc at 110
	crossing[11] = disc(110, 0, 1)

	for _, tc := range []struct {
		name    string
		regions []momentinput.Profile
		want    bool
	}{
		{name: "apart", regions: discRow(20, 10), want: true},
		{name: "apart with a close pair", regions: discRow(20, 2.5), want: true},
		{name: "overlapping neighbours", regions: discRow(20, 1.5)},
		{name: "one crossing pair", regions: crossing},
		{name: "one tangent pair", regions: tangent},
		{name: "nested", regions: nested},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			got, _, err := pairwise(t, tc.regions, 0)
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}

// TestGroupApartPairwiseInflation shows the box slack deciding: a gap that the
// charged displacement can close sends the pair to sketch's own scene, where
// a gap the geometry really has is still proven and a touch is not.
func TestGroupApartPairwiseInflation(t *testing.T) {
	t.Parallel()
	a, b := disc(0, 0, 1), disc(2.5, 0, 1) // a gap of 0.5 between the circles
	far := discRow(18, 10)
	for i := range far {
		far[i] = disc(100+float64(i)*10, 0, 1)
	}
	regions := append([]momentinput.Profile{a, b}, far...)
	got, _, err := pairwise(t, regions, 0)
	require.NoError(t, err)
	require.True(t, got)
	got, _, err = pairwise(t, regions, 5) // boxes now overlap: sketch proves the pair
	require.NoError(t, err)
	require.True(t, got)

	touching := append([]momentinput.Profile{disc(0, 0, 1), disc(2, 0, 1)}, far...)
	got, _, err = pairwise(t, touching, 0)
	require.NoError(t, err)
	require.False(t, got, "tangent circles share one point")
	got, _, err = pairwise(t, touching, 1e-12)
	require.NoError(t, err)
	require.False(t, got)
}

// TestGroupApartPairwiseLimits covers the two limits: a group whose boxes all
// overlap needs more scenes than four per region, and a pair too large for
// one scene keeps today's per-scene cap.
func TestGroupApartPairwiseLimits(t *testing.T) {
	t.Parallel()
	t.Run("scene count", func(t *testing.T) {
		t.Parallel()
		concentric := make([]momentinput.Profile, 20)
		for i := range concentric {
			concentric[i] = disc(0, 0, float64(i+1))
		}
		_, _, err := pairwise(t, concentric, 0)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		var limit *prismcells.WorkLimitError
		require.True(t, errors.As(err, &limit))
		require.ErrorContains(t, err, "pairwise")
	})
	t.Run("per scene cap", func(t *testing.T) {
		t.Parallel()
		// One region of 17 circle segments is 4352 arranger segments, over
		// the cap on its own, so its scene refuses as the single scene does.
		var segs []sectionrecord.CurveSegment
		for i := range 17 {
			segs = append(segs, sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: float64(i)}, Radius: units.Millimeters(1), CCW: true, TEnd: 1})
		}
		big := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
		regions := append([]momentinput.Profile{big}, discRow(19, 10)...)
		for i := 1; i < len(regions); i++ {
			regions[i] = disc(1000+float64(i)*10, 0, 1)
		}
		_, _, err := pairwise(t, regions, 0)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
	})
}

// TestGroupApartPairwiseAgreesWithOneScene compares the pairwise verdict with
// the single scene's on random small groups of discs and squares, so a pair
// the boxes skip is never one the one scene refuses.
func TestGroupApartPairwiseAgreesWithOneScene(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(4, 112))
	agree := map[bool]int{}
	for range 400 {
		n := 2 + rng.IntN(6)
		regions := make([]momentinput.Profile, n)
		for i := range regions {
			u, v := float64(rng.IntN(9)), float64(rng.IntN(5))
			r := 0.5 + float64(rng.IntN(4))*0.5
			if rng.IntN(3) == 0 {
				regions[i] = square(u, u+r*2)
				continue
			}
			regions[i] = disc(u, v, r)
		}
		inflate := []float64{0, 0.25, 1}[rng.IntN(3)]
		want, _, err := prismcells.ProveGroupApart(t.Context(), proofbound.NewWorkBudget(t.Context()), regions)
		require.NoError(t, err)
		got, _, err := pairwise(t, regions, inflate)
		require.NoError(t, err)
		require.Equal(t, want, got)
		agree[want]++
	}
	require.Positive(t, agree[true], "the premise: some groups are apart")
	require.Positive(t, agree[false], "the premise: some groups are not")
}

// TestGroupApartPairwiseCost shows the proof of a 36-region row, 9216 arranger
// segments, costing a fraction of a second; the one scene it replaces refuses.
func TestGroupApartPairwiseCost(t *testing.T) {
	t.Parallel()
	start := time.Now()
	got, _, err := pairwise(t, discRow(36, 4), 0)
	require.NoError(t, err)
	require.True(t, got)
	require.Less(t, time.Since(start), 5*time.Second)
}

// BenchmarkGroupApartPairwise measures the pairwise proof of rows of discs
// that no single scene admits.
func BenchmarkGroupApartPairwise(b *testing.B) {
	for _, count := range []int{20, 36} {
		regions := discRow(count, 4)
		b.Run(fmt.Sprintf("discs=%d", count), func(b *testing.B) {
			b.ReportAllocs()
			for b.Loop() {
				got, _, err := prismcells.ProveGroupApartPairwise(b.Context(),
					proofbound.NewWorkBudget(b.Context()), regions, 0)
				if err != nil || !got {
					b.Fatalf("proof = %v, err = %v", got, err)
				}
			}
		})
	}
}
