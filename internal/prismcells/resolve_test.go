package prismcells_test

import (
	"sort"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// square is the counter-clockwise whole-line square [lo, hi]^2.
func square(lo, hi float64) momentinput.Profile {
	corners := []sectionrecord.Point2{{U: lo, V: lo}, {U: hi, V: lo}, {U: hi, V: hi}, {U: lo, V: hi}}
	segments := make([]sectionrecord.CurveSegment, len(corners))
	for i, start := range corners {
		segments[i] = sectionrecord.LineSeg{Start: start, End: corners[(i+1)%len(corners)], TStart: 0, TEnd: 1}
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segments}}
}

// walkedStarts lists every outer segment's walked start, Start + TStart*(End-Start),
// sorted by (U, V). A Partial segment keeps its source line's Start and End.
func walkedStarts(t *testing.T, profile momentinput.Profile) [][2]float64 {
	t.Helper()
	var out [][2]float64
	for _, segment := range profile.Outer.Segments {
		line, ok := segment.(sectionrecord.LineSeg)
		require.True(t, ok)
		out = append(out, [2]float64{
			line.Start.U + line.TStart*(line.End.U-line.Start.U),
			line.Start.V + line.TStart*(line.End.V-line.Start.V),
		})
	}
	sort.Slice(out, func(i, j int) bool {
		if out[i][0] != out[j][0] {
			return out[i][0] < out[j][0]
		}
		return out[i][1] < out[j][1]
	})
	return out
}

func TestResolveSceneOperands(t *testing.T) {
	identity := func() *prismcells.Reexpression { return &prismcells.Reexpression{Identity: true} }
	budget := func() *proofbound.WorkBudget { return proofbound.NewWorkBudget(t.Context()) }

	t.Run("union of overlapping squares", func(t *testing.T) {
		merged, delta, _, resolved, err := prismcells.ResolveUnion(
			t.Context(), budget(), square(0, 2), square(1, 3), 0, 0, identity())
		require.NoError(t, err)
		require.True(t, resolved)
		require.Zero(t, delta.A)
		require.Zero(t, delta.B)
		require.Empty(t, merged.Holes)
		require.Equal(t, [][2]float64{
			{0, 0}, {0, 2}, {1, 2}, {1, 3}, {2, 0}, {2, 1}, {3, 1}, {3, 3},
		}, walkedStarts(t, merged))
	})

	t.Run("crossing intersect keeps the overlap", func(t *testing.T) {
		merged, _, _, resolved, err := prismcells.ResolveCrossing(
			t.Context(), budget(), square(0, 2), square(1, 3), 0, 0, identity(),
			func(a, b bool) bool { return a && b }, "intersect")
		require.NoError(t, err)
		require.True(t, resolved)
		require.Empty(t, merged.Holes)
		require.Equal(t, [][2]float64{{1, 1}, {1, 2}, {2, 1}, {2, 2}}, walkedStarts(t, merged))
	})

	t.Run("cut of a nested tool", func(t *testing.T) {
		s, match, _, resolved, err := prismcells.ResolveCut(
			t.Context(), budget(), square(0, 4), square(1, 2), identity())
		require.NoError(t, err)
		require.True(t, resolved)
		require.NotNil(t, s)
		require.Len(t, match.Holes, 1)
	})

	t.Run("cut of a disjoint tool is unresolved", func(t *testing.T) {
		_, _, _, resolved, err := prismcells.ResolveCut(
			t.Context(), budget(), square(0, 1), square(2, 3), identity())
		require.NoError(t, err)
		require.False(t, resolved)
	})

	t.Run("intersect of a nested pair", func(t *testing.T) {
		_, match, _, nestedIsB, resolved, err := prismcells.ResolveIntersect(
			t.Context(), budget(), square(0, 4), square(1, 2), identity())
		require.NoError(t, err)
		require.True(t, resolved)
		require.True(t, nestedIsB)
		require.Empty(t, match.Holes)
	})

	t.Run("two regions fit the arranger cap", func(t *testing.T) {
		require.NoError(t, prismcells.RequireRegionsWithinCap(budget(), square(0, 2), square(1, 3)))
	})
}
