package loftcontact_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func fitSegment(points int) sectionrecord.FitSplineSeg {
	return sectionrecord.FitSplineSeg{Fit: make([]sectionrecord.Point2, points), TStart: 0, TEnd: 1}
}

// The work-count overflow check in FitPreflight needs a fit spline of more than
// 2^32 points, and a NaN scene start is also refused by the exact-size gate,
// so neither has an input of its own.
func TestFitPreflight(t *testing.T) {
	t.Parallel()

	budget := func() *proofbound.WorkBudget {
		return proofbound.NewWallWorkBudgetWithOperation(1<<20, proofbound.NewWorkBudget(t.Context()))
	}
	line := sectionrecord.LineSeg{TEnd: 1}

	t.Run("charges every fit spline of the four scanned loops", func(t *testing.T) {
		t.Parallel()
		// The prism loop is scanned twice: first and last.
		visits, ok, err := loftcontact.FitPreflight(budget(), []sectionrecord.CurveSegment{fitSegment(3), line},
			[]sectionrecord.CurveSegment{fitSegment(4)}, []sectionrecord.CurveSegment{line, fitSegment(5)})
		require.NoError(t, err)
		require.True(t, ok)
		per := func(n int) uint64 { return freeform.FitInterpolantCost(n) + uint64(n) }
		require.Equal(t, 2*per(3)+per(4)+per(5), visits)
	})
	t.Run("a fit spline through two points is the smallest admitted", func(t *testing.T) {
		t.Parallel()
		visits, ok, err := loftcontact.FitPreflight(budget(), nil, []sectionrecord.CurveSegment{fitSegment(2)}, nil)
		require.NoError(t, err)
		require.True(t, ok)
		require.Equal(t, freeform.FitInterpolantCost(2)+2, visits)
	})
	t.Run("nothing to charge without fit splines", func(t *testing.T) {
		t.Parallel()
		visits, ok, err := loftcontact.FitPreflight(budget(), []sectionrecord.CurveSegment{line}, nil, nil)
		require.NoError(t, err)
		require.True(t, ok)
		require.Zero(t, visits)
	})
	t.Run("a fit spline through fewer than two points is refused", func(t *testing.T) {
		t.Parallel()
		for _, loops := range [][3][]sectionrecord.CurveSegment{
			{{fitSegment(1)}, nil, nil}, {nil, {fitSegment(0)}, nil}, {nil, nil, {fitSegment(1)}},
		} {
			visits, ok, err := loftcontact.FitPreflight(budget(), loops[0], loops[1], loops[2])
			require.NoError(t, err)
			require.False(t, ok)
			require.Zero(t, visits)
		}
	})
	t.Run("an oversized fit spline exhausts the work budget", func(t *testing.T) {
		t.Parallel()
		small := proofbound.NewWallWorkBudgetWithOperation(4096, proofbound.NewWorkBudget(t.Context()))
		visits, ok, err := loftcontact.FitPreflight(small, nil, []sectionrecord.CurveSegment{fitSegment(65)}, nil)
		require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
		require.False(t, ok)
		require.Zero(t, visits)
	})
}

func TestSameKind(t *testing.T) {
	t.Parallel()

	line, circle, arc := sectionrecord.CurveSegment(sectionrecord.LineSeg{}), sectionrecord.CurveSegment(sectionrecord.CircleSeg{}),
		sectionrecord.CurveSegment(sectionrecord.ArcSeg{})
	fit := sectionrecord.CurveSegment(sectionrecord.FitSplineSeg{})
	ellipse := sectionrecord.CurveSegment(sectionrecord.EllipseSeg{})
	all := []sectionrecord.CurveSegment{line, circle, arc, fit}
	for i, a := range all {
		for j, b := range all {
			require.Equal(t, i == j, loftcontact.SameKind(a, b), "%T against %T", a, b)
		}
	}
	require.False(t, loftcontact.SameKind(ellipse, ellipse), "a kind the loft contact proofs do not read is never the same")
	require.False(t, loftcontact.SameKind(line, ellipse))
}

// The exact-size gate inside SceneRangeFits has no float64 input of its own:
// the finest float needs far fewer bits than the cap allows.
func TestSceneRangeFits(t *testing.T) {
	t.Parallel()

	require.True(t, loftcontact.SceneRangeFits(0, 1))
	require.True(t, loftcontact.SceneRangeFits(0.25, 0.75))
	for name, span := range map[string][2]float64{
		"empty":        {0.5, 0.5},
		"reversed":     {0.75, 0.25},
		"below zero":   {-0.1, 0.5},
		"above one":    {0.5, 1.1},
		"NaN start":    {math.NaN(), 0.5},
		"infinite end": {0.5, math.Inf(1)},
	} {
		require.False(t, loftcontact.SceneRangeFits(span[0], span[1]), name)
	}
}
