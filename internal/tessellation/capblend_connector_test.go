package tessellation_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/stretchr/testify/require"
)

// connectorFixture is one reflex corner at the origin: two straight walls, a
// unit setback and a quarter-turn connector from PA = (1, 0) to PB = (0, 1)
// held at three chords. The cap samples are, in order, wall 1's start, the
// connector's stations PA, p1, p2, and wall 0's start PB.
func connectorFixture(angles [2]float64) tessellation.CapBlendMotionInput {
	line := survey2d.SideWalk{Kind: survey2d.WalkLine}
	pol := func(deg float64) sectionrecord.Point2 {
		r := deg * math.Pi / 180
		return sectionrecord.Point2{U: math.Cos(r), V: math.Sin(r)}
	}
	in := tessellation.CapBlendMotionInput{
		Walks:     []survey2d.SideWalk{line, line},
		Count:     []int{1, 1},
		ArcCount:  []int{3, 0},
		ArcTh0:    []float64{0, 0},
		ArcTh1:    []float64{math.Pi / 2, 0},
		Chamfered: true, OnStart: true,
		CapPts: []sectionrecord.Point2{
			{U: -5, V: 5}, {U: 1, V: 0}, pol(angles[0]), pol(angles[1]), {U: 0, V: 1},
		},
		CapWallStart: []int{4, 0},
		D:            1,
		HasBandDelta: [2]bool{true, false},
		BandDelta:    [2]float64{1e-15, 0},
		Connectors:   true,
		Joins:        []tessellation.CapBlendJoin{{Arc: true}, {}},
		CapArcStart:  []int{1, -1},
		CapBound:     make([]proofbound.WalkEndBound, 5),
		SideBound:    make([]proofbound.WalkEndBound, 2),
		SideStart:    []int{0, 1},
	}
	return in
}

func connectorMotion(t *testing.T, in tessellation.CapBlendMotionInput) []float64 {
	t.Helper()
	motion, err := tessellation.CapBlendCapMotion(proofbound.NewWorkBudget(t.Context()), in, nil)
	require.NoError(t, err)
	return motion
}

func TestCapBlendCapMotionReflexConnector(t *testing.T) {
	t.Parallel()
	t.Run("ordered stations are finite", func(t *testing.T) {
		t.Parallel()
		motion := connectorMotion(t, connectorFixture([2]float64{30, 60}))
		for _, k := range []int{1, 2, 3} {
			require.Less(t, motion[k], 1e-12, `station %d`, k)
		}
		require.InDelta(t, 1e-15, motion[1], 1e-30, `PA takes the contour displacement`)
		require.InDelta(t, 1e-15, motion[4], 1e-30, `a wall that starts at a connector reads PB's displacement, not the previous arc's foot`)
	})
	t.Run("connectors off keeps every station unbounded", func(t *testing.T) {
		t.Parallel()
		in := connectorFixture([2]float64{30, 60})
		in.Connectors = false
		motion := connectorMotion(t, in)
		for _, k := range []int{1, 2, 3} {
			require.True(t, math.IsInf(motion[k], 1), `station %d`, k)
		}
	})
	// The order check refuses a loop whose held stations do not provably run
	// in order inside the window.
	for name, angles := range map[string][2]float64{
		"station outside the window": {120, 60},
		"stations out of order":      {60, 30},
		"coincident stations":        {30, 30},
		"station before PA":          {-10, 60},
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			motion := connectorMotion(t, connectorFixture(angles))
			for _, k := range []int{1, 2, 3} {
				require.True(t, math.IsInf(motion[k], 1), `station %d`, k)
			}
		})
	}
	t.Run("clockwise connector", func(t *testing.T) {
		t.Parallel()
		in := connectorFixture([2]float64{60, 30})
		in.ArcTh0[0], in.ArcTh1[0] = math.Pi/2, 0
		in.CapPts[1], in.CapPts[4] = sectionrecord.Point2{U: 0, V: 1}, sectionrecord.Point2{U: 1, V: 0}
		motion := connectorMotion(t, in)
		for _, k := range []int{1, 2, 3} {
			require.Less(t, motion[k], 1e-12, `station %d`, k)
		}
	})
	t.Run("junction bound that swamps the margin refuses", func(t *testing.T) {
		t.Parallel()
		in := connectorFixture([2]float64{30, 60})
		in.SideBound[0] = proofbound.WalkEndBound{U: 0.5, V: 0.5}
		motion := connectorMotion(t, in)
		for _, k := range []int{1, 2, 3} {
			require.True(t, math.IsInf(motion[k], 1) || motion[k] > 0.5, `station %d`, k)
		}
	})
}

func TestCapBlendConnectorVolume(t *testing.T) {
	t.Parallel()
	in := connectorFixture([2]float64{30, 60})
	motion := connectorMotion(t, in)
	height := [2]float64{0.5, 0}
	got := tessellation.CapBlendConnectorVolume(height, in, in.CapPts, motion, in.D, 0)
	// Three chords of a unit circle over a quarter turn omit a segment of
	// (Δ - sin Δ)/2 each at the cap level, and the cone scales it by s².
	delta := math.Pi / 6
	segment := (delta - math.Sin(delta)) / 2
	trueArea := 3 * segment * height[0]
	require.GreaterOrEqual(t, got, trueArea, `the bound covers the omitted segments`)
	require.Less(t, got, 10*trueArea, `and stays within the chord bound's own looseness`)

	t.Run("no connector costs nothing", func(t *testing.T) {
		t.Parallel()
		none := in
		none.CapArcStart = []int{-1, -1}
		require.Zero(t, tessellation.CapBlendConnectorVolume(height, none, in.CapPts, motion, in.D, 0))
	})
	t.Run("an unbounded station or a vanishing radius refuses", func(t *testing.T) {
		t.Parallel()
		inf := connectorMotion(t, connectorFixture([2]float64{120, 60}))
		require.True(t, math.IsInf(tessellation.CapBlendConnectorVolume(height, in, in.CapPts, inf, in.D, 0), 1))
		require.True(t, math.IsInf(tessellation.CapBlendConnectorVolume(height, in, in.CapPts, motion, 1, 1), 1))
	})
}
