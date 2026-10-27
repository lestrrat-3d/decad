package stackedunion

// An internal test: opposedCells is unexported, and the touch refusals it
// owns are also caught later in the BRep build for the rectangles the
// public API can draw, so only this reading shows each rule deciding alone.

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// clockwiseHole records a polygon's corners, given counter-clockwise, as a
// clockwise hole loop of whole lines.
func clockwiseHole(corners ...[2]float64) sectionrecord.LoopRecord {
	n := len(corners)
	segments := make([]sectionrecord.CurveSegment, n)
	for i := range corners {
		from, to := corners[n-1-i], corners[(2*n-2-i)%n]
		segments[i] = sectionrecord.LineSeg{
			Start: sectionrecord.Point2{U: from[0], V: from[1]},
			End:   sectionrecord.Point2{U: to[0], V: to[1]}, TEnd: 1,
		}
	}
	return sectionrecord.LoopRecord{Segments: segments}
}

func rectHole(u0, v0, u1, v1 float64) sectionrecord.LoopRecord {
	return clockwiseHole([2]float64{u0, v0}, [2]float64{u1, v0}, [2]float64{u1, v1}, [2]float64{u0, v1})
}

// TestOpposedCellsReadsFloorsAndCeilings checks the interface scene's floor
// and ceiling areas against each hole pair's own geometry, and that opposed
// holes touching along an edge, at a corner, or at two reflex corners whose
// outsides meet are refused.
func TestOpposedCellsReadsFloorsAndCeilings(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name             string
		lower, upper     sectionrecord.LoopRecord
		floor, ceiling   float64
		floors, ceilings int
		refused          bool
	}{
		// The floor is the upper hole less the lower one, the ceiling the
		// reverse; the 2 x 4 overlap is open through both slabs.
		{name: "overlap", lower: rectHole(-4, -2, 1, 2), upper: rectHole(-6, -2, -2, 2),
			floor: 8, ceiling: 12, floors: 1, ceilings: 1},
		// Two L shapes meeting the 2 x 2 through square at two crossings.
		{name: "crossing-corners", lower: rectHole(-4, -4, 0, 0), upper: rectHole(-6, -6, -2, -2),
			floor: 12, ceiling: 12, floors: 1, ceilings: 1},
		// The upper hole inside the lower one: one ceiling with a hole.
		{name: "nested", lower: rectHole(-6, -6, 6, 6), upper: rectHole(-2, -2, 2, 2),
			ceiling: 144 - 16, ceilings: 1},
		// Sketch resolves the lower hole's top y = 2 + 1e-12, or one ulp
		// above 2, as the upper hole's top y = 2: a shared span whose two
		// carriers differ, which states no exact face edge.
		{name: "near-shared-1e-12", lower: rectHole(-4, -2, 1, 2+1e-12), upper: rectHole(-6, -2, -2, 2), refused: true},
		{name: "near-shared-ulp", lower: rectHole(-4, -2, 1, math.Nextafter(2, 3)), upper: rectHole(-6, -2, -2, 2),
			refused: true},
		// The same near-shared tops with the displaced line's own endpoint
		// ending the merged span: no span reaches the coincident reading, so
		// the edge ending off its own carrier refuses.
		{name: "near-shared-upper-1e-12", lower: rectHole(-4, -2, 1, 2), upper: rectHole(-6, -2, -2, 2+1e-12),
			refused: true},
		{name: "near-shared-upper-ulp", lower: rectHole(-4, -2, 1, 2), upper: rectHole(-6, -2, -2, math.Nextafter(2, 3)),
			refused: true},
		{name: "near-shared-mirrored-1e-12", lower: rectHole(-6, -2, -2, 2), upper: rectHole(-4, -2, 1, 2+1e-12),
			refused: true},
		{name: "edge-touch", lower: rectHole(-2, -2, 2, 2), upper: rectHole(-6, -2, -2, 2), refused: true},
		{name: "corner-touch", lower: rectHole(-2, 2, 2, 6), upper: rectHole(-6, -2, -2, 2), refused: true},
		// Both holes reflex at the origin: the lower one misses the first
		// quadrant there and the upper one the third, so the material
		// below and above meet only at that point.
		{name: "reflex-touch",
			lower: clockwiseHole([2]float64{-2, -2}, [2]float64{2, -2}, [2]float64{2, 0},
				[2]float64{0, 0}, [2]float64{0, 2}, [2]float64{-2, 2}),
			upper: clockwiseHole([2]float64{0, -3}, [2]float64{3, -3}, [2]float64{3, 3},
				[2]float64{-3, 3}, [2]float64{-3, 0}, [2]float64{0, 0}),
			refused: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			budget := proofbound.NewWorkBudget(t.Context())
			cells, err := opposedCells(t.Context(), budget,
				[]sectionrecord.LoopRecord{tc.lower}, []sectionrecord.LoopRecord{tc.upper})
			if tc.refused {
				require.ErrorIs(t, err, errUBMiss)
				return
			}
			require.NoError(t, err)
			var floors, ceilings int
			var floor, ceiling float64
			for _, cell := range cells {
				area, err := classbgeom.LoopSignedArea(cell.region.Outer)
				require.NoError(t, err)
				for _, hole := range cell.region.Holes {
					inner, err := classbgeom.LoopSignedArea(hole)
					require.NoError(t, err)
					area += inner
				}
				if cell.floor {
					floors++
					floor += area
				} else {
					ceilings++
					ceiling += area
				}
			}
			require.Equal(t, tc.floors, floors)
			require.Equal(t, tc.ceilings, ceilings)
			require.Equal(t, tc.floor, floor)
			require.Equal(t, tc.ceiling, ceiling)
		})
	}
}

// circleHole records a circle about (u, 0) as a clockwise hole loop.
func circleHole(u, r float64) sectionrecord.LoopRecord {
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.CircleSeg{
		Center: sectionrecord.Point2{U: u}, Radius: units.Millimeters(r), TStart: 1, TEnd: 0,
	}}}
}

// TestOpposedCellsReadsArcHoles checks the interface scene's floor and
// ceiling areas where a hole is a circle, against the circular segment's
// closed form, and that a circle crossing a circle, or touching the other
// side's edges, is refused.
//
// Shown red: dropping the CurvedCrossingsCut refusal passes both refused
// cases through opposedCells.
func TestOpposedCellsReadsArcHoles(t *testing.T) {
	t.Parallel()
	// The r = 3 circle about (1, 0) left of x = 0, a segment whose chord lies
	// at distance 1 from its centre: open through both slabs.
	segment := 9*math.Acos(1.0/3) - math.Sqrt(8)
	for _, tc := range []struct {
		name             string
		lower, upper     sectionrecord.LoopRecord
		floor, ceiling   float64
		floors, ceilings int
		refused          bool
	}{
		// The floor is the rectangle less that segment, the ceiling the
		// circle less it.
		{name: "round-across-rect", lower: circleHole(1, 3), upper: rectHole(-6, -3, 0, 3),
			floor: 36 - segment, ceiling: 9*math.Pi - segment, floors: 1, ceilings: 1},
		// A circle inside the rectangle: one floor with a round hole.
		{name: "round-in-rect", lower: circleHole(-3, 1), upper: rectHole(-6, -3, 0, 3),
			floor: 36 - math.Pi, floors: 1},
		{name: "crossing-circles", lower: circleHole(0, 2), upper: circleHole(-2, 2), refused: true},
		// The circle crosses x = 0 and touches y = ±2 from inside.
		{name: "tangent-inside", lower: circleHole(-1, 2), upper: rectHole(-6, -2, 0, 2), refused: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			budget := proofbound.NewWorkBudget(t.Context())
			cells, err := opposedCells(t.Context(), budget,
				[]sectionrecord.LoopRecord{tc.lower}, []sectionrecord.LoopRecord{tc.upper})
			if tc.refused {
				require.ErrorIs(t, err, errUBMiss)
				return
			}
			require.NoError(t, err)
			var floors, ceilings int
			var floor, ceiling float64
			for _, cell := range cells {
				area, err := classbgeom.LoopSignedArea(cell.region.Outer)
				require.NoError(t, err)
				for _, hole := range cell.region.Holes {
					inner, err := classbgeom.LoopSignedArea(hole)
					require.NoError(t, err)
					area += inner
				}
				if cell.floor {
					floors++
					floor += area
				} else {
					ceilings++
					ceiling += area
				}
			}
			require.Equal(t, tc.floors, floors)
			require.Equal(t, tc.ceilings, ceilings)
			require.InDelta(t, tc.floor, floor, 1e-9)
			require.InDelta(t, tc.ceiling, ceiling, 1e-9)
		})
	}
}
