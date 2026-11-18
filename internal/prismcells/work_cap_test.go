package prismcells_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func capPt(u, v float64) momentinput.Point2 { return momentinput.Point2{U: u, V: v} }

func capSquare(x, y, side float64) momentinput.LoopRecord {
	line := func(a, b momentinput.Point2) momentinput.CurveSegment {
		return momentinput.LineSeg{Start: a, End: b, TStart: 0, TEnd: 1}
	}
	return momentinput.LoopRecord{Segments: []momentinput.CurveSegment{
		line(capPt(x, y), capPt(x+side, y)), line(capPt(x+side, y), capPt(x+side, y+side)),
		line(capPt(x+side, y+side), capPt(x, y+side)), line(capPt(x, y+side), capPt(x, y)),
	}}
}

func capCircle(x, y, r float64, ccw bool) momentinput.CurveSegment {
	c := momentinput.CircleSeg{Center: capPt(x, y), Radius: units.Millimeters(r), CCW: ccw, TStart: 0, TEnd: 1}
	if !ccw {
		c.TStart, c.TEnd = 1, 0
	}
	return c
}

// TestCircularSegmentCountMatchesBuilder pins the screen against the
// arranger's own count for the same floats at sweeps near 0, pi and 2pi
// (agent-docs/prism-boolean-design.md Part K §4).
func TestCircularSegmentCountMatchesBuilder(t *testing.T) {
	budget := proofbound.NewWorkBudget(t.Context())
	for name, sweep := range map[string]float64{
		"near zero": 1e-6, "just over a half": math.Pi + 1e-9, "near pi": math.Pi - 1e-9,
		"near two pi": 2*math.Pi - 1e-6, "quarter": math.Pi / 2,
	} {
		t.Run(name, func(t *testing.T) {
			arc := momentinput.ArcSeg{Center: capPt(0, 0), Start: capPt(10, 0),
				End: capPt(10*math.Cos(sweep), 10*math.Sin(sweep)), TStart: 0, TEnd: 1}
			a := momentinput.LoopRecord{Segments: []momentinput.CurveSegment{arc,
				momentinput.LineSeg{Start: arc.End, End: arc.Start, TStart: 0, TEnd: 1}}}
			b := capSquare(100, 100, 1)
			s, _, _, err := prismcells.BuildScene(budget, momentinput.Profile{Outer: a}, momentinput.Profile{Outer: b},
				&prismcells.Reexpression{Identity: true})
			require.NoError(t, err)
			built := prismcells.SketchSceneSegments(s) - 1 - 4 // the closing line and the square
			screen := prismcells.CircularSegmentCount(arc)
			require.GreaterOrEqual(t, screen, built, "the screen never undercounts the builder")
			require.LessOrEqual(t, screen, built+2)
			require.Equal(t, max(2, int(math.Ceil(256*sweep/(2*math.Pi)))), built)
		})
	}
	t.Run("whole circle counts the arranger's own pieces", func(t *testing.T) {
		require.Equal(t, 256, prismcells.CircularSegmentCount(capCircle(0, 0, 1, true)))
	})
	t.Run("narrowed circle counts its range", func(t *testing.T) {
		c := momentinput.CircleSeg{Center: capPt(0, 0), Radius: units.Millimeters(1), CCW: true, TStart: 0.25, TEnd: 0.5}
		require.Equal(t, 66, prismcells.CircularSegmentCount(c))
	})
	t.Run("a sweep that is not finite counts a full turn", func(t *testing.T) {
		arc := momentinput.ArcSeg{Center: capPt(0, 0), Start: capPt(math.NaN(), 0), End: capPt(0, 1), TStart: 0, TEnd: 1}
		require.Equal(t, 257, prismcells.CircularSegmentCount(arc))
	})
}

func TestBuilderRecountRefusesOverCap(t *testing.T) {
	budget := proofbound.NewWorkBudget(t.Context())
	// Seventeen circles recount to 17*256, above the cap, even though no screen ran.
	holes := make([]momentinput.LoopRecord, 0, 16)
	for i := range 16 {
		holes = append(holes, momentinput.LoopRecord{Segments: []momentinput.CurveSegment{capCircle(float64(2*i), 0, 0.5, false)}})
	}
	plate := momentinput.Profile{Outer: capSquare(-1, -1, 40), Holes: holes}
	tool := momentinput.Profile{Outer: momentinput.LoopRecord{Segments: []momentinput.CurveSegment{capCircle(50, 50, 1, true)}}}
	scene, _, _, err := prismcells.BuildScene(budget, plate, tool, &prismcells.Reexpression{Identity: true})
	require.Nil(t, scene)
	require.Error(t, err)
	var limit *prismcells.WorkLimitError
	require.ErrorAs(t, err, &limit)
	require.Equal(t, 17*256+4, limit.Observed)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}

func TestFourteenHolePlateStillFits(t *testing.T) {
	budget := proofbound.NewWorkBudget(t.Context())
	holes := make([]momentinput.LoopRecord, 0, 14)
	for i := range 14 {
		holes = append(holes, momentinput.LoopRecord{Segments: []momentinput.CurveSegment{capCircle(float64(3*i), 0, 1, false)}})
	}
	plate := momentinput.Profile{Outer: capSquare(-5, -5, 60), Holes: holes}
	tool := momentinput.Profile{Outer: momentinput.LoopRecord{Segments: []momentinput.CurveSegment{capCircle(70, 70, 1, true)}}}
	n, within, err := prismcells.RegionsWithinWorkCap(budget, plate, tool)
	require.NoError(t, err)
	require.True(t, within)
	require.Equal(t, 3844, n)
	scene, _, _, err := prismcells.BuildScene(budget, plate, tool, &prismcells.Reexpression{Identity: true})
	require.NoError(t, err)
	require.NotNil(t, scene)
}

func TestWheelLikeSectionFits(t *testing.T) {
	budget := proofbound.NewWorkBudget(t.Context())
	arc := func(r, a0, a1 float64) momentinput.CurveSegment {
		return momentinput.ArcSeg{Center: capPt(0, 0), Start: capPt(r*math.Cos(a0), r*math.Sin(a0)),
			End: capPt(r*math.Cos(a1), r*math.Sin(a1)), TStart: 0, TEnd: 1}
	}
	var holes []momentinput.LoopRecord
	for k := range 5 {
		a := 2 * math.Pi * float64(k) / 5
		w := momentinput.LoopRecord{Segments: []momentinput.CurveSegment{arc(10, a, a+0.8), arc(20, a, a+0.8)}}
		for range 4 { // four sketch-fillet arcs and two lines
			w.Segments = append(w.Segments, arc(1, a, a+math.Pi/2))
		}
		w.Segments = append(w.Segments,
			momentinput.LineSeg{Start: capPt(10, 0), End: capPt(20, 0), TStart: 0, TEnd: 1},
			momentinput.LineSeg{Start: capPt(20, 1), End: capPt(10, 1), TStart: 0, TEnd: 1})
		holes = append(holes, w)
	}
	disc := momentinput.Profile{Outer: momentinput.LoopRecord{Segments: []momentinput.CurveSegment{capCircle(0, 0, 40, true)}}, Holes: holes}
	var teeth []momentinput.Profile
	for range 15 {
		teeth = append(teeth, momentinput.Profile{Outer: capSquare(50, 0, 2)})
	}
	n, within, err := prismcells.RegionsWithinWorkCap(budget, append(teeth, disc)...)
	require.NoError(t, err)
	require.True(t, within)
	require.Less(t, n, 4096)
}
