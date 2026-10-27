package sectionhistory

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

var (
	union     = func(a, b bool) bool { return a || b }
	cut       = func(a, b bool) bool { return a && !b }
	intersect = func(a, b bool) bool { return a && b }
)

// disk is a whole counter-clockwise circle.
func disk(u, v, r float64) momentinput.Profile {
	return momentinput.Profile{Outer: circleLoop(u, v, r, true)}
}

// circleLoop is a whole circle walked counter-clockwise, or clockwise as a hole.
func circleLoop(u, v, r float64, ccw bool) sectionrecord.LoopRecord {
	seg := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: u, V: v}, Radius: units.Millimeters(r), CCW: true, TStart: 0, TEnd: 1}
	if !ccw {
		seg.CCW, seg.TStart, seg.TEnd = false, 1, 0
	}
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{seg}}
}

// archedSquare is [0, 10] x [0, 10] whose top side is the upper half circle
// about (5, 10) with radius 5.
func archedSquare() momentinput.Profile {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: p(0, 0), End: p(10, 0), TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p(10, 0), End: p(10, 10), TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Center: p(5, 10), Start: p(10, 10), End: p(0, 10), TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p(0, 10), End: p(0, 0), TStart: 0, TEnd: 1},
	}}}
}

func loopArea(t *testing.T, loop sectionrecord.LoopRecord) float64 {
	t.Helper()
	area, err := classbgeom.LoopSignedArea(loop)
	require.NoError(t, err)
	return area
}

// regionAreas returns each region's outer and hole areas.
func regionAreas(t *testing.T, regions []momentinput.Profile) [][]float64 {
	t.Helper()
	var out [][]float64
	for _, region := range regions {
		areas := []float64{loopArea(t, region.Outer)}
		for _, hole := range region.Holes {
			areas = append(areas, loopArea(t, hole))
		}
		out = append(out, areas)
	}
	return out
}

// lineCorners lists every walked start of an all-line loop, and fails on any
// other segment kind or a narrowed range.
func lineCorners(t *testing.T, loop sectionrecord.LoopRecord) [][2]float64 {
	t.Helper()
	var out [][2]float64
	for _, seg := range loop.Segments {
		line, ok := seg.(sectionrecord.LineSeg)
		require.True(t, ok)
		require.Equal(t, 0.0, line.TStart)
		require.Equal(t, 1.0, line.TEnd)
		out = append(out, [2]float64{line.Start.U, line.Start.V})
	}
	return out
}

func requireRefusal(t *testing.T, err error, msg string) {
	t.Helper()
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, msg)
}

// TestCombineNestedAndSeveralRegions arranges sections that carry holes, lie
// apart, or split, and checks every merged region and hole exactly.
func TestCombineNestedAndSeveralRegions(t *testing.T) {
	t.Parallel()
	t.Run("disjoint union lists both regions", func(t *testing.T) {
		t.Parallel()
		// No edge of either section bounds a cell of the other, so each
		// cell's membership in the other operand is read from the outside.
		regions, err := combine(t.Context(), []momentinput.Profile{square(0, 0, 10, 10)},
			[]momentinput.Profile{disk(20, 5, 2)}, union)
		require.NoError(t, err)
		require.Len(t, regions, 2)
		require.ElementsMatch(t, [][2]float64{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, lineCorners(t, regions[0].Outer))
		require.Equal(t, disk(20, 5, 2), regions[1])
	})
	t.Run("disjoint intersect is empty", func(t *testing.T) {
		t.Parallel()
		_, err := combine(t.Context(), []momentinput.Profile{square(0, 0, 10, 10)},
			[]momentinput.Profile{disk(20, 5, 2)}, intersect)
		requireRefusal(t, err, msgRegion)
	})
	t.Run("cut leaves a circular hole", func(t *testing.T) {
		t.Parallel()
		regions, err := combine(t.Context(), []momentinput.Profile{square(0, 0, 10, 10)},
			[]momentinput.Profile{disk(5, 5, 2)}, cut)
		require.NoError(t, err)
		require.Len(t, regions, 1)
		require.ElementsMatch(t, [][2]float64{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, lineCorners(t, regions[0].Outer))
		require.Equal(t, []sectionrecord.LoopRecord{circleLoop(5, 5, 2, false)}, regions[0].Holes)
	})
	t.Run("union fills a hole with an island", func(t *testing.T) {
		t.Parallel()
		// The ring is a square with a square hole; the disk sits inside the
		// hole without touching it, so it is a second region.
		ring := square(0, 0, 10, 10)
		hole := square(3, 3, 7, 7)
		ring.Holes = []sectionrecord.LoopRecord{reverseLines(hole.Outer)}
		regions, err := combine(t.Context(), []momentinput.Profile{ring}, []momentinput.Profile{disk(5, 5, 1)}, union)
		require.NoError(t, err)
		require.Len(t, regions, 2)
		require.Equal(t, [][]float64{{100, -16}}, regionAreas(t, regions[:1]))
		require.Equal(t, disk(5, 5, 1), regions[1])
	})
	t.Run("intersect keeps a disk inside the square", func(t *testing.T) {
		t.Parallel()
		// The disk cell reads its membership in the square across its own
		// edge, which the enclosing cell walks as a hole.
		regions, err := combine(t.Context(), []momentinput.Profile{square(0, 0, 10, 10)},
			[]momentinput.Profile{disk(5, 5, 2)}, intersect)
		require.NoError(t, err)
		require.Equal(t, []momentinput.Profile{disk(5, 5, 2)}, regions)
	})
	t.Run("intersect keeps the hole inside the overlap", func(t *testing.T) {
		t.Parallel()
		holed := square(0, 0, 10, 10)
		holed.Holes = []sectionrecord.LoopRecord{circleLoop(5, 5, 2, false)}
		regions, err := combine(t.Context(), []momentinput.Profile{holed},
			[]momentinput.Profile{square(-5, 1, 15, 9)}, intersect)
		require.NoError(t, err)
		require.Len(t, regions, 1)
		require.ElementsMatch(t, [][2]float64{{0, 1}, {10, 1}, {10, 9}, {0, 9}}, lineCorners(t, regions[0].Outer))
		require.Equal(t, []sectionrecord.LoopRecord{circleLoop(5, 5, 2, false)}, regions[0].Holes)
	})
	t.Run("a slot splits a holed square", func(t *testing.T) {
		t.Parallel()
		holed := square(0, 0, 10, 10)
		holed.Holes = []sectionrecord.LoopRecord{circleLoop(5, 5, 2, false)}
		regions, err := combine(t.Context(), []momentinput.Profile{holed},
			[]momentinput.Profile{square(-5, 7.5, 15, 9.5)}, cut)
		require.NoError(t, err)
		require.Len(t, regions, 2)
		require.ElementsMatch(t, [][2]float64{{0, 0}, {10, 0}, {10, 7.5}, {0, 7.5}}, lineCorners(t, regions[0].Outer))
		require.Equal(t, []sectionrecord.LoopRecord{circleLoop(5, 5, 2, false)}, regions[0].Holes)
		require.ElementsMatch(t, [][2]float64{{0, 9.5}, {10, 9.5}, {10, 10}, {0, 10}}, lineCorners(t, regions[1].Outer))
		require.Empty(t, regions[1].Holes)
	})
	t.Run("squares sharing a wall join", func(t *testing.T) {
		t.Parallel()
		// Sketch emits the shared wall x = 10 under one square's line only;
		// the other's walk counts it through the coincident window.
		regions, err := combine(t.Context(), []momentinput.Profile{square(0, 0, 10, 10)},
			[]momentinput.Profile{square(10, 0, 13, 10)}, union)
		require.NoError(t, err)
		require.Len(t, regions, 1)
		require.Empty(t, regions[0].Holes)
		require.Equal(t, 130.0, loopArea(t, regions[0].Outer))
	})
	t.Run("a whole arc survives beside certified line cuts", func(t *testing.T) {
		t.Parallel()
		regions, err := combine(t.Context(), []momentinput.Profile{archedSquare()},
			[]momentinput.Profile{square(2, -5, 4, 3)}, union)
		require.NoError(t, err)
		require.Len(t, regions, 1)
		require.Empty(t, regions[0].Holes)
		require.InDelta(t, 110+12.5*math.Pi, loopArea(t, regions[0].Outer), 1e-12)
		arcs := 0
		for _, seg := range regions[0].Outer.Segments {
			switch seg := seg.(type) {
			case sectionrecord.ArcSeg:
				arcs++
				require.Equal(t, archedSquare().Outer.Segments[2], seg)
			case sectionrecord.LineSeg:
				require.Equal(t, 0.0, seg.TStart)
				require.Equal(t, 1.0, seg.TEnd)
			default:
				t.Fatalf("unexpected %T", seg)
			}
		}
		require.Equal(t, 1, arcs)
	})
}

// TestCombineRefusals reaches each refusal of the curved and nested merge.
func TestCombineRefusals(t *testing.T) {
	t.Parallel()
	t.Run("a line crossing an arc", func(t *testing.T) {
		t.Parallel()
		_, err := combine(t.Context(), []momentinput.Profile{archedSquare()},
			[]momentinput.Profile{square(4, 5, 6, 20)}, union)
		requireRefusal(t, err, msgCurveCut)
	})
	t.Run("a line crossing a circle", func(t *testing.T) {
		t.Parallel()
		_, err := combine(t.Context(), []momentinput.Profile{square(0, 0, 10, 10)},
			[]momentinput.Profile{disk(10, 10, 3)}, cut)
		requireRefusal(t, err, msgCurveCut)
	})
	t.Run("a dart whose map Sketch cannot certify", func(t *testing.T) {
		t.Parallel()
		// The dart (2, 5), (5, 5-eps), (8, 5), (5, 5+eps) is so thin that
		// Sketch's map of it is degenerate. It reports the containment of the
		// square's cell unproven (ReasonCoincident), so the cut refuses rather
		// than return the whole square.
		for _, exp := range []int{-30, -37, -44} {
			eps := math.Ldexp(1, exp)
			p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
			corners := []sectionrecord.Point2{p(2, 5), p(5, 5-eps), p(8, 5), p(5, 5+eps)}
			var dart momentinput.Profile
			for i, c := range corners {
				dart.Outer.Segments = append(dart.Outer.Segments,
					sectionrecord.LineSeg{Start: c, End: corners[(i+1)%len(corners)], TStart: 0, TEnd: 1})
			}
			_, err := combine(t.Context(), []momentinput.Profile{square(0, 0, 10, 10)}, []momentinput.Profile{dart}, cut)
			requireRefusal(t, err, "Sketch does not prove each cell's containment")
		}
	})
	t.Run("a triangle below Sketch's area floor", func(t *testing.T) {
		t.Parallel()
		// The triangle encloses 3*h mm^2, below the area Sketch arranges as a
		// face, so no published cell walks its edges while every containment
		// verdict reads proven. The true cut has a triangular hole and the
		// true intersect is the triangle; each operation refuses.
		p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
		loop := func(corners ...sectionrecord.Point2) momentinput.Profile {
			var out momentinput.Profile
			for i, c := range corners {
				out.Outer.Segments = append(out.Outer.Segments,
					sectionrecord.LineSeg{Start: c, End: corners[(i+1)%len(corners)], TStart: 0, TEnd: 1})
			}
			return out
		}
		for _, exp := range []int{-38, -42, -50} {
			h := math.Ldexp(1, exp)
			for _, sliver := range []momentinput.Profile{
				loop(p(2, 5), p(8, 5), p(5, 5+h)),
				loop(p(2, 3), p(8, 6), p(5-h, 4.5+2*h)),
			} {
				for _, op := range []func(a, b bool) bool{cut, union, intersect} {
					_, err := combine(t.Context(), []momentinput.Profile{square(0, 0, 10, 10)},
						[]momentinput.Profile{sliver}, op)
					require.ErrorIs(t, err, decaderr.ErrUnsupported)
					require.ErrorContains(t, err, msgRegion)
				}
				_, err := combine(t.Context(), []momentinput.Profile{square(0, 0, 10, 10)},
					[]momentinput.Profile{sliver}, cut)
				requireRefusal(t, err, "Sketch publishes no cell along part of a recorded loop")
			}
		}
	})
	t.Run("two regions touching at a corner", func(t *testing.T) {
		t.Parallel()
		_, err := combine(t.Context(), []momentinput.Profile{square(0, 0, 10, 10)},
			[]momentinput.Profile{square(10, 10, 15, 15)}, union)
		requireRefusal(t, err, msgRegion)
	})
}

// reverseLines walks an all-line loop backwards.
func reverseLines(loop sectionrecord.LoopRecord) sectionrecord.LoopRecord {
	n := len(loop.Segments)
	out := make([]sectionrecord.CurveSegment, n)
	for i, seg := range loop.Segments {
		line := seg.(sectionrecord.LineSeg)
		out[n-1-i] = sectionrecord.LineSeg{Start: line.End, End: line.Start, TStart: 0, TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: out}
}
