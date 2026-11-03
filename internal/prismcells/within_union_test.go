package prismcells

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func unionCircle(r float64) momentinput.Profile {
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.CircleSeg{Radius: units.Millimeters(r), CCW: true, TStart: 0, TEnd: 1}}}}
}

func unionRect(x0, y0, x1, y1 float64) momentinput.Profile {
	p := func(x, y float64) sectionrecord.Point2 { return sectionrecord.Point2{U: x, V: y} }
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: p(x0, y0), End: p(x1, y0), TEnd: 1},
		sectionrecord.LineSeg{Start: p(x1, y0), End: p(x1, y1), TEnd: 1},
		sectionrecord.LineSeg{Start: p(x1, y1), End: p(x0, y1), TEnd: 1},
		sectionrecord.LineSeg{Start: p(x0, y1), End: p(x0, y0), TEnd: 1},
	}}}
}

// unionD is the D-shaped region of a circle of radius r cut by the line x = flat.
func unionD(r, flat float64) momentinput.Profile {
	y := math.Sqrt(r*r - flat*flat)
	p := func(x, y float64) sectionrecord.Point2 { return sectionrecord.Point2{U: x, V: y} }
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: p(flat, -y), End: p(flat, y), TEnd: 1},
		sectionrecord.ArcSeg{Center: p(0, 0), Start: p(flat, y), End: p(flat, -y), TStart: 0, TEnd: 1},
	}}}
}

// TestSectionWithinUnion holds the union reading to what the union covers: a
// hole and a cutter that overlap hold a disc neither holds alone, and a union
// with a gap does not.
func TestSectionWithinUnion(t *testing.T) {
	t.Parallel()
	for name, tc := range map[string]struct {
		inner  momentinput.Profile
		outer  []momentinput.Profile
		within bool
	}{
		"disc in a D and a rectangle across its flat": {unionCircle(4), []momentinput.Profile{unionD(4.1, 3.1), unionRect(3, -5, 5, 5)}, true},
		"disc past a D with no rectangle":             {unionCircle(4), []momentinput.Profile{unionD(4.1, 3.1)}, false},
		"disc in a rectangle and a nested disc":       {unionCircle(4), []momentinput.Profile{unionRect(-5, -5, 5, 5), unionCircle(1)}, true},
		"disc in a cross of two rectangles":           {unionCircle(4), []momentinput.Profile{unionRect(-5, -2, 5, 2), unionRect(-2, -5, 2, 5)}, false},
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			within, err := SectionWithinUnion(t.Context(), proofbound.NewWorkBudget(t.Context()),
				[]momentinput.Profile{tc.inner}, tc.outer)
			require.NoError(t, err)
			require.Equal(t, tc.within, within)
		})
	}
}

// TestSectionWithinUnionIdenticalCircle holds the shortcut for a disc and a region
// bounded by the same circle, which Sketch is not asked to arrange: the disc lies
// in the region, and a disc a hair wider does not lie in any region.
func TestSectionWithinUnionIdenticalCircle(t *testing.T) {
	t.Parallel()
	for name, tc := range map[string]struct {
		inner  float64
		within bool
	}{
		"disc of the region's own circle": {4, true},
		"disc 0.1 mm wider":               {4.1, false},
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			within, err := SectionWithinUnion(t.Context(), proofbound.NewWorkBudget(t.Context()),
				[]momentinput.Profile{unionCircle(tc.inner)}, []momentinput.Profile{unionCircle(4), unionRect(30, 30, 31, 31)})
			require.NoError(t, err)
			require.Equal(t, tc.within, within)
		})
	}
}
