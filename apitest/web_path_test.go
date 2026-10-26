package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/rib-design.md §6's public-surface tests for WebPath,
// on the pocketed block of rib_test.go: a floor at z = 5, a rib plane at z = 4 and
// ribs 11 mm tall, so each stands 10 mm above the floor.

// slotPath is a slot-shaped run in the sketch plane z = 4: a line from (10, 12) to
// (20, 12), a half circle of radius 5 about (20, 17) turning counter-clockwise
// from (20, 12) to (20, 22), and a line back to (10, 22).
func slotPath(t *testing.T) (*sketch.Sketch, []sketch.Entity) {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 4)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	fixed := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	a, b, c, d, centre := fixed(10, 12), fixed(20, 12), fixed(20, 22), fixed(10, 22), fixed(20, 17)
	bottom := s.CreateLine(a, b)
	arc := s.CreateArc(centre, b, c)
	top := s.CreateLine(c, d)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, []sketch.Entity{bottom, arc, top}
}

// webSlotArea is the area, by sampling a fine grid, of the union of the three ribs
// of slotPath at thickness 2 and overrun 0.5: two rectangles and an annular
// sector, each lengthened 0.5 mm along its path at both ends.
func webSlotArea() float64 {
	const step = 0.005
	inRect := func(x, y, x0, y0, x1, y1 float64) bool { return x >= x0 && x <= x1 && y >= y0 && y <= y1 }
	extra := 0.5 / 5.0
	covered := 0
	for x := 9.0; x <= 27; x += step {
		for y := 10.0; y <= 24; y += step {
			hit := inRect(x, y, 9.5, 11, 20.5, 13) || inRect(x, y, 9.5, 21, 20.5, 23)
			if !hit {
				r := math.Hypot(x-20, y-17)
				angle := math.Atan2(y-17, x-20) // the sector runs from -π/2 - extra to π/2 + extra
				hit = r >= 4 && r <= 6 && angle >= -math.Pi/2-extra && angle <= math.Pi/2+extra
			}
			if hit {
				covered++
			}
		}
	}
	return float64(covered) * step * step
}

func TestWebPathJoinsALineArcLineChain(t *testing.T) {
	t.Parallel()
	doc, part := ribPart(t)
	s, path := slotPath(t)
	operationCount := len(doc.OperationHistory().Operations())
	out, err := decad.WebPath(t.Context(), part, s, path, units.Millimeters(2),
		decad.Distance{D: units.Millimeters(11), Dir: decad.Along}, decad.WithRibOverrun(units.Millimeters(0.5)))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies(), "the part and the rib tools leave the document")
	operations := doc.OperationHistory().Operations()
	require.Len(t, operations, operationCount+1)
	require.Equal(t, decad.OperationWeb, operations[len(operations)-1].Kind())
	added := 10 * webSlotArea()
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(ribPartVolume+added), decadtest.WithinRel(units.Scalar(0.004)))
	// The slot's open side: its bore stays empty above the floor.
	inside, err := out.LocatePoint(t.Context(), r3.NewVec(15, 17, 10), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, inside, "between the two straight ribs")
	on, err := out.LocatePoint(t.Context(), r3.NewVec(25, 17, 10), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, on, "on the arc rib's crown")
}

func TestWebPathOfOneLineIsARib(t *testing.T) {
	t.Parallel()
	_, part := ribPart(t)
	s, lines := ribSketch(t, [4]float64{3, 20, 37, 20})
	out, err := decad.WebPath(t.Context(), part, s, []sketch.Entity{lines[0]}, units.Millimeters(2),
		decad.Distance{D: units.Millimeters(11), Dir: decad.Along})
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(ribPartVolume+30*2*10), decadtest.WithinRel(units.Scalar(1e-6)))
}

func TestWebPathRefusals(t *testing.T) {
	t.Parallel()
	extent := decad.Distance{D: units.Millimeters(11), Dir: decad.Along}
	thick := units.Millimeters(2)
	doc, part := ribPart(t)
	s, path := slotPath(t)
	_, err := decad.WebPath(t.Context(), part, s, nil, thick, extent)
	require.ErrorIs(t, err, decad.ErrDegenerate, "an empty path")
	_, err = decad.WebPath(t.Context(), part, s, []sketch.Entity{nil}, thick, extent)
	require.ErrorIs(t, err, decad.ErrDegenerate, "a nil entity")
	_, foreignPath := slotPath(t)
	_, err = decad.WebPath(t.Context(), part, s, foreignPath, thick, extent)
	require.ErrorIs(t, err, decad.ErrDegenerate, "entities of another sketch")
	around := s.CreatePoint(30, 30)
	s.Fix(around)
	circle := s.CreateCircle(around, 2)
	_, err = decad.WebPath(t.Context(), part, s, append([]sketch.Entity{circle}, path...), thick, extent)
	require.ErrorIs(t, err, decad.ErrDegenerate, "a circle is neither a line nor an arc")
	_, err = decad.WebPath(t.Context(), part, s, path, units.Millimeters(25), extent)
	require.ErrorIs(t, err, decad.ErrDegenerate, "a rib thicker than the arc's radius")
	require.Equal(t, []*decad.Body{part}, doc.Bodies(), "a refused web leaves the part live and alone")
}
