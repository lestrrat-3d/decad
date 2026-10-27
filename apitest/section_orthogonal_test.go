package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-section-design.md's public-surface tests for Body.Section of a faceted
// solid every face of which squares to a world axis.

func sectionArea(t *testing.T, body *decad.Body, frame r3.Frame, outer, holes int) float64 {
	t.Helper()
	section, profiles, err := body.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.Len(t, profiles[0].Outer, outer)
	require.Len(t, profiles[0].Holes, holes)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, area.Exactness)
	return area.Value.Base()
}

// sectionAreas sections body and returns the exact area of each region, in the order Section lists them.
func sectionAreas(t *testing.T, body *decad.Body, frame r3.Frame) []float64 {
	t.Helper()
	section, profiles, err := body.Section(t.Context(), frame)
	require.NoError(t, err)
	var areas []float64
	for _, profile := range profiles {
		measured, err := decad.MeasureProfile(section, profile)
		require.NoError(t, err)
		area, err := measured.Area()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, area.Exactness)
		areas = append(areas, area.Value.Base())
	}
	return areas
}

func planeFrame(t *testing.T, origin, u, v r3.Vec) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	return f
}

func TestSectionOfAnLShapedMeshSolidIsExact(t *testing.T) {
	t.Parallel()
	l := lBracket(t) // 10 x 4 x 4 bar joined to a 4 x 10 x 4 bar: 64 mm² in plan
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	// The plan view at mid height is the L: six corners.
	require.Equal(t, 64.0, sectionArea(t, l, planeFrame(t, r3.NewVec(0, 0, 2), x, y), 6, 0))
	// The same plane described with its axes swapped, and with a flipped one, is the same L.
	require.Equal(t, 64.0, sectionArea(t, l, planeFrame(t, r3.NewVec(0, 0, 2), y, x), 6, 0))
	require.Equal(t, 64.0, sectionArea(t, l, planeFrame(t, r3.NewVec(0, 0, 2), x, y.Scale(-1)), 6, 0))
	// A vertical cut along the long bar, then beyond the short one, where only the long bar stands.
	require.Equal(t, 40.0, sectionArea(t, l, planeFrame(t, r3.NewVec(2, 0, 0), y, z), 4, 0), "x = 2 cuts the whole long bar and the short one: 10 x 4")
	require.Equal(t, 16.0, sectionArea(t, l, planeFrame(t, r3.NewVec(7, 0, 0), y, z), 4, 0))
	// The section enters a feature: extruded 2 mm it is a 2 mm slab of the L.
	section, profiles, err := l.Section(t.Context(), planeFrame(t, r3.NewVec(0, 0, 2), x, y))
	require.NoError(t, err)
	slab, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := slab.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-128), volume.Bound.Base()+1e-9)
}

func TestSectionOfAMeshSolidWithAThroughHoleHasAHole(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(30, 30, 10))
	bore := blockIn(t, doc, r3.NewVec(10, 10, -1), r3.NewVec(20, 20, 11))
	drilled, err := decad.Cut(t.Context(), plate, bore)
	require.NoError(t, err)
	frame := planeFrame(t, r3.NewVec(0, 0, 5), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.Equal(t, 900.0-100, sectionArea(t, drilled, frame, 4, 1))
}

func TestSectionOfAMeshSolidRefusals(t *testing.T) {
	t.Parallel()
	l := lBracket(t)
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	for name, frame := range map[string]r3.Frame{
		"a plane on a face":        planeFrame(t, r3.NewVec(0, 0, 4), x, y),
		"a plane on an inner line": planeFrame(t, r3.NewVec(4, 0, 0), y, z),
		"a plane past the solid":   planeFrame(t, r3.NewVec(0, 0, 9), x, y),
		"an oblique plane":         planeFrame(t, r3.NewVec(0, 0, 2), x, r3.NewVec(0, 0.6, 0.8)),
	} {
		_, _, err := l.Section(t.Context(), frame)
		require.ErrorIs(t, err, decad.ErrUnsupported, name)
	}
}

func TestSectionOfAMeshSolidListsEachRegion(t *testing.T) {
	t.Parallel()
	x, y := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)
	doc := decad.New()
	bar := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(30, 10, 10))
	slot := blockIn(t, doc, r3.NewVec(10, -1, -1), r3.NewVec(20, 11, 11))
	split, err := decad.Cut(t.Context(), bar, slot)
	require.NoError(t, err)
	// A slot through the bar leaves two blocks, each a profile of the one sketch.
	require.ElementsMatch(t, []float64{100, 100}, sectionAreas(t, split, planeFrame(t, r3.NewVec(0, 0, 5), x, y)))
	// A post standing inside a hollow frame is a region of its own beside the frame's wall. A small block stands first
	// in the order of the loops, so the frame's hole must find its own outer loop and not the first.
	frameDoc := decad.New()
	wall := blockIn(t, frameDoc, r3.NewVec(10, 10, 0), r3.NewVec(40, 40, 10))
	cavity := blockIn(t, frameDoc, r3.NewVec(15, 15, -1), r3.NewVec(35, 35, 11))
	ring, err := decad.Cut(t.Context(), wall, cavity)
	require.NoError(t, err)
	post := blockIn(t, frameDoc, r3.NewVec(22, 22, 0), r3.NewVec(28, 28, 10))
	both, err := decad.Union(t.Context(), ring, post)
	require.NoError(t, err)
	corner := blockIn(t, frameDoc, r3.NewVec(0, 0, 0), r3.NewVec(4, 4, 10))
	three, err := decad.Union(t.Context(), both, corner)
	require.NoError(t, err)
	require.ElementsMatch(t, []float64{16, 900 - 400, 36}, sectionAreas(t, three, planeFrame(t, r3.NewVec(0, 0, 5), x, y)))
}
