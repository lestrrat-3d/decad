package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/gap-tolerance-design.md §5.2 item 5's public-surface tests: a tool flush with both faces of a plate,
// which no translation of the tool opens, is grown about its centre by the shifted retry.

// slantedPlate is a 30 x 30 x 10 mesh plate whose top is only 20 wide, so one side slants and the plate is not an
// axis-square solid: its volume is 7500.
func slantedPlate(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(30, 0, 0), r3.NewVec(30, 30, 0), r3.NewVec(0, 30, 0),
		r3.NewVec(0, 0, 10), r3.NewVec(20, 0, 10), r3.NewVec(20, 30, 10), r3.NewVec(0, 30, 10),
	}
	var tris [][3]int
	for _, q := range [][4]int{{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}} {
		tris = append(tris, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
	}
	m, err := decad.NewMeshBody(t.Context(), verts, tris)
	require.NoError(t, err)
	plate, err := decad.SolidFromMesh(t.Context(), doc, m)
	require.NoError(t, err)
	return plate
}

// flushPin is a cylinder of radius 3 at (8, 15) from z = 0 to 10, as tall as the plate.
func flushPin(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	centre := s.CreatePoint(8, 15)
	s.Fix(centre)
	s.CreateCircle(centre, 3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	pin, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return pin
}

func TestCutByAToolFlushWithBothFacesOfAMeshPlate(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := slantedPlate(t, doc)
	out, err := decad.Cut(t.Context(), plate, flushPin(t, doc))
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	// A round hole of radius 3 through the 10 mm plate, polygonal within the tessellation's bound.
	require.InDelta(t, 7500-math.Pi*9*10, volume.Value.Base(), volume.Bound.Base()+0.5)
	require.Less(t, volume.Bound.Base(), 2.0)
	// The hole is clear through: the axis is void from the bottom face to the top, and the wall beside it solid.
	for point, want := range map[r3.Vec]decad.PointLocation{
		r3.NewVec(8, 15, 0.5): decad.PointOutside,
		r3.NewVec(8, 15, 5):   decad.PointOutside,
		r3.NewVec(8, 15, 9.5): decad.PointOutside,
		r3.NewVec(13, 15, 5):  decad.PointInside,
	} {
		got, err := out.LocatePoint(t.Context(), point, units.Millimeters(0.01))
		require.NoError(t, err)
		require.Equal(t, want, got, "%v", point)
	}
}

func TestIntersectWithAToolFlushWithBothFacesOfAMeshPlate(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	out, err := decad.Intersect(t.Context(), slantedPlate(t, doc), flushPin(t, doc))
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	require.InDelta(t, math.Pi*9*10, volume.Value.Base(), volume.Bound.Base()+0.5)
}

func TestUnionWithAToolFlushWithBothFacesOfAMeshPlate(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	out, err := decad.Union(t.Context(), slantedPlate(t, doc), flushPin(t, doc))
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	require.InDelta(t, 7500, volume.Value.Base(), volume.Bound.Base()+0.5, "the pin lies inside the plate")
}
