package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-boolean-design.md B20's public-surface tests: a Hole in a planar face of a mesh solid
// that is tilted from every axis.

func TestHoleInATiltedMeshFaceRunsAlongTheFacesNormal(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	ramp, face := rampSolid(t, doc)
	before, err := ramp.Volume()
	require.NoError(t, err)
	out, err := decad.Hole(t.Context(), ramp, decad.HoleSpec{
		Face:     face,
		At:       r3.NewVec(10, 5, 9),
		Diameter: units.Millimeters(4),
		Depth:    decad.BlindHole{Depth: units.Millimeters(3)},
	})
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	after, err := out.Volume()
	require.NoError(t, err)
	// The bore is perpendicular to the face, so its mouth is flat and it removes pi * 2^2 * 3.
	removed := before.Value.Base() - after.Value.Base()
	require.InDelta(t, math.Pi*4*3, removed, before.Bound.Base()+after.Bound.Base()+0.2)
	// The ramp's normal is (-1, 0, 2) / sqrt(5): the axis runs 3 mm along its inverse from (10, 5, 9).
	n := r3.NewVec(-1, 0, 2).Scale(1 / math.Sqrt(5))
	for point, want := range map[r3.Vec]decad.PointLocation{
		r3.NewVec(10, 5, 9).Sub(n.Scale(1.5)):                           decad.PointOutside,
		r3.NewVec(10, 5, 9).Sub(n.Scale(2.5)):                           decad.PointOutside,
		r3.NewVec(10, 5, 9).Sub(n.Scale(3.5)):                           decad.PointInside,
		r3.NewVec(10, 5, 9).Sub(n.Scale(1.5)).Add(r3.NewVec(0, 2.3, 0)): decad.PointInside,
	} {
		got, err := out.LocatePoint(t.Context(), point, units.Millimeters(0.01))
		require.NoError(t, err)
		require.Equal(t, want, got, "%v", point)
	}
}
