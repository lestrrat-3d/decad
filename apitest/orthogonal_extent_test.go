package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/mesh-solid-design.md §6's public-surface tests: a mesh solid's axis-square face is a stop for
// ToFace and a mesh solid is a body ThroughAll can meet.

func TestToFaceStopsAtAMeshSolidFace(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := blockIn(t, doc, r3.NewVec(0, 0, 5), r3.NewVec(40, 30, 10))
	s, profiles := embossSketch(t, 0, [4]float64{10, 10, 20, 16})
	top := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1)))
	tool, err := decad.Extrude(t.Context(), doc, s, profiles[0], decad.ToFace{Body: plate, Face: top})
	require.NoError(t, err)
	box, err := tool.Bounds()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(10, 10, 0), box.Min)
	require.Equal(t, r3.NewVec(20, 16, 10), box.Max)
	requireVolume(t, tool, 600)
	volume, err := tool.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness, "the stop is the face's own plane, with no displacement")

	// The tool pocket through the plate takes the part of it between z = 5 and 10.
	out, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	requireVolume(t, out, 40*30*5-10*6*5)
}

func TestToFaceRefusesAMeshSolidFaceSquareToAnotherAxis(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := blockIn(t, doc, r3.NewVec(0, 0, 5), r3.NewVec(40, 30, 10))
	s, profiles := embossSketch(t, 0, [4]float64{10, 10, 20, 16})
	_, err := decad.Extrude(t.Context(), doc, s, profiles[0],
		decad.ToFace{Body: plate, Face: decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)))})
	require.ErrorIs(t, err, decad.ErrUnsupported, "a side face is no perpendicular cap for a sweep along z")
	require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "a refusal leaves the document as it was")
}

func TestThroughAllMeetsAMeshSolid(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	_ = blockIn(t, doc, r3.NewVec(0, 0, 5), r3.NewVec(40, 30, 10))
	s, profiles := embossSketch(t, 0, [4]float64{10, 10, 20, 16})
	tool, err := decad.Extrude(t.Context(), doc, s, profiles[0], decad.ThroughAll{Dir: decad.Along})
	require.NoError(t, err)
	box, err := tool.Bounds()
	require.NoError(t, err)
	require.InDelta(t, 10, box.Max.Z, 1e-9, "the sweep ends at the far side of the plate")
	require.Less(t, box.Bound.Base(), 1e-9)
	volume, err := tool.Volume()
	require.NoError(t, err)
	require.InDelta(t, 600, volume.Value.Base(), 1e-9)
}
