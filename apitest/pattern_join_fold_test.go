package apitest_test

import (
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/union-fold-design.md's U6 tests: a pattern of a polygonal pin joined to a mesh plate
// that no certified route takes as a set is joined two at a time by Union.

func TestPatternUnionFoldsPolygonalPinsIntoAMeshPlate(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := slantedPlate(t, doc)
	// A hexagon of 5 mm about (8, 15), as tall as the plate and so flush with both its faces, stepped 4 mm along y three
	// times: each pin overlaps the last, and all lie inside the plate.
	hex := polygonPrismAtZ(t, doc, 0, 10,
		[2]float64{11, 15}, [2]float64{9.5, 17.6}, [2]float64{6.5, 17.6}, [2]float64{5, 15}, [2]float64{6.5, 12.4}, [2]float64{9.5, 12.4})
	copies, err := decad.PatternCopies(t.Context(), hex,
		decad.LinearPattern{Dir: r3.NewVec(0, 1, 0), Step: units.Millimeters(4), Count: 3})
	require.NoError(t, err)
	out, err := decad.Union(t.Context(), plate, append([]*decad.Body{hex}, copies...)...)
	require.NoError(t, err)
	require.True(t, slices.Equal([]*decad.Body{out}, doc.Bodies()), "the plate, the pin and the copies are consumed")
	volume, err := out.Volume()
	require.NoError(t, err)
	require.InDelta(t, 7500, volume.Value.Base(), volume.Bound.Base()+1e-9, "the pins lie inside the plate")
	require.Less(t, volume.Bound.Base(), 0.1)
	require.Len(t, out.Faces(), 6, "the shrunk pins lie inside the plate, so the plate's own six faces are all there is")
}

func TestPatternUnionFoldsRoundPinsIntoAMeshPlate(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := slantedPlate(t, doc)
	// Round pins of radius 3 about (8, 15), stepped 4 mm along y: the walls of neighbouring pins cross, and the caps are
	// flush with the plate's faces. A round pin is held within its chord tolerance, so the shrink that keeps it inside
	// the plate must pass that bound, and the result's bound states it.
	pin := flushPin(t, doc)
	copies, err := decad.PatternCopies(t.Context(), pin,
		decad.LinearPattern{Dir: r3.NewVec(0, 1, 0), Step: units.Millimeters(4), Count: 3})
	require.NoError(t, err)
	out, err := decad.Union(t.Context(), plate, append([]*decad.Body{pin}, copies...)...)
	require.NoError(t, err)
	require.True(t, slices.Equal([]*decad.Body{out}, doc.Bodies()))
	volume, err := out.Volume()
	require.NoError(t, err)
	require.InDelta(t, 7500, volume.Value.Base(), volume.Bound.Base()+1e-9)
	require.Less(t, volume.Bound.Base(), 5.0)
	require.Len(t, out.Faces(), 6)
}

func TestPatternUnionFoldRefusalLeavesEveryBodyLive(t *testing.T) {
	t.Parallel()
	doc := decad.New(decad.ExactOnly())
	plate := slantedPlate(t, doc)
	hex := polygonPrismAtZ(t, doc, 0, 10,
		[2]float64{11, 15}, [2]float64{9.5, 17.6}, [2]float64{6.5, 17.6}, [2]float64{5, 15}, [2]float64{6.5, 12.4}, [2]float64{9.5, 12.4})
	before := doc.Bodies()
	copies, err := decad.PatternCopies(t.Context(), hex,
		decad.LinearPattern{Dir: r3.NewVec(0, 1, 0), Step: units.Millimeters(4), Count: 3})
	require.NoError(t, err)
	_, err = decad.Union(t.Context(), plate, append([]*decad.Body{hex}, copies...)...)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.True(t, slices.Equal(append(slices.Clone(before), copies...), doc.Bodies()),
		"a refused Union leaves the plate, the pin and every copy live")
	for _, c := range copies {
		require.NoError(t, decad.Remove(c))
	}
	require.True(t, slices.Equal([]*decad.Body{plate, hex}, doc.Bodies()))
}
