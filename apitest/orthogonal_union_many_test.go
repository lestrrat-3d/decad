package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-boolean-design.md B11's public-surface tests for the union of three or more
// axis-square faceted solids.

func TestOrthogonalUnionOfSeveralBodiesJoinsThemOnOneGrid(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 4, 4))
	b := blockIn(t, doc, r3.NewVec(10, 0, 0), r3.NewVec(20, 4, 4))
	c := blockIn(t, doc, r3.NewVec(20, 0, 0), r3.NewVec(30, 4, 4))
	bar, err := decad.Union(t.Context(), a, b, c)
	require.NoError(t, err)
	requireVolume(t, bar, 480)
	require.Len(t, bar.Faces(), 6, "three bars end to end are one bar")
	require.Equal(t, []*decad.Body{bar}, doc.Bodies())
	operations := doc.OperationHistory().Operations()
	require.Equal(t, decad.OperationUnion, operations[len(operations)-1].Kind())
	require.Len(t, operations[len(operations)-1].Inputs(), 3)
}

func TestOrthogonalUnionOfSeveralBodiesTakesAnExtrudedPartner(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 4, 4))
	b := boxBody(t, doc, 10, 0, 20, 4, 4)
	c := blockIn(t, doc, r3.NewVec(20, 0, 0), r3.NewVec(30, 4, 4))
	bar, err := decad.Union(t.Context(), a, b, c)
	require.NoError(t, err)
	requireVolume(t, bar, 480)
}

func TestPatternJoinOfAMeshSolidIsExact(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	base := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(100, 20, 4))
	tooth := blockIn(t, doc, r3.NewVec(2, 2, 4), r3.NewVec(6, 18, 12))
	comb, err := decad.PatternJoin(t.Context(), base, tooth, decad.LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: units.Millimeters(10), Count: 9})
	require.NoError(t, err)
	requireVolume(t, comb, 100*20*4+9*4*16*8)
	inside, err := comb.LocatePoint(t.Context(), r3.NewVec(84, 10, 8), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, inside, "the ninth tooth, at 82 to 86")
	gap, err := comb.LocatePoint(t.Context(), r3.NewVec(88, 10, 8), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, gap, "between the ninth tooth and the end")
}

func TestOrthogonalUnionOfSeveralBodiesLeavesNearCoincidentFacesToTheGeneralRoute(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 4, 4))
	b := blockIn(t, doc, r3.NewVec(10, 0, 0), r3.NewVec(20, 4, 4))
	// A face almost, but not quite, in the plane of another's: the grid will not merge two coordinates 1e-12 apart,
	// so the set is joined two at a time by the general mesh route.
	c := blockIn(t, doc, r3.NewVec(20+1e-12, 0, 0), r3.NewVec(30, 4, 4))
	bar, err := decad.Union(t.Context(), a, b, c)
	if err != nil {
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Len(t, doc.Bodies(), 3, "a refusal leaves every body live")
		return
	}
	volume, err := bar.Volume()
	require.NoError(t, err)
	require.InDelta(t, 480, volume.Value.Base(), volume.Bound.Base()+1e-6)
}
