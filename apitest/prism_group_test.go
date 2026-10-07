package apitest_test

import (
	"io"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin docs/general-boolean-design.md §9's A5 rows: a Union of
// disjoint footprints builds a prism group, and a prism-group tool cuts a
// plate in one arrangement.

// sixHoleGroup unions six Ø4 discs of height h at x = 5, 15, ..., 55 into a
// prism group, one disjoint Union at a time.
func sixHoleGroup(t *testing.T, doc *decad.Document, h float64) *decad.Body {
	t.Helper()
	group := discBody(t, doc, 5, 2, h)
	for i := 1; i < 6; i++ {
		var err error
		group, err = decad.Union(t.Context(), group, discBody(t, doc, 5+10*float64(i), 2, h))
		require.NoError(t, err)
		require.False(t, anyFaceIsFaceted(group))
		require.Len(t, group.Lumps(), i+1)
	}
	return group
}

func TestPrismGroupDisjointUnion(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 5, 10, 10)
	b := boxBody(t, doc, 10, 0, 15, 10, 10)

	got, err := decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	require.Len(t, got.Lumps(), 2)
	require.Len(t, got.Faces(), 12)
	volume, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	require.Equal(t, 1000.0, volumeMM(t, volume))
	area, err := got.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, area.Exactness)
	require.Equal(t, 2*(2*50+2*50+2*100.0), area.Value.Base())
	starts, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(got))).AtLeast(1).SelectFaces(got)
	require.NoError(t, err)
	require.Len(t, starts, 2, "CapStart selects every lump's bottom face")
	centroid, err := got.Centroid()
	require.NoError(t, err)
	require.Equal(t, 7.5, centroid.Value.X)
	box, err := got.Bounds()
	require.NoError(t, err)
	require.Equal(t, 0.0, box.Min.X)
	require.Equal(t, 15.0, box.Max.X)
	requireMeshWatertightAt(t, got, 0.1)
	// Two shells: STEP's single-shell writer refuses the group.
	require.Len(t, got.Shells(), 2)
	err = export.STEP(t.Context(), io.Discard, got, units.Millimeters(0.1),
		export.WithSTEPName("group"), export.WithSTEPAuthor("apitest"), export.WithSTEPOrganization("decad"))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "single non-void shell")

	moved, err := got.Placed(t.Context(), mustTranslation(t, 0, 0, 7))
	require.NoError(t, err)
	require.Len(t, moved.Lumps(), 2)
	movedVolume, err := moved.Volume()
	require.NoError(t, err)
	require.Equal(t, 1000.0, volumeMM(t, movedVolume))
}

func mustTranslation(t *testing.T, x, y, z float64) r3.Transform {
	t.Helper()
	tr, err := r3.Translation(r3.Vec{X: x, Y: y, Z: z})
	require.NoError(t, err)
	return tr
}

func TestPrismGroupSixHoleCut(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, -20, 60, 20, 5)
	tool := sixHoleGroup(t, doc, 5)

	got, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	require.Len(t, got.Lumps(), 1)
	// Four plate walls, six hole walls, two caps.
	require.Len(t, got.Faces(), 12)
	volume, err := got.Volume()
	require.NoError(t, err)
	want := 12000 - 6*math.Pi*4*5
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-want), boundMM3(t, volume))
	require.Less(t, boundMM3(t, volume), 1e-9)
	requireMeshWatertightAt(t, got, 0.05)

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	require.NotNil(t, report.Bodies[0].Wall.Minimum, "the wall survey measures the holed plate")

	filleted, err := got.Fillet(t.Context(), verticalConvexEdge(), units.Millimeters(1))
	require.NoError(t, err)
	filletVolume, err := filleted.Volume()
	require.NoError(t, err)
	require.Less(t, volumeMM(t, filletVolume), want)
}

// A group tool whose lumps cross the plate's edge notches it: the crossing
// sub-case classifies every cell against the plate and the group's lumps.
func TestPrismGroupCutNotchesTheEdge(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 30, 10, 4)
	left := boxBody(t, doc, 5, 8, 9, 12, 4)
	right := boxBody(t, doc, 20, 8, 24, 12, 4)
	tool, err := decad.Union(t.Context(), left, right)
	require.NoError(t, err)
	require.Len(t, tool.Lumps(), 2)

	got, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	require.Len(t, got.Lumps(), 1)
	volume, err := got.Volume()
	require.NoError(t, err)
	// Two 4×2 notches, 4 deep.
	want := 1200.0 - 2*8*4
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-want), boundMM3(t, volume))
	requireBodyWatertight(t, got)
}

// A prism overlapping one lump of a group merges into that lump; the other
// lump stays whole.
func TestPrismGroupUnionMergesOneLump(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	group, err := decad.Union(t.Context(), boxBody(t, doc, 0, 0, 10, 10, 5), boxBody(t, doc, 20, 0, 30, 10, 5))
	require.NoError(t, err)
	got, err := decad.Union(t.Context(), group, boxBody(t, doc, 5, 5, 15, 15, 5))
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	require.Len(t, got.Lumps(), 2)
	volume, err := got.Volume()
	require.NoError(t, err)
	want := 5 * (100 + 100 + 100 - 25.0)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-want), boundMM3(t, volume))
	requireBodyWatertight(t, got)

	// A bridge across both lumps of an undisplaced group closes them into one
	// prism. (On the merged group above the bridge would cross fragments a
	// cut displaced, which prism-boolean §3.4 sends to the mesh path.)
	group, err = decad.Union(t.Context(), boxBody(t, doc, 0, 0, 10, 10, 5), boxBody(t, doc, 20, 0, 30, 10, 5))
	require.NoError(t, err)
	bridged, err := decad.Union(t.Context(), group, boxBody(t, doc, 5, 2, 25, 4, 5))
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(bridged))
	require.Len(t, bridged.Lumps(), 1)
	volume, err = bridged.Volume()
	require.NoError(t, err)
	// The bridge adds x from 10 to 20 of its 2 mm strip.
	want = 5 * (200 + 10*2.0)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-want), boundMM3(t, volume))
	requireBodyWatertight(t, bridged)
}

// Two lumps and a bridge that closes a ring around an empty cell: select-all
// would fill the cell, so the union takes the mesh path.
func TestPrismGroupUnionLeavesAnEnclosedVoidToTheMeshPath(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	// Two bars, then a U bridging them closes a 6×6 void at x 2..8, y 2..8.
	group, err := decad.Union(t.Context(), boxBody(t, doc, 0, 0, 10, 2, 3), boxBody(t, doc, 0, 8, 10, 10, 3))
	require.NoError(t, err)
	require.Len(t, group.Lumps(), 2)
	bridge := polyPrismBody(t, doc, [][2]float64{{-1, 1}, {2, 1}, {2, 9}, {-1, 9}}, 3)
	other := polyPrismBody(t, doc, [][2]float64{{8, 1}, {11, 1}, {11, 9}, {8, 9}}, 3)
	ring, err := decad.Union(t.Context(), bridge, other)
	require.NoError(t, err)
	require.Len(t, ring.Lumps(), 2)

	// The mesh path refuses the pair's coplanar caps; an analytic fill would
	// have returned a body with the void's 6·6·3 mm³ added.
	_, err = decad.Union(t.Context(), group, ring)
	var be *decad.BooleanError
	require.ErrorAs(t, err, &be)
	require.Equal(t, decad.BooleanUnsupportedContact, be.Code)
}
