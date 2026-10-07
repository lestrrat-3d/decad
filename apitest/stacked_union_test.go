package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin docs/general-boolean-design.md §9's A1 rows: a Union of
// two co-directional prisms whose sweep intervals overlap or touch builds an
// analytic stacked prism.

func requireCapsSelectOneFace(t *testing.T, body *decad.Body) {
	t.Helper()
	for _, ref := range []decad.FeatureRef{decad.CapStart(body), decad.CapEnd(body)} {
		faces, err := decad.Faces(decad.FaceCreatedBy(ref)).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces, 1)
	}
}

func requireMeshWatertightAt(t *testing.T, body *decad.Body, tolerance float64) {
	t.Helper()
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(tolerance))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
	requireWatertight(t, mesh)
}

func TestStackedUnionBossOnPlate(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -20, -20, 20, 20, 10)
	boss := circleBodyAtZ(t, doc, 0, 5, 10, 15)

	got, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	require.Len(t, got.Lumps(), 1)
	// Four plate walls, the boss wall, the plate's bottom cap, the boss's
	// top cap, and one floor: the plate's top with the boss as its hole.
	require.Len(t, got.Faces(), 8)
	requireCapsSelectOneFace(t, got)
	floor := 0
	for _, f := range got.Faces() {
		if f.Surface().Kind() == decad.KindPlane && len(f.Loops()) == 2 {
			floor++
		}
	}
	require.Equal(t, 1, floor, "one planar face carries the plate outer and the boss hole")

	volume, err := got.Volume()
	require.NoError(t, err)
	want := 16000 + 25*15*math.Pi
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-want), boundMM3(t, volume))
	require.Less(t, boundMM3(t, volume), 1e-9)

	area, err := got.Area()
	require.NoError(t, err)
	// Plate: 2·1600 caps less the boss footprint, 4·40·10 walls; boss: its
	// footprint on top and a 2π·5·15 wall.
	wantArea := 3200 + 1600 + 2*math.Pi*5*15
	require.LessOrEqual(t, math.Abs(area.Value.Base()-wantArea), area.Bound.Base())

	box, err := got.Bounds()
	require.NoError(t, err)
	require.Equal(t, -20.0, box.Min.X)
	require.Equal(t, 20.0, box.Max.Y)
	require.Equal(t, 0.0, box.Min.Z)
	require.Equal(t, 25.0, box.Max.Z)

	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.05))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
	requireBodyWatertight(t, got)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

func TestStackedUnionRootedBoss(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -20, -20, 20, 20, 10)
	boss := circleBodyAtZ(t, doc, 0, 5, 5, 20)

	got, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	// The boss's buried wall from z = 5 to 10 is interior and makes no face.
	require.Len(t, got.Faces(), 8)
	requireCapsSelectOneFace(t, got)
	volume, err := got.Volume()
	require.NoError(t, err)
	want := 16000 + 25*15*math.Pi
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-want), boundMM3(t, volume))
	require.Less(t, boundMM3(t, volume), 1e-9)
	requireBodyWatertight(t, got)
}

func TestStackedUnionEqualBoxesStacked(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	lower := boxBody(t, doc, 0, 0, 10, 10, 10)
	upper := boxBodyAtZ(t, doc, 0, 0, 10, 10, 10, 10)

	got, err := decad.Union(t.Context(), lower, upper)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	// One wall per side runs the whole 20 mm, and the shared plane makes no
	// face: the body is a 10×10×20 box.
	require.Len(t, got.Faces(), 6)
	volume, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	require.Equal(t, 2000.0, volumeMM(t, volume))
	area, err := got.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, area.Exactness)
	require.Equal(t, 1000.0, area.Value.Base())
	requireBodyWatertight(t, got)
}

func TestStackedUnionFlangeOnShaft(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	shaft := discBody(t, doc, 0, 5, 40)
	flange := circleBodyAtZ(t, doc, 0, 15, 40, 5)

	got, err := decad.Union(t.Context(), shaft, flange)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	// Shaft wall, flange wall, bottom cap, top cap, and the flange's
	// underside as a ceiling around the shaft.
	require.Len(t, got.Faces(), 5)
	volume, err := got.Volume()
	require.NoError(t, err)
	want := math.Pi * (25*40 + 225*5)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-want), boundMM3(t, volume))
	box, err := got.Bounds()
	require.NoError(t, err)
	// The flange, above the first slab, sets the box's x and y extent.
	require.LessOrEqual(t, math.Abs(box.Max.X-15), box.Bound.Base())
	require.LessOrEqual(t, math.Abs(box.Min.Y+15), box.Bound.Base())
	require.Equal(t, 45.0, box.Max.Z)
	// The ceiling joins the shaft's ring to the flange's, 10 mm apart, so the
	// chords must be fine enough to prove the two rings clear of each other.
	requireMeshWatertightAt(t, got, 0.05)

	// Chained: a second, narrower boss on the flange's top adds a slab.
	top := circleBodyAtZ(t, doc, 0, 3, 45, 2)
	chained, err := decad.Union(t.Context(), got, top)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(chained))
	volume, err = chained.Volume()
	require.NoError(t, err)
	want += math.Pi * 9 * 2
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-want), boundMM3(t, volume))
	requireMeshWatertightAt(t, chained, 0.05)
}

func TestStackedUnionBossCrossingPlateOutlineRefuses(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -20, -20, 20, 20, 10)
	boss := circleBodyAtZ(t, doc, 18, 5, 10, 15)

	_, err := decad.Union(t.Context(), plate, boss)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "crosses the other's outline at a slab interface")
	require.Len(t, doc.Bodies(), 2, "a refused union consumes neither operand")
}

// The ceiling under a near-tangent square boss joins two rings from two slabs:
// the plate's chords cut inside its circle by their sagitta, past the square's
// corners, so the mesh must refuse rather than triangulate crossing rings.
func TestStackedUnionPatchRingsMustClear(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := discBody(t, doc, 0, 10, 10)
	const half = 7.07
	boss := boxBodyAtZ(t, doc, -half, -half, half, half, 10, 5)
	got, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	_, err = got.Tessellate(t.Context(), units.Millimeters(0.05))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Contains(t, err.Error(), "clearance")
	requireMeshWatertightAt(t, got, 0.00001)
}

// A wedge inside a wider, taller wedge, sharing only the apex: the
// clean-nesting match finds the inner wedge's outline whole as a hole of the
// outer wedge's cell, so the union is the outer wedge.
func TestStackedUnionKnifeEdgeNestedWedge(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	w := sketch.NewWorld()
	a := wedgePrism(t, doc, w, w.XY(), [3][2]float64{{0, 0}, {12, -4}, {12, 6}}, 5)
	b := wedgePrism(t, doc, w, w.XY(), [3][2]float64{{0, 0}, {8, -1}, {8, 2}}, 3)
	got, err := decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	require.Len(t, got.Faces(), 5)
	volume, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	require.Equal(t, 600.0, volumeMM(t, volume))
}
