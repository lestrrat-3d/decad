package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func circleBodyAtZ(t *testing.T, doc *decad.Document, x, radius, z, height float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(x, 0)
	s.Fix(center)
	s.CreateCircle(center, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func TestBlindCutBuildsAnalyticPocket(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 10, 10, 10)
	tool := boxBodyAtZ(t, doc, 3, 3, 7, 7, 6, 4)

	pocket, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(pocket))
	require.Len(t, pocket.Lumps(), 1)
	require.Len(t, pocket.Faces(), 11)
	volume, err := pocket.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	require.Equal(t, 936.0, volumeMM(t, volume))
	centroid, err := pocket.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, centroid.Exactness)
	require.Equal(t, 5.0, centroid.Value.X)
	require.Equal(t, 5.0, centroid.Value.Y)
	require.LessOrEqual(t, math.Abs(centroid.Value.Z-187.0/39.0), centroid.Bound.Base())
	mesh, err := pocket.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
	requireBodyWatertight(t, pocket)
}

func TestBlindPocketPlacedAndVerified(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 10, 10, 10)
	tool := boxBodyAtZ(t, doc, 3, 3, 7, 7, 6, 4)
	pocket, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	wallReport, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, wallReport.Status)
	_, staged := findDiagnostic(wallReport.Diagnostics, decad.DiagUnsupportedSurveyPayload)
	require.True(t, staged)
	rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	placed, err := pocket.Placed(t.Context(), rotation)
	require.NoError(t, err)
	require.Len(t, placed.Faces(), len(pocket.Faces()))
	volume, err := placed.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	require.Equal(t, 936.0, volumeMM(t, volume))
	requireBodyWatertight(t, placed)
}

// TestBlindPocketFilletsItsOuterEdges rounds a blind pocket's four convex
// vertical edges, r = 1, through docs/brep-modify-design.md's route E: each
// corner loses (1 − π/4)·10, so the volume 936 becomes 896 + 10π, and the
// result is watertight. π lies in [3.14159265358979, 3.14159265358980], and
// the reading covers both ends.
func TestBlindPocketFilletsItsOuterEdges(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 10, 10, 10)
	tool := boxBodyAtZ(t, doc, 3, 3, 7, 7, 6, 4)
	pocket, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	filleted, err := pocket.Fillet(t.Context(), verticalConvexEdge(), units.Millimeters(1))
	require.NoError(t, err)
	volume, err := filleted.Volume()
	require.NoError(t, err)
	for _, pi := range []string{"3.14159265358979", "3.14159265358980"} {
		exact, ok := new(big.Rat).SetString(pi)
		require.True(t, ok)
		exact.Mul(exact, big.NewRat(10, 1)).Add(exact, big.NewRat(896, 1))
		requireReadingCovers(t, volume, exact)
	}
	requireBodyWatertight(t, filleted)
}

func TestBlindRoundBoreTessellates(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -10, -10, 10, 10, 10)
	tool := circleBodyAtZ(t, doc, 0, 2, 6, 4)
	pocket, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(pocket))
	volume, err := pocket.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-(4000-16*math.Pi)), boundMM3(t, volume))
	requireBodyWatertight(t, pocket)
}

func TestBlindBoreThenCircularThroughHoles(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -10, -10, 10, 10, 10)
	pocketTool := circleBodyAtZ(t, doc, 0, 2, 6, 4)
	part, err := decad.Cut(t.Context(), plate, pocketTool)
	require.NoError(t, err)
	for _, x := range []float64{-5, 5} {
		tool := discBody(t, doc, x, 0.5, 10)
		part, err = decad.Cut(t.Context(), part, tool)
		require.NoError(t, err)
		require.False(t, anyFaceIsFaceted(part))
	}
	require.Len(t, part.Faces(), 10)
	volume, err := part.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-(4000-21*math.Pi)), boundMM3(t, volume))
	requireBodyWatertight(t, part)
}

func TestBlindCutFromBottomFace(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 10, 10, 10)
	tool := boxBody(t, doc, 3, 3, 7, 7, 4)
	pocket, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(pocket))
	require.Len(t, pocket.Faces(), 11)
	volume, err := pocket.Volume()
	require.NoError(t, err)
	require.Equal(t, 936.0, volumeMM(t, volume))
	requireBodyWatertight(t, pocket)
}

func TestBlindCutThenTwoThroughHoles(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 10, 10, 10)
	pocketTool := boxBodyAtZ(t, doc, 3, 3, 7, 7, 6, 4)
	pocket, err := decad.Cut(t.Context(), plate, pocketTool)
	require.NoError(t, err)
	firstTool := boxBody(t, doc, 1, 1, 2, 2, 10)
	first, err := decad.Cut(t.Context(), pocket, firstTool)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(first))
	secondTool := boxBody(t, doc, 8, 8, 9, 9, 10)
	second, err := decad.Cut(t.Context(), first, secondTool)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(second))
	volume, err := second.Volume()
	require.NoError(t, err)
	require.Equal(t, 916.0, volumeMM(t, volume))
	requireBodyWatertight(t, second)
}

func TestBlindCutKeepsExistingThroughHole(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 10, 10, 10)
	throughTool := boxBody(t, doc, 1, 1, 2, 2, 10)
	withThrough, err := decad.Cut(t.Context(), plate, throughTool)
	require.NoError(t, err)
	pocketTool := boxBodyAtZ(t, doc, 3, 3, 7, 7, 6, 4)
	pocket, err := decad.Cut(t.Context(), withThrough, pocketTool)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(pocket))
	volume, err := pocket.Volume()
	require.NoError(t, err)
	require.Equal(t, 926.0, volumeMM(t, volume))
	requireBodyWatertight(t, pocket)
}

func TestBlindPocketCanEnterMeshBoolean(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 10, 10, 10)
	pocketTool := boxBodyAtZ(t, doc, 3, 3, 7, 7, 6, 4)
	pocket, err := decad.Cut(t.Context(), plate, pocketTool)
	require.NoError(t, err)
	crossingTool := boxBodyAtZ(t, doc, 5, 5, 11, 11, -1, 12)
	result, err := decad.Cut(t.Context(), pocket, crossingTool)
	require.NoError(t, err)
	requireBodyWatertight(t, result)
}
