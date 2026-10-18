package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func manyRectangleSections(t *testing.T, widths, depths, levels []float64) []decad.LoftSection {
	t.Helper()
	require.Equal(t, len(widths), len(depths))
	require.Equal(t, len(widths), len(levels))
	world := sketch.NewWorld()
	sections := make([]decad.LoftSection, len(widths))
	for i := range sections {
		plane := world.XY()
		if i != 0 {
			var err error
			plane, err = world.CreateOffsetPlane(world.XY(), levels[i])
			require.NoError(t, err)
		}
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		rect := s.CreateRectangle(-widths[i], -depths[i], widths[i], depths[i])
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		profiles := s.Profiles()
		require.Len(t, profiles, 1)
		require.True(t, profiles[0].Valid)
		sections[i] = decad.LoftSection{Sketch: s, Profile: profiles[0]}
	}
	return sections
}

func TestLoftSectionsFiveIndependentRectanglesKeepVolumeAndMeshProof(t *testing.T) {
	sections := manyRectangleSections(t, []float64{2, 3, 4, 5, 6},
		[]float64{3, 4, 5, 7, 8}, []float64{0, 4, 8, 12, 16})
	doc := decad.New()
	body, err := decad.LoftSections(t.Context(), doc, sections...)
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	require.Len(t, body.Faces(), 6)
	volume, err := body.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-1492), volume.Bound.Base())
	require.Less(t, volume.Bound.Base(), 2.0)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.05)
	for i, section := range [][3]float64{{2, 3, 0}, {3, 4, 4}, {4, 5, 8}, {5, 7, 12}, {6, 8, 16}} {
		found := false
		for _, vertex := range mesh.Vertices() {
			found = found || vertex.X == section[0] && vertex.Y == section[1] && vertex.Z == section[2]
		}
		require.Truef(t, found, "section %d is not held", i)
	}
}

func TestLoftSectionsSixIndependentRectanglesHoldEverySection(t *testing.T) {
	sections := manyRectangleSections(t, []float64{2, 3, 4, 5, 6, 7},
		[]float64{3, 4, 5, 7, 8, 9}, []float64{0, 4, 8, 12, 16, 20})
	body, err := decad.LoftSections(t.Context(), decad.New(), sections...)
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	for _, vertex := range [][3]float64{{2, 3, 0}, {3, 4, 4}, {4, 5, 8}, {5, 7, 12}, {6, 8, 16}, {7, 9, 20}} {
		found := false
		for _, held := range mesh.Vertices() {
			found = found || held.X == vertex[0] && held.Y == vertex[1] && held.Z == vertex[2]
		}
		require.Truef(t, found, "source section at z=%g is not held", vertex[2])
	}
}

func TestLoftSectionsManyRectanglesRefuseUnprovedInputs(t *testing.T) {
	for _, tc := range []struct {
		name, reason   string
		widths, depths []float64
		levels         []float64
	}{
		{"positive cubic", "positive cubic proof", []float64{100, 1, 1, 100, 100},
			[]float64{3, 3, 3, 3, 3}, []float64{0, 4, 8, 12, 16}},
		{"unequal levels", "equally spaced Z levels", []float64{2, 3, 4, 5, 6},
			[]float64{3, 4, 5, 7, 8}, []float64{0, 4, 9, 12, 16}},
		{"station coordinate", "station cannot be held exactly", []float64{2, 3, 4.1, 5, 6},
			[]float64{3, 4, 5, 7, 8}, []float64{0, 4, 8, 12, 16}},
		{"station budget", "mesh bound", []float64{1, 10000, 1, 1, 1},
			[]float64{3, 3, 3, 3, 3}, []float64{0, 4, 8, 12, 16}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			sections := manyRectangleSections(t, tc.widths, tc.depths, tc.levels)
			doc := decad.New()
			body, err := decad.LoftSections(t.Context(), doc, sections...)
			require.Nil(t, body)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.ErrorContains(t, err, tc.reason)
			require.Empty(t, doc.Bodies())
		})
	}
}

func TestLoftSectionsManyRectanglesRefuseExcessSections(t *testing.T) {
	sections := manyRectangleSections(t, []float64{2, 3, 4, 5, 6},
		[]float64{3, 4, 5, 7, 8}, []float64{0, 4, 8, 12, 16})
	tooMany := make([]decad.LoftSection, 33)
	copy(tooMany, sections)
	doc := decad.New()
	body, err := decad.LoftSections(t.Context(), doc, tooMany...)
	require.Nil(t, body)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "at most 32 sections")
	require.Empty(t, doc.Bodies())
}
