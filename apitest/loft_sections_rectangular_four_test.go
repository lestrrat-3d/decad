package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func fourRectangleSections(t *testing.T, widths, depths, levels [4]float64) [4]decad.LoftSection {
	t.Helper()
	world := sketch.NewWorld()
	var sections [4]decad.LoftSection
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

func TestLoftSectionsFourIndependentRectanglesKeepVolumeAndMeshProof(t *testing.T) {
	sections := fourRectangleSections(t, [4]float64{2, 3, 4, 5},
		[4]float64{3, 4, 5, 7}, [4]float64{0, 4, 8, 12})
	doc := decad.New()
	body, err := decad.LoftSections(t.Context(), doc, sections[:]...)
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	require.Len(t, body.Faces(), 6)
	volume, err := body.Volume()
	require.NoError(t, err)
	// Cubics through the four stations are w=2+3t and h=3+4t-4.5t²+4.5t³.
	const trueVolume = 819.6
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-trueVolume), volume.Bound.Base())
	require.Less(t, volume.Bound.Base(), 2.0)
	const steps = 2000
	areaIntegral := 0.0
	for i := 0; i <= steps; i++ {
		x := float64(i) / steps
		w, h := 2+3*x, 3+4*x-4.5*x*x+4.5*x*x*x
		hp := 4 - 9*x + 13.5*x*x
		weight := 2.0
		if i == 0 || i == steps {
			weight = 1
		} else if i%2 == 1 {
			weight = 4
		}
		areaIntegral += weight * 4 * (w*math.Sqrt(12*12+hp*hp) + h*math.Sqrt(12*12+3*3))
	}
	trueArea := 4*(2*3+5*7) + areaIntegral/(3*steps)
	area, err := body.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-trueArea), area.Bound.Base()+1e-7)
	require.Less(t, area.Bound.Base(), 20.0)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.05)
	middleCorner := false
	for _, vertex := range mesh.Vertices() {
		middleCorner = middleCorner || vertex.X == 3.5 && vertex.Y == 4.4375 && vertex.Z == 6
	}
	require.True(t, middleCorner, "the held mesh must evaluate both cubic dimensions at t=1/2")
}

func TestLoftSectionsFourIndependentRectanglesRefuseUnprovedInputs(t *testing.T) {
	for _, tc := range []struct {
		name           string
		widths, depths [4]float64
		levels         [4]float64
		wantReason     string
	}{
		{"positive cubic", [4]float64{5, 1, 1, 5}, [4]float64{3, 3, 3, 3},
			[4]float64{0, 4, 8, 12}, "positive cubic proof"},
		{"uneven levels", [4]float64{2, 3, 4, 5}, [4]float64{3, 4, 5, 7},
			[4]float64{0, 4, 9, 12}, "equally spaced Z levels"},
		{"station coordinate", [4]float64{2, 3, 4.1, 5}, [4]float64{3, 4, 5, 7},
			[4]float64{0, 4, 8, 12}, "station cannot be held exactly"},
		{"station budget", [4]float64{1, 10000, 10000, 1}, [4]float64{3, 3, 3, 3},
			[4]float64{0, 4, 8, 12}, "mesh bound"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			sections := fourRectangleSections(t, tc.widths, tc.depths, tc.levels)
			doc := decad.New()
			body, err := decad.LoftSections(t.Context(), doc, sections[:]...)
			require.Nil(t, body)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.ErrorContains(t, err, tc.wantReason)
			require.Empty(t, doc.Bodies())
		})
	}
}
