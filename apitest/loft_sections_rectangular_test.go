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

func TestLoftSectionsIndependentRectanglesVerifyVolumeAndBoundary(t *testing.T) {
	t.Parallel()
	sections := threeRectangleSections(t, [3]float64{5, 4, 2.5}, [3]float64{4, 5, 6.5},
		[3]float64{}, [3]float64{0, 5, 10})
	doc := decad.New()
	body, err := decad.LoftSections(t.Context(), doc, sections[:]...)
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	require.Equal(t, decad.BodySolid, body.Kind())
	require.Len(t, body.Faces(), 6)

	// The two dimensions have opposite curvature. Their signed held-volume
	// errors partly cancel, while the occupied-volume allowance must not.
	const trueVolume = 775 + 1.0/3.0
	volume, err := body.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-trueVolume), volume.Bound.Base())
	require.Greater(t, volume.Bound.Base(), 0.25)
	require.Less(t, volume.Bound.Base(), 0.5)

	// Numerical integration independently checks the actual curved-wall area.
	const steps = 2000
	areaIntegral := 0.0
	for i := 0; i <= steps; i++ {
		x := float64(i) / steps
		w, h := 5-1.5*x-x*x, 4+1.5*x+x*x
		wp, hp := -1.5-2*x, 1.5+2*x
		weight := 2.0
		if i == 0 || i == steps {
			weight = 1
		} else if i%2 == 1 {
			weight = 4
		}
		areaIntegral += weight * 4 * (h*math.Sqrt(100+wp*wp) + w*math.Sqrt(100+hp*hp))
	}
	trueArea := 4*(5*4+2.5*6.5) + areaIntegral/(3*steps)
	area, err := body.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-trueArea), area.Bound.Base()+1e-7)
	require.Less(t, area.Bound.Base(), 2.0)

	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.05)
	require.Len(t, mesh.Triangles(), 132)
	require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()))
	live := make(map[*decad.Face]struct{}, len(body.Faces()))
	for _, face := range body.Faces() {
		live[face] = struct{}{}
	}
	for _, face := range mesh.SourceFaces() {
		_, ok := live[face]
		require.True(t, ok)
	}
}

func TestLoftSectionsIndependentRectanglesPlaceWithVerifiedMesh(t *testing.T) {
	t.Parallel()
	sections := threeRectangleSections(t, [3]float64{5, 4, 2.5}, [3]float64{4, 4, 4},
		[3]float64{}, [3]float64{0, 5, 10})
	body, err := decad.LoftSections(t.Context(), decad.New(), sections[:]...)
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(13, -7, 2))
	require.NoError(t, err)
	placed, err := decad.Copy(t.Context(), body, decad.WithCopyPlacement(move))
	require.NoError(t, err)
	mesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	volume, err := placed.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-(626+2.0/3.0)), volume.Bound.Base())
	box, err := placed.Bounds()
	require.NoError(t, err)
	require.LessOrEqual(t, box.Min.X-box.Bound.Base(), 8.0)
	require.GreaterOrEqual(t, box.Max.X+box.Bound.Base(), 18.0)
}

func TestLoftSectionsRotatedRectangleWalksKeepVerifiedMesh(t *testing.T) {
	t.Parallel()
	world := sketch.NewWorld()
	var sections [3]decad.LoftSection
	for i := range sections {
		plane := world.XY()
		if i != 0 {
			var err error
			plane, err = world.CreateOffsetPlane(world.XY(), float64(i*5))
			require.NoError(t, err)
		}
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		w, h := float64(4+2*i), float64(2+i)
		coords := [4][2]float64{{-w, -h}, {w, -h}, {w, h}, {-w, h}}
		var points [4]*sketch.Point
		for j, pair := range coords {
			points[j] = s.CreatePoint(pair[0], pair[1])
			s.Fix(points[j])
		}
		for j := range points {
			start := (i + j) % len(points)
			s.CreateLine(points[start], points[(start+1)%len(points)])
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		profiles := s.Profiles()
		require.Len(t, profiles, 1)
		require.True(t, profiles[0].Valid)
		sections[i] = decad.LoftSection{Sketch: s, Profile: profiles[0]}
	}
	start0 := sections[0].Profile.Outer[0].Polyline[0]
	start1 := sections[1].Profile.Outer[0].Polyline[0]
	require.NotEqual(t, start0, start1, "the Sketch walks must start at different corners")
	body, err := decad.LoftSections(t.Context(), decad.New(), sections[:]...)
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestLoftSectionsIndependentRectanglesRefuseUnprovedInputs(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name            string
		widths, heights [3]float64
		centers, levels [3]float64
		wantReason      string
	}{
		{"width positivity", [3]float64{5, 0.5, 5}, [3]float64{4, 4, 4},
			[3]float64{}, [3]float64{0, 5, 10}, ""},
		{"height positivity", [3]float64{5, 4, 2.5}, [3]float64{4, 0.5, 4},
			[3]float64{}, [3]float64{0, 5, 10}, ""},
		{"unequal spacing", [3]float64{5, 4, 2.5}, [3]float64{4, 5, 6.5},
			[3]float64{}, [3]float64{0, 4, 10}, ""},
		{"off-center rectangle", [3]float64{5, 4, 2.5}, [3]float64{4, 5, 6.5},
			[3]float64{0, 1, 0}, [3]float64{0, 5, 10}, ""},
		{"station budget", [3]float64{10000, 1, 10000}, [3]float64{4, 4, 4},
			[3]float64{}, [3]float64{0, 5, 10}, ""},
		{"station coordinate", [3]float64{5, 4.1, 2.5}, [3]float64{4, 5, 6.5},
			[3]float64{}, [3]float64{0, 5, 10}, "station cannot be held exactly"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			sections := threeRectangleSections(t, tc.widths, tc.heights, tc.centers, tc.levels)
			doc := decad.New()
			body, err := decad.LoftSections(t.Context(), doc, sections[:]...)
			require.Nil(t, body)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			if tc.wantReason != "" {
				require.ErrorContains(t, err, tc.wantReason)
			}
			require.Empty(t, doc.Bodies())
		})
	}
}
