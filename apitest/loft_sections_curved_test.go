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

func threeCurvedSections(t *testing.T, mids, tips, heights [3]float64) [3]decad.LoftSection {
	t.Helper()
	w := sketch.NewWorld()
	var sections [3]decad.LoftSection
	for i, z := range heights {
		plane := w.XY()
		if i != 0 {
			var err error
			plane, err = w.CreateOffsetPlane(w.XY(), z)
			require.NoError(t, err)
		}
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		point := func(r, angle float64) *sketch.Point {
			p := s.CreatePoint(r*math.Cos(angle), r*math.Sin(angle))
			s.Fix(p)
			return p
		}
		origin := s.CreatePoint(0, 0)
		s.Fix(origin)
		r0, r1, r2 := point(4, -0.3), point(6, mids[i]), point(tips[i], -0.2)
		l0, l1, l2 := point(4, 0.3), point(6, 0.24), point(tips[i], 0.2)
		_, err = s.CreateFitSpline(r0, r1, r2)
		require.NoError(t, err)
		s.CreateArc(origin, r2, l2)
		_, err = s.CreateFitSpline(l2, l1, l0)
		require.NoError(t, err)
		s.CreateArc(origin, l0, r0)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		for _, profile := range s.Profiles() {
			if !profile.Valid || len(profile.Outer) != 4 || len(profile.Holes) != 0 {
				continue
			}
			fits, arcs := 0, 0
			for _, edge := range profile.Outer {
				switch edge.Entity.(type) {
				case *sketch.FitSpline:
					fits++
				case *sketch.Arc:
					arcs++
				}
			}
			if fits == 2 && arcs == 2 {
				sections[i] = decad.LoftSection{Sketch: s, Profile: profile}
				break
			}
		}
		require.NotNil(t, sections[i].Profile, "Sketch must produce a two-fit/two-arc region")
	}
	return sections
}

func curvedPerimeterReference(t *testing.T, profile *sketch.Profile) float64 {
	t.Helper()
	perimeter := 0.0
	for _, edge := range profile.Outer {
		switch curve := edge.Entity.(type) {
		case *sketch.Arc:
			perimeter += curve.R() * curve.Sweep() * (edge.TEnd - edge.TStart)
		case *sketch.FitSpline:
			const steps = 20000
			prevX, prevY := curve.Eval(edge.TStart)
			for i := 1; i <= steps; i++ {
				parameter := edge.TStart + (edge.TEnd-edge.TStart)*float64(i)/steps
				x, y := curve.Eval(parameter)
				perimeter += math.Hypot(x-prevX, y-prevY)
				prevX, prevY = x, y
			}
		default:
			t.Fatalf("unexpected reference curve %T", edge.Entity)
		}
	}
	return perimeter
}

func TestLoftSectionsConstantCurvesKeepAnalyticFacesAndProof(t *testing.T) {
	t.Parallel()
	sections := threeCurvedSections(t, [3]float64{-0.24, -0.24, -0.24},
		[3]float64{8, 8, 8}, [3]float64{0, 5, 10})
	doc := decad.New()
	body, err := doc.LoftSections(t.Context(), sections[:]...)
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, body.Kind())
	require.Len(t, doc.Bodies(), 1)
	require.Len(t, body.Faces(), 6)

	roles := make(map[string]bool)
	planes, cylinders, nurbs := 0, 0, 0
	for _, face := range body.Faces() {
		for _, origin := range face.Origins() {
			roles[origin.Role] = true
		}
		switch face.Surface().Kind() {
		case decad.KindPlane:
			planes++
		case decad.KindCylinder:
			cylinders++
		case decad.KindNURBS:
			nurbs++
		default:
			t.Fatalf("unexpected face kind %v", face.Surface().Kind())
		}
	}
	require.Equal(t, 2, planes)
	require.Equal(t, 2, cylinders)
	require.Equal(t, 2, nurbs)
	for _, role := range []string{"capStart", "capEnd", "side(0,0)", "side(0,1)", "side(0,2)", "side(0,3)"} {
		require.True(t, roles[role], "missing source role %s", role)
	}

	volume, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, 10*sections[0].Profile.Area, volume.Value.Base(), 1e-5)
	require.Greater(t, volume.Value.Base(), 0.0)
	require.False(t, math.IsInf(volume.Bound.Base(), 0) || math.IsNaN(volume.Bound.Base()))
	area, err := body.Area()
	require.NoError(t, err)
	areaReference := 2*sections[0].Profile.Area +
		10*curvedPerimeterReference(t, sections[0].Profile)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-areaReference), area.Bound.Base()+0.002)
	require.False(t, math.IsInf(area.Bound.Base(), 0) || math.IsNaN(area.Bound.Base()))

	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.1)
	require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()))
	live := make(map[*decad.Face]bool)
	for _, face := range body.Faces() {
		live[face] = true
	}
	for _, face := range mesh.SourceFaces() {
		require.True(t, live[face], "mesh facets must name live analytic source faces")
	}

	move, err := r3.Translation(r3.NewVec(16, 0, 0))
	require.NoError(t, err)
	placed, err := body.PlacedCopy(t.Context(), move)
	require.NoError(t, err)
	placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, placedMesh.VolumeVerified())
}

func TestLoftSectionsConstantCurvesRefuseVaryingProfiles(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		mids    [3]float64
		tips    [3]float64
		heights [3]float64
	}{
		{"changed fitted flank", [3]float64{-0.24, -0.23, -0.24},
			[3]float64{8, 8, 8}, [3]float64{0, 5, 10}},
		{"changed circular tip", [3]float64{-0.24, -0.24, -0.24},
			[3]float64{8, 8.5, 8}, [3]float64{0, 5, 10}},
		{"unequal spacing", [3]float64{-0.24, -0.24, -0.24},
			[3]float64{8, 8, 8}, [3]float64{0, 4, 10}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			sections := threeCurvedSections(t, tc.mids, tc.tips, tc.heights)
			doc := decad.New()
			body, err := doc.LoftSections(t.Context(), sections[:]...)
			require.Nil(t, body)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Empty(t, doc.Bodies())
		})
	}
}
