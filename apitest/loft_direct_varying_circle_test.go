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

func directLoftCircles(t *testing.T, outerCircle bool, r0, r1, x1, y1 float64) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	upper, err := world.CreateOffsetPlane(world.XY(), 2)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, plane := range []*sketch.Plane{world.XY(), upper} {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		if outerCircle {
			outer := s.CreatePoint(0, 0)
			s.Fix(outer)
			s.CreateCircle(outer, 4)
		} else {
			rect := s.CreateRectangle(-5, -5, 5, 5)
			s.Fix(rect.A)
		}
		x, y, radius := 0.0, 0.0, r0
		if i == 1 {
			x, y, radius = x1, y1, r1
		}
		center := s.CreatePoint(x, y)
		s.Fix(center)
		s.CreateCircle(center, radius)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	body, err := decad.New().Loft(t.Context(), sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	return body
}

// directCircleWallArea integrates the ruled circle pair independently of
// Decad's held triangles. The angular trapezoid rule samples the smooth
// periodic integrand at 16,384 equally spaced angles.
func directCircleWallArea(r0, r1, dx, dy float64) float64 {
	const n = 16384
	const height = 2.0
	dr := (r1 - r0) / height
	vx, vy := dx/height, dy/height
	var sum float64
	for i := range n {
		angle := 2 * math.Pi * float64(i) / n
		slope := dr + vx*math.Cos(angle) + vy*math.Sin(angle)
		sum += math.Sqrt(1 + slope*slope)
	}
	return height * (r0 + r1) / 2 * (2 * math.Pi / n) * sum
}

func TestDirectLoftVaryingCircleHolesPublishCurvedWalls(t *testing.T) {
	for _, tc := range []struct {
		name        string
		r0, r1      float64
		x1, y1      float64
		wantCone    bool
		normalZSign float64
	}{
		{name: "growing cone", r0: 1, r1: 2, wantCone: true, normalZSign: 1},
		{name: "shrinking cone", r0: 2, r1: 1, wantCone: true, normalZSign: -1},
		{name: "shifted center", r0: 1, r1: 1, x1: 1, y1: 1},
		{name: "shifted center and radius", r0: 1, r1: 2, x1: 1, y1: 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			body := directLoftCircles(t, false, tc.r0, tc.r1, tc.x1, tc.y1)
			requireManifold(t, body)
			var wall *decad.Face
			for _, face := range body.Faces() {
				switch face.Surface().(type) {
				case decad.Cone, decad.NURBSSurface:
					require.Nil(t, wall)
					wall = face
				}
			}
			require.NotNil(t, wall)
			if tc.wantCone {
				require.IsType(t, decad.Cone{}, wall.Surface())
			} else {
				require.IsType(t, decad.NURBSSurface{}, wall.Surface())
			}
			require.Len(t, wall.Loops(), 2)
			wallUses := make(map[*decad.Edge]decad.CoEdge)
			for _, loop := range wall.Loops() {
				require.Len(t, loop.CoEdges(), 1)
				use := loop.CoEdges()[0]
				wallUses[use.Edge()] = use
			}
			for i, role := range []decad.FeatureRef{decad.CapStart(body), decad.CapEnd(body)} {
				faces, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(body)
				require.NoError(t, err)
				require.Len(t, faces[0].Loops(), 2)
				require.Len(t, faces[0].Loops()[1].CoEdges(), 1)
				use := faces[0].Loops()[1].CoEdges()[0]
				circle, ok := use.Edge().Curve().(decad.Circle3)
				require.True(t, ok)
				wantCenter, wantRadius := r3.NewVec(0, 0, 0), tc.r0
				if i == 1 {
					wantCenter, wantRadius = r3.NewVec(tc.x1, tc.y1, 2), tc.r1
				}
				require.Equal(t, wantCenter, circle.Center)
				require.Equal(t, units.Millimeters(wantRadius), circle.Radius)
				require.Same(t, use.Edge().Start(), use.Edge().End())
				require.Len(t, use.Edge().Faces(), 2)
				wallUse, ok := wallUses[use.Edge()]
				require.True(t, ok)
				require.NotEqual(t, use.IsForward(), wallUse.IsForward())
			}
			area, err := wall.Area()
			require.NoError(t, err)
			wantArea := directCircleWallArea(tc.r0, tc.r1, tc.x1, tc.y1)
			require.LessOrEqual(t, math.Abs(area.Value.Base()-wantArea), area.Bound.Base()+1e-8)
			if tc.wantCone {
				closedForm := math.Pi * (tc.r0 + tc.r1) * math.Hypot(2, tc.r1-tc.r0)
				require.InDelta(t, closedForm, wantArea, 1e-10)
				normal, err := wall.NormalAt(r3.NewVec(tc.r0, 0, 0))
				require.NoError(t, err)
				require.Less(t, normal.Value.X, -0.5)
				require.Greater(t, normal.Value.Z*tc.normalZSign, 0.1)
				require.Less(t, normal.Bound.Base(), 1e-8)
			} else {
				_, err := wall.NormalAt(r3.NewVec(tc.r0, 0, 0))
				require.ErrorIs(t, err, decad.ErrUnsupported)
			}
			volume, err := body.Volume()
			require.NoError(t, err)
			wantVolume := 200 - 2*math.Pi*(tc.r0*tc.r0+tc.r0*tc.r1+tc.r1*tc.r1)/3
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-wantVolume), volume.Bound.Base())
			mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}

func TestDirectLoftVaryingHoleKeepsFacetedTopologyWithCurvedOuterWall(t *testing.T) {
	body := directLoftCircles(t, true, 1, 2, 0, 0)
	faces, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(body))).Exactly(1).SelectFaces(body)
	require.NoError(t, err)
	require.Greater(t, len(faces[0].Loops()[1].Edges()), 1)
	for _, edge := range faces[0].Loops()[1].Edges() {
		require.IsType(t, decad.Line3{}, edge.Curve())
	}
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}
