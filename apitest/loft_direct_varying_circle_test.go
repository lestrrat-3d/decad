package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftDirectCircleRoundedAxisKeepsFacetedTopology(t *testing.T) {
	base, err := r3.NewFrame(r3.NewVec(1e9, 0, 0), r3.NewVec(3, 0, 4), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	world := sketch.NewWorld()
	bottomPlane, err := world.CreatePlaneFromFrame(base)
	require.NoError(t, err)
	topPlane, err := world.CreateOffsetPlane(bottomPlane, 3.125)
	require.NoError(t, err)
	top, err := topPlane.Frame()
	require.NoError(t, err)
	level := base.ToLocal(top.Origin())
	require.Equal(t, r3.NewVec(0, 0, 3.125), level)
	require.Equal(t, top.Origin(), base.ToWorld(level))
	bottomAnchor, err := massmoment.RevolveAnchor(base, 0, 0)
	require.NoError(t, err)
	topAnchor, err := massmoment.RevolveAnchor(top, 0, 0)
	require.NoError(t, err)
	axis, err := massmoment.PlaneMap(base, r3.Identity())
	require.NoError(t, err)
	exactLift := new(big.Rat).Add(bottomAnchor[2],
		new(big.Rat).Mul(axis[2][2], proofarith.FloatRat(level.Z)))
	require.NotZero(t, exactLift.Cmp(topAnchor[2]))

	section := func(plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		rect := s.CreateRectangle(-5, -5, 5, 5)
		s.Fix(rect.A)
		center := s.CreatePoint(0, 0)
		s.Fix(center)
		s.CreateCircle(center, 1)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		return s, s.Profiles()[0]
	}
	s0, p0 := section(bottomPlane)
	s1, p1 := section(topPlane)
	body, err := decad.New().Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)
	cylinders, err := decad.Faces(decad.Cylindrical()).SelectFaces(body)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	require.Empty(t, cylinders)
	for _, role := range []decad.FeatureRef{decad.CapStart(body), decad.CapEnd(body)} {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces[0].Loops(), 2)
		require.Greater(t, len(faces[0].Loops()[1].Edges()), 1)
		for _, edge := range faces[0].Loops()[1].Edges() {
			require.IsType(t, decad.Line3{}, edge.Curve())
		}
	}
}

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

// quarterTurnCircleWallArea numerically integrates the independent analytic
// ruled-wall area integrand at 65,536 midpoint stations.
func quarterTurnCircleWallArea(r0, r1, height float64) float64 {
	const n = 65536
	var sum float64
	for i := range n {
		t := (float64(i) + 0.5) / n
		a, b := (1-t)*r0, t*r1
		radial := (1-t)*r0*r0 - t*r1*r1
		sum += math.Sqrt(height*height*(a*a+b*b) + radial*radial)
	}
	return 2 * math.Pi * sum / n
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

func TestDirectLoftVaryingHoleWithCurvedOuterWallPublishesCone(t *testing.T) {
	body := directLoftCircles(t, true, 1, 2, 0, 0)
	requireManifold(t, body)
	var bore *decad.Face
	for _, face := range body.Faces() {
		if _, ok := face.Surface().(decad.Cone); ok {
			require.Nil(t, bore)
			bore = face
		}
	}
	require.NotNil(t, bore)
	for _, role := range []decad.FeatureRef{decad.CapStart(body), decad.CapEnd(body)} {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces[0].Loops()[1].Edges(), 1)
		require.IsType(t, decad.Circle3{}, faces[0].Loops()[1].Edges()[0].Curve())
	}
	area, err := bore.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-3*math.Pi*math.Sqrt(5)), area.Bound.Base())
	volume, err := body.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-82*math.Pi/3), volume.Bound.Base())
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestDirectLoftMovingHoleWithCurvedOuterWallPublishesRuledFace(t *testing.T) {
	body := directLoftCircles(t, true, 1, 2, 1, 0)
	requireManifold(t, body)
	var bore *decad.Face
	for _, face := range body.Faces() {
		if _, ok := face.Surface().(decad.NURBSSurface); ok {
			require.Nil(t, bore)
			bore = face
		}
	}
	require.NotNil(t, bore)
	area, err := bore.Area()
	require.NoError(t, err)
	require.Less(t, area.Bound.Base(), 0.1)
	require.LessOrEqual(t,
		math.Abs(area.Value.Base()-directCircleWallArea(1, 2, 1, 0)), area.Bound.Base()+1e-7)
	volume, err := body.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-82*math.Pi/3), volume.Bound.Base())
	for _, role := range []decad.FeatureRef{decad.CapStart(body), decad.CapEnd(body)} {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces[0].Loops()[1].Edges(), 1)
		require.IsType(t, decad.Circle3{}, faces[0].Loops()[1].Edges()[0].Curve())
	}
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestDirectLoftRotatedCircleFramesPublishRuledHole(t *testing.T) {
	for _, tc := range []struct {
		name string
		u, v r3.Vec
	}{
		{name: "positive quarter turn", u: r3.NewVec(0, 1, 0), v: r3.NewVec(-1, 0, 0)},
		{name: "negative quarter turn", u: r3.NewVec(0, -1, 0), v: r3.NewVec(1, 0, 0)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			directLoftRotatedCircleFrameCase(t, tc.u, tc.v)
		})
	}
}

func directLoftRotatedCircleFrameCase(t *testing.T, u, v r3.Vec) {
	t.Helper()
	world := sketch.NewWorld()
	upperFrame, err := r3.NewFrame(r3.NewVec(0, 0, 2), u, v)
	require.NoError(t, err)
	upper, err := world.CreatePlaneFromFrame(upperFrame)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, plane := range []*sketch.Plane{world.XY(), upper} {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		center := s.CreatePoint(0, 0)
		s.Fix(center)
		s.CreateCircle(center, 4)
		holeRadius := 1.0
		if i == 1 {
			holeRadius = 2
		}
		s.CreateCircle(center, holeRadius)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	body, err := decad.New().Loft(t.Context(), sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	volume, err := body.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-18*math.Pi), volume.Bound.Base())
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	requireManifold(t, body)
	var bore *decad.Face
	for _, face := range body.Faces() {
		if _, ok := face.Surface().(decad.NURBSSurface); ok {
			require.Nil(t, bore)
			bore = face
		}
	}
	require.NotNil(t, bore)
	area, err := bore.Area()
	require.NoError(t, err)
	require.Less(t, area.Bound.Base(), 0.1)
	require.LessOrEqual(t,
		math.Abs(area.Value.Base()-quarterTurnCircleWallArea(1, 2, 2)), area.Bound.Base()+1e-7)
	require.Len(t, bore.Loops(), 2)
	wallUses := make(map[*decad.Edge]decad.CoEdge)
	for _, loop := range bore.Loops() {
		require.Len(t, loop.CoEdges(), 1)
		use := loop.CoEdges()[0]
		wallUses[use.Edge()] = use
	}
	for _, role := range []decad.FeatureRef{decad.CapStart(body), decad.CapEnd(body)} {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces[0].Loops()[1].Edges(), 1)
		use := faces[0].Loops()[1].CoEdges()[0]
		require.IsType(t, decad.Circle3{}, use.Edge().Curve())
		wallUse, ok := wallUses[use.Edge()]
		require.True(t, ok)
		require.NotEqual(t, use.IsForward(), wallUse.IsForward())
	}
}
