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

func TestLoftCircleBoreRoundedAxisDoesNotPublishCylinder(t *testing.T) {
	baseFrame, err := r3.NewFrame(r3.NewVec(1e9, 0, 0), r3.NewVec(3, 0, 4), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	world := sketch.NewWorld()
	basePlane, err := world.CreatePlaneFromFrame(baseFrame)
	require.NoError(t, err)
	topPlane, err := world.CreateOffsetPlane(basePlane, 3.125)
	require.NoError(t, err)
	topFrame, err := topPlane.Frame()
	require.NoError(t, err)
	level := baseFrame.ToLocal(topFrame.Origin())
	require.Equal(t, r3.NewVec(0, 0, 3.125), level)
	require.Equal(t, topFrame.Origin(), baseFrame.ToWorld(level))
	baseAnchor, err := massmoment.RevolveAnchor(baseFrame, 0, 0)
	require.NoError(t, err)
	topAnchor, err := massmoment.RevolveAnchor(topFrame, 0, 0)
	require.NoError(t, err)
	axis, err := massmoment.PlaneMap(baseFrame, r3.Identity())
	require.NoError(t, err)
	exactLift := new(big.Rat).Add(baseAnchor[2], new(big.Rat).Mul(axis[2][2], proofarith.FloatRat(level.Z)))
	require.NotZero(t, exactLift.Cmp(topAnchor[2]), "the exact section centers must differ")

	section := func(plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		rect := s.CreateRectangle(-5, -5, 5, 5)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		return s, s.Profiles()[0]
	}
	s0, p0 := section(basePlane)
	s1, p1 := section(topPlane)
	doc := decad.New()
	blank, err := doc.Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)
	toolWorld := sketch.NewWorld()
	toolPlane, err := toolWorld.CreatePlaneFromFrame(baseFrame)
	require.NoError(t, err)
	toolSketch, err := toolWorld.CreateSketch(toolPlane)
	require.NoError(t, err)
	center := toolSketch.CreatePoint(0, 0)
	toolSketch.Fix(center)
	toolSketch.CreateCircle(center, 1)
	_, err = toolSketch.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, toolSketch, toolSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(6), Dir: decad.Along})
	require.NoError(t, err)
	cut, err := decad.Cut(t.Context(), blank, tool)
	require.NoError(t, err)
	requireLoftBoreFaceted(t, cut, 3.125)
}

func TestLoftCircleBoreRoundedAdmissionRefusals(t *testing.T) {
	for _, tc := range []struct {
		name       string
		origin     float64
		u, v       r3.Vec
		height     float64
		center     float64
		wantOrigin bool
		wantCenter bool
	}{
		{"origin check", 1e9, r3.NewVec(3, 0, 4), r3.NewVec(0, 1, 0), 0.625, 1.3, false, true},
		{"center check", 1e5, r3.NewVec(5, 0, 1), r3.NewVec(0, 1, 0), 9.394, 0.427, true, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			base, err := r3.NewFrame(r3.NewVec(tc.origin, 0, 0), tc.u, tc.v)
			require.NoError(t, err)
			world := sketch.NewWorld()
			basePlane, err := world.CreatePlaneFromFrame(base)
			require.NoError(t, err)
			topPlane, err := world.CreateOffsetPlane(basePlane, tc.height)
			require.NoError(t, err)
			top, err := topPlane.Frame()
			require.NoError(t, err)
			require.Equal(t, base.U(), top.U())
			require.Equal(t, base.V(), top.V())
			level := base.ToLocal(top.Origin())
			require.Zero(t, level.X)
			require.Zero(t, level.Y)
			require.Positive(t, level.Z)
			require.Equal(t, tc.wantOrigin, base.ToWorld(r3.NewVec(0, 0, level.Z)) == top.Origin())
			require.Equal(t, tc.wantCenter,
				base.ToWorld(r3.NewVec(tc.center, 0, level.Z)) == top.ToWorldUV(tc.center, 0))
			worldCenter := base.ToWorldUV(tc.center, 0)
			require.Equal(t, worldCenter, base.ToWorld(base.ToLocal(worldCenter)))

			section := func(plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
				s, err := world.CreateSketch(plane)
				require.NoError(t, err)
				rect := s.CreateRectangle(-5, -5, 5, 5)
				s.Fix(rect.A)
				_, err = s.Solve(t.Context())
				require.NoError(t, err)
				return s, s.Profiles()[0]
			}
			s0, p0 := section(basePlane)
			s1, p1 := section(topPlane)
			doc := decad.New()
			blank, err := doc.Loft(t.Context(), s0, p0, s1, p1)
			require.NoError(t, err)
			toolWorld := sketch.NewWorld()
			toolPlane, err := toolWorld.CreatePlaneFromFrame(base)
			require.NoError(t, err)
			toolSketch, err := toolWorld.CreateSketch(toolPlane)
			require.NoError(t, err)
			center := toolSketch.CreatePoint(tc.center, 0)
			toolSketch.Fix(center)
			toolSketch.CreateCircle(center, 1)
			_, err = toolSketch.Solve(t.Context())
			require.NoError(t, err)
			tool, err := decad.Extrude(t.Context(), doc, toolSketch, toolSketch.Profiles()[0],
				decad.Distance{D: units.Millimeters(tc.height + 2), Dir: decad.Along})
			require.NoError(t, err)
			cut, err := decad.Cut(t.Context(), blank, tool)
			if err != nil {
				// An unproved tool-center lift takes the general boolean path,
				// which can refuse near-coincident held facets.
				require.ErrorIs(t, err, decad.ErrUnsupported)
				return
			}
			requireLoftBoreFaceted(t, cut, tc.height)
		})
	}
}

func requireLoftBoreFaceted(t *testing.T, cut *decad.Body, height float64) {
	t.Helper()
	cylinders, err := decad.Faces(decad.Cylindrical()).SelectFaces(cut)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	require.Empty(t, cylinders)
	for _, role := range []decad.FeatureRef{decad.CapStart(cut), decad.CapEnd(cut)} {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(cut)
		require.NoError(t, err)
		require.Len(t, faces[0].Loops(), 2)
		require.Greater(t, len(faces[0].Loops()[1].Edges()), 1)
		for _, edge := range faces[0].Loops()[1].Edges() {
			require.IsType(t, decad.Line3{}, edge.Curve())
		}
	}
	volume, err := cut.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-height*(100-math.Pi)), volume.Bound.Base())
}

func TestLoftThroughBoreCutPublishesCircularTopology(t *testing.T) {
	bottom, bottomProfile, top, topProfile := loftSquaresAt(t, r3.NewVec(0, 0, 0), 5, 5, 2)
	doc := decad.New()
	blank, err := doc.Loft(t.Context(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)

	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, -1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(1, 0.5)
	s.Fix(center)
	s.CreateCircle(center, 1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	require.NoError(t, err)
	cut, err := decad.Cut(t.Context(), blank, tool)
	require.NoError(t, err)
	requireManifold(t, cut)

	for _, cap := range []struct {
		ref    decad.FeatureRef
		center r3.Vec
	}{
		{decad.CapStart(cut), r3.NewVec(1, 0.5, 0)},
		{decad.CapEnd(cut), r3.NewVec(1, 0.5, 2)},
	} {
		faces, err := decad.Faces(decad.FaceCreatedBy(cap.ref)).Exactly(1).SelectFaces(cut)
		require.NoError(t, err)
		require.Len(t, faces[0].Loops(), 2)
		require.Len(t, faces[0].Loops()[1].Edges(), 1)
		edge := faces[0].Loops()[1].Edges()[0]
		circle, ok := edge.Curve().(decad.Circle3)
		require.True(t, ok)
		require.Equal(t, cap.center, circle.Center)
		require.Equal(t, units.Millimeters(1), circle.Radius)
		require.Same(t, edge.Start(), edge.End())
		require.Len(t, edge.Faces(), 2)
		area, err := faces[0].Area()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(area.Value.Base()-(100-math.Pi)), area.Bound.Base())
	}
	walls, err := decad.Faces(decad.Cylindrical()).Exactly(1).SelectFaces(cut)
	require.NoError(t, err)
	require.Len(t, walls[0].Loops(), 2)
	require.Greater(t, len(walls[0].Origins()), 2)
	for _, origin := range walls[0].Origins() {
		require.NotEqual(t, "side(1,0)", origin.Role)
	}
	wallArea, err := walls[0].Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(wallArea.Value.Base()-4*math.Pi), wallArea.Bound.Base())
	volume, err := cut.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-(200-2*math.Pi)), volume.Bound.Base())
	mesh, err := cut.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	requireBoreRimsOppositeUses(t, cut)

	for _, placed := range []struct {
		name      string
		transform func() (r3.Transform, error)
	}{
		{"translated", func() (r3.Transform, error) { return r3.Translation(r3.NewVec(12, -7, 3)) }},
		{"reflected", func() (r3.Transform, error) {
			mirror, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
			if err != nil {
				return r3.Transform{}, err
			}
			return r3.Reflection(mirror)
		}},
	} {
		t.Run(placed.name, func(t *testing.T) {
			motion, err := placed.transform()
			require.NoError(t, err)
			placedBody, err := decad.Copy(t.Context(), cut, decad.WithCopyPlacement(motion))
			require.NoError(t, err)
			requireManifold(t, placedBody)
			caps, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(placedBody))).Exactly(1).SelectFaces(placedBody)
			require.NoError(t, err)
			require.Len(t, caps[0].Loops()[1].Edges(), 1)
			require.IsType(t, decad.Circle3{}, caps[0].Loops()[1].Edges()[0].Curve())
			mesh, err := placedBody.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
			requireBoreRimsOppositeUses(t, placedBody)
		})
	}
}

func requireBoreRimsOppositeUses(t *testing.T, body *decad.Body) {
	t.Helper()
	walls, err := decad.Faces(decad.Cylindrical()).Exactly(1).SelectFaces(body)
	require.NoError(t, err)
	wallUses := make(map[*decad.Edge]decad.CoEdge)
	for _, loop := range walls[0].Loops() {
		for _, use := range loop.CoEdges() {
			wallUses[use.Edge()] = use
		}
	}
	var centers [2]r3.Vec
	var capFaces [2]*decad.Face
	var capUses [2]decad.CoEdge
	for i, role := range []decad.FeatureRef{decad.CapStart(body), decad.CapEnd(body)} {
		caps, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, caps[0].Loops()[1].CoEdges(), 1)
		use := caps[0].Loops()[1].CoEdges()[0]
		wallUse, ok := wallUses[use.Edge()]
		require.True(t, ok)
		require.NotEqual(t, wallUse.IsForward(), use.IsForward())
		circle, ok := use.Edge().Curve().(decad.Circle3)
		require.True(t, ok)
		centers[i], capFaces[i], capUses[i] = circle.Center, caps[0], use
	}
	axial := centers[1].Sub(centers[0])
	for i, capUse := range capUses {
		wallUse := wallUses[capUse.Edge()]
		pos := capUse.Edge().Start().Position().Value
		radial := pos.Sub(centers[i])
		capNormal, err := capFaces[i].NormalAt(pos)
		require.NoError(t, err)
		wallNormal, err := walls[0].NormalAt(pos)
		require.NoError(t, err)
		require.Greater(t, capNormal.Value.Cross(circleUseTangent(capUse, radial)).Dot(radial), 0.0)
		insideWall := axial
		if i == 1 {
			insideWall = insideWall.Scale(-1)
		}
		require.Greater(t, wallNormal.Value.Cross(circleUseTangent(wallUse, radial)).Dot(insideWall), 0.0)
	}
}

func circleUseTangent(use decad.CoEdge, radial r3.Vec) r3.Vec {
	circle := use.Edge().Curve().(decad.Circle3)
	tangent := circle.Axis.Cross(radial)
	if !use.IsForward() {
		return tangent.Scale(-1)
	}
	return tangent
}

func TestDirectLoftWholeCircleHolePublishesCircularTopology(t *testing.T) {
	world := sketch.NewWorld()
	upper, err := world.CreateOffsetPlane(world.XY(), 2)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, plane := range []*sketch.Plane{world.XY(), upper} {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		rectangle := s.CreateRectangle(-5, -5, 5, 5)
		s.Fix(rectangle.A)
		center := s.CreatePoint(0, 0)
		s.Fix(center)
		s.CreateCircle(center, 1)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	body, err := decad.New().Loft(t.Context(), sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	requireManifold(t, body)
	for _, role := range []decad.FeatureRef{decad.CapStart(body), decad.CapEnd(body)} {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces[0].Loops(), 2)
		require.Len(t, faces[0].Loops()[1].Edges(), 1)
		edge := faces[0].Loops()[1].Edges()[0]
		circle, ok := edge.Curve().(decad.Circle3)
		require.True(t, ok)
		require.Equal(t, units.Millimeters(1), circle.Radius)
		require.Same(t, edge.Start(), edge.End())
		require.Len(t, edge.Faces(), 2)
	}
	_, err = decad.Faces(decad.Cylindrical()).Exactly(1).SelectFaces(body)
	require.NoError(t, err)
	volume, err := body.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-(200-2*math.Pi)), volume.Bound.Base())
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	requireBoreRimsOppositeUses(t, body)
}

func TestExtrudedBoxThroughBoreCutPublishesCircularTopology(t *testing.T) {
	world := sketch.NewWorld()
	boxSketch, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rectangle := boxSketch.CreateRectangle(-5, -5, 5, 5)
	boxSketch.Fix(rectangle.A)
	_, err = boxSketch.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	box, err := decad.Extrude(t.Context(), doc, boxSketch, boxSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, -1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	toolSketch, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := toolSketch.CreatePoint(0, 0)
	toolSketch.Fix(center)
	toolSketch.CreateCircle(center, 1)
	_, err = toolSketch.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, toolSketch, toolSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	require.NoError(t, err)
	cut, err := decad.Cut(t.Context(), box, tool)
	require.NoError(t, err)
	for _, role := range []decad.FeatureRef{decad.CapStart(cut), decad.CapEnd(cut)} {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(cut)
		require.NoError(t, err)
		require.Len(t, faces[0].Loops()[1].Edges(), 1)
		require.IsType(t, decad.Circle3{}, faces[0].Loops()[1].Edges()[0].Curve())
	}
	_, err = decad.Faces(decad.Cylindrical()).Exactly(1).SelectFaces(cut)
	require.NoError(t, err)
	volume, err := cut.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-(200-2*math.Pi)), volume.Bound.Base())
	mesh, err := cut.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}
