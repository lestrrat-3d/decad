package apitest_test

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestRevolveSectorAnnularCylinderCutVerified(t *testing.T) {
	doc, hub, lug, _, _ := taperedHubAndRadialLug(t, 9, 7.5, 4.3, false)
	joined, err := decad.Union(t.Context(), hub, lug)
	require.NoError(t, err)
	requireRevolveSectorCylinderCut(t, doc, hub, joined, 1)
}

func TestRevolveSectorRepeatedAnnularCylinderCutVerified(t *testing.T) {
	doc, hub, lug, _, _ := taperedHubAndRadialLug(t, 9, 7.5, 4.3, false)
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(180))
	require.NoError(t, err)
	second, err := decad.Copy(t.Context(), lug, decad.WithCopyPlacement(turn))
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), hub, lug, second)
	require.NoError(t, err)
	requireRevolveSectorCylinderCut(t, doc, hub, joined, 1)
}

func TestRevolveSectorStraightBandCylinderCutVerified(t *testing.T) {
	doc, hub, lug, _, _ := taperedHubAndRadialLug(t, 9, 7.5, 4.3, false)
	joined, err := decad.Union(t.Context(), hub, lug)
	require.NoError(t, err)
	requireRevolveSectorCylinderCut(t, doc, hub, joined, 2)
}

func TestRevolveSectorRepeatedStraightBandCylinderCutVerified(t *testing.T) {
	doc, hub, lug, _, _ := taperedHubAndRadialLug(t, 9, 7.5, 4.3, false)
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(180))
	require.NoError(t, err)
	second, err := decad.Copy(t.Context(), lug, decad.WithCopyPlacement(turn))
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), hub, lug, second)
	require.NoError(t, err)
	requireRevolveSectorCylinderCut(t, doc, hub, joined, 2)
}

func TestRevolveSectorCylinderCutRadiusBoundariesLeaveBodiesLive(t *testing.T) {
	doc, hub, lug, _, _ := taperedHubAndRadialLug(t, 9, 7.5, 4.3, false)
	joined, err := decad.Union(t.Context(), hub, lug)
	require.NoError(t, err)
	for _, radius := range []float64{
		math.Nextafter(1.5, 0), 1.5, math.Nextafter(1.5, math.Inf(1)),
		math.Nextafter(2.5, 0), 2.5, 3,
	} {
		tool := revolveSectorCylinderTool(t, doc, radius, 4, 6, 0)
		before := doc.Bodies()
		_, err := decad.Cut(t.Context(), joined, tool)
		require.ErrorIs(t, err, decad.ErrUnsupported, "radius %g", radius)
		require.Equal(t, before, doc.Bodies(), "radius %g", radius)
	}
}

func TestRevolveSectorCylinderCutRequiresSpanningCenteredToolAndPlanarCaps(t *testing.T) {
	doc, hub, lug, _, _ := taperedHubAndRadialLug(t, 9, 7.5, 4.3, false)
	joined, err := decad.Union(t.Context(), hub, lug)
	require.NoError(t, err)
	for _, tc := range []struct {
		name                   string
		start, height, centerY float64
	}{
		{name: "ends inside", start: 4, height: 4.5},
		{name: "starts inside", start: 6.1, height: 4},
		{name: "off axis", start: 4, height: 6, centerY: 0.2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			tool := revolveSectorCylinderTool(t, doc, 1, tc.start, tc.height, tc.centerY)
			before := doc.Bodies()
			_, err := decad.Cut(t.Context(), joined, tool)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Equal(t, before, doc.Bodies())
		})
	}
	shoulderDoc, shoulder, shoulderLug := shoulderedHubAndRadialLug(t)
	joinedShoulder, err := decad.Union(t.Context(), shoulder, shoulderLug)
	require.NoError(t, err)
	tool := revolveSectorCylinderTool(t, shoulderDoc, 1, 4, 6, 0)
	before := shoulderDoc.Bodies()
	_, err = decad.Cut(t.Context(), joinedShoulder, tool)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, before, shoulderDoc.Bodies())
}

func revolveSectorCylinderTool(t testing.TB, doc *decad.Document,
	radius, start, height, centerY float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(start, centerY, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	return tool
}

func requireRevolveSectorCylinderCut(t *testing.T, doc *decad.Document,
	hub, joined *decad.Body, radius float64) {
	t.Helper()
	tool := revolveSectorCylinderTool(t, doc, radius, 4, 6, 0)
	before, err := joined.Volume()
	require.NoError(t, err)
	cut, err := decad.Cut(t.Context(), joined, tool)
	require.NoError(t, err)
	after, err := cut.Volume()
	require.NoError(t, err)
	require.Greater(t, before.Value.Base()-before.Bound.Base(),
		after.Value.Base()+after.Bound.Base())
	mesh, err := cut.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Equal(t, 0, revolveLugEuler(mesh))
	require.Contains(t, doc.Bodies(), cut)
	highCapX := math.Inf(-1)
	for _, original := range hub.Faces() {
		if plane, planar := original.Surface().(decad.Plane); planar {
			highCapX = max(highCapX, plane.Frame.Origin().X)
		}
	}
	checkedCaps := 0
	for _, original := range hub.Faces() {
		plane, planar := original.Surface().(decad.Plane)
		if !planar || (radius > 1.5 && plane.Frame.Origin().X != highCapX) {
			continue
		}
		checkedCaps++
		found := false
		for _, current := range cut.Faces() {
			if _, planar := current.Surface().(decad.Plane); planar &&
				slices.Equal(original.Origins(), current.Origins()) {
				found = true
			}
		}
		require.True(t, found, "a Revolve cap lost its source origins")
	}
	if radius > 1.5 {
		require.Equal(t, 1, checkedCaps)
	} else {
		require.Equal(t, 2, checkedCaps)
	}
	wallFound := false
	for _, original := range tool.Faces() {
		if _, cylindrical := original.Surface().(decad.Cylinder); !cylindrical {
			continue
		}
		for _, current := range cut.Faces() {
			if _, cylindrical := current.Surface().(decad.Cylinder); cylindrical &&
				slices.Equal(original.Origins(), current.Origins()) {
				wallFound = true
			}
		}
	}
	require.True(t, wallFound, "the bore wall lost its source origins")
}
