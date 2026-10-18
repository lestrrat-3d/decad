package apitest_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func roundedLoftSection(t *testing.T, world *sketch.World, halfWidth, radius, z float64) decad.LoftSection {
	t.Helper()
	plane := world.XY()
	if z != 0 {
		var err error
		plane, err = world.CreateOffsetPlane(world.XY(), z)
		require.NoError(t, err)
	}
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	point := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	const halfHeight = 3.0
	p := [8]*sketch.Point{
		point(-halfWidth+radius, -halfHeight),
		point(halfWidth-radius, -halfHeight),
		point(halfWidth, -halfHeight+radius),
		point(halfWidth, halfHeight-radius),
		point(halfWidth-radius, halfHeight),
		point(-halfWidth+radius, halfHeight),
		point(-halfWidth, halfHeight-radius),
		point(-halfWidth, -halfHeight+radius),
	}
	s.CreateLine(p[0], p[1])
	s.CreateArc(point(halfWidth-radius, -halfHeight+radius), p[1], p[2])
	s.CreateLine(p[2], p[3])
	s.CreateArc(point(halfWidth-radius, halfHeight-radius), p[3], p[4])
	s.CreateLine(p[4], p[5])
	s.CreateArc(point(-halfWidth+radius, halfHeight-radius), p[5], p[6])
	s.CreateLine(p[6], p[7])
	s.CreateArc(point(-halfWidth+radius, -halfHeight+radius), p[7], p[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, profile := range s.Profiles() {
		if profile.Valid && len(profile.Outer) == 8 && len(profile.Holes) == 0 {
			return decad.LoftSection{Sketch: s, Profile: profile}
		}
	}
	t.Fatal("Sketch did not produce the rounded rectangle")
	return decad.LoftSection{}
}

func TestLoftSectionsFourRoundedProfilesInterpolateNonAffineWidth(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	widths := [4]float64{4, 5.125, 6.25, 7}
	var sections [4]decad.LoftSection
	for i, width := range widths {
		sections[i] = roundedLoftSection(t, w, width, 1, 4*float64(i))
	}
	doc := decad.New()
	body, err := decad.LoftSections(t.Context(), doc, sections[:]...)
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	require.Len(t, body.Faces(), 10)
	roles := make(map[string]bool)
	for _, face := range body.Faces() {
		for _, origin := range face.Origins() {
			roles[origin.Role] = true
		}
	}
	for _, role := range []string{roleCapStart, roleCapEnd} {
		require.True(t, roles[role], "missing source role %s", role)
	}
	for i := range 8 {
		role := fmt.Sprintf("side(0,%d)", i)
		require.True(t, roles[role], "missing source role %s", role)
	}
	volume, err := body.Volume()
	require.NoError(t, err)
	trueVolume := 12 * (12*5.640625 - 4 + math.Pi)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-trueVolume), volume.Bound.Base())
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.05)
	require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()))
	live := make(map[*decad.Face]bool)
	for _, face := range body.Faces() {
		live[face] = true
	}
	for _, face := range mesh.SourceFaces() {
		require.True(t, live[face], "every facet must name a live source face")
	}
	move, err := r3.Translation(r3.NewVec(20, 0, 0))
	require.NoError(t, err)
	placed, err := decad.Copy(t.Context(), body, decad.WithCopyPlacement(move))
	require.NoError(t, err)
	placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, placedMesh.BoundaryVerified())
	require.True(t, placedMesh.VolumeVerified())
}

func TestLoftSectionsFourRoundedProfilesRefuseUnprovedShape(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		widths [4]float64
		radii  [4]float64
		levels [4]float64
	}{
		{"changed radius", [4]float64{4, 5.125, 6.25, 7}, [4]float64{1, 1.25, 1, 1},
			[4]float64{0, 4, 8, 12}},
		{"unequal levels", [4]float64{4, 5.125, 6.25, 7}, [4]float64{1, 1, 1, 1},
			[4]float64{0, 4, 9, 12}},
		{"nonmonotone width", [4]float64{4, 7, 4, 7}, [4]float64{1, 1, 1, 1},
			[4]float64{0, 4, 8, 12}},
		{"affine width", [4]float64{4, 5, 6, 7}, [4]float64{1, 1, 1, 1},
			[4]float64{0, 4, 8, 12}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			w := sketch.NewWorld()
			var sections [4]decad.LoftSection
			for i := range sections {
				sections[i] = roundedLoftSection(t, w, tc.widths[i], tc.radii[i], tc.levels[i])
			}
			doc := decad.New()
			body, err := decad.LoftSections(t.Context(), doc, sections[:]...)
			require.Nil(t, body)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Empty(t, doc.Bodies())
		})
	}
}
