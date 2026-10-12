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

func TestLoftMirrorCylinderBoreChamfersExteriorCaps(t *testing.T) {
	doc := decad.New()
	world := sketch.NewWorld()
	middleFrame, err := r3.NewFrame(r3.Vec{Z: 1}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	middlePlane, err := world.CreatePlaneFromFrame(middleFrame)
	require.NoError(t, err)
	bottomSketch, bottomProfile := loftMirrorChamferSection(t, world, world.XY(), 0)
	middleSketch, middleProfile := loftMirrorChamferSection(t, world, middlePlane, 0.03)
	lower, err := doc.Loft(t.Context(), bottomSketch, bottomProfile, middleSketch, middleProfile)
	require.NoError(t, err)
	upper, err := lower.MirroredCopy(t.Context(), decad.MirrorFrame{Frame: middleFrame})
	require.NoError(t, err)
	pair, err := decad.Union(t.Context(), lower, upper)
	require.NoError(t, err)
	pattern := []*decad.Body{pair}
	for i := 1; i < 8; i++ {
		turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(float64(45*i)))
		require.NoError(t, err)
		turned, err := pair.PlacedCopy(t.Context(), turn)
		require.NoError(t, err)
		pattern = append(pattern, turned)
	}
	blank := circleBodyAtZ(t, doc, 0, 4, 0, 2)
	joined, err := decad.Union(t.Context(), blank, pattern...)
	require.NoError(t, err)
	bore := circleBodyAtZ(t, doc, 0, 0.5, -1, 4)
	cut, err := decad.Cut(t.Context(), joined, bore)
	require.NoError(t, err)
	outerCaps := decad.Edges(decad.OuterLoopOf(decad.CapStart(cut))).
		Or(decad.OuterLoopOf(decad.CapEnd(cut)))
	chamfered, err := cut.Chamfer(t.Context(), outerCaps, units.Millimeters(0.03))
	require.NoError(t, err)
	mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Len(t, doc.Bodies(), 1)
}

// A broad radial lobe gives its rounded side a resolvable inward offset while
// keeping the Join's six-curve source structure independent of any gear model.
func loftMirrorChamferSection(t *testing.T, world *sketch.World, plane *sketch.Plane,
	turn float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	polar := func(radius, angle float64) *sketch.Point {
		point := s.CreatePoint(radius*math.Cos(angle+turn), radius*math.Sin(angle+turn))
		s.Fix(point)
		return point
	}
	rightRoot, leftRoot := polar(4, -0.12), polar(4, 0.12)
	right := make([]*sketch.Point, 3)
	left := make([]*sketch.Point, 3)
	for i := range right {
		radius := 5 + float64(i)
		angle := 0.09 - 0.02*float64(i)
		right[i] = polar(radius, -angle)
		left[i] = polar(radius, angle)
	}
	s.CreateLine(rightRoot, right[0])
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(center, right[len(right)-1], left[len(left)-1])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateLine(left[0], leftRoot)
	s.CreateArc(center, rightRoot, leftRoot)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	return s, profiles[0]
}
