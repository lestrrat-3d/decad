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
	lower, err := decad.Loft(t.Context(), doc, bottomSketch, bottomProfile, middleSketch, middleProfile)
	require.NoError(t, err)
	upper, err := decad.MirroredCopy(t.Context(), lower, decad.MirrorFrame{Frame: middleFrame})
	require.NoError(t, err)
	pair, err := decad.Union(t.Context(), lower, upper)
	require.NoError(t, err)
	pattern := []*decad.Body{pair}
	// Feed copies in reverse angular order. The certified source Join sorts
	// its teeth, so the near-axis upper shift must follow the sorted tooth.
	for i := 7; i >= 1; i-- {
		angle := units.Degrees(float64(45 * i))
		if i == 5 {
			angle = units.Radians(10 * 2 * math.Pi / 17)
		}
		turn, err := r3.Rotation(r3.Vec{Z: 1}, angle)
		require.NoError(t, err)
		if i == 5 {
			require.Equal(t, math.Nextafter(1, 0), turn.Basis().EZ.Z)
		}
		turned, err := decad.Copy(t.Context(), pair, decad.WithCopyPlacement(turn))
		require.NoError(t, err)
		pattern = append(pattern, turned)
	}
	blank := circleBodyAtZ(t, doc, 0, 4, 0, 2)
	joined, err := decad.Union(t.Context(), blank, pattern...)
	require.NoError(t, err)
	bore := circleBodyAtZ(t, doc, 0, 0.5, -1, 4)
	cut, err := decad.Cut(t.Context(), joined, bore)
	require.NoError(t, err)
	_, err = decad.Chamfer(t.Context(), cut, decad.Edges(decad.OuterLoopOf(decad.CapStart(cut))),
		units.Millimeters(0.03))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	outerCaps := decad.Edges(decad.OuterLoopOf(decad.CapStart(cut))).
		Or(decad.OuterLoopOf(decad.CapEnd(cut)))
	chamfered, err := decad.Chamfer(t.Context(), cut, outerCaps, units.Millimeters(0.03))
	require.NoError(t, err)
	for _, role := range []decad.FeatureRef{decad.CapStart(chamfered), decad.CapEnd(chamfered)} {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(chamfered)
		require.NoError(t, err)
		require.Len(t, faces[0].Loops(), 2)
		for _, edge := range faces[0].Loops()[1].Edges() {
			for _, vertex := range []*decad.Vertex{edge.Start(), edge.End()} {
				p := vertex.Position().Value
				require.InDelta(t, 0.5, math.Hypot(p.X, p.Y), 1e-12)
			}
		}
	}
	mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Equal(t, 0, loftMirrorChamferEuler(mesh))
	require.Len(t, doc.Bodies(), 1)
}

func loftMirrorChamferEuler(mesh *decad.Mesh) int {
	edges := make(map[[2]int]struct{})
	tris := mesh.Triangles()
	for _, tri := range tris {
		for i := range 3 {
			a, b := tri[i], tri[(i+1)%3]
			if a > b {
				a, b = b, a
			}
			edges[[2]int{a, b}] = struct{}{}
		}
	}
	return len(mesh.Vertices()) - len(edges) + len(tris)
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
