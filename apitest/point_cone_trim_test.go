package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func pointConeTooth(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.YZ(), 8)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	pts := []*sketch.Point{
		s.CreatePoint(3, -1), s.CreatePoint(4, -1.5), s.CreatePoint(5, -1),
		s.CreatePoint(5, 1), s.CreatePoint(3, 1),
	}
	for _, p := range pts {
		s.Fix(p)
	}
	_, err = s.CreateFitSpline(pts[0], pts[1], pts[2])
	require.NoError(t, err)
	for i := 2; i < len(pts); i++ {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	tooth, err := doc.LoftFromPoint(t.Context(), r3.Vec{}, s, profiles[0])
	require.NoError(t, err)
	return tooth
}

func pointConeTool(t *testing.T, doc *decad.Document, apex float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := []*sketch.Point{
		s.CreatePoint(-2, 0), s.CreatePoint(-2, apex+2), s.CreatePoint(apex, 0),
	}
	for _, p := range pts {
		s.Fix(p)
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	tool, err := doc.Revolve(s, profiles[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return tool
}

func requirePointConeMesh(t *testing.T, body *decad.Body) {
	t.Helper()
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	var coneFaces, freeformFaces int
	for _, face := range body.Faces() {
		switch face.Surface().(type) {
		case decad.Cone:
			coneFaces++
		case decad.NURBSSurface:
			freeformFaces++
		}
	}
	require.Positive(t, coneFaces)
	require.Positive(t, freeformFaces)
}

func TestPointConeTrimKeepsFittedSidesAndCone(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		apex    float64
		cut     bool
		ordered bool
	}{
		{name: "toe cut", apex: 5, cut: true},
		{name: "toe inside", apex: 5},
		{name: "heel scrap", apex: 12, cut: true},
		{name: "heel inside", apex: 12},
		{name: "toe then heel scrap", apex: 12, cut: true, ordered: true},
		{name: "toe then heel inside", apex: 12, ordered: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			tooth := pointConeTooth(t, doc)
			if tc.ordered {
				toe := pointConeTool(t, doc, 5)
				var err error
				tooth, err = decad.Cut(t.Context(), tooth, toe)
				require.NoError(t, err)
			}
			tool := pointConeTool(t, doc, tc.apex)
			var got *decad.Body
			var err error
			if tc.cut {
				got, err = decad.Cut(t.Context(), tooth, tool)
			} else {
				got, err = decad.Intersect(t.Context(), tooth, tool)
			}
			require.NoError(t, err)
			require.True(t, got.IsSolid())
			requirePointConeMesh(t, got)
			volume, err := got.Volume()
			require.NoError(t, err)
			require.Positive(t, volume.Value.Base())
		})
	}
}

func TestPointConeTrimConeNormalAndPlacedCopy(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	tooth := pointConeTooth(t, doc)
	tool := pointConeTool(t, doc, 5)
	var sourceCone *decad.Face
	for _, face := range tool.Faces() {
		if _, ok := face.Surface().(decad.Cone); ok {
			sourceCone = face
			break
		}
	}
	require.NotNil(t, sourceCone)
	cut, err := decad.Cut(t.Context(), tooth, tool)
	require.NoError(t, err)
	var cone *decad.Face
	for _, face := range cut.Faces() {
		if _, ok := face.Surface().(decad.Cone); ok {
			cone = face
			break
		}
	}
	require.NotNil(t, cone)
	normal, err := cone.NormalAt(r3.NewVec(3, 2, 0))
	require.NoError(t, err)
	sourceNormal, err := sourceCone.NormalAt(r3.NewVec(3, 2, 0))
	require.NoError(t, err)
	require.Less(t, normal.Value.X, 0.0)
	require.Less(t, normal.Value.Y, 0.0)
	require.InDelta(t, -sourceNormal.Value.X, normal.Value.X, 1e-14)
	require.InDelta(t, -sourceNormal.Value.Y, normal.Value.Y, 1e-14)
	require.Equal(t, sourceNormal.Bound, normal.Bound)
	move, err := r3.Translation(r3.NewVec(1, 2, 3))
	require.NoError(t, err)
	placed, err := cut.PlacedCopy(t.Context(), move)
	require.NoError(t, err)
	requirePointConeMesh(t, placed)
	var moved decad.Cone
	for _, face := range placed.Faces() {
		if c, ok := face.Surface().(decad.Cone); ok {
			moved = c
			break
		}
	}
	require.Equal(t, r3.NewVec(6, 2, 3), moved.Origin)
	var placedCone *decad.Face
	for _, face := range placed.Faces() {
		if _, ok := face.Surface().(decad.Cone); ok {
			placedCone = face
			break
		}
	}
	require.NotNil(t, placedCone)
	placedNormal, err := placedCone.NormalAt(r3.NewVec(4, 4, 3))
	require.NoError(t, err)
	require.InDelta(t, normal.Value.X, placedNormal.Value.X, 1e-14)
	require.InDelta(t, normal.Value.Y, placedNormal.Value.Y, 1e-14)
}

func TestPointConeTrimContainingCutCannotBuildInvertedBand(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	tooth := pointConeTooth(t, doc)
	inside, err := decad.Intersect(t.Context(), tooth, pointConeTool(t, doc, 5))
	require.NoError(t, err)
	_, err = decad.Cut(t.Context(), inside, pointConeTool(t, doc, 12))
	require.ErrorIs(t, err, decad.ErrBooleanFailed)
}
