package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func neutralSlabCap(t *testing.T, slab *decad.Body, role decad.FeatureRef) *decad.Face {
	t.Helper()
	faces, err := decad.Faces(decad.FaceCreatedBy(role)).SelectFaces(slab)
	require.NoError(t, err)
	require.Len(t, faces, 1)
	return faces[0]
}

func TestLoftFacesTwoAndThreeCapsFromPointSlabs(t *testing.T) {
	doc := decad.New()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(10, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	p := s.CreateRectangle(-1, -1, 1, 1)
	s.Fix(p.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	loft, err := decad.LoftFromPoint(t.Context(), doc, r3.Vec{}, s, s.Profiles()[0])
	require.NoError(t, err)
	makeSheet := func(x float64) *decad.Body {
		sheetSketch, err := world.CreateSketch(world.XY())
		require.NoError(t, err)
		a := sheetSketch.CreatePoint(x, -20)
		b := sheetSketch.CreatePoint(x, 20)
		sheetSketch.Fix(a)
		sheetSketch.Fix(b)
		sheetSketch.CreateLine(a, b)
		_, err = sheetSketch.Solve(t.Context())
		require.NoError(t, err)
		sheet, err := decad.Extrude(t.Context(), doc, sheetSketch, sheetSketch.Chains()[0], decad.TwoSided{
			One: decad.DistanceSide{D: units.Millimeters(20)},
			Two: decad.DistanceSide{D: units.Millimeters(20)},
		})
		require.NoError(t, err)
		return sheet
	}
	first, err := decad.Split(t.Context(), loft, makeSheet(9))
	require.NoError(t, err)
	second, err := decad.Split(t.Context(), first[0], makeSheet(8))
	require.NoError(t, err)
	near, far := second[1], first[1]
	sections := []*decad.Face{
		neutralSlabCap(t, near, decad.CapStart(near)),
		neutralSlabCap(t, near, decad.CapEnd(near)),
		neutralSlabCap(t, far, decad.CapEnd(far)),
	}
	before := doc.Bodies()
	_, err = decad.LoftFaces(t.Context(), doc, sections[:1]...)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, before, doc.Bodies())
	tooMany := make([]*decad.Face, 33)
	for i := range tooMany {
		tooMany[i] = sections[i%len(sections)]
	}
	_, err = decad.LoftFaces(t.Context(), doc, tooMany...)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, before, doc.Bodies())
	_, err = decad.LoftFaces(t.Context(), doc, sections[0], sections[1], sections[1])
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, before, doc.Bodies())
	_, err = decad.LoftFaces(t.Context(), doc, sections[0], sections[2], sections[1])
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, before, doc.Bodies())

	twoCaps, err := decad.LoftFaces(t.Context(), doc, sections[:2]...)
	require.NoError(t, err)
	verify := func(body *decad.Body, wantVolume float64) {
		t.Helper()
		mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1),
			decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
		reading, err := body.Volume()
		require.NoError(t, err)
		require.InDelta(t, wantVolume, reading.Value.Base(), reading.Bound.Base()+1e-9)
	}
	verify(twoCaps, 217.0/75.0)

	result, err := decad.LoftFaces(t.Context(), doc, sections...)
	require.NoError(t, err)
	verify(result, 488.0/75.0)
}
