package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftFacesThreeCapConeContinuationKeepsSource(t *testing.T) {
	doc := New()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(8, 0, 0), r3.NewVec(0.6, -0.8, 0),
		r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(-3.025, -0.025, -2.975, 0.025)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	point, err := LoftFromPoint(t.Context(), doc, r3.Vec{}, s, s.Profiles()[0])
	require.NoError(t, err)
	sheet := func(x float64) *Body {
		ss, e := world.CreateSketch(world.XY())
		require.NoError(t, e)
		a, b := ss.CreatePoint(x, -20), ss.CreatePoint(x, 20)
		ss.Fix(a)
		ss.Fix(b)
		ss.CreateLine(a, b)
		_, e = ss.Solve(t.Context())
		require.NoError(t, e)
		body, e := Extrude(t.Context(), doc, ss, ss.Chains()[0], TwoSided{
			One: DistanceSide{D: units.Millimeters(20)}, Two: DistanceSide{D: units.Millimeters(20)}})
		require.NoError(t, e)
		return body
	}
	first, err := Split(t.Context(), point, sheet(5))
	require.NoError(t, err)
	second, err := Split(t.Context(), first[0], sheet(4.5))
	require.NoError(t, err)
	near, far := second[1], first[1]
	section := func(body *Body, role provenance.Ref) *Face {
		fs, e := Faces(FaceCreatedBy(featureRefOf(role))).SelectFaces(body)
		require.NoError(t, e)
		require.Len(t, fs, 1)
		return fs[0]
	}
	loft, err := LoftFaces(t.Context(), doc, section(near, refOf(CapStart(near))),
		section(near, refOf(CapEnd(near))), section(far, refOf(CapEnd(far))))
	require.NoError(t, err)
	baseRecord := loft.n.Payload.(facetedPayload).LoftFacesCap
	require.Len(t, baseRecord.Maps, 3)
	require.Len(t, baseRecord.Bounds, 3)
	require.Len(t, baseRecord.StationGroup, len(baseRecord.Source.SourceStation))
	cone := func(apex float64) *Body {
		cs, e := world.CreateSketch(world.XY())
		require.NoError(t, e)
		points := []*sketch.Point{cs.CreatePoint(-2, 0), cs.CreatePoint(-2, apex+2),
			cs.CreatePoint(apex, 0)}
		for _, p := range points {
			cs.Fix(p)
		}
		for i := range points {
			cs.CreateLine(points[i], points[(i+1)%len(points)])
		}
		_, e = cs.Solve(t.Context())
		require.NoError(t, e)
		body, e := Revolve(t.Context(), doc, cs, cs.Profiles()[0], SketchLine{
			Start: planePoint{}, End: planePoint{U: 1}}, FullRevolution{})
		require.NoError(t, e)
		return body
	}
	cut, err := Cut(t.Context(), loft, cone(7.5))
	require.NoError(t, err)
	require.NotNil(t, cut.n.Payload.(facetedPayload).LoftFacesToe)
	trim, err := Intersect(t.Context(), cut, cone(8))
	require.NoError(t, err)
	trimRecord := trim.n.Payload.(facetedPayload).LoftFacesTrim
	require.NotNil(t, trimRecord)
	require.Same(t, baseRecord.Source, trimRecord.Source)
	require.Len(t, trimRecord.Maps, 3)
	require.Len(t, trimRecord.Bounds, 3)
	require.Equal(t, baseRecord.StationGroup, trimRecord.StationGroup)
	loftVolume, err := loft.Volume()
	require.NoError(t, err)
	cutVolume, err := cut.Volume()
	require.NoError(t, err)
	trimVolume, err := trim.Volume()
	require.NoError(t, err)
	require.Less(t, cutVolume.Value.Base(), loftVolume.Value.Base())
	require.Less(t, trimVolume.Value.Base(), cutVolume.Value.Base())
	mesh, err := trim.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}
