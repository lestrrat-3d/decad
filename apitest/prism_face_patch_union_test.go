package apitest_test

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func prismFacePatchRectangle(t *testing.T, w *sketch.World, plane *sketch.Plane,
	u0, v0, u1, v1 float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	r := s.CreateRectangle(u0, v0, u1, v1)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	require.True(t, s.Profiles()[0].Valid)
	return s, s.Profiles()[0]
}

func prismFacePatchPair(t *testing.T, sideX, sideY0, sideY1 float64, dir decad.Direction,
	opts ...decad.DocumentOption) (*decad.Body, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	doc := decad.New(opts...)
	plateSketch, plateProfile := prismFacePatchRectangle(t, w, w.XY(), 0, 0, 20, 20)
	plate, err := decad.Extrude(t.Context(), doc, plateSketch, plateProfile,
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	plane, err := w.CreateOffsetPlane(w.YZ(), sideX)
	require.NoError(t, err)
	lugSketch, lugProfile := prismFacePatchRectangle(t, w, plane, sideY0, 1, sideY1, 4)
	lug, err := decad.Extrude(t.Context(), doc, lugSketch, lugProfile,
		decad.Distance{D: units.Millimeters(5), Dir: dir})
	require.NoError(t, err)
	return plate, lug
}

func TestUnionRectangularFacePatchFromPerpendicularExtrudes(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		x    float64
		dir  decad.Direction
		swap bool
	}{
		{name: "positive side", x: 20, dir: decad.Along},
		{name: "negative side", x: 0, dir: decad.Against},
		{name: "swapped operands", x: 20, dir: decad.Along, swap: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			plate, lug := prismFacePatchPair(t, tc.x, 5, 15, tc.dir)
			if tc.swap {
				plate, lug = lug, plate
			}
			joined, err := decad.Union(t.Context(), plate, lug)
			require.NoError(t, err)
			require.Len(t, joined.Lumps(), 1)
			volume, err := joined.Volume()
			require.NoError(t, err)
			require.Equal(t, units.CubicMillimeters(2150), volume.Value)
			require.Equal(t, decad.Exact, volume.Exactness)
			mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
			box, err := joined.Bounds()
			require.NoError(t, err)
			if tc.x == 20 {
				require.Equal(t, r3.NewVec(25, 20, 5), box.Max)
			} else {
				require.Equal(t, r3.NewVec(-5, 0, 0), box.Min)
			}
		})
	}
}

func TestUnionRectangularFacePatchRefusesEdgeContact(t *testing.T) {
	t.Parallel()
	plate, lug := prismFacePatchPair(t, 20, 20, 25, decad.Along, decad.ExactOnly())
	joined, err := decad.Union(t.Context(), plate, lug)
	require.Nil(t, joined)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	var booleanErr *decad.BooleanError
	require.True(t, errors.As(err, &booleanErr))
	require.Equal(t, decad.BooleanUnsupportedContact, booleanErr.Code)
}

func TestUnionRectangularFacePatchAcrossOtherAxis(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	doc := decad.New()
	plateSketch, plateProfile := prismFacePatchRectangle(t, w, w.XY(), 0, 0, 20, 20)
	plate, err := decad.Extrude(t.Context(), doc, plateSketch, plateProfile,
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	plane, err := w.CreateOffsetPlane(w.XZ(), -20)
	require.NoError(t, err)
	lugSketch, lugProfile := prismFacePatchRectangle(t, w, plane, 5, 1, 15, 4)
	lug, err := decad.Extrude(t.Context(), doc, lugSketch, lugProfile,
		decad.Distance{D: units.Millimeters(5), Dir: decad.Against})
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), plate, lug)
	require.NoError(t, err)
	require.Len(t, joined.Lumps(), 1)
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.Equal(t, units.CubicMillimeters(2150), volume.Value)
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestUnionRectangularFacePatchUsesOtherSlabAxis(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	doc := decad.New()
	plateSketch, plateProfile := prismFacePatchRectangle(t, w, w.XY(), 0, 0, 20, 20)
	plate, err := decad.Extrude(t.Context(), doc, plateSketch, plateProfile,
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	plane, err := w.CreateOffsetPlane(w.YZ(), 5)
	require.NoError(t, err)
	lugSketch, lugProfile := prismFacePatchRectangle(t, w, plane, 20, 0, 25, 5)
	lug, err := decad.Extrude(t.Context(), doc, lugSketch, lugProfile,
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), plate, lug)
	require.NoError(t, err)
	require.Len(t, joined.Lumps(), 1)
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.Equal(t, units.CubicMillimeters(2250), volume.Value)
	require.Equal(t, decad.Exact, volume.Exactness)
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	box, err := joined.Bounds()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(20, 25, 5), box.Max)
}

func TestUnionRectangularFacePatchRefusesTaperedSource(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	doc := decad.New(decad.ExactOnly())
	plateSketch, plateProfile := prismFacePatchRectangle(t, w, w.XY(), 0, 0, 20, 20)
	plate, err := decad.Extrude(t.Context(), doc, plateSketch, plateProfile,
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	plane, err := w.CreateOffsetPlane(w.YZ(), 20)
	require.NoError(t, err)
	lugSketch, lugProfile := prismFacePatchRectangle(t, w, plane, 5, 1, 15, 4)
	lug, err := decad.Extrude(t.Context(), doc, lugSketch, lugProfile,
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along}, decad.WithTaper(units.Degrees(5)))
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), plate, lug)
	require.Nil(t, joined)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
