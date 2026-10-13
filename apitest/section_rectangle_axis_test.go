package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSectionCardinalRectangleFeedsVerifiedExtrude(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	source, err := decad.Extrude(t.Context(), decad.New(), s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(5), Dir: decad.Along,
	})
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(5, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	area, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	areaReading, err := area.Area()
	require.NoError(t, err)
	require.Equal(t, units.SquareMillimeters(50), areaReading.Value)
	require.Equal(t, decad.Exact, areaReading.Exactness)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.Equal(t, units.CubicMillimeters(100), volume.Value)
	require.Equal(t, decad.Exact, volume.Exactness)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionCardinalRectanglePlacementAndAxisSense(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 8, 6)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	source, err := decad.Extrude(t.Context(), decad.New(), s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(5), Dir: decad.Along,
	})
	require.NoError(t, err)
	placement, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{Y: 1}, EY: r3.Vec{X: -1}, EZ: r3.Vec{Z: 1},
	}, r3.NewVec(20, 30, 7))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), source, placement)
	require.NoError(t, err)
	for _, tc := range []struct {
		name   string
		origin r3.Vec
		u, v   r3.Vec
		area   units.Value
	}{
		{"reverse sweep axis", r3.NewVec(17, 30, 9.5), r3.NewVec(0, 1, 0),
			r3.NewVec(0, 0, -1), units.SquareMillimeters(40)},
		{"other radial axis", r3.NewVec(20, 34, 7), r3.NewVec(-1, 0, 0),
			r3.NewVec(0, 0, 1), units.SquareMillimeters(30)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame, err := r3.NewFrame(tc.origin, tc.u, tc.v)
			require.NoError(t, err)
			section, profiles, err := placed.Section(t.Context(), frame)
			require.NoError(t, err)
			require.Len(t, profiles, 1)
			measured, err := decad.MeasureProfile(section, profiles[0])
			require.NoError(t, err)
			area, err := measured.Area()
			require.NoError(t, err)
			require.Equal(t, tc.area, area.Value)
			require.Equal(t, decad.Exact, area.Exactness)
		})
	}
}

func TestSectionCardinalRectangleRefusesTouchAndOtherSources(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	source, err := decad.Extrude(t.Context(), decad.New(), s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(5), Dir: decad.Along,
	})
	require.NoError(t, err)
	for _, tc := range []struct {
		name   string
		origin r3.Vec
		u      r3.Vec
	}{
		{"tangent", r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 0)},
		{"outside", r3.NewVec(-1, 0, 0), r3.NewVec(0, 1, 0)},
		{"unrepresentable corner", r3.NewVec(5, 1e17, 0), r3.NewVec(0, 1, 0)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame, err := r3.NewFrame(tc.origin, tc.u, r3.NewVec(0, 0, 1))
			require.NoError(t, err)
			section, profiles, err := source.Section(t.Context(), frame)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Nil(t, section)
			require.Empty(t, profiles)
		})
	}
	frame, err := r3.NewFrame(r3.NewVec(5, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	tapered, err := decad.Extrude(t.Context(), decad.New(), s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(5), Dir: decad.Along,
	}, decad.WithTaper(units.Degrees(5)))
	require.NoError(t, err)
	section, profiles, err := tapered.Section(t.Context(), frame)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Nil(t, section)
	require.Empty(t, profiles)
}
