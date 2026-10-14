package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSectionSketchFeedsExtrude(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	sourceDoc := decad.New()
	source, err := decad.Extrude(t.Context(), sourceDoc, s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(5), Dir: decad.Along,
	})
	require.NoError(t, err)
	require.NoError(t, decad.Remove(source))
	cutPlane, err := w.CreateOffsetPlane(w.XY(), 2.5)
	require.NoError(t, err)
	frame, err := cutPlane.Frame()
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	require.Len(t, profiles[0].Outer, 4)
	sectionFrame, err := section.Plane().Frame()
	require.NoError(t, err)
	require.Equal(t, frame, sectionFrame)

	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.Equal(t, units.SquareMillimeters(100), area.Value)
	require.Equal(t, decad.Exact, area.Exactness)

	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.Equal(t, units.CubicMillimeters(200), volume.Value)
	require.Equal(t, decad.Exact, volume.Exactness)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Empty(t, sourceDoc.Bodies())
}

func TestSectionPlacedPrismKeepsHole(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(5, 5), 2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var sourceProfile *sketch.Profile
	for _, candidate := range s.Profiles() {
		if candidate.Valid && len(candidate.Outer) == 4 && len(candidate.Holes) == 1 {
			sourceProfile = candidate
			break
		}
	}
	require.NotNil(t, sourceProfile)
	doc := decad.New()
	source, err := decad.Extrude(t.Context(), doc, s, sourceProfile, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	move, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{Y: 1}, EY: r3.Vec{X: -1}, EZ: r3.Vec{Z: 1},
	}, r3.Vec{X: 20, Y: 30, Z: 1})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), source, move)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.Vec{X: 20, Y: 30, Z: 3.5}, r3.Vec{Y: 1}, r3.Vec{X: -1})
	require.NoError(t, err)
	section, profiles, err := placed.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.Len(t, profiles[0].Outer, 4)
	require.Len(t, profiles[0].Holes, 1)
	require.Equal(t, []*decad.Body{placed}, doc.Bodies())

	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-(100-4*math.Pi)), area.Bound.Base()+1e-12)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-2*(100-4*math.Pi)), volume.Bound.Base()+1e-12)
}

func TestSectionRefusesInvalidOrNonparallelPlane(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	capFrame, err := w.XY().Frame()
	require.NoError(t, err)
	_, _, err = body.Section(t.Context(), capFrame)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	oblique, err := r3.NewFrame(r3.Vec{Z: 2.5}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	_, _, err = body.Section(t.Context(), oblique)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	_, _, err = body.Section(t.Context(), r3.Frame{})
	require.ErrorIs(t, err, decad.ErrDegenerate)
	var nilBody *decad.Body
	_, _, err = nilBody.Section(t.Context(), capFrame)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	var nilContext context.Context
	_, _, err = body.Section(nilContext, capFrame)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, _, err = body.Section(canceled, capFrame)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, []*decad.Body{body}, doc.Bodies())
	sheet, err := decad.Extrude(t.Context(), decad.New(), s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(5), Dir: decad.Along,
	}, decad.WithSurfaceResult())
	require.NoError(t, err)
	_, _, err = sheet.Section(t.Context(), capFrame)
	require.ErrorIs(t, err, decad.ErrNotSolid)
}
