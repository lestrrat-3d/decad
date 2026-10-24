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

func straightSweepSquare(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 20, 20)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 10)})
	require.NoError(t, err)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, s.Profiles()[0], path)
	require.NoError(t, err)
	return doc, body
}

func convexVerticalSweepEdges() *decad.EdgeQuery {
	return decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex())
}

func TestSweepLineFilletVerticalEdgesVerified(t *testing.T) {
	t.Parallel()
	doc, body := straightSweepSquare(t)
	selected, err := convexVerticalSweepEdges().SelectEdges(body)
	require.NoError(t, err)
	require.Len(t, selected, 4)

	rounded, err := decad.Fillet(t.Context(), body, convexVerticalSweepEdges(), units.Millimeters(1))
	require.NoError(t, err)
	volume, err := rounded.Volume()
	require.NoError(t, err)
	got, err := volume.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, (400-4+math.Pi)*10, got, 1e-9)
	mesh, err := rounded.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)

	move, err := r3.Translation(r3.NewVec(3, -2, 5))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), rounded, move)
	require.NoError(t, err)
	placedVolume, err := placed.Volume()
	require.NoError(t, err)
	placedValue, err := placedVolume.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, got, placedValue, 1e-9)
	_, err = placed.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
}

func TestSweepLineFilletCapEdgeRefuses(t *testing.T) {
	t.Parallel()
	doc, body := straightSweepSquare(t)
	_, err := decad.Fillet(t.Context(), body, decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0))),
		units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "one cap edge at a time")
	require.Equal(t, []*decad.Body{body}, doc.Bodies())
}

func TestSweepLineFilletSingleTopCapEdgeVerified(t *testing.T) {
	t.Parallel()
	doc, body := straightSweepSquare(t)
	sel := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)),
		decad.EndpointAt(r3.NewVec(0, 0, 10))).Exactly(1)
	selected, err := sel.SelectEdges(body)
	require.NoError(t, err)
	require.Len(t, selected, 1)

	rounded, err := decad.Fillet(t.Context(), body, sel, units.Millimeters(1))
	require.NoError(t, err)
	volume, err := rounded.Volume()
	require.NoError(t, err)
	got, err := volume.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, 4000-20*(1-math.Pi/4), got, 1e-9)
	require.Len(t, rounded.Faces(), 7)
	require.Len(t, rounded.Edges(), 15)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, report.Passed())
	mesh, err := rounded.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSweepLineFilletPlacedBottomCapEdgeVerified(t *testing.T) {
	t.Parallel()
	_, body := straightSweepSquare(t)
	motion, err := r3.Translation(r3.NewVec(3, -2, 5))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), body, motion)
	require.NoError(t, err)
	sel := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)),
		decad.EndpointAt(r3.NewVec(3, -2, 5))).Exactly(1)
	rounded, err := decad.Fillet(t.Context(), placed, sel, units.Millimeters(1))
	require.NoError(t, err)
	volume, err := rounded.Volume()
	require.NoError(t, err)
	got, err := volume.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, 4000-20*(1-math.Pi/4), got, 1e-9)
	mesh, err := rounded.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSweepLineFilletPlacedReceiver(t *testing.T) {
	t.Parallel()
	_, body := straightSweepSquare(t)
	move, err := r3.Translation(r3.NewVec(3, -2, 5))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), body, move)
	require.NoError(t, err)
	rounded, err := decad.Fillet(t.Context(), placed, convexVerticalSweepEdges(), units.Millimeters(1))
	require.NoError(t, err)
	volume, err := rounded.Volume()
	require.NoError(t, err)
	got, err := volume.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, (400-4+math.Pi)*10, got, 1e-9)
	_, err = rounded.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
}

func TestSweepFilletRefusesArcCompositeAndMitredPaths(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name          string
		path          *decad.Path
		opts          []decad.SweepOption
		centredSquare bool
	}{
		{name: "arc", path: mustPathArc(t)},
		{name: "composite", path: mustPath(t, r3.Vec{}, r3.NewVec(0, 0, 5), r3.NewVec(0, 0, 10))},
		{name: "mitred", path: mustPath(t, r3.Vec{}, r3.NewVec(0, 0, 10), r3.NewVec(10, 0, 10)),
			opts: []decad.SweepOption{decad.WithMitredJoins()}, centredSquare: true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			s, profile := plateSketch(t)
			if tc.centredSquare {
				s, profile = squareSweepSketch(t, 2)
			}
			doc := decad.New()
			body, err := decad.Sweep(t.Context(), doc, s, profile, tc.path, tc.opts...)
			require.NoError(t, err)
			_, err = decad.Fillet(t.Context(), body, decad.Edges().AtLeast(1), units.Millimeters(1))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Equal(t, []*decad.Body{body}, doc.Bodies())
		})
	}
}

func mustPathArc(t *testing.T) *decad.Path {
	t.Helper()
	path, err := decad.NewPath(r3.NewVec(0, 5, 0), decad.ArcThrough{
		Through: r3.NewVec(0, 3, 4), End: r3.NewVec(0, 0, 5),
	})
	require.NoError(t, err)
	return path
}
