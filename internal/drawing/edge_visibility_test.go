package drawing_test

import (
	"bytes"
	"context"
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/decad/internal/drawing"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/raster"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func visibilityBody(tb testing.TB, notches int) *decad.Body {
	tb.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(tb, err)
	wantVolume := 2000.0
	if notches == 0 {
		rect := s.CreateRectangle(0, 0, 10, 10)
		s.Fix(rect.A)
	} else {
		points := [][2]float64{{0, 0}, {float64(2 * notches), 0}, {float64(2 * notches), 10}}
		for i := notches - 1; i >= 0; i-- {
			x := float64(2 * i)
			points = append(points, [2]float64{x + 1.5, 10}, [2]float64{x + 1.5, 6},
				[2]float64{x + 0.5, 6}, [2]float64{x + 0.5, 10}, [2]float64{x, 10})
		}
		var nodes []*sketch.Point
		for _, p := range points {
			node := s.CreatePoint(p[0], p[1])
			s.Fix(node)
			nodes = append(nodes, node)
		}
		for i, p := range nodes {
			s.CreateLine(p, nodes[(i+1)%len(nodes)])
		}
		wantVolume = float64(16 * notches * 20)
	}
	_, err = s.Solve(tb.Context())
	require.NoError(tb, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 0 {
			profile = p
			break
		}
	}
	require.NotNil(tb, profile)
	body, err := decad.Extrude(tb.Context(), decad.New(), s, profile,
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(tb, err)
	volume, err := body.Volume()
	require.NoError(tb, err)
	require.InDelta(tb, wantVolume, volume.Value.Base(), 1e-8)
	return body
}

func visibilityView() export.DrawingView {
	return export.DrawingView{Direction: r3.NewVec(0.55, 0.8, 0.45), Up: r3.NewVec(0, 0, 1)}
}

func visibilityProject(tb testing.TB, ctx context.Context, body *decad.Body) drawing.Lines {
	tb.Helper()
	mesh, err := body.Tessellate(ctx, units.Millimeters(0.1), decad.WithVerification(decad.VerifyNone))
	require.NoError(tb, err)
	view := visibilityView()
	lines, err := drawing.Project(ctx, mesh.Vertices(), mesh.Triangles(),
		raster.Camera{Direction: view.Direction, Up: view.Up})
	require.NoError(tb, err)
	return lines
}

func TestVisibilityRealBoxBoundary(t *testing.T) {
	body := visibilityBody(t, 0)
	lines := visibilityProject(t, t.Context(), body)
	require.Len(t, lines.Visible, 9)
	require.Len(t, lines.Hidden, 3)
	var out bytes.Buffer
	require.NoError(t, export.DXF(t.Context(), &out, body, units.Millimeters(0.1), visibilityView()))
	require.Equal(t, 12, bytes.Count(out.Bytes(), []byte("\nLINE\n")))
	require.True(t, bytes.HasSuffix(out.Bytes(), []byte("  0\nEOF\n")))
}

func TestVisibilityWorkersPreserveSegmentsAndDXFBytes(t *testing.T) {
	for _, notches := range []int{0, 16} {
		t.Run(fmt.Sprint(notches), func(t *testing.T) {
			body := visibilityBody(t, notches)
			serialCtx := orderedwork.WithWorkers(t.Context(), 1)
			serial := visibilityProject(t, serialCtx, body)
			require.NotEmpty(t, serial.Visible)
			require.NotEmpty(t, serial.Hidden)
			var serialDXF bytes.Buffer
			require.NoError(t, export.DXF(serialCtx, &serialDXF, body, units.Millimeters(0.1), visibilityView()))
			for _, workers := range []int{2, 4} {
				ctx := orderedwork.WithWorkers(t.Context(), workers)
				got := visibilityProject(t, ctx, body)
				for group, pair := range [][2][]drawing.Segment{{serial.Visible, got.Visible}, {serial.Hidden, got.Hidden}} {
					require.Equal(t, len(pair[0]), len(pair[1]))
					for i, segment := range pair[0] {
						require.True(t, segment == pair[1][i], "segment differs in group %d at index %d", group, i)
					}
				}
				var out bytes.Buffer
				require.NoError(t, export.DXF(ctx, &out, body, units.Millimeters(0.1), visibilityView()))
				require.True(t, bytes.Equal(serialDXF.Bytes(), out.Bytes()), "DXF bytes differ with %d workers", workers)
			}
		})
	}
}

func TestVisibilityCancellationLeavesNoOutput(t *testing.T) {
	body := visibilityBody(t, 0)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)
	ctx, cancel := context.WithCancel(orderedwork.WithWorkers(t.Context(), 4))
	cancel()
	view := visibilityView()
	lines, err := drawing.Project(ctx, mesh.Vertices(), mesh.Triangles(),
		raster.Camera{Direction: view.Direction, Up: view.Up})
	require.ErrorIs(t, err, context.Canceled)
	require.Empty(t, lines.Visible)
	require.Empty(t, lines.Hidden)
	var out bytes.Buffer
	require.ErrorIs(t, export.DXF(ctx, &out, body, units.Millimeters(0.1), view), context.Canceled)
	require.Zero(t, out.Len())
}

func TestVisibilityPerspectiveWorkersPreserveOrder(t *testing.T) {
	body := visibilityBody(t, 16)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)
	verts, tris := mesh.Vertices(), mesh.Triangles()
	view := visibilityView()
	cam := raster.Camera{Direction: view.Direction, Up: view.Up, FieldOfView: 1}
	serial, err := drawing.Project(orderedwork.WithWorkers(t.Context(), 1), verts, tris, cam)
	require.NoError(t, err)
	require.NotEmpty(t, serial.Visible)
	require.NotEmpty(t, serial.Hidden)
	for _, workers := range []int{2, 4} {
		got, err := drawing.Project(orderedwork.WithWorkers(t.Context(), workers), verts, tris, cam)
		require.NoError(t, err)
		for group, pair := range [][2][]drawing.Segment{{serial.Visible, got.Visible}, {serial.Hidden, got.Hidden}} {
			require.Equal(t, len(pair[0]), len(pair[1]))
			for i, segment := range pair[0] {
				require.True(t, segment == pair[1][i], "perspective segment differs in group %d at index %d", group, i)
			}
		}
	}
}

func BenchmarkDXFVisibility(b *testing.B) {
	// The fixture sizes bracket the feature-edge gate: a box and progressively
	// more notched profiles, rather than complete consumer models.
	for _, notches := range []int{0, 2, 4, 8, 10, 16} {
		for _, workers := range []int{1, 4} {
			b.Run(fmt.Sprintf("notches%d/w%d", notches, workers), func(b *testing.B) {
				body := visibilityBody(b, notches)
				ctx := orderedwork.WithWorkers(b.Context(), workers)
				lines := visibilityProject(b, ctx, body)
				b.Logf("notches=%d visible=%d hidden=%d", notches, len(lines.Visible), len(lines.Hidden))
				var out bytes.Buffer
				require.NoError(b, export.DXF(ctx, &out, body, units.Millimeters(0.1), visibilityView()))
				b.ReportAllocs()
				b.ResetTimer()
				for range b.N {
					out.Reset()
					if err := export.DXF(ctx, &out, body, units.Millimeters(0.1), visibilityView()); err != nil {
						b.Fatal(err)
					}
				}
			})
		}
	}
}
