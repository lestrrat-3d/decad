package raster_test

import (
	"context"
	"fmt"
	"image"
	"image/color"
	"math"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/raster"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func shadowRowsScene(tb testing.TB) ([]r3.Vec, [][3]int, []color.RGBA) {
	tb.Helper()
	doc := decad.New()
	bodies := []*decad.Body{
		decadtest.NewBlock(tb, doc, 0, 0, 60, 40, units.Millimeters(2)),
		decadtest.NewBlock(tb, doc, 25, 15, 35, 25, units.Millimeters(20)),
	}
	var vertices []r3.Vec
	var triangles [][3]int
	var tints []color.RGBA
	colors := []color.RGBA{{R: 80, G: 170, B: 210, A: 255}, {R: 210, G: 130, B: 80, A: 255}}
	for i, body := range bodies {
		mesh, err := body.Tessellate(tb.Context(), units.Millimeters(0.1))
		require.NoError(tb, err)
		offset := len(vertices)
		vertices = append(vertices, mesh.Vertices()...)
		for _, tri := range mesh.Triangles() {
			triangles = append(triangles, [3]int{tri[0] + offset, tri[1] + offset, tri[2] + offset})
			tints = append(tints, colors[i])
		}
	}
	return vertices, triangles, tints
}

func requireShadowPixels(t *testing.T, want, got *image.RGBA) {
	t.Helper()
	require.Equal(t, want.Rect, got.Rect)
	for y := range want.Rect.Dy() {
		for x := range want.Rect.Dx() {
			if want.RGBAAt(x, y) != got.RGBAAt(x, y) {
				t.Fatalf("shadow image differs at (%d,%d): want %v, got %v", x, y, want.RGBAAt(x, y), got.RGBAAt(x, y))
			}
		}
	}
}

func TestShadowRowsKeepPixelsAcrossWorkers(t *testing.T) {
	vertices, triangles, tints := shadowRowsScene(t)
	for _, mode := range []struct {
		name        string
		soft        bool
		perspective bool
		backfaces   bool
	}{
		{name: "hard"},
		{name: "soft", soft: true},
		{name: "perspective-backfaces", soft: true, perspective: true, backfaces: true},
	} {
		t.Run(mode.name, func(t *testing.T) {
			tris := append([][3]int(nil), triangles...)
			if mode.backfaces {
				for i, tri := range tris {
					tris[i] = [3]int{tri[0], tri[2], tri[1]}
				}
			}
			cam := raster.Camera{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0), Gloss: 0.5}
			if mode.perspective {
				cam.FieldOfView = math.Pi / 3
			}
			spread, samples := 0.0, 1
			if mode.soft {
				spread, samples = math.Tan(5*math.Pi/180), 32
			}
			back := color.RGBA{R: 90, G: 130, B: 170, A: 255}
			serial, err := raster.RenderSoftShadowed(orderedwork.WithWorkers(t.Context(), 1),
				vertices, tris, tints, back, cam, 256, 256, 512, spread, samples)
			require.NoError(t, err)
			for _, workers := range []int{2, 4} {
				parallel, err := raster.RenderSoftShadowed(orderedwork.WithWorkers(t.Context(), workers),
					vertices, tris, tints, back, cam, 256, 256, 512, spread, samples)
				require.NoError(t, err)
				requireShadowPixels(t, serial, parallel)
			}
		})
	}
}

func TestShadowRowsCancellationReturnsNoImage(t *testing.T) {
	vertices, triangles, tints := shadowRowsScene(t)
	ctx, cancel := context.WithCancel(orderedwork.WithWorkers(t.Context(), 4))
	cancel()
	cam := raster.Camera{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0)}
	img, err := raster.RenderSoftShadowed(ctx, vertices, triangles, tints, raster.Back, cam,
		256, 256, 512, math.Tan(5*math.Pi/180), 32)
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, img == nil, "canceled renderer returned an image")
}

func TestShadowRowsCancellationDuringRenderReturnsNoImage(t *testing.T) {
	vertices, triangles, tints := shadowRowsScene(t)
	ctx, cancel := context.WithCancel(orderedwork.WithWorkers(t.Context(), 4))
	defer cancel()
	require.NoError(t, ctx.Err())
	timer := time.AfterFunc(time.Millisecond, cancel)
	defer timer.Stop()
	cam := raster.Camera{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0)}
	img, err := raster.RenderSoftShadowed(ctx, vertices, triangles, tints, raster.Back, cam,
		256, 256, 512, math.Tan(5*math.Pi/180), 32)
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, img == nil, "canceled renderer returned an image")
}

func BenchmarkShadowRowShading(b *testing.B) {
	for _, size := range []int{64, 128, 256, 512} {
		for _, soft := range []bool{false, true} {
			name := "hard"
			spread, samples := 0.0, 1
			if soft {
				name, spread, samples = "soft", math.Tan(5*math.Pi/180), 32
			}
			for _, workers := range []int{1, 4} {
				b.Run(fmt.Sprintf("%s/%d/w%d", name, size, workers), func(b *testing.B) {
					vertices, triangles, tints := shadowRowsScene(b)
					ctx := orderedwork.WithWorkers(b.Context(), workers)
					cam := raster.Camera{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0)}
					img, err := raster.RenderSoftShadowed(ctx, vertices, triangles, tints, raster.Back, cam,
						size, size, max(2*size, 512), spread, samples)
					require.NoError(b, err)
					require.Equal(b, image.Rect(0, 0, size, size), img.Rect)
					b.ReportAllocs()
					b.ResetTimer()
					for range b.N {
						if _, err := raster.RenderSoftShadowed(ctx, vertices, triangles, tints, raster.Back, cam,
							size, size, max(2*size, 512), spread, samples); err != nil {
							b.Fatal(err)
						}
					}
				})
			}
		}
	}
}
