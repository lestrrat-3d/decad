package export_test

import (
	"bytes"
	"context"
	"image/color"
	"image/png"
	"testing"

	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestPNGShadowWorkersPreserveEncodedBytes(t *testing.T) {
	bodies := shadowScene(t)
	for _, mode := range []struct {
		name        string
		soft        bool
		perspective bool
	}{
		{name: "hard"},
		{name: "soft", soft: true},
		{name: "perspective", soft: true, perspective: true},
	} {
		t.Run(mode.name, func(t *testing.T) {
			view := topDown(256)
			view.Shadows = true
			if mode.soft {
				view.ShadowSoftness = units.Degrees(5)
			}
			view.Colors = []color.RGBA{{R: 80, G: 170, B: 210, A: 255}, {R: 210, G: 130, B: 80, A: 255}}
			back := color.RGBA{R: 90, G: 130, B: 170, A: 255}
			view.BackColor = &back
			if mode.perspective {
				view.FieldOfView = units.Degrees(60)
			}
			var serial bytes.Buffer
			require.NoError(t, export.PNGBodies(orderedwork.WithWorkers(t.Context(), 1), &serial, bodies,
				units.Millimeters(0.1), view))
			decoded, err := png.Decode(bytes.NewReader(serial.Bytes()))
			require.NoError(t, err)
			require.Equal(t, 256, decoded.Bounds().Dx())
			require.Equal(t, 256, decoded.Bounds().Dy())
			for _, workers := range []int{2, 4} {
				var parallel bytes.Buffer
				require.NoError(t, export.PNGBodies(orderedwork.WithWorkers(t.Context(), workers), &parallel, bodies,
					units.Millimeters(0.1), view))
				require.True(t, bytes.Equal(serial.Bytes(), parallel.Bytes()),
					"PNG bytes differ for workers=%d perspective=%t", workers, mode.perspective)
			}
			ctx, cancel := context.WithCancel(orderedwork.WithWorkers(t.Context(), 4))
			cancel()
			var out bytes.Buffer
			require.ErrorIs(t, export.PNGBodies(ctx, &out, bodies, units.Millimeters(0.1), view), context.Canceled)
			require.Zero(t, out.Len())
		})
	}
}
