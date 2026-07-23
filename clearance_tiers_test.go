package decad_test

import (
	"context"
	"math"
	"runtime"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type vertexTierCancelContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	entered         bool
}

func (c *vertexTierCancelContext) Err() error {
	pcs := make([]uintptr, 32)
	frames := runtime.CallersFrames(pcs[:runtime.Callers(2, pcs)])
	for {
		frame, more := frames.Next()
		if strings.HasSuffix(frame.Function, ".vertexTier") {
			c.entered = true
			return context.Canceled
		}
		if !more {
			return nil
		}
	}
}

func TestVerifyClearanceCancellationInsideVertexTier(t *testing.T) {
	const sides = 16
	polygon := func(cx float64) [][2]float64 {
		pts := make([][2]float64, sides)
		for i := range pts {
			th := 2 * math.Pi * float64(i) / sides
			pts[i] = [2]float64{cx + 10*math.Cos(th), 10 * math.Sin(th)}
		}
		return pts
	}
	doc := decad.New()
	for _, cx := range []float64{0, 50} {
		s, p := polygonSketch(t, polygon(cx))
		_, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
		require.NoError(t, err)
	}
	before := snapshotDocument(t, doc)
	ctx := &vertexTierCancelContext{Context: t.Context()}

	report, err := doc.Verify(ctx, decad.WithClearances())

	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, report)
	require.True(t, ctx.entered)
	requireDocumentUnchanged(t, doc, before)
}
