package decad

// CI: isolated-tests

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func privateCaptureBox(t *testing.T, ctx context.Context, d *Document, x0, size, height float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, 0, x0+size, size)
	s.Fix(rect.A)
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	b, err := Extrude(ctx, d, s, s.Profiles()[0], Distance{D: units.Millimeters(height), Dir: Along})
	require.NoError(t, err)
	return b
}

// Under one private construction a temporary tool built with Extrude and a
// Fillet on a live body must take distinct producer ids: the Fillet reads the
// counter through the capture, past the tool's identity.
func TestPrivateConstructionFilletTakesDistinctProducer(t *testing.T) {
	t.Parallel()
	d := New()
	live := privateCaptureBox(t, t.Context(), d, 0, 20, 10)
	ctx, capture := privateConstructionContext(t.Context())
	tool := privateCaptureBox(t, ctx, d, 100, 10, 5)
	rounded, err := Fillet(live, Edges(ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(2)).Do(ctx)
	require.NoError(t, err)
	require.NotEqual(t, tool.n.Origin.Producer, rounded.n.Origin.Producer)
	require.Equal(t, capture.base+1, rounded.n.Origin.Producer)
}

// The same hazard through Chamfer.
func TestPrivateConstructionChamferTakesDistinctProducer(t *testing.T) {
	t.Parallel()
	d := New()
	live := privateCaptureBox(t, t.Context(), d, 0, 20, 10)
	ctx, capture := privateConstructionContext(t.Context())
	tool := privateCaptureBox(t, ctx, d, 100, 10, 5)
	cut, err := Chamfer(live, Edges(ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(2)).Do(ctx)
	require.NoError(t, err)
	require.NotEqual(t, tool.n.Origin.Producer, cut.n.Origin.Producer)
	require.Equal(t, capture.base+1, cut.n.Origin.Producer)
}
