package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestChamferRetainedCylinderRim(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	b := mergeTranslated(t, internalBoxBody(t, doc, 0, 0, 10, 10, 10), 0.5, 0, 10)
	stock, err := Union(ctx, a, b)
	require.NoError(t, err)
	// The pad protrudes 0.2001 mm. The chamfer leaves 0.0001 mm of straight wall.
	frame, err := r3.NewFrame(r3.Vec{X: 4, Y: 9.2001, Z: 5}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(pl)
	require.NoError(t, err)
	s.AddConstraint(sketch.NewRadius(s.CreateCircle(s.Origin(), 2), 2))
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	boss, err := Extrude(ctx, doc, s, s.Profiles()[0], Distance{D: units.Millimeters(1), Dir: Against})
	require.NoError(t, err)
	joined, err := Union(ctx, stock, boss)
	require.NoError(t, err)
	finished, err := Chamfer(ctx, joined, Edges(OuterLoopOf(CapStart(boss))).Exactly(1), units.Millimeters(.2))
	require.NoError(t, err)
	vol, err := finished.Volume()
	require.NoError(t, err)
	want := 2000 + math.Pi*(2*2*.2001-.2*.2*(2-.2/3))
	require.LessOrEqual(t, math.Abs(vol.Value.Base()-want), vol.Bound.Base())
	// Check the realized setback independently of the volume allowance.
	payload := mergeFaceted(t, finished)
	ringRadius := func(y float64) float64 {
		var radius float64
		for _, v := range payload.Verts {
			if math.Abs(v.Y-y) < 1e-10 {
				radius = max(radius, math.Hypot(v.X-4, v.Z-5))
			}
		}
		return radius
	}
	require.InDelta(t, 1.8, ringRadius(10.2001), 1e-10)
	require.InDelta(t, 2.0, ringRadius(10.0001), .01)
}
