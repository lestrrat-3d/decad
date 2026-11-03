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

// A shaft along Y (radius 2, y -75 to -42) takes a flat from a box tool whose
// sweep is perpendicular to the shaft's axis. The flat stops short of the far
// end, so the shaft's long wall fragment meets the flat along a shorter edge
// than the fragments that stay whole beside it. The tool starts `over` mm
// before the shaft's end face, or inside it when `over` is negative.
func TestCutFlatOnPerpendicularShaft(t *testing.T) {
	t.Parallel()
	const radius, length, flat, toolLen = 2.0, 33.0, 1.3, 7.0
	// The chord at x = flat removes a circular segment of this area.
	segment := radius*radius*math.Acos(flat/radius) - flat*math.Sqrt(radius*radius-flat*flat)
	for name, tc := range map[string]struct{ over, lift float64 }{
		"inside":       {over: -1},
		"flush":        {over: 0},
		"past the end": {over: 3},
		"lifted":       {over: 3, lift: 1.7945},
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			frame, err := r3.NewFrame(r3.Vec{X: -96.65835921350012, Y: -75, Z: 600}, r3.Vec{X: 1}, r3.Vec{Z: -1})
			require.NoError(t, err)
			doc := decad.New()
			w := sketch.NewWorld()
			plane, err := w.CreatePlaneFromFrame(frame)
			require.NoError(t, err)
			s, err := w.CreateSketch(plane)
			require.NoError(t, err)
			centre := s.CreatePoint(0, 0)
			s.Fix(centre)
			s.CreateCircle(centre, radius)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			shaft, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
				decad.Distance{D: units.Millimeters(length), Dir: decad.Along})
			require.NoError(t, err)

			w2 := sketch.NewWorld()
			plane2, err := w2.CreateOffsetPlane(w2.XY(), 597)
			require.NoError(t, err)
			s2, err := w2.CreateSketch(plane2)
			require.NoError(t, err)
			x := -96.65835921350012
			s2.CreateRectangle(x+flat, -75-tc.over, x+3, -75+toolLen)
			_, err = s2.Solve(t.Context())
			require.NoError(t, err)
			tool, err := decad.Extrude(t.Context(), doc, s2, s2.Profiles()[0],
				decad.Distance{D: units.Millimeters(6), Dir: decad.Along})
			require.NoError(t, err)

			if tc.lift != 0 {
				xf, err := r3.RotationAround(r3.Vec{X: -96, Y: -60, Z: 600}, r3.Vec{Y: 1}, units.Radians(tc.lift))
				require.NoError(t, err)
				shaft, err = decad.Transform(t.Context(), shaft, xf)
				require.NoError(t, err)
				tool, err = decad.Transform(t.Context(), tool, xf)
				require.NoError(t, err)
			}
			out, err := decad.Cut(t.Context(), shaft, tool)
			require.NoError(t, err)
			v, err := out.Volume()
			require.NoError(t, err)
			// The flat takes the circular segment over the part of the tool inside the shaft.
			inside := toolLen + min(tc.over, 0)
			want := math.Pi*radius*radius*length - segment*inside
			require.InDelta(t, want, v.Value.Base(), v.Bound.Base()+1e-6*want)
		})
	}
}
