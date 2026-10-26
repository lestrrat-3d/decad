package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// halfTurnPairSweep sweeps the 2 x 0.5 rectangle [x0,x0+2] x [0,0.5] on XY
// along two half turns in the same sense about axes parallel to Y: the first
// from the origin over (5,0,5) to (10,0,0), the second from there under the
// X axis to (end,0,0). All four section caps lie in z = 0. The two turns map
// x to 10-x and then to (10+end)-x, so the end cap covers
// [end+x0, end+x0+2] x [0,0.5]. With x0 = 0 the profile lies between the
// path and the first axis; with x0 = 6 it lies beyond both axes, so each arc
// sweeps against the path's own turning direction.
func halfTurnPairSweep(t *testing.T, x0, end float64) (*decad.Document, *decad.Body, error) {
	t.Helper()

	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, 0, x0+2, 0.5)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	center := (10 + end) / 2
	path, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.ArcThrough{Through: r3.NewVec(5, 0, 5), End: r3.NewVec(10, 0, 0)},
		decad.ArcThrough{Through: r3.NewVec(center, 0, -(10 - center)), End: r3.NewVec(end, 0, 0)},
	)
	require.NoError(t, err)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, s.Profiles()[0], path)
	return doc, body, err
}

func TestSweepCompositeHalfTurnsRefuseMeetingEndCaps(t *testing.T) {
	t.Parallel()

	for _, tc := range []struct {
		name string
		x0   float64
		end  float64
	}{
		// The end cap [1,3] x [0,0.5] overlaps the start cap over [1,2] x [0,0.5].
		{name: "overlap", x0: 0, end: 1},
		// The end cap [2,4] x [0,0.5] meets the start cap along the edge x = 2.
		{name: "touch", x0: 0, end: 2},
		// The end cap [7,9] x [0,0.5] overlaps the start cap over [7,8] x [0,0.5].
		{name: "overlap beyond the axes", x0: 6, end: 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, body, err := halfTurnPairSweep(t, tc.x0, tc.end)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.ErrorContains(t, err,
				"sweep spans 0 and 1 both keep their far sections in their shared plane")
			require.Nil(t, body)
			require.Empty(t, doc.Bodies())
		})
	}
}

func TestSweepCompositeHalfTurnsSeparatedEndCapsAdmit(t *testing.T) {
	t.Parallel()

	// The end cap [end+x0, end+x0+2] x [0,0.5] leaves a gap of 1 to the start
	// cap. A half turn of the rectangle over radii [r0,r1] sweeps
	// 0.5 * (pi/2) * (r1^2 - r0^2).
	for _, tc := range []struct {
		name   string
		x0     float64
		volume float64
	}{
		// Radii [3,5] about x = 5, then [1.5,3.5] about x = 6.5.
		{name: "between path and axes", x0: 0, volume: 4*math.Pi + 2.5*math.Pi},
		// Radii [1,3] about x = 5, then [2.5,4.5] about x = 6.5.
		{name: "beyond the axes", x0: 6, volume: 2*math.Pi + 3.5*math.Pi},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, body, err := halfTurnPairSweep(t, tc.x0, 3)
			require.NoError(t, err)
			require.True(t, body.IsSolid())
			require.Len(t, body.Faces(), 10)
			decadtest.MeasuresVolume(t, body, units.CubicMillimeters(tc.volume))
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Equal(t, decad.Sound, report.Status)
		})
	}
}
