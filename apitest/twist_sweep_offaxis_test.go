package apitest_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// The screw gear's bore section is not centred on the twist axis: it spans
// u in [-7.7, 7.7] and v in [-2.075, 2.675], and it turns about the origin.
const (
	offAxisHalfU = 7.7
	offAxisLoV   = -2.075
	offAxisHiV   = 2.675
)

func offAxisTwistTool(t *testing.T, doc *decad.Document, length, deg float64) (*decad.Body, error) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(-offAxisHalfU, offAxisLoV, offAxisHalfU, offAxisHiV)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, length)})
	require.NoError(t, err)
	return decad.Sweep(t.Context(), doc, s, s.Profiles()[0], path, decad.WithSweepTwist(units.Degrees(deg)))
}

func TestSweepTwistOffAxisAngles(t *testing.T) {
	t.Parallel()
	const length = 8.0
	area := 2 * offAxisHalfU * (offAxisHiV - offAxisLoV)
	angles := []float64{58.2}
	for a := 2.5; a <= 360; a += 22.5 {
		angles = append(angles, a)
	}
	for _, deg := range angles {
		t.Run(fmt.Sprintf("%gdeg", deg), func(t *testing.T) {
			t.Parallel()
			body, err := offAxisTwistTool(t, decad.New(), length, deg)
			require.NoError(t, err)
			volume, err := body.Volume()
			require.NoError(t, err)
			require.InDelta(t, area*length, volume.Value.Base(), 1e-6*area*length+volume.Bound.Base())

			// The centroid is the section centroid (0, 0.3) averaged over
			// the turn, integrated here by the midpoint rule.
			cx, cy := 0.0, 0.0
			const n = 20000
			cv := (offAxisLoV + offAxisHiV) / 2
			for k := range n {
				sin, cos := math.Sincos(deg * math.Pi / 180 * (float64(k) + 0.5) / n)
				cx -= cv * sin / n
				cy += cv * cos / n
			}
			centroid, err := body.Centroid()
			require.NoError(t, err)
			bound := centroid.Bound.Base()
			require.Less(t, bound, 1e-6)
			require.InDelta(t, cx, centroid.Value.X, bound+1e-6)
			require.InDelta(t, cy, centroid.Value.Y, bound+1e-6)
			require.InDelta(t, length/2, centroid.Value.Z, bound+1e-6)

			mesh, err := body.Tessellate(t.Context(), units.Millimeters(5), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}

// offAxisBoreOverlap integrates, independently of the evaluator, the volume
// the placed tool shares with the tube wall. The tool sweeps radial
// x in [11.5, 11.5+length] after a 90 degree turn about Y, centred at
// z = mid on the tube.
func offAxisBoreOverlap(length, deg, radialStart, mid, height, inner, outer float64) float64 {
	const nu, nv, nw = 308, 190, 160
	du, dv, dw := 2*offAxisHalfU/nu, (offAxisHiV-offAxisLoV)/nv, length/nw
	count := 0
	for k := range nw {
		w := (float64(k) + 0.5) * dw
		sin, cos := math.Sincos(deg * math.Pi / 180 * w / length)
		for i := range nu {
			u := -offAxisHalfU + (float64(i)+0.5)*du
			for j := range nv {
				v := offAxisLoV + (float64(j)+0.5)*dv
				lx, ly := u*cos-v*sin, u*sin+v*cos
				x, z := radialStart+w, mid-lx
				if r2 := x*x + ly*ly; r2 >= inner*inner && r2 <= outer*outer && z >= 0 && z <= height {
					count++
				}
			}
		}
	}
	return float64(count) * du * dv * dw
}

// The bore: the off-axis tool runs radially through the tube wall.
func TestCutTwistedOffAxisSweepThroughTubeWall(t *testing.T) {
	t.Parallel()
	const outer, inner, height, length, deg = 18.5, 12.5, 37.5, 8.0, 58.2
	const radialStart, mid = 11.5, height/2 + 6.9
	setup := func() (*decad.Body, *decad.Body) {
		doc := decad.New()
		tube := twistBoreTube(t, doc, outer, inner, height)
		tool, err := offAxisTwistTool(t, doc, length, deg)
		require.NoError(t, err)
		turn, err := r3.Rotation(r3.NewVec(0, 1, 0), units.Degrees(90))
		require.NoError(t, err)
		tool, err = decad.Transform(t.Context(), tool, turn)
		require.NoError(t, err)
		move, err := r3.Translation(r3.NewVec(radialStart, 0, mid))
		require.NoError(t, err)
		tool, err = decad.Transform(t.Context(), tool, move)
		require.NoError(t, err)
		return tube, tool
	}
	overlap := offAxisBoreOverlap(length, deg, radialStart, mid, height, inner, outer)
	require.Greater(t, overlap, 100.0)

	tube, tool := setup()
	tubeVolume, err := tube.Volume()
	require.NoError(t, err)
	out, err := decad.Cut(t.Context(), tube, tool)
	require.NoError(t, err)
	require.True(t, out.IsSolid())
	volume, err := out.Volume()
	require.NoError(t, err)
	slack := volume.Bound.Base() + tubeVolume.Bound.Base() + 0.01*overlap
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-(tubeVolume.Value.Base()-overlap)), slack,
		"%v want %v (overlap %v)", volume, tubeVolume.Value.Base()-overlap, overlap)
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(3), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}
