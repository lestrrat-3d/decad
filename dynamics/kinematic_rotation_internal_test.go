package dynamics

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The driver turns 90° about the x axis through (5, 5) while it advances
// 20 mm in 1 s. Its velocity field moves the point (10, 5, 5) on the axis
// at the slide alone, (20, 0, 0) mm/s, and a point 0.01 mm off the axis
// also at (π/2)·0.01 mm/s across it: the island reads the field at each
// contact point, not one velocity for the whole driver.
func TestDriverMotionReadsFieldOffTheAxis(t *testing.T) {
	t.Parallel()
	turn, err := r3.RotationAround(r3.Vec{X: 5, Y: 5, Z: 5}, r3.Vec{X: 1}, units.Degrees(90))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{X: 20})
	require.NoError(t, err)
	end, err := turn.Then(shift)
	require.NoError(t, err)
	motion, ok := driverMotionOf(decad.PoseSegment{From: r3.Identity(), To: end, Duration: units.Seconds(1)})
	require.True(t, ok)
	require.True(t, motion.rotates())
	float := func(x [3]*big.Rat) r3.Vec {
		v, _ := x[0].Float64()
		w, _ := x[1].Float64()
		z, _ := x[2].Float64()
		return r3.Vec{X: v, Y: w, Z: z}
	}
	point := func(v r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{ratFloat(v.X), ratFloat(v.Y), ratFloat(v.Z)}
	}
	onAxis := float(motion.at(point(r3.Vec{X: 10, Y: 5, Z: 5})))
	require.InDelta(t, 20, onAxis.X, 1e-12)
	require.InDelta(t, 0, onAxis.Y, 1e-12)
	require.InDelta(t, 0, onAxis.Z, 1e-12)
	offAxis := float(motion.at(point(r3.Vec{X: 10, Y: 5.01, Z: 5})))
	require.InDelta(t, 20, offAxis.X, 1e-12)
	require.InDelta(t, 0, offAxis.Y, 1e-12)
	require.InDelta(t, math.Pi/2*.01, offAxis.Z, 1e-12)
}
