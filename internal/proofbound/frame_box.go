package proofbound

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// FrameBoxWorld maps Sketch's outward-rounded local curve bounds through
// the exact binary64 coefficients of its recorded frame. It does not use a
// rounded world point to decide which side of a cone contains the curve.
func FrameBoxWorld(frame r3.Frame, origin r3.Vec,
	lower, upper [2]float64) ([3]RatInterval, bool) {
	var world [3]RatInterval
	if !FiniteVec(r3.Vec{X: lower[0], Y: lower[1], Z: upper[0]}) ||
		!FiniteVec(r3.Vec{X: upper[1]}) ||
		lower[0] > upper[0] || lower[1] > upper[1] {
		return world, false
	}
	u := Interval(proofarith.FloatRat(lower[0]), proofarith.FloatRat(upper[0]))
	v := Interval(proofarith.FloatRat(lower[1]), proofarith.FloatRat(upper[1]))
	for i, axis := range [3][3]float64{
		{origin.X, frame.U().X, frame.V().X},
		{origin.Y, frame.U().Y, frame.V().Y},
		{origin.Z, frame.U().Z, frame.V().Z},
	} {
		world[i] = IntervalAdd(
			PointInterval(proofarith.FloatRat(axis[0])),
			IntervalAdd(
				IntervalScale(u, proofarith.FloatRat(axis[1])),
				IntervalScale(v, proofarith.FloatRat(axis[2]))))
	}
	return world, true
}

// BoxMidStation chooses a held mesh station inside a proven world box
// and returns a spatial allowance containing every exact point in that box.
func BoxMidStation(box [3]RatInterval) (r3.Vec, float64, bool) {
	var held [3]float64
	maxError := 0.0
	for i, interval := range box {
		if interval.Lo == nil || interval.Hi == nil || proofarith.CmpRat(interval.Lo, interval.Hi) > 0 {
			return r3.Vec{}, 0, false
		}
		mid := proofarith.QuoRat(new(big.Rat), proofarith.AddRat(new(big.Rat), interval.Lo, interval.Hi), big.NewRat(2, 1))
		held[i], _ = mid.Float64()
		if math.IsNaN(held[i]) || math.IsInf(held[i], 0) {
			return r3.Vec{}, 0, false
		}
		maxError = max(maxError, IntervalFloatError(interval, held[i]))
	}
	bound := Radius3D(maxError)
	return r3.Vec{X: held[0], Y: held[1], Z: held[2]},
		bound, !IsNonFinite(bound)
}
