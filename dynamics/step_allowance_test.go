package dynamics

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This proof gate is private, so the regression exercises it inside the package.
func TestCorrectionAllowanceUsesElapsedBoundsAndActualPose(t *testing.T) {
	// The published elapsed centers coincide, yet their certified time balls
	// permit a nonzero bracket. A difference of the rounded centers would be zero.
	bracket := decad.SweepInterval{
		From: decad.SweepInstant{Elapsed: decad.Measurement{
			Value: units.Seconds(0.1), Bound: units.Seconds(1e-12),
		}},
		To: decad.SweepInstant{Elapsed: decad.Measurement{
			Value: units.Seconds(0.1), Bound: units.Seconds(1e-12),
		}},
	}
	travel, ok := boundBracketTravel(bracket, 100)
	require.True(t, ok)
	require.GreaterOrEqual(t, travel, 2e-10)

	before := r3.Identity()
	after, err := r3.Translation(r3.Vec{Z: 0.5})
	require.NoError(t, err)
	require.False(t, correctionWithin(before, after, 0.49))
	require.True(t, correctionWithin(before, after, 0.5000000000000002))
}

func TestImpulseResidualIncludesEachRoundedOperation(t *testing.T) {
	e := units.Scalar(math.Float64frombits(0x3feefe3d0913e87c))
	velocity := units.MillimetersPerSecond(math.Float64frombits(0xc0b621f00706a76b))
	mass := decad.Measurement{
		Value: units.Kilograms(2.6875), Bound: units.Kilograms(0),
	}
	// Hold the published float outputs fixed. The host may round the
	// intermediate operations differently, while the exact input law and its
	// error against these outputs must have the same answer everywhere.
	target := math.Float64frombits(0x40b56fa89ec6df9e)
	impulse := math.Float64frombits(0x40dd45d28f6616b9)
	ulp := math.Nextafter(impulse, math.Inf(1)) - impulse

	// The exact response differs from this nominal impulse by more than one ULP.
	// Charging only the final multiplication would admit the tight limit.
	require.False(t, responseResidualsWithin(velocity, 1, e, mass, target, impulse,
		units.MillimetersPerSecond(1e-8), units.KilogramMillimetersPerSecond(1.1*ulp)))
	require.True(t, responseResidualsWithin(velocity, 1, e, mass, target, impulse,
		units.MillimetersPerSecond(1e-8), units.KilogramMillimetersPerSecond(2*ulp)))
}
