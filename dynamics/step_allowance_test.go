package dynamics

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This proof gate is private, so the regression exercises it inside the package.
func TestCorrectionAllowanceUsesElapsedBounds(t *testing.T) {
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
}
