package revolveangle

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestHalfTurnExcessUsesStatedAngles(t *testing.T) {
	t.Parallel()
	angle := func(value units.Value) (float64, Angle) {
		held, err := value.In(units.Radian)
		require.NoError(t, err)
		return held, FromValue(value)
	}
	for _, tc := range []struct {
		name string
		deg  float64
		sign int
	}{
		{name: "exact half turn", deg: 180},
		{name: "below half turn", deg: 179.99, sign: -1},
		{name: "above half turn", deg: 180.01, sign: 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			held, stated := angle(units.Degrees(tc.deg))
			excess, ok := (Sweep{Phi0: Zero(), Phi1: stated}).HalfTurnExcessFor(0, held)
			require.True(t, ok)
			if tc.sign < 0 {
				require.Negative(t, excess.Hi.Sign())
				return
			}
			if tc.sign > 0 {
				require.Positive(t, excess.Lo.Sign())
				return
			}
			require.Zero(t, excess.Lo.Sign())
			require.Zero(t, excess.Hi.Sign())
		})
	}
}

func TestHalfTurnExcessFallsBackToHeldAngles(t *testing.T) {
	t.Parallel()
	excess, ok := (Sweep{}).HalfTurnExcessFor(0, 3.2)
	require.True(t, ok)
	require.Positive(t, excess.Lo.Sign())

	excess, ok = (Sweep{}).HalfTurnExcessFor(0, 3.1)
	require.True(t, ok)
	require.Negative(t, excess.Hi.Sign())

	_, ok = (Sweep{}).HalfTurnExcessFor(0, math.NaN())
	require.False(t, ok)
}
