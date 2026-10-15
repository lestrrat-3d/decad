package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestTessellateRefusalNamesChainRevolvePayload pins the payload class the
// unsupported-payload refusal prints for a revolved open chain.
func TestTessellateRefusalNamesChainRevolvePayload(t *testing.T) {
	t.Parallel()
	s, ch, axis := coneChainAtRadius(t, 0)
	body, err := decad.Revolve(t.Context(), decad.New(), s, ch, axis, decad.FullRevolution{})
	require.NoError(t, err)
	_, err = body.Tessellate(t.Context(), units.Millimeters(0.5))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "tessellation does not support payload decad.chainRevolvePayload;")
}

// TestTessellateRefusalNamesChainLoftPayload pins the payload class the
// unsupported-payload refusal prints for a lofted pair of open chains.
func TestTessellateRefusalNamesChainLoftPayload(t *testing.T) {
	t.Parallel()
	s0, ch0, s1, ch1 := chainLoftPair(t, 10,
		[][2]float64{{0, 0}, {40, 0}},
		[][2]float64{{0, 0}, {40, 0}},
	)
	body, err := decad.New().LoftChain(t.Context(), s0, ch0, s1, ch1)
	require.NoError(t, err)
	_, err = body.Tessellate(t.Context(), units.Millimeters(0.5))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "tessellation does not support payload decad.chainLoftPayload;")
}
