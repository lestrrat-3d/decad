package capband_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/stretchr/testify/require"
)

// TestDenotedNormalAllow checks the term's three regimes: an exactly held band
// owes nothing, a larger displacement owes more, and a height the axial
// displacement can close bounds nothing.
//
// The covering case is the exact turn of a flat patch: a 1 mm high, 45° wall
// whose cap corner moves 1e-9 mm in the plane turns its normal by
// atan(1/(1 − 1e-9)) − π/4, about 5e-10, which the term must cover.
func TestDenotedNormalAllow(t *testing.T) {
	t.Parallel()
	require.Zero(t, capband.DenotedNormalAllow(0, 0, 0, 1, 1))

	small := capband.DenotedNormalAllow(1e-15, 0, 0, 10, 0)
	large := capband.DenotedNormalAllow(1e-12, 0, 0, 10, 0)
	require.Positive(t, small)
	require.Greater(t, large, small)
	require.GreaterOrEqual(t, small, 2e-16, `2·delta/h at the least`)

	const e = 1e-9
	turn := math.Atan(1/(1-e)) - math.Pi/4
	require.Positive(t, turn)
	require.LessOrEqual(t, turn, capband.DenotedNormalAllow(e, 0, 0, 1, 0))

	require.True(t, math.IsInf(capband.DenotedNormalAllow(1e-15, 6, 6, 10, 0), 1))
	require.True(t, math.IsInf(capband.DenotedNormalAllow(1e-15, 0, 0, 0, 0), 1))
}
