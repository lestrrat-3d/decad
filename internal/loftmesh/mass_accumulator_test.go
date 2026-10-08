package loftmesh_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/stretchr/testify/require"
)

// TestCentroidClearanceIsExact pins Centroid's clearance as |vol| − allow over
// exact rationals, rounded down once. The volume 1/10 rounds UP to its
// float64, and allow sits one float below that, inside the volume's own
// half-ulp: the exact gap is about 8.3e-18 while the float difference is a
// whole ulp of 0.1, about 1.39e-17.
//
// Shown to fail: the retired form nextafter(|volValue| − allow, −Inf), which
// subtracts from the ROUNDED volume, overstates the gap here by about 1.7x;
// the assertion on it below records that, and substituting it for
// CentroidClearance fails the enclosure assertion.
func TestCentroidClearanceIsExact(t *testing.T) {
	t.Parallel()
	vol := big.NewRat(1, 10)
	volValue, _ := vol.Float64()
	require.Positive(t, new(big.Rat).SetFloat64(volValue).Cmp(vol), "the fixture needs a volume that rounds up")
	allow := math.Nextafter(volValue, 0)
	gap := new(big.Rat).Sub(vol, new(big.Rat).SetFloat64(allow))
	require.Positive(t, gap.Sign(), "the fixture needs allow strictly below the exact volume")

	retired := math.Nextafter(math.Abs(volValue)-allow, math.Inf(-1))
	require.Positive(t, new(big.Rat).SetFloat64(retired).Cmp(gap), "the retired form overstates the clearance")

	for _, v := range []*big.Rat{vol, new(big.Rat).Neg(vol)} {
		got := loftmesh.CentroidClearance(v, allow)
		require.Positive(t, got)
		require.LessOrEqual(t, new(big.Rat).SetFloat64(got).Cmp(gap), 0, "the clearance must not exceed the exact gap")
		next := new(big.Rat).SetFloat64(math.Nextafter(got, math.Inf(1)))
		require.Positive(t, next.Cmp(gap), "the clearance must be the largest float at or below the exact gap")
	}

	huge := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 1100))
	require.Equal(t, math.MaxFloat64, loftmesh.CentroidClearance(huge, 1),
		"a gap past float64's range answers the largest finite float, never a panic")

	require.Zero(t, loftmesh.CentroidClearance(vol, volValue), "an allowance at or above the volume leaves no clearance")
	require.Zero(t, loftmesh.CentroidClearance(vol, math.NaN()), "a NaN allowance states no bound")
	require.Zero(t, loftmesh.CentroidClearance(vol, math.Inf(1)), "an infinite allowance states no bound")
}
