package revolveproof_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/stretchr/testify/require"
)

func ratInt(v int64) *big.Rat { return big.NewRat(v, 1) }

func requireInterval(t *testing.T, got proofbound.RatInterval, lo, hi *big.Rat) {
	t.Helper()
	require.Zero(t, got.Lo.Cmp(lo), "lo %v want %v", got.Lo, lo)
	require.Zero(t, got.Hi.Cmp(hi), "hi %v want %v", got.Hi, hi)
}

func TestChainArcSourcePoint(t *testing.T) {
	z, rho, ok := revolveproof.ChainArcSourcePoint(revolveaxis.Frame{DU: 1}, 2, 1, proofbound.WalkEndBound{})
	require.True(t, ok)
	requireInterval(t, z, ratInt(2), ratInt(2))
	requireInterval(t, rho, ratInt(1), ratInt(1))

	// The anchor bound AUBound widens the u offset du = u - AU to [3/2, 5/2].
	z, rho, ok = revolveproof.ChainArcSourcePoint(revolveaxis.Frame{DU: 1, AUBound: 0.5}, 2, 1, proofbound.WalkEndBound{})
	require.True(t, ok)
	requireInterval(t, z, big.NewRat(3, 2), big.NewRat(5, 2))
	requireInterval(t, rho, ratInt(1), ratInt(1))
}

func TestChainArcAreaDensityAllow(t *testing.T) {
	point := func(n, d int64) proofbound.RatInterval { return proofbound.PointInterval(big.NewRat(n, d)) }
	allow := func(heldStep proofbound.RatInterval, n int) (float64, error) {
		return revolveproof.ChainArcAreaDensityAllow(
			point(0, 1), point(2, 1), point(1, 1), point(2, 1),
			0, 2, 1, 2, point(1, 1), heldStep, n)
	}

	got, err := allow(point(1, 2), 2)
	require.NoError(t, err)
	require.Equal(t, 0.0, got)

	// A held step 1/4 short of the true 1/2 step costs 1 * 1/4 * rho(2).
	got, err = allow(point(1, 4), 2)
	require.NoError(t, err)
	require.GreaterOrEqual(t, got, 0.5)
	require.InDelta(t, 0.5, got, 1e-12)

	_, err = allow(point(1, 2), 0)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "no enclosed area density")
}
