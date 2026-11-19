package meshbool

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

func TestRoundedCoverBoxRejectStrictFiniteOnly(t *testing.T) {
	triangle := [3]proof.Xpt{
		proof.XptOf(r3.Vec{}),
		proof.XptOf(r3.Vec{X: 2}),
		proof.XptOf(r3.Vec{Y: 2}),
	}
	bounds := roundedCoverBoundsFor(triangle)
	require.True(t, bounds.valid)

	cases := []struct {
		name   string
		point  proof.Xpt
		reject bool
	}{
		{name: "interior", point: proof.XptOf(r3.Vec{X: 0.5, Y: 0.5})},
		{name: "diagonal boundary", point: proof.XptOf(r3.Vec{X: 1, Y: 1})},
		{name: "vertex", point: triangle[0]},
		{name: "other vertex", point: triangle[1]},
		{name: "strictly outside x", point: proof.XptOf(r3.Vec{X: -1}), reject: true},
		{name: "strictly outside z", point: proof.XptOf(r3.Vec{Z: 1}), reject: true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			probe, valid := roundedProbeCoords(tc.point)
			require.True(t, valid)
			require.Equal(t, tc.reject, roundedCoverBoxReject(probe, valid, bounds))
		})
	}
}

func TestRoundedCoverBoxRejectFallsThroughForNonfiniteValues(t *testing.T) {
	triangle := [3]proof.Xpt{
		proof.XptOf(r3.Vec{}),
		proof.XptOf(r3.Vec{X: 2}),
		proof.XptOf(r3.Vec{Y: 2}),
	}
	bounds := roundedCoverBoundsFor(triangle)
	huge := proof.Xpt{
		X: new(big.Int).Lsh(big.NewInt(1), 1100),
		Y: big.NewInt(0),
		Z: big.NewInt(0),
		W: big.NewInt(1),
	}
	_, probeValid := roundedProbeCoords(huge)
	require.False(t, probeValid)
	require.False(t, roundedCoverBoxReject([3]float64{}, probeValid, bounds))

	triangle[0] = huge
	require.False(t, roundedCoverBoundsFor(triangle).valid)
}
