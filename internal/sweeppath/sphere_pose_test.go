package sweeppath_test

import (
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSpherePoseAt(t *testing.T) {
	t.Parallel()

	delta := [3]proofarith.Dyadic{proofarith.MustDyOf(4), proofarith.MustDyOf(2), proofarith.MustDyOf(-6)}
	half := big.NewRat(1, 2)

	t.Run("straight path takes the plain pose", func(t *testing.T) {
		t.Parallel()
		path := sweeppath.AffinePath{From: r3.Identity(), Delta: delta, Duration: big.NewRat(1, 1)}
		got, err := path.SpherePoseAt(half)
		require.NoError(t, err)
		require.Equal(t, r3.Vec{X: 2, Y: 1, Z: -3}, got.Translation())
	})

	t.Run("drifting path keeps the rounded turn and moves the centre exactly", func(t *testing.T) {
		t.Parallel()
		drift := &sweeppath.RigidDriftSegment{From: r3.Identity(), Center: r3.Vec{X: 5},
			LinearVelocity: sweeppath.QuantityVec{X: units.MillimetersPerSecond(4),
				Y: units.MillimetersPerSecond(2), Z: units.MillimetersPerSecond(-6)},
			AngularVelocity: sweeppath.QuantityVec{X: units.RadiansPerSecond(0),
				Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(1)},
			Duration: units.Seconds(1)}
		start, err := r3.Translation(r3.Vec{X: 10, Y: 20, Z: 30})
		require.NoError(t, err)
		path := sweeppath.AffinePath{From: start, Delta: delta, Duration: big.NewRat(1, 1), Drift: drift}

		rounded, err := path.RoundedPoseAt(half)
		require.NoError(t, err)
		got, err := path.SpherePoseAt(half)
		require.NoError(t, err)
		require.Equal(t, rounded.Basis(), got.Basis(), "the rotation is the rounded pose's")
		require.Equal(t, r3.Vec{X: 12, Y: 21, Z: 27}, got.Translation(),
			"the centre is the start plus the exact delta at the fraction")
		require.NotEqual(t, rounded.Translation(), got.Translation(), "the rounded pose turns about its own centre")

		atStart, err := path.SpherePoseAt(new(big.Rat))
		require.NoError(t, err)
		require.Equal(t, start, atStart, "fraction zero is the start pose")
	})
}
