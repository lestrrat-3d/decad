package compositesweep_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestCertifyRimPair(t *testing.T) {
	post := func(x, bound float64) *compositesweep.RimPost {
		return &compositesweep.RimPost{Position: r3.Vec{X: x, Y: 1, Z: 2}, Bound: bound}
	}
	from := [2]*compositesweep.RimPost{post(0, 1e-6), post(5, 1e-6)}

	t.Run("equal", func(t *testing.T) {
		require.NoError(t, compositesweep.CertifyRimPair(from, from))
	})
	t.Run("within bounds", func(t *testing.T) {
		to := [2]*compositesweep.RimPost{post(1.5e-6, 1e-6), post(5, 1e-6)}
		require.NoError(t, compositesweep.CertifyRimPair(from, to))
	})
	t.Run("beyond bounds on start", func(t *testing.T) {
		to := [2]*compositesweep.RimPost{post(3e-6, 1e-6), post(5, 1e-6)}
		err := compositesweep.CertifyRimPair(from, to)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "composite chain source rims disagree beyond their coordinate bounds")
	})
	t.Run("beyond bounds on end", func(t *testing.T) {
		to := [2]*compositesweep.RimPost{post(0, 1e-6), post(5.1, 1e-6)}
		require.ErrorContains(t, compositesweep.CertifyRimPair(from, to),
			"composite chain source rims disagree beyond their coordinate bounds")
	})
	t.Run("missing post", func(t *testing.T) {
		to := [2]*compositesweep.RimPost{post(0, 1e-6), nil}
		err := compositesweep.CertifyRimPair(from, to)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "a composite chain join has a missing source post")
		require.Error(t, compositesweep.CertifyRimPair([2]*compositesweep.RimPost{nil, from[1]}, from))
	})
}
