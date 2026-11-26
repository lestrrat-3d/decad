package opcommand_test

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/stretchr/testify/require"
)

func TestSlotLastSetWins(t *testing.T) {
	t.Parallel()
	var s opcommand.Slot[float64]
	require.False(t, s.IsSet())
	v, err := s.Get()
	require.NoError(t, err)
	require.Zero(t, v)

	bad := errors.New("bad value")
	s.Set(-1, bad)
	require.True(t, s.IsSet())
	v, err = s.Get()
	require.Same(t, bad, err)
	require.InDelta(t, -1, v, 0)

	// A valid second call clears the earlier call's error.
	s.Set(2, nil)
	v, err = s.Get()
	require.NoError(t, err)
	require.InDelta(t, 2, v, 0)
}
