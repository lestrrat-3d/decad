package gaptol_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/stretchr/testify/require"
)

func TestReach(t *testing.T) {
	t.Parallel()
	require.Zero(t, gaptol.Reach(t.Context()), "a context no root operation touched is exact only")
	require.InDelta(t, 0.02, gaptol.Reach(gaptol.With(t.Context(), 0.02)), 0)
	inner := gaptol.With(gaptol.With(t.Context(), 0.02), 0)
	require.Zero(t, gaptol.Reach(inner), "the innermost reach wins, and zero is exact only")
	require.InDelta(t, 0.01, gaptol.Default, 0)
}
