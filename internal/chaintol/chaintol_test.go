package chaintol_test

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/chaintol"
)

func TestOf(t *testing.T) {
	t.Parallel()
	require.Zero(t, chaintol.Of(t.Context()), "a context no document touched carries none")
	require.InDelta(t, 0.05, chaintol.Of(chaintol.With(t.Context(), 0.05)), 0)
	inner := chaintol.With(chaintol.With(t.Context(), 0.05), 0)
	require.Zero(t, chaintol.Of(inner), "the innermost tolerance wins")
}
