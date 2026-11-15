package rimsplit_test

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/rimsplit"
)

func TestOf(t *testing.T) {
	t.Parallel()
	require.False(t, rimsplit.Of(t.Context()), "a context no document touched carries none")
	require.True(t, rimsplit.Of(rimsplit.With(t.Context(), true)))
	require.False(t, rimsplit.Of(rimsplit.With(rimsplit.With(t.Context(), true), false)), "the innermost choice wins")
}
