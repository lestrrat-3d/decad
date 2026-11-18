package coplanarcontact_test

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/coplanarcontact"
)

func TestOf(t *testing.T) {
	t.Parallel()
	require.False(t, coplanarcontact.Of(t.Context()), "a context no document touched carries none")
	require.True(t, coplanarcontact.Of(coplanarcontact.With(t.Context(), true)))
	require.False(t, coplanarcontact.Of(coplanarcontact.With(coplanarcontact.With(t.Context(), true), false)), "the innermost choice wins")
}
