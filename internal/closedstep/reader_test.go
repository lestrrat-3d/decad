package closedstep

import (
	"context"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestReadAbsentAndCanceled(t *testing.T) {
	_, ok, err := Read(t.Context(), nil, nil)
	require.NoError(t, err)
	require.False(t, ok)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, ok, err = Read(ctx, nil, nil)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
}
