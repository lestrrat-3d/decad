package booleandispatch

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestTargetChordRetry(t *testing.T) {
	t.Parallel()
	geometric := meshbool.ExpectedBoolean(meshbool.BooleanExpectedStaging, decaderr.ErrUnsupported)
	require.True(t, retryTargetChord(geometric))
	for _, cause := range []error{nil, errors.New("unexpected"), decaderr.ErrBooleanFailed,
		errors.Join(geometric, context.Canceled), proofbound.WithWorkLimit(geometric, "work", 2, 1), prismcells.WithWorkLimit(geometric, 2)} {
		require.False(t, retryTargetChord(cause))
	}
}
