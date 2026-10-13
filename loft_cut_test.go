package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/stretchr/testify/require"
)

func TestLoftCertifiedRewriteWorkLimitKeepsDefault(t *testing.T) {
	t.Parallel()
	const pairs = uint64(360)
	ordinary := loftmesh.StationWorkLimit(0, pairs)
	require.Equal(t, ordinary, (loftPayload{}).rewriteWorkLimit(pairs))
	trusted := loftPayload{authenticatedReconstruction: true}
	require.Equal(t, 2*ordinary, trusted.rewriteWorkLimit(pairs))
	require.LessOrEqual(t, trusted.rewriteWorkLimit(pairs), uint64(1<<27))

	defaultWork := freeform.NewFreeformWork()
	defaultWork.RaiseLimit((loftPayload{}).rewriteWorkLimit(pairs))
	loftWork := freeform.NewFreeformWork()
	loftWork.RaiseLimit(trusted.rewriteWorkLimit(pairs))
	for range ordinary / freeform.FreeformWorkLimit {
		require.NoError(t, defaultWork.Step(freeform.FreeformWorkLimit))
		require.NoError(t, loftWork.Step(freeform.FreeformWorkLimit))
	}
	require.ErrorIs(t, defaultWork.Step(1), ErrUnsupported)
	require.NoError(t, loftWork.Step(1))
}
