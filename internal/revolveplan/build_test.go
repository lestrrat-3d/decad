package revolveplan

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/stretchr/testify/require"
)

// cancelAfterContext reports context.Canceled from its cancelAt-th Err poll.
type cancelAfterContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	cancelAt        int
	calls           int
}

func (c *cancelAfterContext) Err() error {
	c.calls++
	if c.calls >= c.cancelAt {
		return context.Canceled
	}
	return c.Context.Err()
}

func TestFullRevolveShellsContextPollsEachLoop(t *testing.T) {
	t.Parallel()
	ctx := &cancelAfterContext{Context: t.Context(), cancelAt: 2}
	perLoop := []revLoopParts{{}, {faces: []*topo.Face{{}}, runs: []int{0}}}

	shells, err := fullRevolveShellsContext(ctx, topotest.Alloc{}, perLoop, false)

	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, shells)
	require.Equal(t, 2, ctx.calls)
}
