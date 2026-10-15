package cupwall

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
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

func TestRenameCavityRolesContextPollsNestedMetadata(t *testing.T) {
	t.Parallel()
	ref := provenance.Producer(1)
	face := &topo.Face{Origins: []provenance.Ref{{Producer: ref, Role: "side(0,0)"}}}
	ctx := &cancelAfterContext{Context: t.Context(), cancelAt: 3}

	err := renameCavityRoles(ctx, []*topo.Face{face}, ref)

	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, 3, ctx.calls)
	require.Equal(t, []provenance.Ref{{Producer: ref, Role: "side(0,0)"}}, face.Origins)
}
