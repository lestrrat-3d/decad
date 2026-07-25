package decad

import (
	"context"
	"testing"

	"github.com/stretchr/testify/require"
)

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

func TestSideOriginsContextPollsEachSegment(t *testing.T) {
	ctx := &cancelAfterContext{Context: t.Context(), cancelAt: 3}

	origins, err := sideOriginsContext(ctx, StepRef(1), 2, []int{4, 5, 6})

	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, origins)
	require.Equal(t, 3, ctx.calls)
}

func TestFullRevolveShellsContextPollsEachLoop(t *testing.T) {
	ctx := &cancelAfterContext{Context: t.Context(), cancelAt: 2}
	perLoop := [][]*Face{{}, {{}}}

	shells, err := fullRevolveShellsContext(ctx, perLoop)

	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, shells)
	require.Equal(t, 2, ctx.calls)
}

func TestAddBlendRolesContextPollsNestedMetadata(t *testing.T) {
	ref := StepRef(1)
	face := &Face{origins: []FeatureRef{{Step: ref, Role: "side(0,0)"}}}
	body := &Body{lumps: []*Lump{{shells: []*Shell{{faces: []*Face{face}}}}}}
	ctx := &cancelAfterContext{Context: t.Context(), cancelAt: 5}

	err := addBlendRoles(ctx, body, ref, []map[int]struct{}{{0: {}}}, "fillet")

	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, 5, ctx.calls)
	require.Equal(t, []FeatureRef{{Step: ref, Role: "side(0,0)"}}, face.origins)
}

func TestRenameCavityRolesContextPollsNestedMetadata(t *testing.T) {
	ref := StepRef(1)
	face := &Face{origins: []FeatureRef{{Step: ref, Role: "side(0,0)"}}}
	ctx := &cancelAfterContext{Context: t.Context(), cancelAt: 3}

	err := renameCavityRoles(ctx, []*Face{face}, ref)

	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, 3, ctx.calls)
	require.Equal(t, []FeatureRef{{Step: ref, Role: "side(0,0)"}}, face.origins)
}
