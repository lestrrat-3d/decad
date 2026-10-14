package decad

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
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
	t.Parallel()
	ctx := &cancelAfterContext{Context: t.Context(), cancelAt: 3}

	origins, err := prismrecord.SideOrigins(ctx, producerID(1), 2, []int{4, 5, 6})

	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, origins)
	require.Equal(t, 3, ctx.calls)
}

func TestFullRevolveShellsContextPollsEachLoop(t *testing.T) {
	t.Parallel()
	ctx := &cancelAfterContext{Context: t.Context(), cancelAt: 2}
	perLoop := []revLoopParts{{}, {faces: []*Face{bornFace(&Face{})}, runs: []int{0}}}

	shells, err := fullRevolveShellsContext(ctx, perLoop, false)

	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, shells)
	require.Equal(t, 2, ctx.calls)
}

func TestAddBlendRolesContextPollsNestedMetadata(t *testing.T) {
	t.Parallel()
	ref := producerID(1)
	face := bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Producer: ref, Role: "side(0,0)"}}}})
	body := bornBody(&Body{n: topo.Body{Lumps: []*topo.Lump{lumpNode(bornLump(&Lump{n: topo.Lump{Shells: []*topo.FaceShell{faceShellNode(bornFaceShell(&FaceShell{n: topo.FaceShell{Faces: []*topo.Face{faceNode(face)}}}))}}}))}}})
	ctx := &cancelAfterContext{Context: t.Context(), cancelAt: 5}

	err := topobuild.AddBlendRoles(ctx, bodyNode(body), ref, []map[int]struct{}{{0: {}}}, "fillet")

	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, 5, ctx.calls)
	require.Equal(t, []provenance.Ref{{Producer: ref, Role: "side(0,0)"}}, face.n.Origins)
}

func TestRenameCavityRolesContextPollsNestedMetadata(t *testing.T) {
	t.Parallel()
	ref := producerID(1)
	face := bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Producer: ref, Role: "side(0,0)"}}}})
	ctx := &cancelAfterContext{Context: t.Context(), cancelAt: 3}

	err := renameCavityRoles(ctx, []*Face{face}, ref)

	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, 3, ctx.calls)
	require.Equal(t, []provenance.Ref{{Producer: ref, Role: "side(0,0)"}}, face.n.Origins)
}
