package loftcylinder_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestMirrorBuildsRefuseLostHalves(t *testing.T) {
	t.Parallel()
	env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
	notFaceted := &topo.Body{Payload: prismrecord.Payload{}}
	unbored := &topo.Body{Payload: facetedrecord.Payload{ContactAudited: true, Xform: r3.Identity()}}
	t.Run("join without an audited lower slab", func(t *testing.T) {
		body, err := loftcylinder.BuildMirrorJoin(t.Context(), env, &topo.Body{}, nil, nil, unbored,
			r3.Identity(), 0, 0)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "lower slab has no audited source")
		require.Nil(t, body)
	})
	t.Run("bore without a bored lower half", func(t *testing.T) {
		record := &facetedrecord.LoftMirrorCylinderJoinRecord{Source: &facetedrecord.LoftMirrorJoinRecord{
			Lower: &topo.Body{}}}
		body, err := loftcylinder.BuildMirrorBore(t.Context(), env, record, unbored)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "lost its lower annulus")
		require.Nil(t, body)
	})
	t.Run("cap chamfer without a faceted lower half", func(t *testing.T) {
		body, err := loftcylinder.BuildMirrorCapChamfer(t.Context(), env, facetedrecord.Payload{},
			&facetedrecord.LoftMirrorCylinderBoreRecord{}, notFaceted, loftcylinder.ChamferHalf{}, 0, 0, 0)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "lost its audited lower half")
		require.Nil(t, body)
	})
}
