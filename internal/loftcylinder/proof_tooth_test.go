package loftcylinder_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestProofTooth(t *testing.T) {
	t.Run("an axial tooth is its own proof body", func(t *testing.T) {
		tooth := &topo.Body{Payload: loftrecord.Payload{Xform: r3.Identity()}}
		proof, shift, admitted, err := loftcylinder.ProofTooth(t.Context(), topo.Env{}, tooth, 0, 2)
		require.NoError(t, err)
		require.True(t, admitted)
		require.Same(t, tooth, proof)
		require.Zero(t, shift)
	})
	t.Run("a translated tooth is not admitted", func(t *testing.T) {
		shifted, err := r3.Translation(r3.Vec{Z: 1})
		require.NoError(t, err)
		tooth := &topo.Body{Payload: loftrecord.Payload{Xform: shifted}}
		proof, shift, admitted, err := loftcylinder.ProofTooth(t.Context(), topo.Env{}, tooth, 0, 2)
		require.NoError(t, err)
		require.False(t, admitted)
		require.Nil(t, proof)
		require.Zero(t, shift)
	})
	t.Run("a tooth that is not a Loft is not admitted", func(t *testing.T) {
		tooth := &topo.Body{Payload: prismrecord.Payload{Xform: r3.Identity()}}
		proof, _, admitted, err := loftcylinder.ProofTooth(t.Context(), topo.Env{}, tooth, 0, 2)
		require.NoError(t, err)
		require.False(t, admitted)
		require.Nil(t, proof)
	})
}
