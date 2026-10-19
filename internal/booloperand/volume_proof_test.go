package booloperand_test

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestRequireVolumeProof(t *testing.T) {
	t.Run("admitted payload classes", func(t *testing.T) {
		for _, payload := range []topo.Payload{
			nil,
			prismrecord.Payload{},
			breprecord.Payload{},
		} {
			b := &topo.Body{Kind: topo.BodySolid, Payload: payload}
			require.NoError(t, booloperand.RequireVolumeProof(t.Context(), b, 0), "%T", payload)
		}
	})
	refusals := []struct {
		name string
		body *topo.Body
	}{
		{"sheet", &topo.Body{Kind: topo.BodySheet, Payload: prismrecord.Payload{}}},
		{"stitched body without a recorded triangle set", &topo.Body{Kind: topo.BodySolid, Payload: stitchrecord.Payload{}}},
	}
	for _, c := range refusals {
		t.Run(c.name, func(t *testing.T) {
			err := booloperand.RequireVolumeProof(t.Context(), c.body, 1)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			var expected *meshbool.BooleanExpectedError
			require.True(t, errors.As(err, &expected))
			require.Equal(t, meshbool.BooleanExpectedVolumeProof, expected.Kind)
			require.Equal(t, 1, expected.Operand)
		})
	}
}
