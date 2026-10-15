package reportvocab_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
	"github.com/stretchr/testify/require"
)

func TestPayloadTypeName(t *testing.T) {
	t.Parallel()
	require.Equal(t, "decad.chainRevolvePayload", reportvocab.PayloadTypeName(revolveplan.ChainPayload{}))
	require.Equal(t, "decad.chainLoftPayload", reportvocab.PayloadTypeName(loftrecord.ChainPayload{}))
	require.Equal(t, "decad.twistedSweepPayload", reportvocab.PayloadTypeName(twistsweep.Payload{}))
	require.Equal(t, "decad.compositeTwistedSweepPayload", reportvocab.PayloadTypeName(twistsweep.CompositePayload{}))
	require.Equal(t, "decad.draftPayload", reportvocab.PayloadTypeName(draftrecord.Payload{}))
	require.Equal(t, "decad.twoSidedDraftPayload", reportvocab.PayloadTypeName(draftrecord.TwoSidedPayload{}))
	require.Equal(t, "decad.sweepPayload", reportvocab.PayloadTypeName(sweeprecord.Payload{}))
	require.Equal(t, "decad.chainSweepPayload", reportvocab.PayloadTypeName(sweeprecord.ChainPayload{}))
	require.Equal(t, "decad.stitchPayload", reportvocab.PayloadTypeName(stitchrecord.Payload{}))
	require.Equal(t, "decad.unstitchPayload", reportvocab.PayloadTypeName(stitchrecord.UnstitchPayload{}))
	require.Equal(t, "int", reportvocab.PayloadTypeName(3))
}
