package payloadclass_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/payloadclass"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
	"github.com/stretchr/testify/require"
)

func TestPayloadTypeName(t *testing.T) {
	t.Parallel()
	require.Equal(t, "decad.chainRevolvePayload", payloadclass.TypeName(revolveplan.ChainPayload{}))
	require.Equal(t, "decad.chainLoftPayload", payloadclass.TypeName(loftrecord.ChainPayload{}))
	require.Equal(t, "decad.twistedSweepPayload", payloadclass.TypeName(twistsweep.Payload{}))
	require.Equal(t, "decad.compositeTwistedSweepPayload", payloadclass.TypeName(twistsweep.CompositePayload{}))
	require.Equal(t, "decad.draftPayload", payloadclass.TypeName(draftrecord.Payload{}))
	require.Equal(t, "decad.twoSidedDraftPayload", payloadclass.TypeName(draftrecord.TwoSidedPayload{}))
	require.Equal(t, "decad.sweepPayload", payloadclass.TypeName(sweeprecord.Payload{}))
	require.Equal(t, "decad.chainSweepPayload", payloadclass.TypeName(sweeprecord.ChainPayload{}))
	require.Equal(t, "decad.stitchPayload", payloadclass.TypeName(stitchrecord.Payload{}))
	require.Equal(t, "decad.unstitchPayload", payloadclass.TypeName(stitchrecord.UnstitchPayload{}))
	require.Equal(t, "decad.facetedPayload", payloadclass.TypeName(facetedrecord.Payload{}))
	require.Equal(t, "int", payloadclass.TypeName(3))
}
