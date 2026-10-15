package reportvocab_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/stretchr/testify/require"
)

func TestPayloadTypeName(t *testing.T) {
	t.Parallel()
	require.Equal(t, "decad.chainRevolvePayload", reportvocab.PayloadTypeName(revolveplan.ChainPayload{}))
	require.Equal(t, "decad.chainLoftPayload", reportvocab.PayloadTypeName(loftrecord.ChainPayload{}))
	require.Equal(t, "int", reportvocab.PayloadTypeName(3))
}
