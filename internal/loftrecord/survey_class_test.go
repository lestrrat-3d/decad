package loftrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/stretchr/testify/require"
)

// The staged-survey diagnostic text publishes these class names verbatim.
func TestSurveyClass(t *testing.T) {
	require.Equal(t, "loftPayload", loftrecord.Payload{}.SurveyClass())
	require.Equal(t, "chainLoftPayload", loftrecord.ChainPayload{}.SurveyClass())
}
