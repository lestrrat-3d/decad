package sweeprecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/stretchr/testify/require"
)

// The staged-survey diagnostic text publishes these class names verbatim.
func TestSurveyClass(t *testing.T) {
	t.Parallel()
	require.Equal(t, "sweepPayload", sweeprecord.Payload{}.SurveyClass())
	require.Equal(t, "chainSweepPayload", sweeprecord.ChainPayload{}.SurveyClass())
}
