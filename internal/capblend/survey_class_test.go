package capblend_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/stretchr/testify/require"
)

// The staged-survey diagnostic text publishes this class name verbatim.
func TestSurveyClass(t *testing.T) {
	require.Equal(t, "capBlendPayload", capblend.Payload{}.SurveyClass())
}
