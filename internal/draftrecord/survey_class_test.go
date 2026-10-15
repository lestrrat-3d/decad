package draftrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/stretchr/testify/require"
)

// The staged-survey diagnostic text publishes these class names verbatim.
func TestSurveyClass(t *testing.T) {
	t.Parallel()
	require.Equal(t, "draftPayload", draftrecord.Payload{}.SurveyClass())
	require.Equal(t, "twoSidedDraftPayload", draftrecord.TwoSidedPayload{}.SurveyClass())
}
