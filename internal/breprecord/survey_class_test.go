package breprecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/stretchr/testify/require"
)

// The staged-survey diagnostic text publishes this class name verbatim.
func TestSurveyClass(t *testing.T) {
	require.Equal(t, "allEdgeChamferPayload", breprecord.AllEdgeChamferPayload{}.SurveyClass())
	require.Equal(t, "brepPayload", breprecord.Payload{}.SurveyClass())
}
