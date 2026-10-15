package stitchrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/stretchr/testify/require"
)

// The staged-survey diagnostic and the tessellate refusal publish these
// class names verbatim.
func TestSurveyClass(t *testing.T) {
	t.Parallel()
	require.Equal(t, "stitchPayload", stitchrecord.Payload{}.SurveyClass())
	require.Equal(t, "unstitchPayload", stitchrecord.UnstitchPayload{}.SurveyClass())
}
