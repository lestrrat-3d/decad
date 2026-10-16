package facetedrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/stretchr/testify/require"
)

// The staged-survey diagnostic and the tessellate refusal publish this class
// name verbatim: the root type name the payload replaced.
func TestSurveyClass(t *testing.T) {
	t.Parallel()
	require.Equal(t, "facetedPayload", facetedrecord.Payload{}.SurveyClass())
}
