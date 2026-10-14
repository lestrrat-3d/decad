package provenance_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/stretchr/testify/require"
)

// TestRoleNames pins the spellings of the roles every analytic evaluator
// mints: they are the stable names a FeatureRef exposes, so a change would
// retarget every selector that names a cap or a body origin.
func TestRoleNames(t *testing.T) {
	t.Parallel()

	require.Equal(t, "body", provenance.RoleBody)
	require.Equal(t, "capStart", provenance.RoleCapStart)
	require.Equal(t, "capEnd", provenance.RoleCapEnd)
}
