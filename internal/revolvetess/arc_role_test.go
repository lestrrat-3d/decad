package revolvetess_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolvetess"
	"github.com/stretchr/testify/require"
)

func TestArcRole(t *testing.T) {
	t.Parallel()

	for _, reverse := range []bool{false, true} {
		role := revolvetess.ArcRole(reverse)
		require.Equal(t, "side(0,2)", role("side(2)"), "a side role gains the zero turn")
		require.Equal(t, "other", role("other"))
		require.Equal(t, "", role(""))
	}
	keep, swap := revolvetess.ArcRole(false), revolvetess.ArcRole(true)
	require.Equal(t, provenance.RoleCapStart, keep(provenance.RoleCapStart))
	require.Equal(t, provenance.RoleCapEnd, keep(provenance.RoleCapEnd))
	require.Equal(t, provenance.RoleCapEnd, swap(provenance.RoleCapStart))
	require.Equal(t, provenance.RoleCapStart, swap(provenance.RoleCapEnd))
}
