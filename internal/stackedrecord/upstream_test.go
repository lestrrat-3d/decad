package stackedrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestUpstreamRolesTravelOnlyThroughPlaced pins that Build stores a record
// without its upstream roles, so a record derived by copying the payload
// carries none, while Placed re-attaches them to the faces they name.
func TestUpstreamRolesTravelOnlyThroughPlaced(t *testing.T) {
	t.Parallel()
	inherited := []provenance.Ref{{Producer: 1, Role: "capStart"}}
	sp := firstPocket(t)
	sp.Upstream = topo.UpstreamRoles{{Roles: []string{"capStart"}, Origins: inherited}}
	env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}

	built, err := stackedrecord.Build(t.Context(), env, sp, stackedrecord.OwnPlan{})
	require.NoError(t, err)
	require.Nil(t, built.Payload.(stackedrecord.Payload).Upstream)
	for _, f := range topobuild.Faces(built) {
		require.Nil(t, f.Upstream)
	}

	shift, err := r3.Translation(r3.NewVec(0, 0, 5))
	require.NoError(t, err)
	placed, err := sp.Placed(t.Context(), env, shift)
	require.NoError(t, err)
	require.Equal(t, sp.Upstream, placed.Payload.(stackedrecord.Payload).Upstream)
	var named int
	for _, f := range topobuild.Faces(placed) {
		if f.Origins[0].Role == "capStart" {
			named++
			require.Equal(t, inherited, f.Upstream)
			continue
		}
		require.Nil(t, f.Upstream)
	}
	require.Equal(t, 1, named)
}
