package pairkernel_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance/pairkernel"
	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestPlanarPairAdmits(t *testing.T) {
	t.Parallel()

	const (
		mitred  = "mitred"
		faceted = "faceted"
		coil    = "coil"
		prism   = "prism"
		stitch  = "stitch"
		revolve = "revolve"
	)
	classes := map[string]topo.Payload{
		mitred: sweepmitre.Payload{}, faceted: facetedrecord.Payload{}, coil: coilshell.Payload{},
		prism: prismrecord.Payload{}, stitch: stitchrecord.Payload{}, revolve: revolveplan.Payload{},
	}
	// served lists the pairs the exact planar arm answers, order aside.
	served := map[[2]string]bool{
		{mitred, mitred}: true, {mitred, faceted}: true, {mitred, prism}: true, {mitred, stitch}: true,
		{faceted, faceted}: true, {faceted, prism}: true, {faceted, stitch}: true,
		{coil, prism}: true, {coil, stitch}: true,
	}
	for an, ap := range classes {
		for bn, bp := range classes {
			want := served[[2]string{an, bn}] || served[[2]string{bn, an}]
			got := pairkernel.PlanarPairAdmits(&topo.Body{Payload: ap}, &topo.Body{Payload: bp})
			require.Equal(t, want, got, "%s with %s", an, bn)
		}
	}
	require.False(t, pairkernel.PlanarPairAdmits(&topo.Body{}, &topo.Body{Payload: facetedrecord.Payload{}}))
	require.False(t, pairkernel.PlanarPairAdmits(&topo.Body{Payload: facetedrecord.Payload{}}, &topo.Body{}))
}
