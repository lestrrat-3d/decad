package revolveshell_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The bore route reads a join in +X arithmetic. A join whose tooth trim holds
// a limit about another axis keeps ordinary dispatch: the route reports no
// match before it reads the cap-ring record. The +X join with the same empty
// record is the control: the route matches it and refuses the missing record
// (agent-docs/revolve-axis-design.md §4.6).
func TestSectorCylinderCutDeclinesJoinAboutAnotherAxis(t *testing.T) {
	t.Parallel()
	yAxis, ok := coneaxis.New(r3.Vec{Y: 1})
	require.True(t, ok)
	joined := func(lower, upper coneaxis.Axis) *topo.Body {
		trim := &facetedrecord.PointConeTrimRecord{
			Lower: &facetedrecord.PointConeLimit{Axis: lower},
			Upper: &facetedrecord.PointConeLimit{Axis: upper},
		}
		return &topo.Body{Payload: facetedrecord.Payload{Xform: r3.Identity(),
			RevolveLoftJoin: &facetedrecord.RevolveLoftJoinRecord{
				Certificate: &facetedrecord.RevolveLoftSectorCertificate{Trim: trim}}}}
	}
	_, ok, err := revolveshell.SectorCylinderCut(t.Context(), meshbool.OpCut, topo.Env{},
		joined(coneaxis.X(), coneaxis.X()), &topo.Body{})
	require.True(t, ok)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	for _, axes := range [][2]coneaxis.Axis{{yAxis, yAxis}, {coneaxis.X(), yAxis}, {yAxis, coneaxis.X()}} {
		out, ok, err := revolveshell.SectorCylinderCut(t.Context(), meshbool.OpCut, topo.Env{},
			joined(axes[0], axes[1]), &topo.Body{})
		require.NoError(t, err)
		require.False(t, ok)
		require.Nil(t, out)
	}
	// A certificate from the general record branch declines even when its
	// axis value and both limits are X, as for an XZ-plane blank revolved
	// about U: its ring stands at angles in the blank's own basis.
	general := joined(coneaxis.X(), coneaxis.X())
	payload := general.Payload.(facetedrecord.Payload)
	payload.RevolveLoftJoin.Certificate.Section = facetedrecord.RevolveSectionCertificate{
		Axis: coneaxis.X(), General: &facetedrecord.RevolveSectionAxis{}}
	out, ok, err := revolveshell.SectorCylinderCut(t.Context(), meshbool.OpCut, topo.Env{}, general, &topo.Body{})
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, out)
}
