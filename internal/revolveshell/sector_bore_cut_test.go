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

// The bore route reads a join about its certificate's axis. A join whose
// tooth trim holds a limit about another axis keeps ordinary dispatch: the
// route reports no match before it reads the cap-ring record. That covers a +X
// certificate with a limit about Y, and a general certificate about Y with a
// limit about X. A join whose limits match its certificate is the control:
// the route matches it, +X or general, and refuses the missing record
// (agent-docs/revolve-axis-design.md §4.6, §9.1).
func TestSectorCylinderCutDeclinesJoinAboutAnotherAxis(t *testing.T) {
	t.Parallel()
	yAxis, ok := coneaxis.New(r3.Vec{Y: 1})
	require.True(t, ok)
	joined := func(section facetedrecord.RevolveSectionCertificate, lower, upper coneaxis.Axis) *topo.Body {
		trim := &facetedrecord.PointConeTrimRecord{
			Lower: &facetedrecord.PointConeLimit{Axis: lower},
			Upper: &facetedrecord.PointConeLimit{Axis: upper},
		}
		return &topo.Body{Payload: facetedrecord.Payload{Xform: r3.Identity(),
			RevolveLoftJoin: &facetedrecord.RevolveLoftJoinRecord{
				Certificate: &facetedrecord.RevolveLoftSectorCertificate{Section: section, Trim: trim}}}}
	}
	plusX := facetedrecord.RevolveSectionCertificate{Axis: coneaxis.X()}
	aboutX := facetedrecord.RevolveSectionCertificate{Axis: coneaxis.X(), General: &facetedrecord.RevolveSectionAxis{}}
	aboutY := facetedrecord.RevolveSectionCertificate{Axis: yAxis, General: &facetedrecord.RevolveSectionAxis{}}
	for name, body := range map[string]*topo.Body{
		"+X":              joined(plusX, coneaxis.X(), coneaxis.X()),
		"general about X": joined(aboutX, coneaxis.X(), coneaxis.X()),
		"general about Y": joined(aboutY, yAxis, yAxis),
	} {
		out, ok, err := revolveshell.SectorCylinderCut(t.Context(), meshbool.OpCut, topo.Env{}, body, &topo.Body{})
		require.True(t, ok, name)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, name)
		require.ErrorContains(t, err, "cap-ring record", name)
		require.Nil(t, out, name)
	}
	for name, body := range map[string]*topo.Body{
		"+X with both about Y":    joined(plusX, yAxis, yAxis),
		"+X with lower about Y":   joined(plusX, yAxis, coneaxis.X()),
		"+X with upper about Y":   joined(plusX, coneaxis.X(), yAxis),
		"about Y with X limits":   joined(aboutY, coneaxis.X(), coneaxis.X()),
		"about Y with upper at X": joined(aboutY, yAxis, coneaxis.X()),
	} {
		out, ok, err := revolveshell.SectorCylinderCut(t.Context(), meshbool.OpCut, topo.Env{}, body, &topo.Body{})
		require.NoError(t, err, name)
		require.False(t, ok, name)
		require.Nil(t, out, name)
	}
}
