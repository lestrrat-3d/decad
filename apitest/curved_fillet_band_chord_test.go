package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The patch on a rounded corner has curvature along its contour and across
// its fillet. Both contribute to the bound consumed by a later mesh boolean.
func TestCurvedFilletBandChord(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	part := wedgeRoundedRectAtZ(t, doc, -3, -3, 3, 3, 1, 0, 2)
	rim := decad.Edges(decad.OuterLoopOf(decad.CapStart(part)))
	fillet, err := decad.Fillet(part, rim, units.Millimeters(.5)).Do(t.Context())
	require.NoError(t, err)
	const chord = .01
	mesh, err := fillet.Tessellate(t.Context(), units.Millimeters(chord), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.LessOrEqual(t, mesh.Bound().Base(), chord)
}
