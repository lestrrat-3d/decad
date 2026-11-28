package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestTessellateRefusalNamesChainRevolvePayload pins the payload class the
// unsupported-payload refusal prints for a revolved open chain.
func TestTessellateRefusalNamesChainRevolvePayload(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a, b := s.CreatePoint(0, 0), s.CreatePoint(3, 3)
	s.Fix(a)
	s.CreateArc(s.CreatePoint(0, 3), a, b)
	s.CreateLine(b, s.CreatePoint(5, 3))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	ch := s.Chains()[0]
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	body, err := decad.Revolve(decad.New(), s, ch, axis, decad.FullRevolution{}).Do(t.Context())
	require.NoError(t, err)
	_, err = body.Tessellate(t.Context(), units.Millimeters(0.5))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, refusalText("decad.chainRevolvePayload"), err.Error())
}

// refusalText is the complete unsupported-payload message for a payload class.
func refusalText(class string) string {
	return "decad: not supported by the current evaluator: tessellation does not support payload " + class +
		"; supported payload classes are prism, patch, stacked prism, brep, chain-fed prism, sweep, mitred sweep, " +
		"coil, revolve, cup, loft, cap-loop chamfer, draft (tapered extrude), stitch, and faceted"
}
