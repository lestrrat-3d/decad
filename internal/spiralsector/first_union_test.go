package spiralsector_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestJoinAreaAllowance(t *testing.T) {
	// One held right triangle of area 0.5 mm².
	held := &meshbool.StitchedMesh{
		Verts: []r3.Vec{{}, {X: 1}, {Y: 1}},
		Tris:  [][3]int{{0, 1, 2}},
	}
	body := func(value, bound float64) *topo.Body {
		return &topo.Body{Area: measurement.Measurement{
			Value: units.SquareMillimeters(value), Bound: units.SquareMillimeters(bound)}}
	}
	t.Run("sums held and both operand area ceilings", func(t *testing.T) {
		allowance, ok, err := spiralsector.JoinAreaAllowance(t.Context(), body(40, 1), body(10, 0.5), held)
		require.NoError(t, err)
		require.True(t, ok)
		// 0.5 + (40 + 1) + (10 + 0.5) = 52, raised only by outward rounding.
		require.GreaterOrEqual(t, allowance, 52.0)
		require.Less(t, allowance, 52.01)
	})
	t.Run("refuses a negative operand area", func(t *testing.T) {
		_, ok, err := spiralsector.JoinAreaAllowance(t.Context(), body(40, 1), body(-10, 0.5), held)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("refuses a negative operand bound", func(t *testing.T) {
		_, ok, err := spiralsector.JoinAreaAllowance(t.Context(), body(40, -1), body(10, 0.5), held)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("refuses a missing operand", func(t *testing.T) {
		_, ok, err := spiralsector.JoinAreaAllowance(t.Context(), body(40, 1), nil, held)
		require.NoError(t, err)
		require.False(t, ok)
	})
}
