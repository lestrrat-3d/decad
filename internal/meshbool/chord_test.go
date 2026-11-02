package meshbool

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestOverlapDiameter holds a Cut's overlap chord to the boxes' common part:
// a short tool across one end of a long bar overlaps it by the tool's own
// extent, and boxes that only share a face share no volume.
//
// Leg shown to fail: taking the union box instead returns the bar's length.
func TestOverlapDiameter(t *testing.T) {
	t.Parallel()
	bar := ChordOperand{Lo: r3.Vec{}, Hi: r3.Vec{X: 400, Y: 10, Z: 20}}
	tool := ChordOperand{Lo: r3.Vec{X: -2, Y: 2, Z: 4}, Hi: r3.Vec{X: 10, Y: 8, Z: 10}}
	d, ok := OverlapDiameter(bar, tool)
	require.True(t, ok)
	require.InDelta(t, r3.Vec{X: 10, Y: 6, Z: 6}.Len(), d, 1e-12)
	require.Greater(t, bar.Hi.Sub(bar.Lo).Len(), OverlapChordRatio*d, "premise: the bar passes the ratio")

	face := ChordOperand{Lo: r3.Vec{X: 400}, Hi: r3.Vec{X: 410, Y: 10, Z: 20}}
	_, ok = OverlapDiameter(bar, face)
	require.False(t, ok)
}
