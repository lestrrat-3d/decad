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

func TestExtendedCutterTargetChord(t *testing.T) {
	t.Parallel()
	target := ChordOperand{Hi: r3.Vec{X: 3, Y: 4}}
	pair := 7 * ChordFactor
	require.Equal(t, 5*ChordFactor, target.EvaluationChord(pair, 7, 1, false, true))
	require.Equal(t, pair, target.EvaluationChord(pair, 7, 1, true, true))
	require.Equal(t, pair, target.EvaluationChord(pair, 7, 1, false, false))
	require.Equal(t, 2*ChordFactor, target.EvaluationChord(pair, 2, 1, false, true))
	target.Floor = 6 * ChordFactor
	require.Equal(t, target.Floor/2, target.EvaluationChord(pair/2, 7, 0.5, false, true))
}
