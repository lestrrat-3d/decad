package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// oldProveSaddleTool is proveSaddleTool before the base cells were shared:
// each saddle proved its own.
func oldProveSaddleTool(t *testing.T, tool *saddleTool, s saddle, d float64) {
	t.Helper()
	proof, err := crossbore.ProveToolBand(t.Context(), s.key.R, s.key.r, d, s.key.pose, func(j, i int) r3.Vec {
		return tool.verts[rowVertex(j, i)]
	})
	require.NoError(t, err)
	tool.proof = proof
	tool.beta = make([]float64, len(tool.verts))
	for j := range crossbore.ToolStations {
		for i := range crossbore.ToolRows + 1 {
			tool.beta[rowVertex(j, i)] = proof.Chord
		}
	}
	var rows [crossbore.ToolRows][][3]int
	for k, f := range tool.tris {
		if tool.row[k] >= 0 {
			rows[tool.row[k]] = append(rows[tool.row[k]], f)
		}
	}
	for i := range rows {
		tool.rowArea[i], tool.rowSlop[i] = heldArea(tool.verts, rows[i])
	}
}

func sameBits(a, b float64) bool { return math.Float64bits(a) == math.Float64bits(b) }

// sameToolProof reports the first differing field name, or "".
func sameToolProof(a, b crossbore.ToolProof) string {
	switch {
	case !sameBits(a.Chord, b.Chord):
		return "Chord"
	case !sameBits(a.SampleError, b.SampleError):
		return "SampleError"
	case !sameBits(a.AreaLo, b.AreaLo):
		return "AreaLo"
	case !sameBits(a.AreaHi, b.AreaHi):
		return "AreaHi"
	}
	for i := range crossbore.ToolRows {
		if !sameBits(a.RowLo[i], b.RowLo[i]) {
			return "RowLo"
		}
		if !sameBits(a.RowHi[i], b.RowHi[i]) {
			return "RowHi"
		}
	}
	return ""
}

// TestSharedToolBasesMatchOldPath proves the two mouths of one through bore,
// sharing one proof of the base cells through sharedToolBases, get the bits
// the per-saddle proof gave, and that a saddle of other radii gets its own.
func TestSharedToolBasesMatchOldPath(t *testing.T) {
	t.Parallel()
	const d = 1.5
	mouth := func(R, r float64, pose crossbore.Pose) saddle {
		return saddle{key: saddleKey{pose: pose, R: R, r: r}}
	}
	saddles := []saddle{
		mouth(10, 3, crossbore.Pose{Axis: [3]int{2, 1, 0}, Sign: [3]float64{-1, -1, 1}, Shift: r3.Vec{X: 40}}),
		mouth(10, 3, crossbore.Pose{Axis: [3]int{2, 1, 0}, Sign: [3]float64{1, 1, 1}, Shift: r3.Vec{X: -40}}),
		mouth(9, 2.5, crossbore.Pose{Axis: [3]int{2, 1, 0}, Sign: [3]float64{1, 1, 1}, Shift: r3.Vec{X: 5}}),
	}
	var shared sharedToolBases
	for i, s := range saddles {
		got, err := placeSaddleTool(s, d)
		require.NoError(t, err)
		want, err := placeSaddleTool(s, d)
		require.NoError(t, err)
		bases, err := shared.get(t.Context(), s, d)
		require.NoError(t, err)
		require.NoError(t, proveSaddleTool(t.Context(), got, s, bases))
		oldProveSaddleTool(t, want, s, d)
		require.Empty(t, sameToolProof(want.proof, got.proof), "saddle %d", i)
		require.Len(t, got.beta, len(want.beta))
		for k := range want.beta {
			require.True(t, sameBits(want.beta[k], got.beta[k]), "saddle %d vertex %d bound", i, k)
		}
		for k := range crossbore.ToolRows {
			require.True(t, sameBits(want.rowArea[k], got.rowArea[k]), "saddle %d row %d area", i, k)
			require.True(t, sameBits(want.rowSlop[k], got.rowSlop[k]), "saddle %d row %d slop", i, k)
		}
	}
}
