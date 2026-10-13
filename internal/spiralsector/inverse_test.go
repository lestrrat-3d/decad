package spiralsector_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func inverseTetra(o r3.Vec) ([]r3.Vec, [][3]int) {
	return []r3.Vec{
		o, o.Add(r3.Vec{X: 1}), o.Add(r3.Vec{Y: 1}), o.Add(r3.Vec{Z: 1}),
	}, [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
}

func TestInverseHeldResultKeepsHeldX(t *testing.T) {
	t.Parallel()

	rot, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(90))
	require.NoError(t, err)
	baseVerts, baseTris := inverseTetra(r3.Vec{})
	blank := &meshbool.BoolMesh{Verts: baseVerts, Tris: baseTris, VertexBound: make([]float64, 4)}
	heldVerts, _ := inverseTetra(r3.Vec{Y: 5})
	newHeld := func() *meshbool.StitchedMesh {
		return &meshbool.StitchedMesh{
			Verts: heldVerts, VertexBound: []float64{0.01, 0.01, 0.01, 0.01}, Round: 1e-9,
		}
	}

	t.Run("identity", func(t *testing.T) {
		t.Parallel()
		held := newHeld()
		verts, beta, round, _, ok := spiralsector.InverseHeldResult(
			t.Context(), blank, baseVerts, 0.001, held, r3.Identity())
		require.True(t, ok)
		require.Equal(t, held.Verts, verts)
		require.Len(t, beta, 4)
		for _, b := range beta {
			require.GreaterOrEqual(t, b, 0.01)
		}
		require.GreaterOrEqual(t, round, 1e-9)
	})
	t.Run("rotated", func(t *testing.T) {
		t.Parallel()
		held := newHeld()
		inv, err := rot.Inverse()
		require.NoError(t, err)
		verts, _, _, _, ok := spiralsector.InverseHeldResult(t.Context(), blank, baseVerts, 0.001, held, rot)
		require.True(t, ok)
		for i, v := range verts {
			want := inv.Apply(held.Verts[i])
			require.Equal(t, held.Verts[i].X, v.X)
			require.InDelta(t, want.Y, v.Y, 1e-12)
			require.InDelta(t, want.Z, v.Z, 1e-12)
		}
	})
	t.Run("refusals", func(t *testing.T) {
		t.Parallel()
		_, _, _, _, ok := spiralsector.InverseHeldResult(t.Context(), blank, nil, 0.001, newHeld(), r3.Identity())
		require.False(t, ok, "empty first vertices")

		short := newHeld()
		short.VertexBound = short.VertexBound[:3]
		_, _, _, _, ok = spiralsector.InverseHeldResult(t.Context(), blank, baseVerts, 0.001, short, r3.Identity())
		require.False(t, ok, "mismatched bounds")

		nan := newHeld()
		nan.VertexBound[0] = math.NaN()
		_, _, _, _, ok = spiralsector.InverseHeldResult(t.Context(), blank, baseVerts, 0.001, nan, r3.Identity())
		require.False(t, ok, "NaN bound")
	})
}
