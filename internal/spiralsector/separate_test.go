package spiralsector_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func tetra(o r3.Vec) spiralsector.Shell {
	return spiralsector.Shell{
		Verts: []r3.Vec{
			o, o.Add(r3.Vec{X: 1}), o.Add(r3.Vec{Y: 1}), o.Add(r3.Vec{Z: 1}),
		},
		Tris:        [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}},
		VertexBound: make([]float64, 4),
	}
}

func quarterTurn(t *testing.T) r3.Transform {
	t.Helper()
	rot, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(90))
	require.NoError(t, err)
	return rot
}

func TestSeparateProvesRotatedTeethApart(t *testing.T) {
	t.Parallel()

	rot := quarterTurn(t)
	a := tetra(r3.Vec{Y: 10})
	b := spiralsector.Shell{Tris: a.Tris, VertexBound: a.VertexBound}
	for _, v := range a.Verts {
		b.Verts = append(b.Verts, rot.Apply(v))
	}
	moves := []r3.Transform{r3.Identity(), rot}
	require.True(t, spiralsector.Separate(t.Context(), []spiralsector.Shell{a, b}, moves, false))
}

func TestSeparateRefusesOverlappingTeeth(t *testing.T) {
	t.Parallel()

	rot := quarterTurn(t)
	a := tetra(r3.Vec{Y: 10})
	moves := []r3.Transform{r3.Identity(), rot}
	pair := []spiralsector.Shell{a, a}

	require.False(t, spiralsector.Separate(t.Context(), pair, moves, false))
	require.False(t, spiralsector.Separate(t.Context(), pair, moves[:1], false))

	negative := a
	negative.MeshBound = -1
	require.False(t, spiralsector.Separate(t.Context(), []spiralsector.Shell{a, negative}, moves, false))

	// Separated shells still refuse under a cancelled context.
	b := spiralsector.Shell{Tris: a.Tris, VertexBound: a.VertexBound}
	for _, v := range a.Verts {
		b.Verts = append(b.Verts, rot.Apply(v))
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	require.False(t, spiralsector.Separate(ctx, []spiralsector.Shell{a, b}, moves, false))
}

func TestPlacedHeldShellsBoundsEachCopy(t *testing.T) {
	t.Parallel()

	move, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	moves := []r3.Transform{r3.Identity(), move}
	base := tetra(r3.Vec{})
	first := spiralsector.HeldCopy{Verts: base.Verts, Tris: base.Tris, Linked: true}

	shifted := func(dz float64) spiralsector.HeldCopy {
		verts := make([]r3.Vec, len(base.Verts))
		for i, v := range base.Verts {
			verts[i] = v.Add(r3.Vec{X: 1})
		}
		verts[3].Z += dz
		return spiralsector.HeldCopy{Verts: verts, Tris: base.Tris, Linked: true}
	}

	t.Run("exact copy", func(t *testing.T) {
		t.Parallel()
		shells, err := spiralsector.PlacedHeldShells(t.Context(), []spiralsector.HeldCopy{first, shifted(0)}, moves)
		require.NoError(t, err)
		require.Len(t, shells, 2)
		require.Equal(t, make([]float64, 4), shells[0].VertexBound)
		for _, b := range shells[1].VertexBound {
			require.Less(t, b, 1e-12)
		}
	})
	t.Run("displaced vertex", func(t *testing.T) {
		t.Parallel()
		shells, err := spiralsector.PlacedHeldShells(t.Context(), []spiralsector.HeldCopy{first, shifted(0.25)}, moves)
		require.NoError(t, err)
		require.GreaterOrEqual(t, shells[1].VertexBound[3], 0.25)
		for _, b := range shells[1].VertexBound[:3] {
			require.Less(t, b, 1e-12)
		}
		require.GreaterOrEqual(t, shells[1].MeshBound, 0.25)
	})
}

func TestPlacedHeldShellsRefusals(t *testing.T) {
	t.Parallel()

	move, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	moves := []r3.Transform{r3.Identity(), move}
	base := tetra(r3.Vec{})
	first := spiralsector.HeldCopy{Verts: base.Verts, Tris: base.Tris, Linked: true}
	verts := make([]r3.Vec, len(base.Verts))
	for i, v := range base.Verts {
		verts[i] = v.Add(r3.Vec{X: 1})
	}
	copyOK := spiralsector.HeldCopy{Verts: verts, Tris: base.Tris, Linked: true}

	unlinkedFirst := first
	unlinkedFirst.Linked = false
	unlinkedCopy := copyOK
	unlinkedCopy.Linked = false
	reordered := copyOK
	reordered.Tris = [][3]int{base.Tris[1], base.Tris[0], base.Tris[2], base.Tris[3]}

	tests := []struct {
		name   string
		copies []spiralsector.HeldCopy
		moves  []r3.Transform
		want   string
	}{
		{"one copy", []spiralsector.HeldCopy{first}, moves[:1], "held copies lack their source placement"},
		{
			"first move not identity", []spiralsector.HeldCopy{first, copyOK}, []r3.Transform{move, move},
			"held copies lack their source placement",
		},
		{"first unlinked", []spiralsector.HeldCopy{unlinkedFirst, copyOK}, moves, "held source is incomplete"},
		{
			"copy unlinked", []spiralsector.HeldCopy{first, unlinkedCopy}, moves,
			"held copy 2 lost source correspondence",
		},
		{"reordered facets", []spiralsector.HeldCopy{first, reordered}, moves, "held copy 2 changed source facets"},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, err := spiralsector.PlacedHeldShells(t.Context(), tc.copies, tc.moves)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, tc.want)
		})
	}
}

func TestInverseHeldResultKeepsHeldX(t *testing.T) {
	t.Parallel()

	rot := quarterTurn(t)
	base := tetra(r3.Vec{})
	blank := &meshbool.BoolMesh{Verts: base.Verts, Tris: base.Tris, VertexBound: make([]float64, 4)}
	heldShell := tetra(r3.Vec{Y: 5})
	newHeld := func() *meshbool.StitchedMesh {
		return &meshbool.StitchedMesh{
			Verts: heldShell.Verts, VertexBound: []float64{0.01, 0.01, 0.01, 0.01}, Round: 1e-9,
		}
	}

	t.Run("identity", func(t *testing.T) {
		t.Parallel()
		held := newHeld()
		verts, beta, round, _, ok := spiralsector.InverseHeldResult(
			t.Context(), blank, base.Verts, 0.001, held, r3.Identity())
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
		verts, _, _, _, ok := spiralsector.InverseHeldResult(t.Context(), blank, base.Verts, 0.001, held, rot)
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
		_, _, _, _, ok = spiralsector.InverseHeldResult(t.Context(), blank, base.Verts, 0.001, short, r3.Identity())
		require.False(t, ok, "mismatched bounds")

		nan := newHeld()
		nan.VertexBound[0] = math.NaN()
		_, _, _, _, ok = spiralsector.InverseHeldResult(t.Context(), blank, base.Verts, 0.001, nan, r3.Identity())
		require.False(t, ok, "NaN bound")
	})
}
