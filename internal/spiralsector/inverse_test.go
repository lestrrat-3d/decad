package spiralsector_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
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

func axisRecord(t *testing.T, axis r3.Vec, angle units.Value) (*exactrotation.Rotation, r3.Transform) {
	t.Helper()
	rec, move, err := exactrotation.New(r3.Vec{}, axis, angle)
	require.NoError(t, err)
	return rec, move
}

func TestInverseHeldResultKeepsHeldX(t *testing.T) {
	t.Parallel()

	rec90, rot := axisRecord(t, r3.Vec{X: 1}, units.Degrees(90))
	rec0, id := axisRecord(t, r3.Vec{X: 1}, units.Radians(0))
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
			t.Context(), blank, baseVerts, 0.001, held, id, rec0)
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
		verts, _, _, _, ok := spiralsector.InverseHeldResult(t.Context(), blank, baseVerts, 0.001, held, rot, rec90)
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
		_, _, _, _, ok := spiralsector.InverseHeldResult(t.Context(), blank, nil, 0.001, newHeld(), id, rec0)
		require.False(t, ok, "empty first vertices")

		short := newHeld()
		short.VertexBound = short.VertexBound[:3]
		_, _, _, _, ok = spiralsector.InverseHeldResult(t.Context(), blank, baseVerts, 0.001, short, id, rec0)
		require.False(t, ok, "mismatched bounds")

		nan := newHeld()
		nan.VertexBound[0] = math.NaN()
		_, _, _, _, ok = spiralsector.InverseHeldResult(t.Context(), blank, baseVerts, 0.001, nan, id, rec0)
		require.False(t, ok, "NaN bound")

		_, _, _, _, ok = spiralsector.InverseHeldResult(t.Context(), blank, baseVerts, 0.001, newHeld(), rot, nil)
		require.False(t, ok, "a nil exact record")

		recY, aboutY := axisRecord(t, r3.Vec{Y: 1}, units.Degrees(90))
		_, _, _, _, ok = spiralsector.InverseHeldResult(t.Context(), blank, baseVerts, 0.001, newHeld(), aboutY, recY)
		require.False(t, ok, "a record that does not fix the X axis")

		shift, err := r3.Translation(r3.Vec{Y: 1})
		require.NoError(t, err)
		moved, err := rot.Then(shift)
		require.NoError(t, err)
		_, _, _, _, ok = spiralsector.InverseHeldResult(t.Context(), blank, baseVerts, 0.001, newHeld(), moved, rec90)
		require.False(t, ok, "a move with a translation")

		huge := newHeld()
		huge.VertexBound[0] = math.MaxFloat64
		_, _, _, _, ok = spiralsector.InverseHeldResult(t.Context(), blank, baseVerts, 0.001, huge, rot, rec90)
		require.False(t, ok, "a non-finite charge")
	})
}

// TestInverseHeldResultBetaCoversExactTranspose checks, in exact rationals, that
// each returned vertex lies within its returned bound of Q^T applied to the
// held vertex, for a recorded-style angle whose float inverse is not exact.
func TestInverseHeldResultBetaCoversExactTranspose(t *testing.T) {
	t.Parallel()

	rec, move := axisRecord(t, r3.Vec{X: 1}, units.Radians(2*math.Pi*23/31))
	baseVerts, baseTris := inverseTetra(r3.Vec{})
	blank := &meshbool.BoolMesh{Verts: baseVerts, Tris: baseTris, VertexBound: make([]float64, 4)}
	heldVerts := []r3.Vec{{X: 3, Y: 11.5, Z: -7.25}, {X: -2, Y: 0.1, Z: 9}, {X: 0.5, Y: -8, Z: 0.3}, {X: 1, Y: 1, Z: 1}}
	held := &meshbool.StitchedMesh{Verts: heldVerts, VertexBound: []float64{0.01, 0, 0.02, 0.01}, Round: 1e-9}
	verts, beta, round, radial, ok := spiralsector.InverseHeldResult(t.Context(), blank, baseVerts, 0.001, held, move, rec)
	require.True(t, ok)
	require.GreaterOrEqual(t, round, held.Round)
	require.Positive(t, radial, "the float inverse of this angle is not exactly the transpose")

	q := rec.Linear()
	ratOf := func(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }
	for i, v := range heldVerts {
		in := [3]*big.Rat{ratOf(v.X), ratOf(v.Y), ratOf(v.Z)}
		var want [3]*big.Rat
		for r := range want {
			want[r] = new(big.Rat)
			for c := range in {
				// Q^T row r is column r of Q.
				want[r].Add(want[r], new(big.Rat).Mul(q[c][r], in[c]))
			}
		}
		got := [3]*big.Rat{ratOf(verts[i].X), ratOf(verts[i].Y), ratOf(verts[i].Z)}
		require.Zero(t, got[0].Cmp(in[0]), "X is kept exactly")
		sq := new(big.Rat)
		for r := range got {
			d := new(big.Rat).Sub(got[r], want[r])
			sq.Add(sq, d.Mul(d, d))
		}
		extra := new(big.Rat).Sub(ratOf(beta[i]), ratOf(held.VertexBound[i]))
		require.GreaterOrEqual(t, extra.Sign(), 0)
		// |got - Q^T v| <= extra, compared squared.
		require.LessOrEqual(t, sq.Cmp(new(big.Rat).Mul(extra, extra)), 0, "vertex %d", i)
	}
}

// TestInverseHeldResultGapCoversMoveDefect pairs an exact record with a float
// move that is off the record's rotation by far more than rounding, at vertices
// far from the axis, so the gap charge dominates the rounding allowance. Each
// returned vertex must still lie within its bound of the exact Q^T applied to
// the held vertex.
func TestInverseHeldResultGapCoversMoveDefect(t *testing.T) {
	t.Parallel()

	angle := 2 * math.Pi * 23 / 31
	rec, _ := axisRecord(t, r3.Vec{X: 1}, units.Radians(angle))
	_, off := axisRecord(t, r3.Vec{X: 1}, units.Radians(angle+1e-6))
	baseVerts, baseTris := inverseTetra(r3.Vec{})
	blank := &meshbool.BoolMesh{Verts: baseVerts, Tris: baseTris, VertexBound: make([]float64, 4)}
	heldVerts := []r3.Vec{{X: 3, Y: 4e3, Z: -3e3}, {X: -2, Y: -5e3, Z: 1e3}, {X: 0.5, Y: 2e3, Z: 6e3}, {X: 1, Y: 1, Z: 1}}
	held := &meshbool.StitchedMesh{Verts: heldVerts, VertexBound: []float64{0.01, 0, 0.02, 0.01}, Round: 1e-9}
	verts, beta, _, _, ok := spiralsector.InverseHeldResult(t.Context(), blank, baseVerts, 0.001, held, off, rec)
	require.True(t, ok)

	q := rec.Linear()
	ratOf := func(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }
	for i, v := range heldVerts {
		in := [3]*big.Rat{ratOf(v.X), ratOf(v.Y), ratOf(v.Z)}
		got := [3]*big.Rat{ratOf(verts[i].X), ratOf(verts[i].Y), ratOf(verts[i].Z)}
		sq := new(big.Rat)
		for r := range got {
			want := new(big.Rat)
			for c := range in {
				want.Add(want, new(big.Rat).Mul(q[c][r], in[c]))
			}
			d := new(big.Rat).Sub(got[r], want)
			sq.Add(sq, d.Mul(d, d))
		}
		extra := new(big.Rat).Sub(ratOf(beta[i]), ratOf(held.VertexBound[i]))
		require.GreaterOrEqual(t, extra.Sign(), 0)
		require.LessOrEqual(t, sq.Cmp(new(big.Rat).Mul(extra, extra)), 0, "vertex %d", i)
	}
}
