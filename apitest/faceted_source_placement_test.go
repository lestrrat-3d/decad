package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// Increment 4 of agent-docs/faceted-source-retention-design.md §3.4: a placed
// Boolean result keeps a usable normal bound on every retained face.

func TestFacetedSourcePlacement(t *testing.T) {
	t.Parallel()

	drill := func(t *testing.T) *decad.Body {
		t.Helper()
		sc := newRetentionScene()
		drilled, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
		require.NoError(t, err)
		return drilled
	}
	type reading struct {
		at    r3.Vec
		value r3.Vec
		bound float64
	}
	// read takes each face's normal at its own first vertex, or, given at, at
	// move applied to at[i]: the image of the unplaced reading's point.
	read := func(t *testing.T, b *decad.Body, at []r3.Vec, move *r3.Transform) []reading {
		t.Helper()
		var out []reading
		for i, f := range b.Faces() {
			p := firstVertex(t, f)
			if at != nil {
				p = move.Apply(at[i])
			}
			n, err := f.NormalAt(p)
			require.NoError(t, err)
			out = append(out, reading{at: p, value: n.Value, bound: n.Bound.Base()})
		}
		return out
	}

	t.Run("a translated result reports a bound within rounding of the unplaced one", func(t *testing.T) {
		t.Parallel()
		drilled := drill(t)
		move, err := r3.Translation(r3.NewVec(7, -3, 100))
		require.NoError(t, err)
		placed, err := decad.Transform(t.Context(), drilled, move)
		require.NoError(t, err)
		before := read(t, drilled, nil, nil)
		var points []r3.Vec
		for _, r := range before {
			points = append(points, r.at)
		}
		after := read(t, placed, points, &move)
		require.Len(t, after, len(before))
		for i := range after {
			require.LessOrEqual(t, after[i].bound, before[i].bound+1e-9, "face %d", i)
			// The motion is a translation, so the normal itself is unchanged.
			require.Less(t, after[i].value.Sub(before[i].value).Len(), after[i].bound+before[i].bound+1e-12, "face %d", i)
		}
	})

	t.Run("a rotated result reports a bound within rounding of the unplaced one that covers the rotated normal", func(t *testing.T) {
		t.Parallel()
		drilled := drill(t)
		turn, err := r3.RotationAround(r3.NewVec(20, 10, 0), r3.NewVec(0, 0, 1), units.Degrees(20))
		require.NoError(t, err)
		placed, err := decad.Transform(t.Context(), drilled, turn)
		require.NoError(t, err)
		before := read(t, drilled, nil, nil)
		var points []r3.Vec
		for _, r := range before {
			points = append(points, r.at)
		}
		after := read(t, placed, points, &turn)
		require.Len(t, after, len(before))
		for i := range after {
			require.LessOrEqual(t, after[i].bound, before[i].bound+1e-9, "face %d", i)
			// The true normal at the rotated point is the rotated normal, and
			// each reading is within its bound of the true normal at its own
			// patch, so the two readings are at most the bounds apart.
			want := turn.ApplyDir(before[i].value)
			require.LessOrEqual(t, after[i].value.Sub(want).Len(), after[i].bound+before[i].bound+1e-12, "face %d", i)
		}
	})

	t.Run("a placed retained plane is not an exact mirror plane under a rotation", func(t *testing.T) {
		t.Parallel()
		drilled := drill(t)
		turn, err := r3.RotationAround(r3.NewVec(20, 10, 0), r3.NewVec(0, 0, 1), units.Degrees(20))
		require.NoError(t, err)
		placed, err := decad.Transform(t.Context(), drilled, turn)
		require.NoError(t, err)
		wall := decad.Faces(decad.Planar(), decad.Facing(turn.ApplyDir(r3.NewVec(1, 0, 0))))
		_, err = decad.MirroredCopy(t.Context(), placed, decad.MirrorFace{Body: placed, Face: wall})
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
}
