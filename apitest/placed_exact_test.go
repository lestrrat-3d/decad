package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-boolean-design.md B9's tests: a placement whose vertex images are floats
// charges no rounding, so a placed mesh solid stays on the exact grid routes.

func TestPlacedMeshSolidByAWholeTranslationStaysExact(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	block := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
	move, err := r3.Translation(r3.NewVec(100, -3, 7))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), block, move)
	require.NoError(t, err)
	for _, v := range placed.Vertices() {
		require.Zero(t, v.Position().Bound.Mag(), "the vertex at %v pays no rounding", v.Position().Value)
	}
	requireVolume(t, placed, 480)
	box, err := placed.Bounds()
	require.NoError(t, err)
	require.True(t, box.Min.Equal(r3.NewVec(100, -3, 7), 1e-12))
	require.True(t, box.Max.Equal(r3.NewVec(110, 5, 13), 1e-12))
	// It takes the exact operations a mesh solid takes.
	longer, err := decad.Stretch(t.Context(), placed, r3.NewVec(100, -3, 7), units.Scalar(2), units.Scalar(1), units.Scalar(1))
	require.NoError(t, err)
	requireVolume(t, longer, 960)
}

func TestPlacedCopiesOfAMeshSolidJoinExactly(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	bar := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 4, 4))
	copyOf, err := decad.Copy(t.Context(), bar)
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(10, 0, 0))
	require.NoError(t, err)
	shifted, err := decad.Transform(t.Context(), copyOf, move)
	require.NoError(t, err)
	// The copy, moved one bar's length along, touches the original at a face: the grid joins them into one bar.
	joined, err := decad.Union(t.Context(), bar, shifted)
	require.NoError(t, err)
	requireVolume(t, joined, 320)
	require.Len(t, joined.Faces(), 6)
	require.Len(t, doc.Bodies(), 1)
}

func TestPlacementByARoundedMotionStillPaysItsRounding(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	block := blockIn(t, doc, r3.NewVec(1, 1, 1), r3.NewVec(11, 9, 7))
	turn, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Degrees(30))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), block, turn)
	require.NoError(t, err)
	paid := false
	for _, v := range placed.Vertices() {
		paid = paid || v.Position().Bound.Mag() > 0
	}
	require.True(t, paid, "a 30 degree turn leaves vertex images that are not floats")
	// And the sound bound still holds: the volume is the block's within its bound.
	volume, err := placed.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-480), volume.Bound.Base()+1e-9)
}
