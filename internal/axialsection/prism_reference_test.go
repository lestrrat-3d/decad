package axialsection_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/axialsection"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

func TestAsReference(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	prism := axialsection.Operand{
		Frame0: frame, Frame1: frame, Xform: r3.Identity(),
		Level0: big.NewRat(2, 1), Level1: big.NewRat(5, 1),
	}

	t.Run("an unplaced prism records the raised planes", func(t *testing.T) {
		ref, ok := prism.AsReference()
		require.True(t, ok)
		require.True(t, ref.FromLoft)
		require.Equal(t, r3.Vec{Z: 2}, ref.Frame0.Origin())
		require.Equal(t, r3.Vec{Z: 5}, ref.Frame1.Origin())
		require.Equal(t, sectionrecord.PlaneRecord{Origin: r3.Vec{Z: 2}, U: r3.Vec{X: 1}, V: r3.Vec{Y: 1}}, ref.Plane0)
		require.Equal(t, 0, ref.Level0.Sign())
		require.Zero(t, ref.Level1.Cmp(big.NewRat(3, 1)))
		require.Zero(t, ref.Lateral)
	})

	t.Run("a placed prism is not a reference", func(t *testing.T) {
		turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Radians(0.5))
		require.NoError(t, err)
		placed := prism
		placed.Xform = turn
		_, ok := placed.AsReference()
		require.False(t, ok)
	})

	t.Run("a flat sweep is not a reference", func(t *testing.T) {
		flat := prism
		flat.Level1 = flat.Level0
		_, ok := flat.AsReference()
		require.False(t, ok)
	})

	t.Run("a loft operand is returned unchanged", func(t *testing.T) {
		loft := prism
		loft.FromLoft = true
		got, ok := loft.AsReference()
		require.True(t, ok)
		require.Equal(t, loft, got)
	})
}
