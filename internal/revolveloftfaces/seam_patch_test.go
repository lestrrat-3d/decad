package revolveloftfaces

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftface"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestSeamHeldPointBoundsNonDyadicRounding(t *testing.T) {
	exact := loftface.RatVec{big.NewRat(1, 10), new(big.Rat), new(big.Rat)}
	held, bound, ok := HeldPoint(exact)
	require.True(t, ok)
	require.Positive(t, bound)
	delta := new(big.Rat).Sub(proofarith.FloatRat(held.X), exact[0])
	delta.Abs(delta)
	require.LessOrEqual(t, delta.Cmp(proofarith.FloatRat(bound)), 0)
	require.Equal(t, 0.0, held.Y)
	require.Equal(t, 0.0, held.Z)

	_, _, ok = HeldPoint(loftface.RatVec{nil, new(big.Rat), new(big.Rat)})
	require.False(t, ok)
}

func TestSeamMeridianPointPreservesRadialRay(t *testing.T) {
	seam := loftface.RatVec{big.NewRat(1, 1), big.NewRat(3, 1), big.NewRat(4, 1)}
	point, bound, ok := meridianPoint(seam, sectionrecord.Point2{U: 7, V: 10})
	require.True(t, ok)
	require.Equal(t, r3.Vec{X: 7, Y: 6, Z: 8}, point)
	require.Zero(t, bound)

	_, _, ok = meridianPoint(seam, sectionrecord.Point2{U: 7, V: 0})
	require.False(t, ok)
}
