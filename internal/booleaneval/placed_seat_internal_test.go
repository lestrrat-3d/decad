package booleaneval

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestRatPlaneImage checks the cofactor image of a plane against the image of three of its points (§17.2).
func TestRatPlaneImage(t *testing.T) {
	t.Parallel()
	turn, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(31))
	require.NoError(t, err)
	linear, err := massmoment.PlacementRotation(turn)
	require.NoError(t, err)
	shift := [3]*big.Rat{big.NewRat(1, 3), big.NewRat(-7, 2), big.NewRat(5, 1)}
	// The plane z = 6 through (0, 0, 6).
	p := ratPlane{o: [3]*big.Rat{new(big.Rat), new(big.Rat), big.NewRat(6, 1)}, n: [3]*big.Rat{new(big.Rat), new(big.Rat), big.NewRat(1, 1)}}
	img := p.image(linear, shift)
	require.NotNil(t, img)
	for _, x := range [][3]int64{{0, 0, 6}, {5, -3, 6}, {-2, 7, 6}} {
		var y [3]*big.Rat
		for i := range 3 {
			s := new(big.Rat).Set(shift[i])
			for k := range 3 {
				s.Add(s, new(big.Rat).Mul(linear[i][k], big.NewRat(x[k], 1)))
			}
			y[i] = s
		}
		d := new(big.Rat)
		for i := range 3 {
			d.Add(d, new(big.Rat).Mul(img.n[i], new(big.Rat).Sub(y[i], img.o[i])))
		}
		require.Zero(t, d.Sign(), "image of %v", x)
	}
}
