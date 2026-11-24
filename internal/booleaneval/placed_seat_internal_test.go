package booleaneval

import (
	"math"
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

// TestRatPlaneHoldsExact checks the exact corner test of a placed plane image against n·(e − o) = 0 read directly:
// corners on the image hold, a corner one ulp off does not, and a vertex shared by two triangles reads the same in
// both.
func TestRatPlaneHoldsExact(t *testing.T) {
	t.Parallel()
	linear, err := massmoment.PlacementRotation(r3.Identity())
	require.NoError(t, err)
	shift := [3]*big.Rat{big.NewRat(-2, 1), big.NewRat(3, 1), big.NewRat(1, 4)}
	// The plane x = 1 maps to x = 1 - 2 = -1 under the shift.
	p := ratPlane{o: [3]*big.Rat{big.NewRat(1, 1), new(big.Rat), new(big.Rat)}, n: [3]*big.Rat{big.NewRat(1, 1), new(big.Rat), new(big.Rat)}}
	img := p.image(linear, shift)
	require.NotNil(t, img)
	off := math.Nextafter(-1, 0)
	verts := []r3.Vec{r3.NewVec(-1, 0, 0), r3.NewVec(-1, 5, 2), r3.NewVec(-1, -3, 7), r3.NewVec(off, 1, 1)}
	direct := func(tri [3]int) bool {
		for _, v := range tri {
			e, ok := massmoment.ExactVec(verts[v])
			require.True(t, ok)
			s := new(big.Rat)
			for i := range 3 {
				s.Add(s, new(big.Rat).Mul(img.n[i], new(big.Rat).Sub(e[i], img.o[i])))
			}
			if s.Sign() != 0 {
				return false
			}
		}
		return true
	}
	exact := exactVerts{verts: verts}
	for _, tc := range []struct {
		tri  [3]int
		want bool
	}{{[3]int{0, 1, 2}, true}, {[3]int{0, 1, 3}, false}, {[3]int{2, 1, 0}, true}, {[3]int{3, 2, 1}, false}} {
		require.Equal(t, tc.want, direct(tc.tri), "premise %v", tc.tri)
		require.Equal(t, tc.want, img.holdsExact(&exact, tc.tri), "%v", tc.tri)
	}
}
