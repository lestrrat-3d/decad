package meshbool_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func TestCross2xSignMatchesExactValue(t *testing.T) {
	for n := int64(1); n <= 64; n++ {
		points := [3]meshbool.Xp2{
			meshbool.NewXP2(big.NewRat(3*n+1, 7*n+2), big.NewRat(5*n-2, 11*n+3)),
			meshbool.NewXP2(big.NewRat(2*n-5, 13*n+1), big.NewRat(7*n+4, 17*n+2)),
			meshbool.NewXP2(big.NewRat(9*n-1, 19*n+3), big.NewRat(4*n+2, 23*n+4)),
		}
		require.Equal(t, meshbool.Cross2x(points[0], points[1], points[2]).Sign(),
			meshbool.Cross2xSign(points[0], points[1], points[2]), "rational case %d", n)
	}
	base := meshbool.NewXP2(big.NewRat(1, 3), big.NewRat(2, 5))
	step := meshbool.NewXP2(big.NewRat(2, 3), big.NewRat(4, 5))
	near := new(big.Rat).Add(big.NewRat(6, 5), new(big.Rat).SetFrac(big.NewInt(1),
		new(big.Int).Lsh(big.NewInt(1), 120)))
	for _, endV := range []*big.Rat{big.NewRat(6, 5), near,
		new(big.Rat).Sub(big.NewRat(6, 5), new(big.Rat).Sub(near, big.NewRat(6, 5)))} {
		end := meshbool.NewXP2(big.NewRat(1, 1), endV)
		require.Equal(t, meshbool.Cross2x(base, step, end).Sign(), meshbool.Cross2xSign(base, step, end))
	}
	hom := [3]meshbool.Xp2{
		meshbool.NewXP2FromXpt(proof.Xpt{X: big.NewInt(1), Y: big.NewInt(2), Z: big.NewInt(0), W: big.NewInt(3)}, 0, 1),
		meshbool.NewXP2FromXpt(proof.Xpt{X: big.NewInt(4), Y: big.NewInt(5), Z: big.NewInt(0), W: big.NewInt(7)}, 0, 1),
		meshbool.NewXP2FromXpt(proof.Xpt{X: big.NewInt(8), Y: big.NewInt(9), Z: big.NewInt(0), W: big.NewInt(11)}, 0, 1),
	}
	require.Equal(t, meshbool.Cross2x(hom[0], hom[1], hom[2]).Sign(),
		meshbool.Cross2xSign(hom[0], hom[1], hom[2]))
	require.Equal(t, meshbool.Cross2x(hom[0], hom[1], base).Sign(),
		meshbool.Cross2xSign(hom[0], hom[1], base))
}
