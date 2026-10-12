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
	point := func(un, ud, vn, vd int64) meshbool.Xp2 {
		return meshbool.NewXP2(big.NewRat(un, ud), big.NewRat(vn, vd))
	}
	axisBase := point(1, 3, 2, 5)
	axisCases := [][2]meshbool.Xp2{
		{point(1, 3, 4, 5), point(1, 2, 3, 5)},
		{point(2, 3, 2, 5), point(1, 2, 3, 5)},
		{point(2, 3, 3, 5), point(1, 3, 4, 5)},
		{point(2, 3, 3, 5), point(1, 2, 2, 5)},
	}
	for i, pair := range axisCases {
		require.Equal(t, meshbool.Cross2x(axisBase, pair[0], pair[1]).Sign(),
			meshbool.Cross2xSign(axisBase, pair[0], pair[1]), "shared axis case %d", i)
	}
	epsilon := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 120))
	nearU := new(big.Rat).Add(big.NewRat(1, 3), epsilon)
	farU := new(big.Rat).Add(big.NewRat(1, 3), new(big.Rat).Mul(big.NewRat(2, 1), epsilon))
	nearV := new(big.Rat).Add(big.NewRat(6, 5), epsilon)
	nearAxisB := meshbool.NewXP2(nearU, big.NewRat(4, 5))
	nearAxisC := meshbool.NewXP2(farU, nearV)
	require.Equal(t, meshbool.Cross2x(axisBase, nearAxisB, nearAxisC).Sign(),
		meshbool.Cross2xSign(axisBase, nearAxisB, nearAxisC))
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
