package massmoment_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestTransformVolumeMomentsIsTheImage(t *testing.T) {
	r := [3][3]*big.Rat{
		{big.NewRat(3, 5), big.NewRat(-4, 5), new(big.Rat)},
		{big.NewRat(4, 5), big.NewRat(3, 5), new(big.Rat)},
		{new(big.Rat), new(big.Rat), big.NewRat(1, 1)},
	}
	scale := new(big.Rat).Add(big.NewRat(1, 1), big.NewRat(1, 1<<20))
	var transform [3][3]*big.Rat
	for i := range transform {
		for j := range transform[i] {
			transform[i][j] = new(big.Rat).Mul(scale, r[i][j])
		}
	}
	det := new(big.Rat).Mul(scale, new(big.Rat).Mul(scale, scale))
	box := massmoment.Moments{Volume: proofbound.PointInterval(big.NewRat(48, 1))}
	for i, half := range []int64{1, 2, 3} {
		box.First[i] = proofbound.PointInterval(new(big.Rat))
		for j := range box.Second[i] {
			box.Second[i][j] = proofbound.PointInterval(new(big.Rat))
		}
		box.Second[i][i] = proofbound.PointInterval(big.NewRat(48*half*half, 3))
	}
	local := massmoment.Shift(box, [3]*big.Rat{big.NewRat(1, 1), big.NewRat(2, 1), big.NewRat(3, 1)})
	exact := func(iv proofbound.RatInterval) *big.Rat {
		require.Zero(t, iv.Lo.Cmp(iv.Hi))
		return iv.Lo
	}
	contains := func(iv proofbound.RatInterval, want *big.Rat) {
		t.Helper()
		require.True(t, iv.Lo.Cmp(want) <= 0 && want.Cmp(iv.Hi) <= 0)
	}

	got := massmoment.Transform(local, transform)
	contains(got.Volume, new(big.Rat).Mul(det, exact(local.Volume)))
	for i := range 3 {
		first := new(big.Rat)
		for k := range 3 {
			first.Add(first, new(big.Rat).Mul(transform[i][k], exact(local.First[k])))
		}
		contains(got.First[i], first.Mul(first, det))
		for j := range 3 {
			second := new(big.Rat)
			for k := range 3 {
				for l := range 3 {
					term := new(big.Rat).Mul(transform[i][k], transform[j][l])
					second.Add(second, term.Mul(term, exact(local.Second[k][l])))
				}
			}
			contains(got.Second[i][j], second.Mul(second, det))
		}
	}
}
