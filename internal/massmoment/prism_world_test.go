package massmoment

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestWidenTranslationEnclosesMomentShift(t *testing.T) {
	input := massmomentPointValues()
	original := input.Second[0][1].Lo.String()
	shifted, err := WidenTranslation(input, 1.0)
	require.NoError(t, err)
	require.Equal(t, original, input.Second[0][1].Lo.String())
	require.Equal(t, shifted.Second[0][1], shifted.Second[1][0])
	for _, delta := range [][3]*big.Rat{
		{big.NewRat(1, 2), big.NewRat(-1, 3), big.NewRat(1, 1)},
		{big.NewRat(-1, 1), big.NewRat(1, 4), big.NewRat(-1, 2)},
	} {
		for i := range 3 {
			want := new(big.Rat).Add(input.First[i].Lo, new(big.Rat).Mul(input.Volume.Lo, delta[i]))
			require.True(t, shifted.First[i].Lo.Cmp(want) <= 0 && shifted.First[i].Hi.Cmp(want) >= 0)
			for j := range 3 {
				wantSecond := new(big.Rat).Set(input.Second[i][j].Lo)
				wantSecond.Add(wantSecond, new(big.Rat).Mul(delta[i], input.First[j].Lo))
				wantSecond.Add(wantSecond, new(big.Rat).Mul(input.First[i].Lo, delta[j]))
				wantSecond.Add(wantSecond, new(big.Rat).Mul(input.Volume.Lo,
					new(big.Rat).Mul(delta[i], delta[j])))
				bound := shifted.Second[i][j]
				require.True(t, bound.Lo.Cmp(wantSecond) <= 0 && bound.Hi.Cmp(wantSecond) >= 0)
			}
		}
	}
	zero, err := WidenTranslation(input, 0)
	require.NoError(t, err)
	require.Equal(t, input.First[0], zero.First[0])
	require.Equal(t, input.Second[0][1], zero.Second[0][1])
	_, err = WidenTranslation(input, -1)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}

func massmomentPointValues() Moments {
	point := func(value int64) proofbound.RatInterval { return proofbound.PointInterval(big.NewRat(value, 1)) }
	moments := Moments{Volume: point(2)}
	moments.First = [3]proofbound.RatInterval{point(1), point(-2), point(3)}
	for i := range 3 {
		for j := range 3 {
			moments.Second[i][j] = point(int64(2 + i + j))
		}
	}
	return moments
}
