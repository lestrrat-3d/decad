package proofbound_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestIntervalQuoMatchesAllCorners(t *testing.T) {
	ends := []int64{-7, -3, 0, 2, 9}
	for i, lowA := range ends {
		for _, highA := range ends[i:] {
			a := proofbound.Interval(big.NewRat(lowA, 3), big.NewRat(highA, 3))
			for j, lowB := range ends {
				for _, highB := range ends[j:] {
					b := proofbound.Interval(big.NewRat(lowB, 5), big.NewRat(highB, 5))
					got, ok := proofbound.IntervalQuo(a, b)
					if lowB <= 0 && highB >= 0 {
						require.False(t, ok)
						continue
					}
					require.True(t, ok)
					corners := [4]*big.Rat{
						new(big.Rat).Quo(a.Lo, b.Lo),
						new(big.Rat).Quo(a.Lo, b.Hi),
						new(big.Rat).Quo(a.Hi, b.Lo),
						new(big.Rat).Quo(a.Hi, b.Hi),
					}
					lo, hi := corners[0], corners[0]
					for _, corner := range corners[1:] {
						if corner.Cmp(lo) < 0 {
							lo = corner
						}
						if corner.Cmp(hi) > 0 {
							hi = corner
						}
					}
					require.Zero(t, got.Lo.Cmp(lo), "a=%v b=%v", a, b)
					require.Zero(t, got.Hi.Cmp(hi), "a=%v b=%v", a, b)
				}
			}
		}
	}
}
