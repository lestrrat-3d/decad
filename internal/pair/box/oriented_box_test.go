package box_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func TestOrientedInteriorWitnessRequiresStrictErodedOverlap(t *testing.T) {
	unitBox := func(start int64) box.OrientedBox {
		var result box.OrientedBox
		for axis := range 3 {
			result.Edge[axis][axis] = proof.DyInt(2)
		}
		for index := range result.Corner {
			for axis := range 3 {
				origin := int64(0)
				if axis == 0 {
					origin = start
				}
				result.Corner[index][axis] = proof.DyInt(origin + 2*int64((index>>axis)&1))
			}
		}
		return result
	}
	a, b := unitBox(0), unitBox(1)
	quarter, half := big.NewRat(1, 4), big.NewRat(1, 2)
	require.True(t, box.OrientedInteriorWitness(a, b, quarter, quarter))
	require.True(t, box.OrientedInteriorWitness(b, a, quarter, quarter))
	require.False(t, box.OrientedInteriorWitness(a, b, half, half))
}
