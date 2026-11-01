package box_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func TestClosestOrthogonalBoxPoint(t *testing.T) {
	orthogonalBox := func() box.OrientedBox {
		result := box.OrientedBox{
			Edge: [3]proof.DyV3{
				{proof.DyInt(2), proof.DyZero(), proof.DyZero()},
				{proof.DyZero(), proof.DyInt(3), proof.DyZero()},
				{proof.DyZero(), proof.DyZero(), proof.DyInt(4)},
			},
		}
		for index := range result.Corner {
			for axis := range 3 {
				result.Corner[index][axis] = proof.DyInt(int64((index>>axis)&1) * []int64{2, 3, 4}[axis])
			}
		}
		return result
	}
	point := func(x, y, z int64) proof.DyV3 {
		return proof.DyV3{proof.DyInt(x), proof.DyInt(y), proof.DyInt(z)}
	}
	cases := []struct {
		name   string
		center proof.DyV3
		want   int64
	}{
		{"interior", point(1, 1, 2), 0},
		{"face", point(-1, 1, 2), 1},
		{"edge", point(-1, 4, 2), 2},
		{"corner", point(-1, 4, 6), 6},
		{"endpoint equality", point(0, 1, 2), 0},
		{"tie between corners", point(-1, -1, 2), 2},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got, ok := box.ClosestOrthogonalBoxPoint(tc.center, orthogonalBox())
			require.True(t, ok)
			require.Equal(t, big.NewRat(tc.want, 1), got)
		})
	}
}

func TestClosestOrthogonalBoxPointRotated(t *testing.T) {
	oriented := box.OrientedBox{
		Edge: [3]proof.DyV3{
			{proof.DyInt(2), proof.DyInt(2), proof.DyZero()},
			{proof.DyInt(-3), proof.DyInt(3), proof.DyZero()},
			{proof.DyZero(), proof.DyZero(), proof.DyInt(4)},
		},
	}
	oriented.Corner[0] = proof.DyV3{proof.DyInt(5), proof.DyInt(-2), proof.DyInt(1)}
	for index := range oriented.Corner {
		for axis := range 3 {
			if index&(1<<axis) != 0 {
				oriented.Corner[index] = proof.DvAdd(oriented.Corner[index], oriented.Edge[axis])
			}
		}
		if index != 0 {
			oriented.Corner[index] = proof.DvAdd(oriented.Corner[index], oriented.Corner[0])
		}
	}
	require.True(t, box.OrthogonalSourceBox(oriented))
	cases := []struct {
		name   string
		center proof.DyV3
		want   int64
	}{
		{"interior rational projection", proof.DyV3{proof.DyInt(4), proof.DyZero(), proof.DyInt(3)}, 0},
		{"face", proof.DyV3{proof.DyInt(3), proof.DyInt(-2), proof.DyInt(3)}, 2},
		{"edge", proof.DyV3{proof.DyInt(-2), proof.DyInt(3), proof.DyInt(3)}, 20},
		{"corner", proof.DyV3{proof.DyInt(-2), proof.DyInt(3), proof.DyInt(9)}, 36},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got, ok := box.ClosestOrthogonalBoxPoint(tc.center, oriented)
			require.True(t, ok)
			require.Equal(t, big.NewRat(tc.want, 1), got)
		})
	}
}

func TestClosestOrthogonalBoxPointRejectsDegenerateEdge(t *testing.T) {
	var degenerate box.OrientedBox
	degenerate.Edge[0][0] = proof.DyInt(1)
	degenerate.Edge[1][1] = proof.DyInt(1)
	got, ok := box.ClosestOrthogonalBoxPoint(proof.DyV3{}, degenerate)
	require.False(t, ok)
	require.Nil(t, got)
}

func TestOrthogonalSourceBoxRejectsSkewEdges(t *testing.T) {
	var skew box.OrientedBox
	skew.Edge[0] = proof.DyV3{proof.DyInt(1), proof.DyZero(), proof.DyZero()}
	skew.Edge[1] = proof.DyV3{proof.DyInt(1), proof.DyInt(1), proof.DyZero()}
	skew.Edge[2] = proof.DyV3{proof.DyZero(), proof.DyZero(), proof.DyInt(1)}
	require.False(t, box.OrthogonalSourceBox(skew))
}
