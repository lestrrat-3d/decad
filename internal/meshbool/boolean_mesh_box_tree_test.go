package meshbool

import (
	"context"
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestBooleanMeshBoxTreeMatchesNestedScan(t *testing.T) {
	boxes := make([][2]r3.Vec, 0, 257)
	for x := range 16 {
		for y := range 16 {
			for z := range 1 {
				minimum := r3.NewVec(float64(x), float64(y), float64(z))
				maximum := minimum.Add(r3.NewVec(0.75, 0.75, 0.75))
				boxes = append(boxes, [2]r3.Vec{minimum, maximum})
			}
		}
	}
	boxes = append(boxes, [2]r3.Vec{
		r3.NewVec(math.NaN(), 0, 0), r3.NewVec(1, 1, 1),
	})

	tree := newBoxTree(boxes)
	queries := [][2]r3.Vec{
		{r3.NewVec(2.5, 4.5, -1), r3.NewVec(5, 7, 1)},
		{r3.NewVec(10, 10, 0), r3.NewVec(10, 10, 0)},
		{r3.NewVec(32, 32, 32), r3.NewVec(33, 33, 33)},
	}
	for _, query := range queries {
		var got []int
		err := tree.visitOverlaps(query, t.Context().Err, func(facet int) error {
			got = append(got, facet)
			return nil
		})
		require.NoError(t, err)
		sort.Ints(got)

		var want []int
		for i, box := range boxes {
			if BoxesOverlap(query, box) {
				want = append(want, i)
			}
		}
		require.Equal(t, want, got)
	}

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	require.ErrorIs(t, tree.visitOverlaps(queries[0], ctx.Err, func(int) error {
		t.Fatal("canceled query visited a facet")
		return nil
	}), context.Canceled)
}
