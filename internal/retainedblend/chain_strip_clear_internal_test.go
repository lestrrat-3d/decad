package retainedblend

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestBoxGridKeepsEveryMeetingPair(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(1, 2))
	box := func(scale float64) facetBox {
		lo := r3.NewVec(rng.Float64()*10, rng.Float64()*10, rng.Float64()*10)
		return facetBox{lo: lo, hi: lo.Add(r3.NewVec(rng.Float64()*scale, rng.Float64()*scale, rng.Float64()*scale))}
	}
	strip := make([]facetBox, 500)
	for i := range strip {
		strip[i] = box(0.5)
	}
	grid := newBoxGrid(strip)
	var near []int
	pairs := 0
	for range 2000 {
		b := box(2)
		near = grid.near(b, near[:0])
		var want []int
		for i, s := range strip {
			if b.meets(s) {
				want = append(want, i)
			}
		}
		var got []int
		for _, i := range near {
			if b.meets(strip[i]) {
				got = append(got, i)
			}
		}
		require.Equal(t, want, got)
		pairs += len(want)
	}
	require.Positive(t, pairs)
}
