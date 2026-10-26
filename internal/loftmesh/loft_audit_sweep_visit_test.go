package loftmesh

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// sweepVisitBoxes returns n random boxes spread along x, each overlapping a
// few neighbours.
func sweepVisitBoxes(rng *rand.Rand, n int) [][2]r3.Vec {
	boxes := make([][2]r3.Vec, n)
	for i := range boxes {
		lo := r3.Vec{X: rng.Float64() * float64(n) / 4, Y: rng.Float64() * 20, Z: rng.Float64() * 20}
		ext := r3.Vec{X: rng.Float64() * 3, Y: rng.Float64() * 6, Z: rng.Float64() * 6}
		boxes[i] = [2]r3.Vec{lo, lo.Add(ext)}
	}
	return boxes
}

// referenceSweepVisit is the box-loading inner loop visit replaced: it reads
// the sweep-axis bounds from the boxes on every scanned pair.
func referenceSweepVisit(s sweepOrder, limit uint64) ([][2]int, uint64, bool) {
	var pairs [][2]int
	scanned := uint64(0)
	for a, ia := range s.order {
		boxA := s.boxes[ia]
		_, hiA := boxAxis(boxA, s.axis)
		for _, ib := range s.order[a+1:] {
			boxB := s.boxes[ib]
			if loB, _ := boxAxis(boxB, s.axis); loB > hiA {
				break
			}
			scanned++
			if scanned > limit {
				return pairs, scanned, true
			}
			if !meshbool.BoxesOverlap(boxA, boxB) {
				continue
			}
			i, j := int(ia), int(ib)
			if i > j {
				i, j = j, i
			}
			pairs = append(pairs, [2]int{i, j})
		}
	}
	return pairs, scanned, false
}

func TestSweepVisitMatchesReference(t *testing.T) {
	rng := rand.New(rand.NewPCG(7, 11))
	for trial := range 40 {
		boxes := sweepVisitBoxes(rng, 50+rng.IntN(400))
		var members []int
		for i := range boxes {
			if trial%2 == 0 || rng.IntN(3) != 0 {
				members = append(members, i)
			}
		}
		s := newSweepOrder(boxes, members)
		for _, limit := range []uint64{^uint64(0), 0, 100} {
			want, wantScanned, wantExceeded := referenceSweepVisit(s, limit)
			var got [][2]int
			scanned, exceeded, err := s.visit(proofbound.NewWorkBudget(t.Context()), limit, func(i, j int) {
				got = append(got, [2]int{i, j})
			})
			require.NoError(t, err)
			require.Equal(t, wantScanned, scanned, "trial %d limit %d scanned", trial, limit)
			require.Equal(t, wantExceeded, exceeded, "trial %d limit %d exceeded", trial, limit)
			require.Equal(t, len(want), len(got), "trial %d limit %d pair count", trial, limit)
			for k := range want {
				require.Equal(t, want[k], got[k], "trial %d limit %d pair %d", trial, limit, k)
			}
		}
	}
}

func BenchmarkSweepOrder(b *testing.B) {
	rng := rand.New(rand.NewPCG(3, 5))
	boxes := sweepVisitBoxes(rng, 4000)
	members := make([]int, len(boxes))
	for i := range members {
		members[i] = i
	}
	b.ReportAllocs()
	b.ResetTimer()
	for range b.N {
		newSweepOrder(boxes, members)
	}
}
