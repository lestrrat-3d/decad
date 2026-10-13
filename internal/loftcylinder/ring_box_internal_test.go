package loftcylinder

import (
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestRingEdgeBoxPerToothMatchesHoisted(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(73, 79))
	coord := func() *big.Rat {
		if rng.IntN(2) == 0 {
			return proofarith.FloatRat(rng.Float64()*20 - 10)
		}
		return big.NewRat(int64(rng.IntN(2001)-1000), int64(rng.IntN(97)+1))
	}
	vec := func() RingVec { return RingVec{coord(), coord(), coord()} }
	for range 2000 {
		a, b, level := vec(), vec(), coord()
		mappedA, mappedB := a, b
		mappedA[2], mappedB[2] = level, level
		hoisted := RingEdgeBox(a, b)
		hoisted[2] = [2]float64{proofbound.RatFloatDown(level), proofbound.RatFloatUp(level)}
		require.True(t, hoisted == RingEdgeBox(mappedA, mappedB))
	}
}
