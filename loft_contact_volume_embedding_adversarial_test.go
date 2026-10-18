package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestLoftVolumeEmbeddingRefusesNearCoplanarCrossing(t *testing.T) {
	zero, one, two := big.NewRat(0, 1), big.NewRat(1, 1), big.NewRat(2, 1)
	half := big.NewRat(1, 2)
	epsilon := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 1000))
	below := new(big.Rat).Neg(epsilon)
	points := []loftVolumeVec{
		{zero, zero, zero}, {two, zero, zero}, {zero, two, zero},
		{half, half, below}, {half, half, epsilon}, {half, one, epsilon},
	}

	// Both crossing points have z=0 at parameter 1/2 on the edges from
	// point 3. Their (x,y) coordinates are (1/2,1/2) and (1/2,3/4), strictly
	// inside x>0, y>0, x+y<2. The triangles have no common vertex ID.
	time := new(big.Rat).Quo(new(big.Rat).Neg(below), new(big.Rat).Sub(epsilon, below))
	require.Equal(t, "1/2", time.RatString())
	firstY := new(big.Rat).Add(half, new(big.Rat).Mul(time, new(big.Rat).Sub(half, half)))
	secondY := new(big.Rat).Add(half, new(big.Rat).Mul(time, new(big.Rat).Sub(one, half)))
	require.Equal(t, "1/2", firstY.RatString())
	require.Equal(t, "3/4", secondY.RatString())
	require.Negative(t, new(big.Rat).Add(half, secondY).Cmp(two))

	a := loftVolumeTriangle{ids: [3]int{0, 1, 2},
		normal:  loftVolumeVec{zero, zero, one},
		minimum: loftVolumeVec{zero, zero, zero}, maximum: loftVolumeVec{two, two, zero}}
	b := loftVolumeTriangle{ids: [3]int{3, 4, 5},
		normal:  loftVolumeVec{one, zero, zero},
		minimum: loftVolumeVec{half, half, below}, maximum: loftVolumeVec{half, one, epsilon}}
	c := &loftVolumeArithmetic{ok: true}
	allowed, err := loftVolumePairAllowed(proofbound.NewWorkBudget(t.Context()), c, points, a, b)
	require.NoError(t, err)
	require.True(t, c.ok)
	require.False(t, allowed)
}

func TestLoftVolumeEmbeddingRefusesOneVertexTJunction(t *testing.T) {
	zero, one, two := big.NewRat(0, 1), big.NewRat(1, 1), big.NewRat(2, 1)
	minusOne := big.NewRat(-1, 1)
	points := []loftVolumeVec{
		{zero, zero, zero}, {two, zero, zero}, {zero, two, zero},
		{one, zero, zero}, {zero, minusOne, zero},
	}

	// The triangles share only vertex ID 0. Their coplanar edges from 0
	// overlap on 0<x<1; (1/2,0,0) is an exact contact beyond that vertex.
	witness := big.NewRat(1, 2)
	require.Positive(t, witness.Cmp(zero))
	require.Negative(t, witness.Cmp(one))
	require.Negative(t, witness.Cmp(two))

	a := loftVolumeTriangle{ids: [3]int{0, 1, 2},
		normal:  loftVolumeVec{zero, zero, one},
		minimum: loftVolumeVec{zero, zero, zero}, maximum: loftVolumeVec{two, two, zero}}
	b := loftVolumeTriangle{ids: [3]int{0, 3, 4},
		normal:  loftVolumeVec{zero, zero, minusOne},
		minimum: loftVolumeVec{zero, minusOne, zero}, maximum: loftVolumeVec{one, zero, zero}}
	c := &loftVolumeArithmetic{ok: true}
	allowed, err := loftVolumePairAllowed(proofbound.NewWorkBudget(t.Context()), c, points, a, b)
	require.NoError(t, err)
	require.True(t, c.ok)
	require.False(t, allowed)
}
