package facetproof_test

import (
	"math"
	"math/big"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The rational reference checks that the dyadic path publishes the same
// outward-rounded box error, including cancellation near an extreme.
func TestFacetedExtremeErrorMatchesRationalReference(t *testing.T) {
	near := math.Nextafter(1, 0)
	verts := []r3.Vec{{X: 0}, {X: 1}, {X: near, Y: 0.5}}
	beta := []float64{0, 0, math.SmallestNonzeroFloat64}
	reach := []float64{0, 0, math.Ldexp(1, -52)}
	got, err := facetproof.FacetedExtremeError(proofbound.NewWorkBudget(t.Context()),
		verts, beta, reach, r3.Vec{}, r3.Vec{X: 1, Y: 0.5})
	require.NoError(t, err)
	require.Equal(t, rationalExtremeError(verts, beta, reach,
		r3.Vec{}, r3.Vec{X: 1, Y: 0.5}), got)

	rng := rand.New(rand.NewSource(17))
	for range 80 {
		count := 3 + rng.Intn(8)
		verts := make([]r3.Vec, count)
		beta, reach := make([]float64, count), make([]float64, count)
		lo := r3.Vec{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
		hi := r3.Vec{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)}
		for i := range verts {
			coordinate := func() float64 {
				return math.Ldexp(2*rng.Float64()-1, rng.Intn(80)-40)
			}
			verts[i] = r3.Vec{X: coordinate(), Y: coordinate(), Z: coordinate()}
			beta[i] = math.Ldexp(rng.Float64(), rng.Intn(35)-35)
			reach[i] = math.Ldexp(rng.Float64(), rng.Intn(35)-35)
			lo.X, lo.Y, lo.Z = min(lo.X, verts[i].X), min(lo.Y, verts[i].Y), min(lo.Z, verts[i].Z)
			hi.X, hi.Y, hi.Z = max(hi.X, verts[i].X), max(hi.Y, verts[i].Y), max(hi.Z, verts[i].Z)
		}
		got, err := facetproof.FacetedExtremeError(proofbound.NewWorkBudget(t.Context()),
			verts, beta, reach, lo, hi)
		require.NoError(t, err)
		require.Equal(t, rationalExtremeError(verts, beta, reach, lo, hi), got)
	}
}

func rationalExtremeError(verts []r3.Vec, beta, reach []float64, lo, hi r3.Vec) float64 {
	coordinates := func(v r3.Vec) [3]float64 { return [3]float64{v.X, v.Y, v.Z} }
	var worst big.Rat
	for axis := range 3 {
		low := new(big.Rat).SetFloat64(coordinates(lo)[axis])
		high := new(big.Rat).SetFloat64(coordinates(hi)[axis])
		var trueLow, trueHigh, heldLow, heldHigh *big.Rat
		for i, vertex := range verts {
			v := new(big.Rat).SetFloat64(coordinates(vertex)[axis])
			b := new(big.Rat).SetFloat64(reach[i])
			own := new(big.Rat).SetFloat64(beta[i])
			if candidate := new(big.Rat).Sub(v, b); trueLow == nil || candidate.Cmp(trueLow) < 0 {
				trueLow = candidate
			}
			if candidate := new(big.Rat).Add(v, b); trueHigh == nil || candidate.Cmp(trueHigh) > 0 {
				trueHigh = candidate
			}
			if candidate := new(big.Rat).Add(v, own); heldLow == nil || candidate.Cmp(heldLow) < 0 {
				heldLow = candidate
			}
			if candidate := new(big.Rat).Sub(v, own); heldHigh == nil || candidate.Cmp(heldHigh) > 0 {
				heldHigh = candidate
			}
		}
		for _, gap := range []*big.Rat{
			new(big.Rat).Sub(low, trueLow), new(big.Rat).Sub(heldLow, low),
			new(big.Rat).Sub(trueHigh, high), new(big.Rat).Sub(high, heldHigh),
		} {
			if gap.Cmp(&worst) > 0 {
				worst.Set(gap)
			}
		}
	}
	if worst.Sign() == 0 {
		return 0
	}
	f, _ := worst.Float64()
	return proofbound.ProvenUpRound(f)
}
