package proof_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func requireSameDeterminant(t *testing.T, a, b, c r3.Vec, label string) {
	t.Helper()
	got := proof.FloatDeterminant(a, b, c)
	legacy := proof.LegacyFloatDeterminant(a, b, c)
	general := proof.DvDot(proof.DyVec(a), proof.DvCross(proof.DyVec(b), proof.DyVec(c)))
	require.Zero(t, proof.DyCmp(got, legacy), "%s: differs from the previous implementation", label)
	require.Zero(t, proof.DyCmp(got, general), "%s: differs from the general dyadic path", label)
	require.Equal(t, legacy.Exp(), got.Exp(), "%s: exponent differs from the previous implementation", label)
	require.Equal(t, legacy.Sign(), got.Sign(), "%s: sign differs", label)
	require.Zero(t, got.Mant().Cmp(legacy.Mant()), "%s: mantissa differs", label)
}

func TestFloatDeterminantMatchesPreviousAndGeneralPaths(t *testing.T) {
	t.Parallel()
	edge := []float64{
		0, math.Copysign(0, -1), 1, -1, 0.5, 3, -7.25,
		math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64, 0x1p-1022, -0x1p-1022,
		math.Float64frombits(0x000fffffffffffff), math.MaxFloat64, -math.MaxFloat64,
		0x1p-600, 0x1p600, 0x1p300, 1 + 0x1p-52, 31.7, -12.3,
	}
	rng := rand.New(rand.NewPCG(31, 37))
	pick := func() float64 { return edge[rng.IntN(len(edge))] }
	for range 6000 {
		a := r3.Vec{X: pick(), Y: pick(), Z: pick()}
		b := r3.Vec{X: pick(), Y: pick(), Z: pick()}
		c := r3.Vec{X: pick(), Y: pick(), Z: pick()}
		requireSameDeterminant(t, a, b, c, "edge")
	}
	randVec := func(span int) r3.Vec {
		f := func() float64 { return math.Ldexp(rng.Float64()-0.5, rng.IntN(span+1)-span/2) }
		return r3.Vec{X: f(), Y: f(), Z: f()}
	}
	for i := range 20000 {
		span := []int{1, 8, 60, 400, 2000}[i%5]
		a, b, c := randVec(span), randVec(span), randVec(span)
		switch i % 7 {
		case 0:
			b = a
		case 1:
			c = r3.Vec{X: 2 * a.X, Y: 2 * a.Y, Z: 2 * a.Z}
		case 2:
			c = r3.Vec{X: -a.X, Y: -a.Y, Z: -a.Z}
		case 3:
			a, b = b, a
		}
		requireSameDeterminant(t, a, b, c, "random")
	}
	for _, f := range tessellatedFacets(3000) {
		requireSameDeterminant(t, f[0], f[1], f[2], "tessellated")
	}
}

func TestFloatDeterminantCancelsToZero(t *testing.T) {
	t.Parallel()
	a, b := r3.Vec{X: 1.5, Y: -2.25, Z: 3e-5}, r3.Vec{X: 7, Y: 0.125, Z: -9e9}
	got := proof.FloatDeterminant(a, b, a)
	require.True(t, got.IsZero())
	require.Zero(t, got.Exp())
}

func TestFloatDeterminantPanicsOnNonFinite(t *testing.T) {
	t.Parallel()
	one := r3.Vec{X: 1, Y: 2, Z: 3}
	for _, bad := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
		require.Panics(t, func() { proof.FloatDeterminant(r3.Vec{X: bad}, one, one) })
		require.Panics(t, func() { proof.FloatDeterminant(one, one, r3.Vec{Z: bad}) })
	}
}
