package twistsweep

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestTwistShellDistanceUpperMatchesRationalFormula(t *testing.T) {
	subnormal := math.SmallestNonzeroFloat64
	maxFloat := math.MaxFloat64
	values := []struct {
		name string
		a, b r3.Vec
	}{
		{name: "zeros and signed zero", a: r3.NewVec(0, math.Copysign(0, -1), 0), b: r3.Vec{}},
		{name: "three four five", a: r3.Vec{}, b: r3.NewVec(3, 4, 0)},
		{name: "subnormals", a: r3.NewVec(subnormal, -subnormal, subnormal), b: r3.NewVec(-subnormal, subnormal, 0)},
		{name: "cancellation", a: r3.NewVec(maxFloat, maxFloat, 1), b: r3.NewVec(math.Nextafter(maxFloat, 0), maxFloat, -1)},
		{name: "mixed exponents", a: r3.NewVec(math.Ldexp(1, 500), math.Ldexp(1, -500), -7), b: r3.NewVec(-math.Ldexp(1, 450), -math.Ldexp(1, -520), 11)},
		{name: "exact square neighbors", a: r3.NewVec(math.Nextafter(3, 0), 4, 0), b: r3.NewVec(0, 0, 0)},
		{name: "distance near max", a: r3.NewVec(math.Nextafter(maxFloat, 0), 0, 0), b: r3.Vec{}},
		{name: "distance over max", a: r3.NewVec(maxFloat, maxFloat, 0), b: r3.Vec{}},
	}
	for _, tc := range values {
		t.Run(tc.name, func(t *testing.T) {
			require.Equal(t, math.Float64bits(oldTwistShellDistanceUpper(tc.a, tc.b)),
				math.Float64bits(twistShellDistanceUpper(tc.a, tc.b)))
		})
	}
}

func TestTwistShellDistanceUpperRefusesNonFiniteCoordinates(t *testing.T) {
	for _, coordinate := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
		for slot := range 6 {
			values := [6]float64{}
			values[slot] = coordinate
			require.Nil(t, proof.RatSquaredDistance3(values[0], values[1], values[2],
				values[3], values[4], values[5]))
			a := r3.NewVec(values[0], values[1], values[2])
			b := r3.NewVec(values[3], values[4], values[5])
			require.True(t, math.IsInf(twistShellDistanceUpper(a, b), 1), "coordinate slot %d", slot)
		}
	}
}

func oldTwistShellDistanceUpper(a, b r3.Vec) float64 {
	dx := new(big.Rat).Sub(TwistRat(a.X), TwistRat(b.X))
	dy := new(big.Rat).Sub(TwistRat(a.Y), TwistRat(b.Y))
	dz := new(big.Rat).Sub(TwistRat(a.Z), TwistRat(b.Z))
	squared := new(big.Rat).Add(new(big.Rat).Add(
		new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy)), new(big.Rat).Mul(dz, dz))
	return proofbound.RatSqrtUp(squared)
}

var benchmarkTwistShellDistanceUpperResult float64

func BenchmarkTwistShellDistanceUpper(b *testing.B) {
	pairs := [...]struct{ a, b r3.Vec }{
		{a: r3.NewVec(8.2, 1.1, 3.75), b: r3.NewVec(7.6, 1.8, 3.125)},
		{a: r3.NewVec(-4.3, 6.1, 1.25), b: r3.NewVec(-3.7, 5.5, 1.875)},
		{a: r3.NewVec(2.25, -8.5, 9.75), b: r3.NewVec(2.875, -7.9, 9.125)},
	}
	b.ReportAllocs()
	b.ResetTimer()
	var result float64
	for i := range b.N {
		pair := pairs[i%len(pairs)]
		result = twistShellDistanceUpper(pair.a, pair.b)
	}
	benchmarkTwistShellDistanceUpperResult = result
}
