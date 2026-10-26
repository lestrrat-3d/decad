package proof_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestOrientSignExpansionMatchesExactDeterminant(t *testing.T) {
	t.Parallel()
	check := func(points [4]r3.Vec) {
		t.Helper()
		got, certain := proof.OrientSignExpansion(points[0], points[1], points[2], points[3])
		require.True(t, certain)
		want := proof.OrientSignExact(proof.XptOf(points[0]), proof.XptOf(points[1]),
			proof.XptOf(points[2]), proof.XptOf(points[3]))
		require.Equal(t, want, got)
	}
	for _, z := range []float64{math.Nextafter(0.5, 0), 0.5, math.Nextafter(0.5, 1)} {
		check([4]r3.Vec{{}, {X: 1, Z: 1}, {Y: 1, Z: 1}, {X: 0.25, Y: 0.25, Z: z}})
	}
	// The triple products have about 150 significant bits, but their exact
	// determinant is -1, 0 or 1 before scaling. A rounded determinant loses it.
	n := float64(1 << 50)
	for _, exponent := range []int{-140, 0, 140} {
		for _, offset := range []float64{-1, 0, 1} {
			points := [4]r3.Vec{{}, {X: n, Y: n - 1, Z: n - 2},
				{X: n + 1, Y: n, Z: n - 1}, {X: n + 2, Y: n + 1, Z: n + offset}}
			for i := range points {
				points[i] = points[i].Scale(math.Ldexp(1, exponent))
			}
			check(points)
		}
	}
	rng := rand.New(rand.NewPCG(0x7b28a1, 0x519d63))
	exponents := [...]int{-200, -100, -1, 0, 1, 100, 199}
	coordinate := func() float64 {
		v := math.Ldexp(1+float64(rng.Uint64()>>11)*0x1p-53, exponents[rng.IntN(len(exponents))])
		if rng.IntN(2) == 0 {
			return -v
		}
		return v
	}
	for range 2000 {
		points := [4]r3.Vec{}
		for i := 1; i < len(points); i++ {
			points[i] = r3.NewVec(coordinate(), coordinate(), coordinate())
		}
		check(points)
		// Repeated points force exact cancellation rather than a large sign.
		points[3] = points[1]
		check(points)
	}
	for range 500 {
		var points [4]r3.Vec
		for i := range points {
			points[i] = r3.NewVec(float64(rng.IntN(2049)-1024)/8,
				float64(rng.IntN(2049)-1024)/8, float64(rng.IntN(2049)-1024)/8)
		}
		check(points)
	}
	for range 1000 {
		var points [4]r3.Vec
		for i := range points {
			points[i] = r3.NewVec(coordinate(), coordinate(), coordinate())
		}
		got, certain := proof.OrientSignExpansion(points[0], points[1], points[2], points[3])
		if !certain {
			continue
		}
		want := proof.OrientSignExact(proof.XptOf(points[0]), proof.XptOf(points[1]),
			proof.XptOf(points[2]), proof.XptOf(points[3]))
		require.Equal(t, want, got)
	}
	for _, scale := range []float64{0x1p-200, 0x1p200} {
		check([4]r3.Vec{{}, {X: scale}, {Y: scale}, {Z: scale}})
	}
}

func TestOrientSignExpansionDeclinesRoundedDifferencesAndExtremeScales(t *testing.T) {
	for _, value := range []float64{math.SmallestNonzeroFloat64, math.Nextafter(0x1p-200, 0),
		math.Nextafter(0x1p200, math.Inf(1)), math.MaxFloat64, math.Inf(1), math.NaN()} {
		got, certain := proof.OrientSignExpansion(r3.Vec{}, r3.Vec{X: value}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
		require.False(t, certain)
		require.Zero(t, got)
	}
	// Subtracting a tiny, nonzero input from one rounds to one. Its tail
	// must reach the integer fallback instead of being treated as exact zero.
	a := r3.Vec{X: 1, Y: 1, Z: 1}
	b := r3.Vec{X: 0x1p-100}
	c, d := r3.Vec{Y: 2}, r3.Vec{Z: 2}
	got, certain := proof.OrientSignExpansion(a, b, c, d)
	require.False(t, certain)
	require.Zero(t, got)
	want := proof.OrientSignExact(proof.XptOf(a), proof.XptOf(b), proof.XptOf(c), proof.XptOf(d))
	require.Equal(t, want, proof.OrientSign(a, b, c, d))
}

func TestOrientSignExpansionAllocatesNoStorage(t *testing.T) {
	a := r3.Vec{}
	b, c := r3.Vec{X: 1, Z: 1}, r3.Vec{Y: 1, Z: 1}
	d := r3.Vec{X: 0.25, Y: 0.25, Z: math.Nextafter(0.5, 1)}
	allocations := testing.AllocsPerRun(100, func() {
		_, _ = proof.OrientSignExpansion(a, b, c, d)
	})
	require.Zero(t, allocations)
}

func TestOrientationFiltersDecideAboveProvenErrorBound(t *testing.T) {
	for _, delta := range []float64{-0x1p-44, 0x1p-44} {
		a := r3.Vec{X: 0x1p-100, Y: 0x1p-100, Z: 0x1p-100}
		b, c := r3.Vec{X: 1, Z: 1}, r3.Vec{Y: 1, Z: 1}
		d := r3.Vec{X: 0.25, Y: 0.25, Z: 0.5 + delta}
		// This probe has nonzero subtraction tails, so the expansion cannot
		// decide it. The permanent bound must prove the sign independently.
		_, expansionCertain := proof.OrientSignExpansion(a, b, c, d)
		require.False(t, expansionCertain)
		want := proof.OrientSignExact(proof.XptOf(a), proof.XptOf(b), proof.XptOf(c), proof.XptOf(d))
		got, certain := proof.OrientSignFloat(a, b, c, d)
		require.True(t, certain)
		require.Equal(t, want, got)
		plane := proof.NewFloatOrientationPlane(a, b, c)
		got, certain = plane.Sign(d)
		require.True(t, certain)
		require.Equal(t, want, got)
	}
}
