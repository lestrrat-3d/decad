package revolvemesh

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// legacyRevDown and legacyRevUp are the float steps RevDown and RevUp had
// before the plain build began calling proof directly. Both builds must
// return the same value as them for every x.
func legacyRevDown(x float64) float64 {
	if x <= -0x1p-969 {
		return x + float64(x*0x1.0000000000001p-53)
	}
	return proof.NextDown(x)
}

func legacyRevUp(x float64) float64 {
	if x >= 0x1p-969 {
		return x + float64(x*0x1.0000000000001p-53)
	}
	return proof.NextUp(x)
}

// sameFloat reports whether a and b are both NaN or have identical bits. NaN
// payloads are not compared: they differ between architectures.
func sameFloat(a, b float64) bool {
	if math.IsNaN(a) || math.IsNaN(b) {
		return math.IsNaN(a) && math.IsNaN(b)
	}
	return math.Float64bits(a) == math.Float64bits(b)
}

// TestRevStepMatchesLegacyFloat compares RevUp and RevDown with the legacy
// float steps on the special values, every power of two with its neighbours,
// and seeded random bit patterns.
func TestRevStepMatchesLegacyFloat(t *testing.T) {
	t.Parallel()
	check := func(x float64) {
		if !sameFloat(RevUp(x), legacyRevUp(x)) {
			require.Failf(t, "RevUp", "x = %#x", math.Float64bits(x))
		}
		if !sameFloat(RevDown(x), legacyRevDown(x)) {
			require.Failf(t, "RevDown", "x = %#x", math.Float64bits(x))
		}
	}
	for _, x := range filterSpecials() {
		check(x)
	}
	for _, x := range []float64{
		math.NaN(), math.Inf(1), math.Inf(-1), 0, math.Copysign(0, -1),
		math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64,
		math.MaxFloat64, -math.MaxFloat64, 1, -1,
	} {
		check(x)
		check(math.Nextafter(x, math.Inf(1)))
		check(math.Nextafter(x, math.Inf(-1)))
	}
	for e := -1074; e <= 1023; e++ {
		p := math.Ldexp(1, e)
		for _, x := range []float64{p, math.Nextafter(p, 0), math.Nextafter(p, math.Inf(1))} {
			check(x)
			check(-x)
		}
	}
	r := rand.New(rand.NewPCG(11, 13))
	for range 200_000 {
		check(math.Float64frombits(r.Uint64()))
	}
}
