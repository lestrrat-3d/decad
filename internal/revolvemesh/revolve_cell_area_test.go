package revolvemesh_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// intervalTwoTriangleArea is the interval-arithmetic formula for twice the
// area of a triangle; IvTwoTriangleArea must return exactly its enclosure.
func intervalTwoTriangleArea(p0, p1, p2 proofbound.IvVec3) (proofbound.RatInterval, bool) {
	n := proofbound.IvVec3Cross(proofbound.IvVec3Sub(p1, p0), proofbound.IvVec3Sub(p2, p0))
	return proofbound.IntervalSqrt(proofbound.IvVec3NormSq(n))
}

func requireSameRat(t *testing.T, want, got *big.Rat, what string) {
	t.Helper()
	require.Zero(t, want.Cmp(got), "%s: value differs", what)
	require.Equal(t, want.String(), got.String(), "%s: text differs", what)
}

func TestIvTwoTriangleAreaMatchesIntervalFormula(t *testing.T) {
	rng := rand.New(rand.NewPCG(1, 2))
	coord := func() float64 {
		switch rng.IntN(6) {
		case 0:
			return 0
		case 1:
			return float64(rng.IntN(7) - 3)
		case 2:
			return math.Ldexp(rng.Float64()-0.5, rng.IntN(40)-20)
		case 3:
			return math.Ldexp(rng.Float64()-0.5, rng.IntN(600)-300)
		default:
			return (rng.Float64() - 0.5) * 40
		}
	}
	point := func() proofbound.IvVec3 {
		v, ok := proofbound.IvVec3Of(r3.Vec{X: coord(), Y: coord(), Z: coord()})
		require.True(t, ok)
		return v
	}
	// widen turns one coordinate into a non-point interval.
	widen := func(v proofbound.IvVec3) proofbound.IvVec3 {
		k := rng.IntN(3)
		w := new(big.Rat).SetFloat64(math.Ldexp(rng.Float64(), rng.IntN(20)-30))
		lo := new(big.Rat).Sub(v[k].Lo, w)
		hi := new(big.Rat).Add(v[k].Hi, w)
		v[k] = proofbound.Interval(lo, hi)
		return v
	}
	triangles := 20000
	if revolvemesh.RaceDetector {
		triangles = 4000
	}
	for range triangles {
		p0, p1, p2 := point(), point(), point()
		switch rng.IntN(10) {
		case 0:
			p2 = p1
		case 1:
			p1, p2 = p0, p0
		case 2:
			// Collinear: the origin, (1,2,3) and (2,4,6).
			o, ok := proofbound.IvVec3Of(r3.Vec{})
			require.True(t, ok)
			a, ok := proofbound.IvVec3Of(r3.Vec{X: 1, Y: 2, Z: 3})
			require.True(t, ok)
			b, ok := proofbound.IvVec3Of(r3.Vec{X: 2, Y: 4, Z: 6})
			require.True(t, ok)
			p0, p1, p2 = o, a, b
		case 3:
			p0 = widen(p0)
		case 4:
			p1 = widen(p1)
			p2 = widen(p2)
		}
		// Snapshot the inputs: the call must not write them.
		var before [9]string
		for i, p := range []proofbound.IvVec3{p0, p1, p2} {
			for k, c := range p {
				before[i*3+k] = c.Lo.String() + "|" + c.Hi.String()
			}
		}
		want, wantOK := intervalTwoTriangleArea(p0, p1, p2)
		got, gotOK := revolvemesh.IvTwoTriangleArea(p0, p1, p2)
		require.Equal(t, wantOK, gotOK)
		if wantOK {
			requireSameRat(t, want.Lo, got.Lo, "lo")
			requireSameRat(t, want.Hi, got.Hi, "hi")
		}
		for i, p := range []proofbound.IvVec3{p0, p1, p2} {
			for k, c := range p {
				require.Equal(t, before[i*3+k], c.Lo.String()+"|"+c.Hi.String())
			}
		}
	}
}
