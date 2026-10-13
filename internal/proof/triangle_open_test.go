package proof_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func exactNormalNonZero(a, b, c r3.Vec) bool {
	xa := proof.XptOf(a)
	n := proof.Xcross(proof.Xsub(proof.XptOf(b), xa), proof.Xsub(proof.XptOf(c), xa))
	return n.X.Sign() != 0 || n.Y.Sign() != 0 || n.Z.Sign() != 0
}

// TestTriangleOpenFloatImpliesExactNormal checks that a true answer is never
// contradicted by the exact cross product, and that exactly collinear,
// repeated and out-of-range triangles are never reported open.
func TestTriangleOpenFloatImpliesExactNormal(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	coord := func(lo, hi float64) float64 {
		m := lo * math.Pow(hi/lo, rng.Float64())
		if rng.IntN(2) == 0 {
			return -m
		}
		return m
	}
	vec := func(lo, hi float64) r3.Vec {
		return r3.NewVec(coord(lo, hi), coord(lo, hi), coord(lo, hi))
	}
	t.Run("random", func(t *testing.T) {
		open := 0
		for range 20000 {
			a, b, c := vec(1e-3, 1e3), vec(1e-3, 1e3), vec(1e-3, 1e3)
			if proof.TriangleOpenFloat(a, b, c) {
				open++
				require.True(t, exactNormalNonZero(a, b, c), "%v %v %v", a, b, c)
			}
		}
		require.Greater(t, open, 19000)
	})
	t.Run("collinear", func(t *testing.T) {
		for range 2000 {
			a := r3.NewVec(float64(rng.IntN(9)-4), float64(rng.IntN(9)-4), float64(rng.IntN(9)-4))
			b := r3.NewVec(float64(rng.IntN(9)-4), float64(rng.IntN(9)-4), float64(rng.IntN(9)-4))
			c := a.Add(b.Sub(a).Scale(math.Ldexp(1, -rng.IntN(6))))
			require.False(t, exactNormalNonZero(a, b, c))
			require.False(t, proof.TriangleOpenFloat(a, b, c), "%v %v %v", a, b, c)
		}
	})
	t.Run("near collinear", func(t *testing.T) {
		for range 2000 {
			a, b := vec(1e-3, 1e3), vec(1e-3, 1e3)
			c := a.Add(b.Sub(a).Scale(rng.Float64()))
			c.X = math.Nextafter(c.X, math.Inf(1))
			if proof.TriangleOpenFloat(a, b, c) {
				require.True(t, exactNormalNonZero(a, b, c), "%v %v %v", a, b, c)
			}
		}
	})
	t.Run("extreme", func(t *testing.T) {
		for _, scale := range []float64{1e-301, 1e301} {
			a, b, c := r3.NewVec(0, 0, 0), r3.NewVec(scale, 0, 0), r3.NewVec(0, scale, 0)
			require.True(t, exactNormalNonZero(a, b, c))
			require.False(t, proof.TriangleOpenFloat(a, b, c), "scale=%g", scale)
		}
	})
	t.Run("repeated vertex", func(t *testing.T) {
		a, b := r3.NewVec(1, 2, 3), r3.NewVec(-4, 5, 6)
		require.False(t, proof.TriangleOpenFloat(a, a, b))
		require.False(t, proof.TriangleOpenFloat(a, b, a))
		require.False(t, proof.TriangleOpenFloat(b, a, a))
	})
}
