package pointquery

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This test reads the private float bound directly and compares it against
// the exact rational box distance, independently of the pruning decision.
func TestBoxDistanceSquaredLowerEnclosesExactDistance(t *testing.T) {
	t.Parallel()
	check := func(p, a, b, c r3.Vec) {
		t.Helper()
		lower := boxDistanceSquaredLower(p, a, b, c)
		require.False(t, math.IsNaN(lower))
		require.False(t, math.IsInf(lower, 0))
		require.GreaterOrEqual(t, lower, 0.0)
		exact := boxDistanceSquared(exactVec(p), exactVec(a), exactVec(b), exactVec(c))
		require.LessOrEqual(t, proof.FloatRat(lower).Cmp(exact), 0,
			"lower %g exceeds exact box distance %s", lower, exact.RatString())
	}
	for _, scale := range []float64{math.SmallestNonzeroFloat64, 1e-200, 0.1, 1, 1e100, math.MaxFloat64} {
		check(r3.NewVec(scale, scale, scale), r3.Vec{}, r3.Vec{}, r3.Vec{})
		check(r3.NewVec(-scale, -scale, -scale), r3.NewVec(scale, scale, scale),
			r3.NewVec(scale, scale, scale), r3.NewVec(scale, scale, scale))
		check(r3.NewVec(math.Nextafter(scale, 0), scale, 0),
			r3.NewVec(scale, scale, 0), r3.NewVec(scale, scale, 0), r3.NewVec(scale, scale, 0))
	}
	rng := rand.New(rand.NewPCG(79, 131))
	finite := func() float64 {
		bits := rng.Uint64() &^ (uint64(0x7ff) << 52)
		return math.Float64frombits(bits | (rng.Uint64()%0x7ff)<<52)
	}
	vec := func() r3.Vec { return r3.NewVec(finite(), finite(), finite()) }
	for range 500 {
		check(vec(), vec(), vec(), vec())
	}
}

// Compare both queries against an exhaustive exact search with no box test.
// The first minimum also checks that tied closest points keep their order.
func TestMeshQueriesMatchExhaustiveExactSearch(t *testing.T) {
	t.Parallel()
	for _, scale := range []float64{math.SmallestNonzeroFloat64, 1e-200, 0.1, 1, 1e100, 1e160} {
		for _, reverse := range []bool{false, true} {
			verts := []r3.Vec{
				{X: -scale, Y: -scale}, {X: scale, Y: -scale}, {Y: scale},
				{X: -scale, Y: -scale, Z: scale}, {X: scale, Y: -scale, Z: scale}, {Y: scale, Z: scale},
				{X: -scale, Y: -scale, Z: math.Nextafter(scale, 0)},
				{X: scale, Y: -scale, Z: math.Nextafter(scale, 0)},
				{Y: scale, Z: math.Nextafter(scale, 0)},
			}
			tris := [][3]int{{0, 1, 2}, {3, 4, 5}, {6, 7, 8}}
			if reverse {
				tris[0], tris[2] = tris[2], tris[0]
			}
			for _, p := range []r3.Vec{{}, {Z: scale / 2}, {X: scale, Y: scale, Z: scale}, {Z: 2 * scale}} {
				point := exactVec(p)
				var want *big.Rat
				var nearest ratVec
				for _, tri := range tris {
					d, q, err := triangleNearest(point, exactVec(verts[tri[0]]), exactVec(verts[tri[1]]), exactVec(verts[tri[2]]))
					require.NoError(t, err)
					if want == nil || d.Cmp(want) < 0 {
						want, nearest = d, q
					}
				}
				d, err := MeshDistanceSquared(t.Context(), p, verts, tris)
				require.NoError(t, err)
				require.Zero(t, d.Cmp(want), "scale %g reverse %t", scale, reverse)
				d, q, err := MeshClosest(t.Context(), p, verts, tris)
				require.NoError(t, err)
				require.Zero(t, d.Cmp(want), "closest scale %g reverse %t", scale, reverse)
				for axis := range 3 {
					require.Zero(t, q[axis].Cmp(nearest[axis]), "closest coordinate %d", axis)
				}
			}
		}
	}
}
