package loftmesh

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// collapsedExact is the exact-only reference for TriangleCollapsed.
func collapsedExact(verts []r3.Vec, tri [3]int) bool {
	a, b, c := proof.XptOf(verts[tri[0]]), proof.XptOf(verts[tri[1]]), proof.XptOf(verts[tri[2]])
	n := proof.Xcross(proof.Xsub(b, a), proof.Xsub(c, a))
	return n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0
}

func TestTriangleCollapsedMatchesExact(t *testing.T) {
	rng := rand.New(rand.NewPCG(7, 11))
	tri := [3]int{0, 1, 2}
	randVec := func(lo, hi float64) r3.Vec {
		f := func() float64 {
			m := lo + rng.Float64()*(hi-lo)
			if rng.IntN(2) == 0 {
				return -m
			}
			return m
		}
		return r3.Vec{X: f(), Y: f(), Z: f()}
	}
	intVec := func() r3.Vec {
		return r3.Vec{X: float64(rng.IntN(41) - 20), Y: float64(rng.IntN(41) - 20), Z: float64(rng.IntN(41) - 20)}
	}
	nudge := func(v r3.Vec) r3.Vec {
		switch rng.IntN(3) {
		case 0:
			v.X = math.Nextafter(v.X, math.Inf(1))
		case 1:
			v.Y = math.Nextafter(v.Y, math.Inf(-1))
		default:
			v.Z = math.Nextafter(v.Z, math.Inf(1))
		}
		return v
	}
	check := func(verts []r3.Vec) {
		t.Helper()
		require.Equal(t, collapsedExact(verts, tri), TriangleCollapsed(verts, tri), "%v", verts)
	}

	for range 3000 {
		check([]r3.Vec{randVec(1e-3, 1e3), randVec(1e-3, 1e3), randVec(1e-3, 1e3)})
	}
	for range 1500 {
		a, b := intVec(), intVec()
		s := math.Ldexp(1, -rng.IntN(5))
		c := r3.Vec{X: a.X + s*(b.X-a.X), Y: a.Y + s*(b.Y-a.Y), Z: a.Z + s*(b.Z-a.Z)}
		check([]r3.Vec{a, b, c})
		check([]r3.Vec{a, b, nudge(c)})
	}
	// Exactly collinear (all on a ray from the origin) with a difference that
	// rounds in float, so the float determinant is noise around an exact zero.
	for range 1500 {
		p := randVec(1e-3, 1e3)
		sc := func(k float64) r3.Vec { return r3.Vec{X: p.X * k, Y: p.Y * k, Z: p.Z * k} }
		check([]r3.Vec{sc(1), sc(2), sc(4)})
		check([]r3.Vec{sc(1), sc(4), sc(0.5)})
	}
	for range 300 {
		for _, scale := range []float64{1e-301, 1e301} {
			a, b, c := randVec(1, 2), randVec(1, 2), randVec(1, 2)
			sc := func(v r3.Vec) r3.Vec { return r3.Vec{X: v.X * scale, Y: v.Y * scale, Z: v.Z * scale} }
			check([]r3.Vec{sc(a), sc(b), sc(c)})
		}
	}
	for range 300 {
		a, b := randVec(1e-3, 1e3), randVec(1e-3, 1e3)
		check([]r3.Vec{a, a, b})
		check([]r3.Vec{a, b, a})
		check([]r3.Vec{a, a, a})
	}
	nan := math.NaN()
	// Non-finite inputs never reach the filter's proof.
	require.False(t, triangleOpenFloat(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: nan}))
	require.False(t, triangleOpenFloat(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: math.Inf(1)}))
	check([]r3.Vec{{}, {X: 1}, {X: 2}})
}
