package meshbool_test

import (
	"math"
	"math/big"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestSetSlabBounds(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	prep := func(t *testing.T, verts []r3.Vec, beta float64) *meshbool.BoolMesh {
		t.Helper()
		bm, err := meshbool.PrepContext(t.Context(), verts, [][3]int{{0, 1, 2}}, []int{0})
		require.NoError(t, err)
		bm.SetVertexBounds([]float64{beta, beta, beta})
		return bm
	}
	always := func(int) (r3.Frame, bool) { return frame, true }

	t.Run("a facet in the plane states a thin slab", func(t *testing.T) {
		t.Parallel()
		bm := prep(t, []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(10, 0, 0), r3.NewVec(0, 10, 0)}, 0.003)
		bm.SetSlabBounds(always)
		require.Zero(t, bm.SlabBound[0])
		require.Equal(t, 0.003, bm.FacetBound[0])
	})

	t.Run("a tilted facet charges its corner distance and its slope", func(t *testing.T) {
		t.Parallel()
		const beta, rise = 0.003, 1e-6
		bm := prep(t, []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(10, 0, rise), r3.NewVec(0, 10, 0)}, beta)
		bm.SetSlabBounds(always)
		// The facet normal is (-rise*10, 0, 100): slope sin = rise*10/100.
		want := rise + beta*(rise*10/math.Hypot(rise*10, 100))
		require.GreaterOrEqual(t, bm.SlabBound[0], want)
		require.LessOrEqual(t, bm.SlabBound[0], want*(1+1e-9))
	})

	t.Run("a steep facet keeps the displacement bound", func(t *testing.T) {
		t.Parallel()
		bm := prep(t, []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(10, 0, 0), r3.NewVec(0, 0, 10)}, 0.003)
		bm.SetSlabBounds(always)
		require.Equal(t, 0.003, bm.SlabBound[0])
	})

	t.Run("a facet with no plane keeps the displacement bound", func(t *testing.T) {
		t.Parallel()
		bm := prep(t, []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(10, 0, 0), r3.NewVec(0, 10, 0)}, 0.003)
		bm.SetSlabBounds(func(int) (r3.Frame, bool) { return r3.Frame{}, false })
		require.Equal(t, 0.003, bm.SlabBound[0])
	})
}

func TestSetSlabBoundsMatchesRational(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewSource(29))
	vec := func() r3.Vec {
		return r3.NewVec(2*rng.Float64()-1, 2*rng.Float64()-1, 2*rng.Float64()-1)
	}
	tightened := 0
	for i := range 600 {
		exp := []int{-900, -400, -1, 0, 1, 400, 900}[i%7]
		scale := math.Ldexp(1, exp)
		origin := vec().Scale(scale)
		frame, err := r3.NewFrame(origin, vec(), vec())
		require.NoError(t, err)
		verts := []r3.Vec{vec().Scale(scale).Add(origin), vec().Scale(scale).Add(origin), vec().Scale(scale).Add(origin)}
		if i%4 == 0 {
			// Nearly planar facets force cancellation in the exact normal.
			verts = []r3.Vec{r3.NewVec(0, 0, scale), r3.NewVec(scale, 0, scale),
				r3.NewVec(0, scale, math.Nextafter(scale, math.Inf(1)))}
		}
		beta := math.Ldexp(scale, 10)
		bm := &meshbool.BoolMesh{Verts: verts, Tris: [][3]int{{0, 1, 2}}, FacetBound: []float64{beta}, SlabBound: []float64{beta}}
		before := append([]r3.Vec(nil), verts...)
		want := beta
		if slab, ok := rationalPlaneSlab(verts, beta, frame); ok && slab < want {
			want = slab
			tightened++
		}
		bm.SetSlabBounds(func(int) (r3.Frame, bool) { return frame, true })
		require.Equal(t, math.Float64bits(want), math.Float64bits(bm.SlabBound[0]), "case %d, exponent %d", i, exp)
		require.Equal(t, beta, bm.FacetBound[0], "case %d preserves the displacement bound", i)
		for k := range verts {
			require.Equal(t, before[k], verts[k], "case %d, vertex %d stays unchanged", i, k)
		}
	}
	require.Greater(t, tightened, 500, "the comparison must exercise computed bounds below the displacement bound")
}

// rationalPlaneSlab independently evaluates Claim S using only math/big for
// exact arithmetic. The directed square root and final float bounds are the
// unchanged consumer of the exact quantities.
func rationalPlaneSlab(verts []r3.Vec, beta float64, frame r3.Frame) (float64, bool) {
	point := func(v r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
	}
	sub := func(a, b [3]*big.Rat) [3]*big.Rat {
		return [3]*big.Rat{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1]), new(big.Rat).Sub(a[2], b[2])}
	}
	dot := func(a, b [3]*big.Rat) *big.Rat {
		out := new(big.Rat)
		for k := range a {
			out.Add(out, new(big.Rat).Mul(a[k], b[k]))
		}
		return out
	}
	cross := func(a, b [3]*big.Rat) [3]*big.Rat {
		c := func(x, y, z, w int) *big.Rat {
			return new(big.Rat).Sub(new(big.Rat).Mul(a[x], b[y]), new(big.Rat).Mul(a[z], b[w]))
		}
		return [3]*big.Rat{c(1, 2, 2, 1), c(2, 0, 0, 2), c(0, 1, 1, 0)}
	}
	o := point(frame.Origin())
	n := cross(point(frame.U()), point(frame.V()))
	nn := dot(n, n)
	if nn.Sign() == 0 {
		return 0, false
	}
	corners := [3][3]*big.Rat{point(verts[0]), point(verts[1]), point(verts[2])}
	farthest := new(big.Rat)
	for _, c := range corners {
		s := dot(n, sub(c, o))
		s.Mul(s, s)
		if s.Cmp(farthest) > 0 {
			farthest = s
		}
	}
	h := proofbound.RatSqrtUp(new(big.Rat).Quo(farthest, nn))
	nt := cross(sub(corners[1], corners[0]), sub(corners[2], corners[0]))
	ntnt := dot(nt, nt)
	if ntnt.Sign() == 0 {
		return 0, false
	}
	c := cross(nt, n)
	sin2 := new(big.Rat).Quo(dot(c, c), new(big.Rat).Mul(ntnt, nn))
	slab := proofbound.AbsSumUpper(h, proofbound.ProductUpper(beta, proofbound.RatSqrtUp(sin2)))
	return slab, !proofbound.IsNonFinite(slab)
}
