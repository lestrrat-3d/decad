package meshplane_test

import (
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/lattice"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshplane"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// referenceGroup is one group of referenceGroups.
type referenceGroup struct {
	planar      bool
	first       int
	normal      [3]*big.Rat
	weight      *big.Rat
	moment      [3]*big.Rat
	centroid    measurement.VecMeasurement
	hasCentroid bool
}

// referenceGroups is the plane grouping in big.Rat arithmetic: each triangle's
// plane keyed by the decimal strings of its normal and offset divided by the
// magnitude of the normal's first non-zero part, and each group's sums
// accumulated as reduced rationals. Groups must answer exactly what it does.
func referenceGroups(verts []r3.Vec, tris [][3]int) ([]int, []referenceGroup) {
	exact := func(v r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{proof.FloatRat(v.X), proof.FloatRat(v.Y), proof.FloatRat(v.Z)}
	}
	sub := func(a, b [3]*big.Rat) [3]*big.Rat {
		var out [3]*big.Rat
		for i := range out {
			out[i] = new(big.Rat).Sub(a[i], b[i])
		}
		return out
	}
	cross := func(u, v [3]*big.Rat) [3]*big.Rat {
		term := func(a, b, c, d *big.Rat) *big.Rat {
			return new(big.Rat).Sub(new(big.Rat).Mul(a, b), new(big.Rat).Mul(c, d))
		}
		return [3]*big.Rat{term(u[1], v[2], u[2], v[1]), term(u[2], v[0], u[0], v[2]), term(u[0], v[1], u[1], v[0])}
	}
	index := map[string]int{}
	src := make([]int, len(tris))
	var groups []referenceGroup
	for ti, t := range tris {
		a, b, c := exact(verts[t[0]]), exact(verts[t[1]]), exact(verts[t[2]])
		n := cross(sub(b, a), sub(c, a))
		key, planar := fmt.Sprintf("triangle %d", ti), false
		for _, part := range n {
			if part.Sign() == 0 {
				continue
			}
			scale := new(big.Rat).Abs(part)
			offset := new(big.Rat)
			for i := range n {
				offset.Add(offset, new(big.Rat).Mul(n[i], a[i]))
			}
			key = fmt.Sprintf("%s %s %s %s", new(big.Rat).Quo(n[0], scale).RatString(),
				new(big.Rat).Quo(n[1], scale).RatString(), new(big.Rat).Quo(n[2], scale).RatString(),
				new(big.Rat).Quo(offset, scale).RatString())
			planar = true
			break
		}
		gi, ok := index[key]
		if !ok {
			gi = len(groups)
			index[key] = gi
			groups = append(groups, referenceGroup{planar: planar, first: ti, normal: n})
		}
		src[ti] = gi
		if !planar {
			continue
		}
		g := &groups[gi]
		w := new(big.Rat)
		for i := range n {
			w.Add(w, new(big.Rat).Mul(n[i], g.normal[i]))
		}
		if g.weight == nil {
			g.weight = new(big.Rat)
			for i := range g.moment {
				g.moment[i] = new(big.Rat)
			}
		}
		g.weight.Add(g.weight, w)
		for i := range g.moment {
			corner := new(big.Rat).Add(a[i], b[i])
			corner.Add(corner, c[i])
			g.moment[i].Add(g.moment[i], corner.Mul(corner, w))
		}
	}
	for gi := range groups {
		g := &groups[gi]
		if g.planar {
			g.centroid, g.hasCentroid = referenceCentroid(g.weight, g.moment)
		}
	}
	return src, groups
}

// referenceCentroid is the centroid read of referenceGroups.
func referenceCentroid(weight *big.Rat, moment [3]*big.Rat) (measurement.VecMeasurement, bool) {
	if weight == nil || weight.Sign() <= 0 {
		return measurement.VecMeasurement{}, false
	}
	var coords [3]float64
	var bound float64
	three := big.NewRat(3, 1)
	for i := range coords {
		q := new(big.Rat).Quo(moment[i], new(big.Rat).Mul(weight, three))
		f, _ := proof.RatFloat64(q)
		coords[i] = f
		bound = math.Max(bound, proof.RationalFloatError(q, f))
	}
	if math.IsInf(bound, 0) || math.IsNaN(bound) {
		return measurement.VecMeasurement{}, false
	}
	return measurement.VecMeasurement{
		Value:     r3.NewVec(coords[0], coords[1], coords[2]),
		Exactness: measurement.ExactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, true
}

// requireSameRat requires the exact value r to equal want, and its reduced
// form to be want's.
func requireSameRat(t *testing.T, want, r *big.Rat, what string) {
	t.Helper()
	require.Zero(t, want.Cmp(r), "%s: %s, want %s", what, r.RatString(), want.RatString())
	require.Zero(t, want.Num().Cmp(r.Num()), what)
	require.Zero(t, want.Denom().Cmp(r.Denom()), what)
}

// requireSameFloat requires got to have want's bits.
func requireSameFloat(t *testing.T, want, got float64, what string) {
	t.Helper()
	require.Equal(t, math.Float64bits(want), math.Float64bits(got), "%s: %v, want %v", what, got, want)
}

// requireMatchesReference requires Groups to answer referenceGroups' grouping,
// group order, exact normals and centroids, and centroid bits. It compares field by
// field and reports only scalars.
func requireMatchesReference(t *testing.T, verts []r3.Vec, tris [][3]int) []meshplane.Group {
	t.Helper()
	wantSrc, want := referenceGroups(verts, tris)
	gotSrc, got := meshplane.Groups(verts, tris)
	require.Equal(t, len(wantSrc), len(gotSrc))
	for ti := range wantSrc {
		require.Equal(t, wantSrc[ti], gotSrc[ti], "triangle %d", ti)
	}
	require.Equal(t, len(want), len(got))
	for gi := range want {
		w, g := want[gi], got[gi]
		what := fmt.Sprintf("group %d", gi)
		require.Equal(t, w.planar, g.Planar, what)
		require.Equal(t, w.first, g.First, what)
		require.Equal(t, w.hasCentroid, g.HasCentroid, what)
		if !w.planar {
			require.True(t, proof.DvIsZero(g.Normal), what)
			require.True(t, g.CentroidDen.IsZero(), what)
			continue
		}
		three := big.NewRat(3, 1)
		for i := range 3 {
			requireSameRat(t, w.normal[i], g.Normal[i].Rat(), what+" normal")
			// The exact centroid coordinate, the rational the float reading rounds.
			q := new(big.Rat).Quo(w.moment[i], new(big.Rat).Mul(w.weight, three))
			requireSameRat(t, q, new(big.Rat).Quo(g.CentroidNum[i].Rat(), g.CentroidDen.Rat()), what+" centroid")
		}
		requireSameFloat(t, w.centroid.Value.X, g.Centroid.Value.X, what+" centroid x")
		requireSameFloat(t, w.centroid.Value.Y, g.Centroid.Value.Y, what+" centroid y")
		requireSameFloat(t, w.centroid.Value.Z, g.Centroid.Value.Z, what+" centroid z")
		requireSameFloat(t, w.centroid.Bound.Base(), g.Centroid.Bound.Base(), what+" bound")
		require.Equal(t, w.centroid.Exactness, g.Centroid.Exactness, what)
	}
	return got
}

// cube is the closed cube of side s at corner o, two triangles a face.
func cube(o r3.Vec, s float64) ([]r3.Vec, [][3]int) {
	var verts []r3.Vec
	for i := range 8 {
		verts = append(verts, o.Add(r3.NewVec(s*float64(i&1), s*float64(i>>1&1), s*float64(i>>2&1))))
	}
	tris := [][3]int{
		{0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6},
		{0, 1, 4}, {1, 5, 4}, {2, 6, 3}, {3, 6, 7},
		{0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5},
	}
	return verts, tris
}

// planeSoup is n triangles on a few exact planes z = p·x + q·y + r with small
// integer p, q, r, the coordinates integers scaled by 2^scale, wound at random,
// interleaved with triangles of random float vertices.
func planeSoup(rng *rand.Rand, n, scale int) ([]r3.Vec, [][3]int) {
	type coeffs struct{ p, q, r float64 }
	planes := []coeffs{{0, 0, 0}, {0, 0, 5}, {1, 0, 0}, {2, -3, 7}, {-1, 1, 1}, {3, 3, -2}}
	unit := math.Ldexp(1, scale)
	var verts []r3.Vec
	var tris [][3]int
	for range n {
		base := len(verts)
		if rng.IntN(4) == 0 {
			for range 3 {
				verts = append(verts, r3.NewVec(rng.Float64()*unit, rng.Float64()*unit, rng.Float64()*unit))
			}
		} else {
			c := planes[rng.IntN(len(planes))]
			for range 3 {
				x, y := float64(rng.IntN(64)-32), float64(rng.IntN(64)-32)
				verts = append(verts, r3.NewVec(x*unit, y*unit, (c.p*x+c.q*y+c.r)*unit))
			}
		}
		if rng.IntN(2) == 0 {
			tris = append(tris, [3]int{base, base + 1, base + 2})
		} else {
			tris = append(tris, [3]int{base, base + 2, base + 1})
		}
	}
	return verts, tris
}

func TestGroupsMatchesRationalReference(t *testing.T) {
	t.Parallel()
	t.Run("lattices", func(t *testing.T) {
		t.Parallel()
		lo, hi := r3.NewVec(0.3, -0.7, 1.1), r3.NewVec(10.3, 9.3, 11.1)
		for kind, level := range map[lattice.Kind]float64{lattice.KindGyroid: 0.6, lattice.KindSchwarzP: 1,
			lattice.KindDiamond: 0.5, lattice.KindNeovius: 4, lattice.KindIWP: 2} {
			verts, tris, err := lattice.Surface(t.Context(), kind, lo, hi, 10, level, 8)
			require.NoError(t, err)
			got := requireMatchesReference(t, verts, tris)
			multi := 0
			for _, g := range got {
				if g.Planar && proof.DyCmp(g.CentroidDen, proof.DyInt(3)) != 0 {
					multi++
				}
			}
			require.Positive(t, multi, "kind %d has groups of several triangles", kind)
		}
	})
	t.Run("cube", func(t *testing.T) {
		t.Parallel()
		verts, tris := cube(r3.NewVec(0.1, -2.75, 1e-3), 3.3)
		got := requireMatchesReference(t, verts, tris)
		require.Equal(t, 6, len(got))
	})
	t.Run("degenerate", func(t *testing.T) {
		t.Parallel()
		verts := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1, 1, 1), r3.NewVec(2, 2, 2), r3.NewVec(1, 0, 0),
			r3.NewVec(0, 1, 0), r3.NewVec(0.1, 0.2, 0.3)}
		tris := [][3]int{{0, 1, 2}, {0, 3, 4}, {0, 0, 5}, {0, 1, 2}, {0, 4, 3}, {5, 5, 5}, {3, 0, 4}}
		got := requireMatchesReference(t, verts, tris)
		require.Equal(t, 6, len(got), "each triangle of no area is its own group; one plane of each orientation")
	})
	t.Run("near_coplanar", func(t *testing.T) {
		t.Parallel()
		// The second triangle lifts one corner by one ulp and the third lowers it by one, so each tilts.
		verts := []r3.Vec{r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 1), r3.NewVec(0, 1, 1),
			r3.NewVec(1, 1, math.Nextafter(1, 2)), r3.NewVec(1, 1, math.Nextafter(1, 0)), r3.NewVec(1, 1, 1)}
		tris := [][3]int{{0, 1, 2}, {1, 3, 2}, {1, 4, 2}, {1, 5, 2}}
		got := requireMatchesReference(t, verts, tris)
		require.Equal(t, 3, len(got))
	})
	t.Run("bucket_collision", func(t *testing.T) {
		t.Parallel()
		// The planes through the z axis with normals (−q, p, 0) for p = 2^52+1 and 2^52+3, each with q = 3p+1,
		// are distinct, yet p/q rounds down to one float for both, so the two share a key.
		p1, p2 := math.Ldexp(1, 52)+1, math.Ldexp(1, 52)+3
		q1, q2 := math.Ldexp(3, 52)+4, math.Ldexp(3, 52)+10
		require.Equal(t, proof.DyQuoFloatDown(proof.MustDyOf(p1), proof.MustDyOf(q1)),
			proof.DyQuoFloatDown(proof.MustDyOf(p2), proof.MustDyOf(q2)))
		require.NotZero(t, new(big.Rat).SetFrac(big.NewInt(int64(p1)), big.NewInt(int64(q1))).Cmp(
			new(big.Rat).SetFrac(big.NewInt(int64(p2)), big.NewInt(int64(q2)))))
		verts := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), r3.NewVec(p1, q1, 0), r3.NewVec(p2, q2, 0),
			r3.NewVec(0, 0, 2), r3.NewVec(2*p1, 2*q1, 0)}
		tris := [][3]int{{0, 1, 2}, {0, 1, 3}, {0, 4, 2}, {0, 1, 5}, {0, 4, 3}}
		src, _ := meshplane.Groups(verts, tris)
		require.Equal(t, []int{0, 1, 0, 0, 1}, src)
		requireMatchesReference(t, verts, tris)
	})
	t.Run("plane_soup", func(t *testing.T) {
		t.Parallel()
		rng := rand.New(rand.NewPCG(7, 11))
		for _, scale := range []int{0, -3, 40, -40} {
			verts, tris := planeSoup(rng, 600, scale)
			requireMatchesReference(t, verts, tris)
		}
	})
	t.Run("large_and_tiny", func(t *testing.T) {
		t.Parallel()
		rng := rand.New(rand.NewPCG(3, 5))
		for _, scale := range []int{900, 1000, -1000, -1070} {
			verts, tris := planeSoup(rng, 200, scale)
			requireMatchesReference(t, verts, tris)
		}
		// A cube spanning large and tiny coordinates at once, and one of subnormal side.
		verts, tris := cube(r3.NewVec(1e300, -1e-300, 3e-310), 1e290)
		requireMatchesReference(t, verts, tris)
		verts, tris = cube(r3.NewVec(5e-324, 0, -1e-320), 4e-322)
		requireMatchesReference(t, verts, tris)
	})
}
