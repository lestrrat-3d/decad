package meshbool

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

type parityIndexFixture struct {
	name  string
	verts []r3.Vec
	tris  [][3]int
}

func fr(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }

// parityCylinder is a closed z-axis prism, radius 3 from z = -1 to 2, over a
// regular n-gon with fan caps.
// For a ray sweeping x, both caps project onto horizontal lines (one line per
// cap, n facets each) and the two side quads facing ±y onto vertical lines.
func parityCylinder(n int) ([]r3.Vec, [][3]int) {
	const r, z0, z1 = 3, -1.0, 2.0
	var verts []r3.Vec
	for k := range n {
		th := 2 * math.Pi * float64(k) / float64(n)
		x, y := r*math.Cos(th), r*math.Sin(th)
		if k == n/4 {
			x, y = 0, r
		}
		if k == 3*n/4 {
			x, y = 0, -r
		}
		verts = append(verts, r3.Vec{X: x, Y: y, Z: z0}, r3.Vec{X: x, Y: y, Z: z1})
	}
	verts = append(verts, r3.Vec{Z: z0}, r3.Vec{Z: z1})
	c0, c1 := 2*n, 2*n+1
	var tris [][3]int
	for k := range n {
		a0, a1 := 2*k, 2*k+1
		b0, b1 := 2*((k+1)%n), 2*((k+1)%n)+1
		tris = append(tris, [3]int{a0, b0, b1}, [3]int{a0, b1, a1},
			[3]int{c0, b0, a0}, [3]int{c1, a1, b1})
	}
	return verts, tris
}

// parityLatticeSoup is a triangle soup on a coarse lattice: many projections
// collapse to segments on axis-parallel and slanted lines, and many share
// projected vertices. No facet projects to a single point on any axis.
func parityLatticeSoup(rng *rand.Rand, n int, step float64) ([]r3.Vec, [][3]int) {
	var verts []r3.Vec
	var tris [][3]int
	pt := func() r3.Vec {
		return r3.Vec{X: step * float64(rng.IntN(7)-3), Y: step * float64(rng.IntN(7)-3), Z: step * float64(rng.IntN(7)-3)}
	}
	for len(tris) < n {
		a, b, c := pt(), pt(), pt()
		if pointProjection(a, b, c) {
			continue
		}
		// Every third facet is forced to contain the x direction, so its x
		// projection collapses to a segment of a slanted or axis-parallel line.
		if len(tris)%3 == 0 {
			c = r3.Vec{X: c.X, Y: b.Y, Z: b.Z}
			if pointProjection(a, b, c) {
				continue
			}
		}
		k := len(verts)
		verts = append(verts, a, b, c)
		tris = append(tris, [3]int{k, k + 1, k + 2})
	}
	return verts, tris
}

func pointProjection(a, b, c r3.Vec) bool {
	same := func(f func(r3.Vec) float64) bool { return f(a) == f(b) && f(a) == f(c) }
	x := func(v r3.Vec) float64 { return v.X }
	y := func(v r3.Vec) float64 { return v.Y }
	z := func(v r3.Vec) float64 { return v.Z }
	return (same(y) && same(z)) || (same(x) && same(z)) || (same(x) && same(y))
}

// paritySlanted is random float triangles, a third of them long and thin
// across the whole box, and a third whose x projection is a segment of a line
// with an arbitrary float slope.
func paritySlanted(rng *rand.Rand, n int, scale float64) ([]r3.Vec, [][3]int) {
	var verts []r3.Vec
	var tris [][3]int
	f := func() float64 { return scale * (rng.Float64()*2 - 1) }
	for len(tris) < n {
		a := r3.Vec{X: f(), Y: f(), Z: f()}
		b := r3.Vec{X: f(), Y: f(), Z: f()}
		var c r3.Vec
		switch len(tris) % 3 {
		case 0:
			c = r3.Vec{X: f(), Y: b.Y, Z: b.Z}
		case 1:
			c = r3.Vec{X: a.X + scale*1e-3, Y: a.Y + scale*1e-3*rng.Float64(), Z: a.Z - scale*1e-3}
		default:
			c = r3.Vec{X: f(), Y: f(), Z: f()}
		}
		if pointProjection(a, b, c) {
			continue
		}
		k := len(verts)
		verts = append(verts, a, b, c)
		tris = append(tris, [3]int{k, k + 1, k + 2})
	}
	return verts, tris
}

func parityIndexFixtures() []parityIndexFixture {
	rng := rand.New(rand.NewPCG(0x9a41, 0x7e3d))
	var fx []parityIndexFixture
	v, t := parityCylinder(24)
	fx = append(fx, parityIndexFixture{`cylinder`, v, t})
	v, t = parityLatticeSoup(rng, 120, 0.5)
	fx = append(fx, parityIndexFixture{`lattice`, v, t})
	v, t = paritySlanted(rng, 150, 4)
	fx = append(fx, parityIndexFixture{`slanted`, v, t})
	v, t = paritySlanted(rng, 90, 1e150)
	fx = append(fx, parityIndexFixture{`slanted huge`, v, t})
	v, t = paritySlanted(rng, 90, 1e-150)
	fx = append(fx, parityIndexFixture{`slanted tiny`, v, t})
	return fx
}

// parityIndexQueries returns exact queries for one fixture: every vertex,
// edge midpoint and centroid; points on each facet's projected line outside
// its segment; points on and just beside the index's own cell boundaries; and
// random rationals around the box.
func parityIndexQueries(rng *rand.Rand, fx parityIndexFixture) []proof.Xpt {
	var qs []proof.Xpt
	third := big.NewRat(1, 3)
	for _, tri := range fx.tris {
		a, b, c := fx.verts[tri[0]], fx.verts[tri[1]], fx.verts[tri[2]]
		qs = append(qs, proof.XptOf(a))
		xa, xb, xc := proof.XptOf(a), proof.XptOf(b), proof.XptOf(c)
		qs = append(qs, XCentroid(xa, xb, xc))
		// Off the segment on its own line, at 7/3 and -4/3 of the way from a
		// to b, with a random x so every x ray meets the line there.
		for _, t := range []*big.Rat{big.NewRat(7, 3), big.NewRat(-4, 3), third} {
			lerp := func(p, q float64) *big.Rat {
				d := new(big.Rat).Sub(fr(q), fr(p))
				return new(big.Rat).Add(fr(p), d.Mul(d, t))
			}
			qs = append(qs, ratXpt(lerp(a.X, b.X), lerp(a.Y, b.Y), lerp(a.Z, b.Z)))
			qs = append(qs, ratXpt(fr(c.X), lerp(a.Y, b.Y), lerp(a.Z, b.Z)))
		}
	}
	lo, hi := fx.verts[0], fx.verts[0]
	for _, v := range fx.verts {
		lo = r3.Vec{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
		hi = r3.Vec{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
	}
	span := func(l, h float64) *big.Rat {
		w := new(big.Rat).Sub(fr(h), fr(l))
		f := new(big.Rat).Mul(w, big.NewRat(rng.Int64N(1400)-200, 1000))
		return f.Add(f, fr(l))
	}
	for range 200 {
		qs = append(qs, ratXpt(span(lo.X, hi.X), span(lo.Y, hi.Y), span(lo.Z, hi.Z)))
	}
	// Cell boundaries of the x-ray index, exactly and one tiny step either side
	// (the side steps round back onto the boundary).
	pm := NewParityMesh(fx.verts, fx.tris)
	if idx := pm.buildParityIndex(0, 1, 2); idx != nil {
		for k := 1; k < len(idx.u.b)-1; k += 3 {
			for _, d := range []int64{0, 1, -1} {
				nudge := func(f float64) *big.Rat {
					e := new(big.Rat).SetFrac(big.NewInt(d), new(big.Int).Lsh(big.NewInt(1), 1200))
					return e.Add(e, fr(f))
				}
				j := 1 + rng.IntN(len(idx.v.b)-2)
				qs = append(qs, ratXpt(span(lo.X, hi.X), nudge(idx.u.b[k]), nudge(idx.v.b[j])))
			}
		}
	}
	return qs
}

func parityAllFacets(n int) []int {
	all := make([]int, n)
	for i := range all {
		all[i] = i
	}
	return all
}

// flatGroupFirst maps each facet whose projection lies on one axis-parallel
// line to the first facet on that line, the one the index keeps.
func flatGroupFirst(pm *ParityMesh, axis, u, v int) map[int]int {
	boxes := pm.FacetBoxes(axis)
	first := map[[2]float64]int{}
	out := map[int]int{}
	for ti, tri := range pm.Tris {
		if !boxes[ti].Built {
			boxes[ti] = pm.BuildFacetBox(axis, u, v, ti)
		}
		if boxes[ti].Nondegenerate {
			continue
		}
		a, b, c := pm.Verts[tri[0]], pm.Verts[tri[1]], pm.Verts[tri[2]]
		var key [2]float64
		switch {
		case CoordOf(a, v) == CoordOf(b, v) && CoordOf(a, v) == CoordOf(c, v):
			key = [2]float64{0, CoordOf(a, v)}
		case CoordOf(a, u) == CoordOf(b, u) && CoordOf(a, u) == CoordOf(c, u):
			key = [2]float64{1, CoordOf(a, u)}
		default:
			continue
		}
		if f, ok := first[key]; ok {
			out[ti] = f
		} else {
			first[key] = ti
		}
	}
	return out
}

// TestParityIndexListsEveryFacetThatMatters checks the index at facet
// granularity: a facet missing from a query's cell list classifies strictly
// outside, or is a later facet of an axis-parallel line whose first facet is
// listed and ambiguous for the query.
func TestParityIndexListsEveryFacetThatMatters(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x31d7, 0x0c55))
	for _, fx := range parityIndexFixtures() {
		t.Run(fx.name, func(t *testing.T) {
			queries := parityIndexQueries(rng, fx)
			pruned, mattered := 0, 0
			for _, ray := range AxisRays {
				pm := NewParityMesh(fx.verts, fx.tris)
				idx := pm.buildParityIndex(ray.Axis, ray.U, ray.V)
				require.NotNil(t, idx, `axis %d`, ray.Axis)
				group := flatGroupFirst(pm, ray.Axis, ray.U, ray.V)
				for qi, p := range queries {
					m, pr := requireLeftOutFacetsEndOutside(t, pm, idx, group, ray.Axis, ray.U, ray.V, ray.Dir, p, qi)
					mattered += m
					pruned += pr
				}
			}
			require.Positive(t, mattered, `the queries must reach facets`)
			require.Positive(t, pruned, `the index must prune`)
		})
	}
}

// requireLeftOutFacetsEndOutside checks one query on one ray: every facet
// missing from the query's cell list classifies strictly outside, or is a
// later facet of an axis-parallel line whose first facet is listed and
// ambiguous. It returns the number of facets that were not outside and the
// number left out.
func requireLeftOutFacetsEndOutside(t *testing.T, pm *ParityMesh, idx *ParityIndex, group map[int]int,
	axis, u, v, dir int, p proof.Xpt, qi int) (int, int) {
	t.Helper()
	pa := NewXP2(RatCoordOf(p, u), RatCoordOf(p, v))
	require.True(t, pa.FloatFinite)
	listed := map[int]bool{}
	for _, ti := range idx.candidates(pa.Fu, pa.Fv) {
		listed[int(ti)] = true
	}
	mattered, pruned := 0, 0
	for ti := range pm.Tris {
		got := pm.classifyFacet(p, pa, axis, u, v, dir, ti)
		if got != facetOutside {
			mattered++
		}
		if listed[ti] {
			continue
		}
		pruned++
		if got == facetOutside {
			continue
		}
		f, ok := group[ti]
		require.True(t, ok && f < ti && listed[f],
			`query %d axis %d dir %d: facet %d (outcome %d) is not listed`, qi, axis, dir, ti, got)
		require.Equal(t, facetAmbiguous, pm.classifyFacet(p, pa, axis, u, v, dir, f),
			`query %d: facet %d stands for %d but is not ambiguous`, qi, f, ti)
	}
	return mattered, pruned
}

type parityAnswer struct {
	inside, on bool
	err        string
}

func answerOf(in, on bool, err error) parityAnswer {
	a := parityAnswer{inside: in, on: on}
	if err != nil {
		a.err = err.Error()
	}
	return a
}

// TestParityIndexMatchesTheUnindexedScan is the end-to-end differential: the
// indexed all-facets scan against the subset scan over the identity subset,
// with and without the box filter, each arm keeping its caches across queries.
func TestParityIndexMatchesTheUnindexedScan(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	rng := rand.New(rand.NewPCG(0x77a3, 0x1f09))
	for _, fx := range parityIndexFixtures() {
		t.Run(fx.name, func(t *testing.T) {
			all := parityAllFacets(len(fx.tris))
			indexed := NewParityMesh(fx.verts, fx.tris)
			filtered := NewParityMesh(fx.verts, fx.tris)
			plain := NewParityMesh(fx.verts, fx.tris)
			plain.Unfiltered = true
			kinds := map[parityAnswer]int{}
			for qi, p := range parityIndexQueries(rng, fx) {
				want := answerOf(MeshParityPreparedContext(ctx, p, plain, all))
				require.Equal(t, want, answerOf(MeshParityPreparedContext(ctx, p, filtered, all)), `query %d`, qi)
				require.Equal(t, want, answerOf(MeshParityAllContext(ctx, p, indexed)), `query %d`, qi)
				kinds[want]++
			}
			require.NotNil(t, indexed.index[0], `the x-ray index must have been built`)
			require.GreaterOrEqual(t, len(kinds), 2, `answers must vary: %v`, kinds)
		})
	}
}

// TestParityIndexRefusesWhatItCannotPlace pins the cases the grid leaves to
// the plain scan, and that the scan then still answers alike.
func TestParityIndexRefusesWhatItCannotPlace(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	base, tris := parityCylinder(24)
	cases := map[string]func() ([]r3.Vec, [][3]int){
		`too small`: func() ([]r3.Vec, [][3]int) { return parityCylinder(8) },
		`one-point projection`: func() ([]r3.Vec, [][3]int) {
			// A flat facet on the line z = 0.5 comes first, so a dedup that
			// ran before the one-point check would hide the one-point facet.
			v := append(append([]r3.Vec(nil), base...),
				r3.Vec{X: 0, Y: -5, Z: 0.5}, r3.Vec{X: 1, Y: -4, Z: 0.5}, r3.Vec{X: 0, Y: -4.5, Z: 0.5},
				r3.Vec{X: -5, Y: 0.5, Z: 0.5}, r3.Vec{X: 5, Y: 0.5, Z: 0.5}, r3.Vec{X: 7, Y: 0.5, Z: 0.5})
			k := len(base)
			return v, append(append([][3]int(nil), tris...), [3]int{k, k + 1, k + 2}, [3]int{k + 3, k + 4, k + 5})
		},
		`overflowing line`: func() ([]r3.Vec, [][3]int) {
			v := append(append([]r3.Vec(nil), base...), r3.Vec{X: 1, Y: -1.7e308, Z: 0},
				r3.Vec{X: 1, Y: 1.7e308, Z: 1}, r3.Vec{X: 2, Y: 1.7e308, Z: 1})
			k := len(base)
			return v, append(append([][3]int(nil), tris...), [3]int{k, k + 1, k + 2})
		},
	}
	for name, mk := range cases {
		t.Run(name, func(t *testing.T) {
			v, tr := mk()
			pm := NewParityMesh(v, tr)
			require.Nil(t, pm.buildParityIndex(0, 1, 2))
			ref := NewParityMesh(v, tr)
			ref.Unfiltered = true
			all := parityAllFacets(len(tr))
			for _, p := range []proof.Xpt{
				proof.XptOf(r3.Vec{X: 0.1, Y: 0.5, Z: 0.5}),
				proof.XptOf(r3.Vec{X: 9, Y: 0.25, Z: 0.5}),
				proof.XptOf(r3.Vec{X: 0.1, Y: 0.2, Z: 3}),
			} {
				require.Equal(t, answerOf(MeshParityPreparedContext(ctx, p, ref, all)),
					answerOf(MeshParityAllContext(ctx, p, NewParityMesh(v, tr))))
			}
		})
	}
	t.Run(`a canceled context builds nothing`, func(t *testing.T) {
		canceled, cancel := context.WithCancel(ctx)
		cancel()
		pm := NewParityMesh(base, tris)
		_, _, err := MeshParityAllContext(canceled, proof.XptOf(r3.Vec{X: 0.1}), pm)
		require.ErrorIs(t, err, context.Canceled)
		require.Equal(t, [3]bool{}, pm.indexBuilt)
	})
}

// TestParityGridCellHoldsTheExactCoordinate checks the rounding step of the
// proof: an exact coordinate whose nearest float lands in cell k lies in
// [lo[k], hi[k]].
func TestParityGridCellHoldsTheExactCoordinate(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x4b1e, 0x2d2d))
	for _, scale := range []float64{1, 1e-300, 1e300} {
		ax := newParityGridAxis(-3*scale, 5*scale, 32)
		checked := 0
		for k := 1; k < len(ax.b)-1; k++ {
			for _, d := range []int64{-3, -1, 0, 1, 3} {
				for _, sh := range []uint{52, 53, 54, 60, 1100} {
					b := fr(ax.b[k])
					e := new(big.Rat).Abs(b)
					if e.Sign() == 0 {
						e.SetFrac64(1, 1)
					}
					e.Mul(e, new(big.Rat).SetFrac(big.NewInt(d), new(big.Int).Lsh(big.NewInt(1), sh)))
					r := new(big.Rat).Add(b, e)
					f, _ := r.Float64()
					c := ax.cellOf(f)
					require.True(t, math.IsInf(ax.lo[c], -1) || fr(ax.lo[c]).Cmp(r) <= 0, `scale %g k %d d %d sh %d`, scale, k, d, sh)
					require.True(t, math.IsInf(ax.hi[c], 1) || fr(ax.hi[c]).Cmp(r) >= 0, `scale %g k %d d %d sh %d`, scale, k, d, sh)
					checked++
				}
			}
		}
		for range 2000 {
			r := new(big.Rat).SetFrac(big.NewInt(rng.Int64N(1<<40)-(1<<39)), big.NewInt(1+rng.Int64N(1<<20)))
			r.Mul(r, fr(scale))
			f, _ := r.Float64()
			c := ax.cellOf(f)
			require.True(t, math.IsInf(ax.lo[c], -1) || fr(ax.lo[c]).Cmp(r) <= 0)
			require.True(t, math.IsInf(ax.hi[c], 1) || fr(ax.hi[c]).Cmp(r) >= 0)
		}
		require.Positive(t, checked)
	}
}

// TestParityLineMarginCoversTheError checks lineAt's margin against the exact
// height of the line through two float points, with a 256-fold headroom
// demanded, over slopes up to and beyond one and magnitudes from 1e-300 to
// 1e300.
func TestParityLineMarginCoversTheError(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x6a6a, 0x3c3c))
	worst := 0.0
	for i := range 20000 {
		scale := math.Ldexp(1, rng.IntN(1900)-950)
		f := func() float64 { return scale * (rng.Float64()*2 - 1) }
		a, b, ra, rb, x := f(), f(), f(), f(), f()
		if i%4 == 0 {
			rb = b + (ra-a)*float64(rng.IntN(3)-1)/3 // slopes ±1/3 and 0
		}
		du, dv := ra-a, rb-b
		if du == 0 || proofIsNonFinite(du) || proofIsNonFinite(dv) {
			continue
		}
		s := dv / du
		if proofIsNonFinite(s) {
			continue
		}
		y, m := lineAt(a, b, s, absf(b), 1+absf(s), x, absf(x))
		if proofIsNonFinite(y) || proofIsNonFinite(m) {
			continue
		}
		// exact: b + (x-a)*(rb-b)/(ra-a)
		exact := new(big.Rat).Sub(fr(rb), fr(b))
		exact.Mul(exact, new(big.Rat).Sub(fr(x), fr(a)))
		exact.Quo(exact, new(big.Rat).Sub(fr(ra), fr(a)))
		exact.Add(exact, fr(b))
		diff := new(big.Rat).Sub(fr(y), exact)
		diff.Abs(diff)
		ratio, _ := new(big.Rat).Quo(diff, fr(m)).Float64()
		worst = math.Max(worst, ratio)
		require.LessOrEqual(t, ratio, 1.0/256, `a=%g b=%g ra=%g rb=%g x=%g`, a, b, ra, rb, x)
	}
	t.Logf(`worst error / margin: %g`, worst)
}

func proofIsNonFinite(f float64) bool { return f != f || math.IsInf(f, 0) }

// TestParityIndexListsAscend requires every cell list to hold strictly
// ascending facet indices: the exact scan must meet facets in index order.
func TestParityIndexListsAscend(t *testing.T) {
	t.Parallel()
	for _, fx := range parityIndexFixtures() {
		pm := NewParityMesh(fx.verts, fx.tris)
		idx := pm.buildParityIndex(0, 1, 2)
		require.NotNil(t, idx, fx.name)
		longest := 0
		for c := range len(idx.start) - 1 {
			list := idx.ids[idx.start[c]:idx.start[c+1]]
			longest = max(longest, len(list))
			for i := 1; i < len(list); i++ {
				require.Less(t, list[i-1], list[i], `%s cell %d`, fx.name, c)
			}
		}
		require.Greater(t, longest, 1, fx.name)
	}
}

func TestParityIndexCountsAreSmall(t *testing.T) {
	t.Parallel()
	v, tr := parityCylinder(96)
	pm := NewParityMesh(v, tr)
	idx := pm.buildParityIndex(0, 1, 2)
	require.NotNil(t, idx)
	cands := idx.candidates(0.25, 0.5)
	require.Less(t, len(cands), len(tr)/8, fmt.Sprintf(`%d of %d`, len(cands), len(tr)))
}

// TestParityIndexUnboundedCells sends queries far beyond the shell, into the
// unbounded end cells of each axis, including coordinates whose nearest float
// is infinite (no float filter at all). Every left-out facet must end outside and every
// answer must match the plain scan.
func TestParityIndexUnboundedCells(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	huge := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 1100)) // nearest float is +Inf
	far := []*big.Rat{fr(40), fr(-40), fr(1e300), fr(-1e300), fr(math.MaxFloat64), huge, new(big.Rat).Neg(huge)}
	mid := []*big.Rat{fr(0.25), fr(-1.5), fr(2.5)}
	for _, fx := range parityIndexFixtures()[:3] {
		t.Run(fx.name, func(t *testing.T) {
			var qs []proof.Xpt
			for _, f := range far {
				for _, m := range mid {
					qs = append(qs, ratXpt(f, m, m), ratXpt(m, f, m), ratXpt(m, m, f), ratXpt(f, f, m),
						ratXpt(m, f, f), ratXpt(f, m, f), ratXpt(f, f, f))
				}
			}
			all := parityAllFacets(len(fx.tris))
			indexed := NewParityMesh(fx.verts, fx.tris)
			plain := NewParityMesh(fx.verts, fx.tris)
			plain.Unfiltered = true
			for qi, p := range qs {
				require.Equal(t, answerOf(MeshParityPreparedContext(ctx, p, plain, all)),
					answerOf(MeshParityAllContext(ctx, p, indexed)), `query %d`, qi)
			}
			endCell := 0
			for _, ray := range AxisRays {
				pm := NewParityMesh(fx.verts, fx.tris)
				idx := pm.buildParityIndex(ray.Axis, ray.U, ray.V)
				require.NotNil(t, idx)
				group := flatGroupFirst(pm, ray.Axis, ray.U, ray.V)
				for qi, p := range qs {
					pa := NewXP2(RatCoordOf(p, ray.U), RatCoordOf(p, ray.V))
					if !pa.FloatFinite {
						continue
					}
					cu, cv := idx.u.cellOf(pa.Fu), idx.v.cellOf(pa.Fv)
					if cu == 0 || cu == len(idx.u.b)-2 || cv == 0 || cv == len(idx.v.b)-2 {
						endCell++
					}
					requireLeftOutFacetsEndOutside(t, pm, idx, group, ray.Axis, ray.U, ray.V, ray.Dir, p, qi)
				}
			}
			require.Positive(t, endCell, `queries must land in unbounded cells`)
		})
	}
	// A non-finite shell coordinate is not reachable through the scan (the
	// exact lift of every vertex panics for it, indexed or not), but the build
	// must still refuse it rather than place it.
	t.Run(`infinite shell coordinate`, func(t *testing.T) {
		v, tr := parityCylinder(24)
		k := len(v)
		v = append(v, r3.Vec{X: 1, Y: math.Inf(1), Z: 1}, r3.Vec{X: 1, Y: 2, Z: 1}, r3.Vec{X: 2, Y: 1, Z: 0})
		tr = append(tr, [3]int{k, k + 1, k + 2})
		pm := NewParityMesh(v, tr)
		require.True(t, pm.buildParityIndex(0, 1, 2) == nil, `an infinite projected coordinate gets no index`)
	})
}
