package meshbool

import (
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// refCutter is the facet-interning half of TriCutter as it was keyed by
// decimal text: every point is projected, then interned by
// U.RatString()+"|"+V.RatString(). It is the reference the hashed tables
// must reproduce.
type refCutter struct {
	verts []CutVert
	index map[string]int
	edges []CutEdge
	swap  bool
	u, v  int
	steps int
}

func (r *refCutter) proj(p proof.Xpt) Xp2 {
	u, v := r.u, r.v
	if r.swap {
		u, v = v, u
	}
	return NewXP2FromXpt(p, u, v)
}

func (r *refCutter) addVert(p2 Xp2, p3 proof.Xpt, boundary bool) int {
	if r.index == nil {
		r.index = map[string]int{}
	}
	k := p2.U.RatString() + "|" + p2.V.RatString()
	if i, ok := r.index[k]; ok {
		if boundary {
			r.verts[i].Boundary = true
		}
		return i
	}
	r.index[k] = len(r.verts)
	r.verts = append(r.verts, CutVert{P2: p2, P3: p3, Boundary: boundary})
	return len(r.verts) - 1
}

func refInternFacet(xtri [3]proof.Xpt, normal proof.Xpt, segs []Xseg) (*refCutter, [3]int) {
	r := &refCutter{}
	r.u, r.v = ProjAxes(normal)
	corner := func(p proof.Xpt) Xp2 { return NewXP2FromXpt(p, r.u, r.v) }
	if Cross2xSign(corner(xtri[0]), corner(xtri[1]), corner(xtri[2])) < 0 {
		r.swap = true
	}
	var cornerIdx [3]int
	for i := range 3 {
		r.steps++
		cornerIdx[i] = r.addVert(r.proj(xtri[i]), xtri[i], true)
	}
	seen := map[[2]int]struct{}{}
	for _, s := range segs {
		r.steps++
		ia := r.addVert(r.proj(s.A), s.A, s.AOnEdge)
		ib := r.addVert(r.proj(s.B), s.B, s.BOnEdge)
		if ia == ib {
			continue
		}
		k := [2]int{min(ia, ib), max(ia, ib)}
		if _, ok := seen[k]; ok {
			continue
		}
		seen[k] = struct{}{}
		r.edges = append(r.edges, CutEdge{A: ia, B: ib, Partner: s.Partner, ViaParity: s.ViaParity})
	}
	return r, cornerIdx
}

// countingCutter is a TriCutter whose budget counts its steps.
func countingCutter(normal proof.Xpt, projections *cutProjectionScratch) (*TriCutter, *int) {
	steps := new(int)
	tc := &TriCutter{
		Work: &proofbound.WorkBudget{
			StepFn: func() error { *steps++; return nil },
			ErrFn:  func() error { return nil },
		},
		projections: projections,
	}
	tc.U, tc.V = ProjAxes(normal)
	return tc, steps
}

// requireSameVert compares one vertex field by field and reports only the
// vertex index and the field that differs.
func requireSameVert(t *testing.T, want, got CutVert, label string) {
	t.Helper()
	require.Equal(t, want.P2.U.RatString(), got.P2.U.RatString(), "%s U", label)
	require.Equal(t, want.P2.V.RatString(), got.P2.V.RatString(), "%s V", label)
	require.Equal(t, math.Float64bits(want.P2.Fu), math.Float64bits(got.P2.Fu), "%s Fu", label)
	require.Equal(t, math.Float64bits(want.P2.Fv), math.Float64bits(got.P2.Fv), "%s Fv", label)
	require.Equal(t, want.P2.FloatFinite, got.P2.FloatFinite, "%s FloatFinite", label)
	require.Equal(t, want.P2.floatFromExact, got.P2.floatFromExact, "%s floatFromExact", label)
	for k, pair := range [3][2]*big.Int{{want.P2.Hu, got.P2.Hu}, {want.P2.Hv, got.P2.Hv}, {want.P2.Hw, got.P2.Hw}} {
		require.Zero(t, pair[0].Cmp(pair[1]), "%s homogeneous %d", label, k)
	}
	// The vertex keeps the very point first interned, not an equal copy.
	require.True(t, want.P3.X == got.P3.X && want.P3.Y == got.P3.Y && want.P3.Z == got.P3.Z && want.P3.W == got.P3.W,
		"%s P3 identity", label)
	require.Equal(t, want.Boundary, got.Boundary, "%s Boundary", label)
}

func requireSameInterning(t *testing.T, f cutFixtureFacet, label string) {
	t.Helper()
	ref, refCorners := refInternFacet(f.xtri, f.normal, f.segs)
	for _, projections := range []*cutProjectionScratch{nil, {}} {
		tc, steps := countingCutter(f.normal, projections)
		corners, err := tc.internFacet(f.xtri, f.segs)
		require.NoError(t, err, label)
		require.Equal(t, ref.swap, tc.Swap, "%s Swap", label)
		require.Equal(t, refCorners, corners, "%s corners", label)
		require.Equal(t, ref.steps, *steps, "%s steps", label)
		require.Len(t, tc.Verts, len(ref.verts), label)
		for i := range ref.verts {
			requireSameVert(t, ref.verts[i], tc.Verts[i], fmt.Sprintf("%s vertex %d", label, i))
		}
		require.Equal(t, ref.edges, tc.Edges, "%s edges", label)
	}
}

func copyXpt(p proof.Xpt) proof.Xpt {
	return proof.Xpt{X: new(big.Int).Set(p.X), Y: new(big.Int).Set(p.Y), Z: new(big.Int).Set(p.Z), W: new(big.Int).Set(p.W)}
}

func scaleXpt(p proof.Xpt, k int64) proof.Xpt {
	s := big.NewInt(k)
	return proof.Xpt{X: new(big.Int).Mul(p.X, s), Y: new(big.Int).Mul(p.Y, s), Z: new(big.Int).Mul(p.Z, s), W: new(big.Int).Mul(p.W, s)}
}

// cutInterningVariants rewrites one facet's input in ways that reach every
// interning branch: the reversed winding sets Swap; fresh copies give equal
// integers behind new pointers (the raw table's hits); scaled points give
// one value two integer forms (raw misses that the 2D identity must merge);
// corner endpoints, collapsed and repeated segments, and random boundary
// flags exercise the sticky boundary and the edge dedup.
func cutInterningVariants(f cutFixtureFacet, rng *rand.Rand) map[string]cutFixtureFacet {
	out := map[string]cutFixtureFacet{"as built": f}
	rev := f
	rev.xtri = [3]proof.Xpt{f.xtri[0], f.xtri[2], f.xtri[1]}
	out["reversed"] = rev

	edit := func(fn func(i int, s Xseg) Xseg) []Xseg {
		segs := make([]Xseg, len(f.segs))
		for i, s := range f.segs {
			segs[i] = fn(i, s)
		}
		return segs
	}
	copies := f
	copies.segs = edit(func(_ int, s Xseg) Xseg { s.A, s.B = copyXpt(s.A), copyXpt(s.B); return s })
	out["fresh copies"] = copies
	scaled := f
	scaled.segs = edit(func(i int, s Xseg) Xseg {
		if i%2 == 0 {
			s.A = scaleXpt(s.A, 3)
		} else {
			s.B = scaleXpt(s.B, 5)
		}
		return s
	})
	out["scaled"] = scaled
	flags := f
	flags.segs = edit(func(_ int, s Xseg) Xseg { s.AOnEdge, s.BOnEdge = rng.IntN(2) == 0, rng.IntN(2) == 0; return s })
	out["boundary flags"] = flags

	extra := f
	extra.segs = append([]Xseg(nil), f.segs...)
	s0 := f.segs[0]
	extra.segs = append(extra.segs,
		Xseg{A: copyXpt(f.xtri[1]), B: copyXpt(s0.A), AOnEdge: false, BOnEdge: true, Partner: s0.Partner},
		Xseg{A: s0.B, B: copyXpt(s0.B), Partner: s0.Partner},
		Xseg{A: copyXpt(s0.B), B: scaleXpt(s0.A, 7), Partner: s0.Partner, ViaParity: true},
		Xseg{A: scaleXpt(f.xtri[2], 2), B: f.xtri[0], AOnEdge: true, Partner: s0.Partner},
	)
	rng.Shuffle(len(extra.segs), func(i, j int) { extra.segs[i], extra.segs[j] = extra.segs[j], extra.segs[i] })
	out["corners and repeats"] = extra
	return out
}

// TestInternFacetMatchesTextKeyedInterning checks the hashed interning
// against the decimal-text reference on crossing-grid facets and their
// variants: the same Swap, corners, step count, vertex table and edges.
func TestInternFacetMatchesTextKeyedInterning(t *testing.T) {
	t.Parallel()
	facets := cutFixtureFacets(t, 12)
	require.Greater(t, len(facets), 40)
	rng := rand.New(rand.NewPCG(5, 11))
	for fi, f := range facets {
		for name, v := range cutInterningVariants(f, rng) {
			requireSameInterning(t, v, fmt.Sprintf("facet %d %s", fi, name))
		}
	}
}

// TestCutTriangleRegionsMatchAcrossIntegerForms cuts each facet with its
// endpoints as built, as fresh copies and scaled: one value, three integer
// forms. The raw table hits on the first two and misses on the third, and the
// regions must not depend on which.
func TestCutTriangleRegionsMatchAcrossIntegerForms(t *testing.T) {
	t.Parallel()
	facets := cutFixtureFacets(t, 12)
	rng := rand.New(rand.NewPCG(3, 7))
	var scratch cutProjectionScratch
	for fi, f := range facets {
		variants := cutInterningVariants(f, rng)
		want, err := CutTriangle(t.Context(), f.xtri, f.normal, f.segs)
		require.NoError(t, err)
		for _, name := range []string{"fresh copies", "scaled"} {
			v := variants[name]
			got, err := cutTriangle(t.Context(), v.xtri, v.normal, v.segs, &scratch)
			require.NoError(t, err)
			// A region corner keeps the integer form first interned, so
			// compare exact values, not integers.
			requireSameRegionValues(t, want, got, fmt.Sprintf("facet %d %s", fi, name))
		}
	}
}

// requireSameRegionValues compares regions by exact point value, so an
// endpoint scaled to another integer form still matches.
func requireSameRegionValues(t *testing.T, want, got []CutRegion, label string) {
	t.Helper()
	require.Len(t, got, len(want), label)
	same := func(a, b proof.Xpt) bool { return a.Key() == b.Key() }
	for r := range want {
		require.Equal(t, want[r].HasAnchor, got[r].HasAnchor, "%s region %d anchor", label, r)
		require.Equal(t, want[r].Partner, got[r].Partner, "%s region %d partner", label, r)
		require.True(t, same(want[r].Probe, got[r].Probe), "%s region %d probe", label, r)
		require.Len(t, got[r].Tris, len(want[r].Tris), "%s region %d", label, r)
		for k := range want[r].Tris {
			for c := range 3 {
				require.True(t, same(want[r].Tris[k][c], got[r].Tris[k][c]), "%s region %d tri %d corner %d", label, r, k, c)
			}
		}
	}
}

// nonReducedRat is n/d with its numerator and denominator written as given,
// not reduced, the form TestChainProbeMidpointOfNonReducedEndpoint pins.
func nonReducedRat(n, d int64) *big.Rat {
	r := big.NewRat(1, 7)
	r.Num().SetInt64(n)
	r.Denom().SetInt64(d)
	return r
}

// TestAddVertMatchesTextKeyedIndex replays random vertex sequences, with
// equal values behind distinct rationals, integers with and without a stored
// denominator, and non-reduced forms, through AddVert and the text-keyed
// reference: the same index for every call and the same table.
func TestAddVertMatchesTextKeyedIndex(t *testing.T) {
	t.Parallel()
	pool := []func() *big.Rat{
		func() *big.Rat { return new(big.Rat) },
		func() *big.Rat { return big.NewRat(0, 5) },
		func() *big.Rat { return new(big.Rat).SetInt64(3) },
		func() *big.Rat { return big.NewRat(3, 1) },
		func() *big.Rat { return big.NewRat(-3, 1) },
		func() *big.Rat { return big.NewRat(1, 3) },
		func() *big.Rat { return big.NewRat(-1, 3) },
		func() *big.Rat { return nonReducedRat(3, 9) },
		func() *big.Rat { return nonReducedRat(6, 18) },
		func() *big.Rat { return nonReducedRat(9, 3) },
		func() *big.Rat { return big.NewRat(2, 3) },
		func() *big.Rat {
			r, _ := new(big.Rat).SetString("123456789012345678901234567890/98765432109876543210987")
			return r
		},
		func() *big.Rat {
			r, _ := new(big.Rat).SetString("123456789012345678901234567891/98765432109876543210987")
			return r
		},
	}
	rng := rand.New(rand.NewPCG(1, 2))
	for round := range 200 {
		ref := &refCutter{}
		tc := &TriCutter{}
		for call := range 60 {
			p2 := NewXP2(pool[rng.IntN(len(pool))](), pool[rng.IntN(len(pool))]())
			p3 := proof.XptOf(r3.NewVec(float64(call), 0, 0))
			boundary := rng.IntN(3) == 0
			want := ref.addVert(p2, p3, boundary)
			got := tc.AddVert(p2, p3, boundary)
			require.Equal(t, want, got, "round %d call %d", round, call)
		}
		require.Len(t, tc.Verts, len(ref.verts), "round %d", round)
		for i := range ref.verts {
			requireSameVert(t, ref.verts[i], tc.Verts[i], fmt.Sprintf("round %d vertex %d", round, i))
		}
	}
}

// TestCutVertIndexConfirmsBothCoordinates forces every vertex onto one hash,
// so only the exact comparison of U and of V can tell them apart.
func TestCutVertIndexConfirmsBothCoordinates(t *testing.T) {
	t.Parallel()
	const h = 42
	verts := []CutVert{
		{P2: NewXP2(big.NewRat(1, 2), big.NewRat(1, 3))},
		{P2: NewXP2(big.NewRat(1, 5), big.NewRat(1, 3))}, // V shared with vertex 0
		{P2: NewXP2(big.NewRat(1, 2), big.NewRat(1, 7))}, // U shared with vertex 0
	}
	var x cutVertIndex
	for i := range verts {
		x.add(h, i)
	}
	for i, v := range verts {
		got, ok := x.lookup(verts, h, NewXP2(new(big.Rat).Set(v.P2.U), new(big.Rat).Set(v.P2.V)))
		require.True(t, ok, "vertex %d", i)
		require.Equal(t, i, got, "vertex %d", i)
	}
	_, ok := x.lookup(verts, h, NewXP2(big.NewRat(1, 5), big.NewRat(1, 7)))
	require.False(t, ok, "a pair whose coordinates each match some vertex")
	_, ok = x.lookup(verts, h+1, verts[0].P2)
	require.False(t, ok, "another hash")
}

// TestCutRawIndexConfirmsEveryInteger forces every entry onto one hash, so
// only the exact comparison of each integer can tell them apart.
func TestCutRawIndexConfirmsEveryInteger(t *testing.T) {
	t.Parallel()
	const h = 7
	n := func(v int64) *big.Int { return big.NewInt(v) }
	entries := [][3]*big.Int{{n(1), n(2), n(3)}, {n(9), n(2), n(3)}, {n(1), n(9), n(3)}, {n(1), n(2), n(9)}}
	var x cutRawIndex
	for i, e := range entries {
		x.add(h, e[0], e[1], e[2], 10+i)
	}
	for i, e := range entries {
		got, ok := x.lookup(h, new(big.Int).Set(e[0]), new(big.Int).Set(e[1]), new(big.Int).Set(e[2]))
		require.True(t, ok, "entry %d", i)
		require.Equal(t, 10+i, got, "entry %d", i)
	}
	for _, miss := range [][3]*big.Int{{n(9), n(9), n(3)}, {n(9), n(2), n(9)}, {n(1), n(9), n(9)}} {
		_, ok := x.lookup(h, miss[0], miss[1], miss[2])
		require.False(t, ok, "%v", miss)
	}
}
