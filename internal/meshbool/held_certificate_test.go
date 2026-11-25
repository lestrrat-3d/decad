package meshbool

// CI: isolated-tests

import (
	"context"
	"errors"
	"math"
	"math/big"
	"math/rand/v2"
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// Every padded box contains the exact box widened by 2τ on each side, also where
// 2τ is far below the spacing of the coordinates, so the padding is never
// rounded down.
func TestCertificatePaddedBoxesRoundOutward(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(11, 3))
	pad := proof.FloatRat(2 * CertificateTau)
	for _, scale := range []float64{1, 0x1p10, 0x1p20, 0x1p30, 0x1p40, 0x1p-5} {
		verts, tris := certRandomSoup(rng, 40, 3)
		for i := range verts {
			verts[i] = verts[i].Scale(scale * (1 + rng.Float64()))
		}
		boxes := certPaddedBoxes(verts, tris)
		for i, tri := range tris {
			raw := TriBox(verts, tri)
			for k, p := range [2][3]float64{{boxes[i][0].X, boxes[i][0].Y, boxes[i][0].Z},
				{boxes[i][1].X, boxes[i][1].Y, boxes[i][1].Z}} {
				rawAxes := [3]float64{raw[k].X, raw[k].Y, raw[k].Z}
				for axis := range 3 {
					want := new(big.Rat).Set(proof.FloatRat(rawAxes[axis]))
					if k == 0 {
						want.Sub(want, pad)
						require.LessOrEqual(t, proof.FloatRat(p[axis]).Cmp(want), 0, "scale %g facet %d", scale, i)
					} else {
						want.Add(want, pad)
						require.GreaterOrEqual(t, proof.FloatRat(p[axis]).Cmp(want), 0, "scale %g facet %d", scale, i)
					}
				}
			}
		}
	}
}

// The padded box tree visits exactly the non-adjacent pairs whose padded boxes
// overlap, whatever the node partition.
func TestCertificatePairEnumerationMatchesBruteForce(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(5, 8))
	type mesh struct {
		verts []r3.Vec
		tris  [][3]int
	}
	var meshes []mesh
	for _, spread := range []float64{1, 4, 12} {
		for _, count := range []int{1, 7, 17, 90, 260} {
			v, tr := certRandomSoup(rng, count, spread)
			meshes = append(meshes, mesh{v, tr})
		}
	}
	// Pairs whose boxes end within 4τ of each other, and pairs that touch.
	near, nearTris := certRandomSoup(rng, 40, 1)
	for i := range 40 {
		shifted := near[3*i : 3*i+3]
		for k := range shifted {
			shifted[k].X += float64(i%5) * 3 * CertificateTau
		}
	}
	meshes = append(meshes, mesh{near, nearTris})
	sv, st := certIcosphere(2, 1, r3.Vec{})
	meshes = append(meshes, mesh{sv, st})
	for mi, m := range meshes {
		boxes := certPaddedBoxes(m.verts, m.tris)
		want := map[[2]int]struct{}{}
		for i := range m.tris {
			for j := i + 1; j < len(m.tris); j++ {
				if !sharesCorner(m.tris[i], m.tris[j]) && BoxesOverlap(boxes[i], boxes[j]) {
					want[[2]int{i, j}] = struct{}{}
				}
			}
		}
		for _, limit := range []int{1, 16, 64} {
			tree := newBoxTree(boxes)
			tasks, err := tree.selfPairTasks(t.Context(), limit)
			require.NoError(t, err)
			got := map[[2]int]struct{}{}
			for _, task := range tasks {
				require.NoError(t, certScanTask(t.Context(), m.tris, &tree, task, func(i, j int) bool {
					key := [2]int{min(i, j), max(i, j)}
					_, dup := got[key]
					require.False(t, dup, "mesh %d pair %v visited twice", mi, key)
					got[key] = struct{}{}
					return true
				}))
			}
			require.Equal(t, len(want), len(got), "mesh %d limit %d", mi, limit)
			for key := range want {
				_, ok := got[key]
				require.True(t, ok, "mesh %d limit %d misses pair %v", mi, limit, key)
			}
		}
	}
}

// The adjacent list holds every pair of facets that share at least one index
// (one, two or three), once each, duplicates and reversed windings included.
func TestCertificateAdjacentPairsCoverEverySharedIndex(t *testing.T) {
	t.Parallel()
	verts, tris := certIcosphere(2, 1, r3.Vec{})
	tris = append(tris, tris[7], [3]int{tris[9][0], tris[9][2], tris[9][1]})
	got := certAdjacentPairs(verts, tris)
	require.NotNil(t, got)
	seen := map[[2]int32]struct{}{}
	for _, p := range got {
		require.Less(t, p[0], p[1])
		_, dup := seen[p]
		require.False(t, dup, "pair %v listed twice", p)
		seen[p] = struct{}{}
	}
	for i := range tris {
		for j := i + 1; j < len(tris); j++ {
			_, listed := seen[[2]int32{int32(i), int32(j)}]
			require.Equal(t, sharesCorner(tris[i], tris[j]), listed, "pair %d %d", i, j)
		}
	}
}

// A source holding a facet twice, or the same facet reversed, shares three
// indices between two facets. Today's scan refuses that pair, so no copy of such
// a source may be certified; a facet that repeats an index is refused outright.
func TestCertificateDuplicateAndRepeatedFacets(t *testing.T) {
	t.Parallel()
	verts, tris := certIcosphere(2, 3, r3.Vec{X: 1})
	basis := certRotationBasis(t, r3.Vec{X: 1, Y: 2, Z: 3}, 37)
	for name, extra := range map[string][3]int{
		"duplicate": tris[11],
		"reversed":  {tris[11][0], tris[11][2], tris[11][1]},
	} {
		src := append(append([][3]int(nil), tris...), extra)
		cert, err := BuildPlacementCertificate(t.Context(), verts, src)
		require.NoError(t, err)
		require.NotNil(t, cert, name)
		held, copyTris, allow := certPlace(basis, r3.Vec{X: 5, Y: -2, Z: 1}, verts, src)
		ok, err := cert.Check(t.Context(), basis, allow, held, copyTris)
		require.NoError(t, err)
		require.False(t, ok, name)
		clean := certScanClean(t, held, copyTris)
		require.False(t, clean, name)
	}
	repeated := append(append([][3]int(nil), tris...), [3]int{3, 3, 4})
	cert, err := BuildPlacementCertificate(t.Context(), verts, repeated)
	require.ErrorIs(t, err, ErrNoCertificate)
	require.Nil(t, cert)
}

func TestCertificateRefusesUnusableSources(t *testing.T) {
	t.Parallel()
	verts, tris := certIcosphere(2, 1, r3.Vec{})
	ok, err := BuildPlacementCertificate(t.Context(), verts, tris)
	require.NoError(t, err)
	require.NotNil(t, ok)
	bad := map[string]func([]r3.Vec, [][3]int) ([]r3.Vec, [][3]int){
		"nan": func(v []r3.Vec, tr [][3]int) ([]r3.Vec, [][3]int) {
			v = append([]r3.Vec(nil), v...)
			v[5].Y = math.NaN()
			return v, tr
		},
		"infinite": func(v []r3.Vec, tr [][3]int) ([]r3.Vec, [][3]int) {
			v = append([]r3.Vec(nil), v...)
			v[5].Z = math.Inf(1)
			return v, tr
		},
		"huge": func(v []r3.Vec, tr [][3]int) ([]r3.Vec, [][3]int) {
			v = append([]r3.Vec(nil), v...)
			v[5].X = 1e70
			return v, tr
		},
		"index out of range": func(v []r3.Vec, tr [][3]int) ([]r3.Vec, [][3]int) {
			tr = append([][3]int(nil), tr...)
			tr[3][1] = len(v)
			return v, tr
		},
		"negative index": func(v []r3.Vec, tr [][3]int) ([]r3.Vec, [][3]int) {
			tr = append([][3]int(nil), tr...)
			tr[3][1] = -1
			return v, tr
		},
		"no facets": func(v []r3.Vec, _ [][3]int) ([]r3.Vec, [][3]int) { return v, nil },
	}
	for name, mutate := range bad {
		v, tr := mutate(verts, tris)
		cert, err := BuildPlacementCertificate(t.Context(), v, tr)
		require.ErrorIs(t, err, ErrNoCertificate, name)
		require.Nil(t, cert, name)
	}
}

// certPair builds the offsets of two triangles from a's first corner, as the
// separation test reads them.
func certPair(a, b [3]r3.Vec) [6]r3.Vec {
	o := a[0]
	return [6]r3.Vec{{}, a[1].Sub(o), a[2].Sub(o), b[0].Sub(o), b[1].Sub(o), b[2].Sub(o)}
}

// disjointExactly decides the pair with the held embedding scan's own exact
// predicate, over the floats given.
func disjointExactly(t *testing.T, a, b [3]r3.Vec) bool {
	t.Helper()
	verts := []r3.Vec{a[0], a[1], a[2], b[0], b[1], b[2]}
	e := newHeldEmbeddingScan(HeldRounding{Verts: verts, Tris: [][3]int{{0, 1, 2}, {3, 4, 5}}},
		proofbound.NewWorkBudget(t.Context()), false)
	return e.pairEmbedded(0, 1)
}

type certKind struct {
	name string
	// build returns the pair with a normal gap of g millimetres along axis, and
	// the axis whose test is the one under examination.
	build func(g float64) (a, b [3]r3.Vec, axis func(d *[6]r3.Vec) r3.Vec)
}

func certKinds() []certKind {
	tri := [3]r3.Vec{{}, {X: 1}, {Y: 1}}
	return []certKind{
		{"normal of the first triangle", func(g float64) ([3]r3.Vec, [3]r3.Vec, func(*[6]r3.Vec) r3.Vec) {
			b := [3]r3.Vec{{X: 0.2, Y: 0.2, Z: g}, {X: 0.5, Y: 0.2, Z: g}, {X: 0.2, Y: 0.5, Z: g}}
			return tri, b, func(d *[6]r3.Vec) r3.Vec { return d[1].Cross(d[2].Sub(d[1])) }
		}},
		{"normal of the second triangle", func(g float64) ([3]r3.Vec, [3]r3.Vec, func(*[6]r3.Vec) r3.Vec) {
			// The first triangle's plane crosses the large second triangle.
			a := [3]r3.Vec{{Z: g}, {X: 0.1, Z: g}, {Y: 0.1, Z: g + 0.1}}
			b := [3]r3.Vec{{X: -10, Y: -10}, {X: 10, Y: -10}, {Y: 10}}
			return a, b, func(d *[6]r3.Vec) r3.Vec { return d[4].Sub(d[3]).Cross(d[5].Sub(d[4])) }
		}},
		{"edge normal inside a shared plane", func(g float64) ([3]r3.Vec, [3]r3.Vec, func(*[6]r3.Vec) r3.Vec) {
			s := g / math.Sqrt2
			b := [3]r3.Vec{{X: 0.5 + s, Y: 0.5 + s}, {X: 1 + s, Y: 0.4 + s}, {X: 0.4 + s, Y: 1 + s}}
			return tri, b, func(d *[6]r3.Vec) r3.Vec {
				na := d[1].Cross(d[2].Sub(d[1]))
				return na.Cross(d[2].Sub(d[1]))
			}
		}},
		{"cross of two edges", func(g float64) ([3]r3.Vec, [3]r3.Vec, func(*[6]r3.Vec) r3.Vec) {
			a := [3]r3.Vec{{X: -1}, {X: 1}, {Y: 1, Z: -1}}
			b := [3]r3.Vec{{Y: -1, Z: g}, {Y: 1, Z: g}, {X: 1, Z: g + 1}}
			return a, b, func(d *[6]r3.Vec) r3.Vec { return d[1].Sub(d[0]).Cross(d[4].Sub(d[3])) }
		}},
	}
}

// For each axis kind, a pair whose gap is under 2τ is not certified, a pair
// well over it is, and a certified pair stays disjoint when every vertex is
// moved by the certified radius toward the other triangle, along the axis and
// along random directions, and in every sign corner.
func TestCertificateSeparationKindsAtTheThreshold(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(2, 77))
	for _, kind := range certKinds() {
		// The test charges |n|₁ for |n|₂, which costs up to a factor √2 on an
		// in-plane diagonal axis, so only gaps under 2τ and over 3.2τ are decided.
		for _, factor := range []float64{0.5, 1.5, 1.9, 3.2, 4, 40} {
			g := factor * CertificateTau
			a, b, axisOf := kind.build(g)
			d := certPair(a, b)
			n := axisOf(&d)
			certified := certAxis(&d, n)
			if factor < 2 {
				require.False(t, certified, "%s gap %g tau", kind.name, factor)
			} else {
				require.True(t, certified, "%s gap %g tau", kind.name, factor)
			}
			if factor < 2 {
				continue
			}
			require.True(t, certSeparated(a, b), kind.name)
			dirs := []r3.Vec{n.Scale(1 / n.Len()), n.Scale(-1 / n.Len())}
			for range 40 {
				dirs = append(dirs, r3.Vec{X: rng.NormFloat64(), Y: rng.NormFloat64(), Z: rng.NormFloat64()})
			}
			for sx := -1.0; sx <= 1; sx += 2 {
				for sy := -1.0; sy <= 1; sy += 2 {
					for sz := -1.0; sz <= 1; sz += 2 {
						dirs = append(dirs, r3.Vec{X: sx, Y: sy, Z: sz})
					}
				}
			}
			radius := CertificateTau * (1 - 1e-6)
			for _, da := range dirs {
				for _, db := range dirs[:2+40] {
					na, nb := a, b
					ua, ub := da.Scale(radius/da.Len()), db.Scale(radius/db.Len())
					for k := range 3 {
						na[k], nb[k] = a[k].Add(ua), b[k].Add(ub)
					}
					require.True(t, disjointExactly(t, na, nb), "%s gap %g tau: nudge %v %v", kind.name, factor, ua, ub)
				}
			}
		}
	}
}

// Two triangles a thin gap apart along a face normal are broken by the radius
// the certificate refuses to cover, which is why a gap under 2τ is thin.
func TestCertificateNudgeBreaksAThinGap(t *testing.T) {
	t.Parallel()
	a, b, _ := certKinds()[0].build(1.5 * CertificateTau)
	require.True(t, disjointExactly(t, a, b))
	// Each vertex moves by τ: the first triangle up, and one corner of the second
	// down through it.
	for k := range 3 {
		a[k].Z += CertificateTau
	}
	b[0].Z -= CertificateTau
	require.False(t, disjointExactly(t, a, b))
}

// A margin that equals the threshold exactly is not certified: the test is
// strict.
func TestCertificateMarginEqualToThresholdIsNotCertified(t *testing.T) {
	t.Parallel()
	a := [3]r3.Vec{{}, {X: 1}, {Y: 1}}
	found := false
	_, threshold := certMargin(&[6]r3.Vec{}, r3.Vec{Z: 1})
	for k := range int(1 << 15) {
		z := math.Float64frombits(math.Float64bits(threshold) + uint64(k))
		b := [3]r3.Vec{{X: 0.2, Y: 0.2, Z: z}, {X: 0.5, Y: 0.2, Z: z}, {X: 0.2, Y: 0.5, Z: z}}
		d := certPair(a, b)
		gap, threshold := certMargin(&d, r3.Vec{Z: 1})
		if gap != threshold {
			continue
		}
		found = true
		require.False(t, certAxis(&d, r3.Vec{Z: 1}), "gap equals threshold %g", gap)
		break
	}
	require.True(t, found, "no pair with a gap equal to its threshold in the search window")
}

func TestCertificateUnusableNumbersAreNotCertified(t *testing.T) {
	t.Parallel()
	good := [3]r3.Vec{{}, {X: 1}, {Y: 1}}
	far := [3]r3.Vec{{Z: 5}, {X: 1, Z: 5}, {Y: 1, Z: 5}}
	require.True(t, certSeparated(good, far))
	scaled := func(s float64, tri [3]r3.Vec) [3]r3.Vec {
		return [3]r3.Vec{tri[0].Scale(s), tri[1].Scale(s), tri[2].Scale(s)}
	}
	nan, inf := math.NaN(), math.Inf(1)
	cases := map[string][2][3]r3.Vec{
		"nan corner":        {good, {{X: nan, Z: 5}, {X: 1, Z: 5}, {Y: 1, Z: 5}}},
		"infinite corner":   {good, {{Z: inf}, {X: 1, Z: 5}, {Y: 1, Z: 5}}},
		"huge":              {scaled(1e300, good), scaled(1e300, far)},
		"tiny":              {scaled(1e-100, good), scaled(1e-100, far)},
		"subnormal":         {scaled(1e-310, good), scaled(1e-310, far)},
		"tiny next to huge": {scaled(1e-100, good), scaled(1e200, far)},
	}
	for name, c := range cases {
		require.False(t, certSeparated(c[0], c[1]), name)
	}
	d := certPair(good, far)
	for name, n := range map[string]r3.Vec{"nan": {Z: nan}, "inf": {Z: inf}, "huge": {Z: 1e150}, "tiny": {Z: 1e-150}} {
		gap, threshold := certMargin(&d, n)
		require.False(t, gap > threshold, name)
		require.False(t, certAxis(&d, n), name)
	}
}

func TestCertificateRefusesLargeOrchestratedDefects(t *testing.T) {
	t.Parallel()
	verts, tris := certIcosphere(2, 3, r3.Vec{})
	cert, err := BuildPlacementCertificate(t.Context(), verts, tris)
	require.NoError(t, err)
	require.NotNil(t, cert)
	scale := func(s float64) r3.Basis { return r3.Basis{EX: r3.Vec{X: s}, EY: r3.Vec{Y: 1}, EZ: r3.Vec{Z: 1}} }
	for name, tc := range map[string]struct {
		scale float64
		want  bool
	}{
		"defect 0.49": {1.2207, true}, "defect over a half": {1.2250, false}, "defect 0.69": {1.3, false},
	} {
		held, copyTris, allow := certPlace(scale(tc.scale), r3.Vec{}, verts, tris)
		ok, err := cert.Check(t.Context(), scale(tc.scale), allow, held, copyTris)
		require.NoError(t, err)
		require.Equal(t, tc.want, ok, name)
	}
	held, copyTris, allow := certPlace(scale(1), r3.Vec{}, verts, tris)
	for name, b := range map[string]r3.Basis{
		"nan basis": {EX: r3.Vec{X: math.NaN()}, EY: r3.Vec{Y: 1}, EZ: r3.Vec{Z: 1}},
		"inf basis": {EX: r3.Vec{X: math.Inf(1)}, EY: r3.Vec{Y: 1}, EZ: r3.Vec{Z: 1}},
	} {
		ok, err := cert.Check(t.Context(), b, allow, held, copyTris)
		require.NoError(t, err)
		require.False(t, ok, name)
	}
	for name, a := range map[string]float64{"nan allowance": math.NaN(), "negative": -1, "infinite": math.Inf(1)} {
		ok, err := cert.Check(t.Context(), scale(1), a, held, copyTris)
		require.NoError(t, err)
		require.False(t, ok, name)
	}
}

// A copy whose rounding allowance exceeds τ is not certified, and one just under
// it is.
func TestCertificateRefusesRoundingBeyondTau(t *testing.T) {
	t.Parallel()
	verts, tris := certIcosphere(2, 3, r3.Vec{})
	cert, err := BuildPlacementCertificate(t.Context(), verts, tris)
	require.NoError(t, err)
	basis := certRotationBasis(t, r3.Vec{X: 1, Y: 1}, 20)
	for name, tc := range map[string]struct {
		shift float64
		want  bool
	}{"100 m": {1e5, true}, "300 m": {3e5, false}, "origin": {0, true}} {
		tr := r3.Vec{X: tc.shift, Y: -tc.shift / 3, Z: tc.shift / 2}
		held, copyTris, allow := certPlace(basis, tr, verts, tris)
		ok, err := cert.Check(t.Context(), basis, allow, held, copyTris)
		require.NoError(t, err)
		require.Equal(t, tc.want, ok, name)
	}
}

// A reflected copy swaps two corners of every facet. The certificate reads the
// adjacency of the source, so the reflected copy is certified; a facet list that
// is neither the source nor its reflection is not.
func TestCertificateReflectedCopyUsesSourceAdjacency(t *testing.T) {
	t.Parallel()
	verts, tris := certIcosphere(3, 2, r3.Vec{X: 1})
	cert, err := BuildPlacementCertificate(t.Context(), verts, tris)
	require.NoError(t, err)
	require.NotNil(t, cert)
	mirror := r3.Basis{EX: r3.Vec{X: -1}, EY: r3.Vec{Y: 1}, EZ: r3.Vec{Z: 1}}
	rot := certRotationBasis(t, r3.Vec{Z: 1}, 31)
	mirrored := r3.Basis{EX: rot.EX.Scale(-1), EY: rot.EY, EZ: rot.EZ}
	for name, basis := range map[string]r3.Basis{"mirror": mirror, "mirror and turn": mirrored} {
		held, copyTris, allow := certPlace(basis, r3.Vec{X: 3, Y: 4}, verts, tris)
		require.NotEqual(t, tris[0], copyTris[0], name)
		ok, err := cert.Check(t.Context(), basis, allow, held, copyTris)
		require.NoError(t, err)
		require.True(t, ok, name)
		clean := certScanClean(t, held, copyTris)
		require.True(t, clean, name)
	}
	held, _, allow := certPlace(rot, r3.Vec{}, verts, tris)
	other := append([][3]int(nil), tris...)
	other[4] = [3]int{tris[4][1], tris[4][0], tris[4][2]}
	other[5] = [3]int{tris[5][2], tris[5][1], tris[5][0]}
	for name, bad := range map[string][][3]int{"permuted corners": other, "fewer facets": tris[:len(tris)-1]} {
		ok, err := cert.Check(t.Context(), rot, allow, held, bad)
		require.NoError(t, err)
		require.False(t, ok, name)
	}
	ok, err := cert.Check(t.Context(), rot, allow, held[:len(held)-1], tris)
	require.NoError(t, err)
	require.False(t, ok)
}

// plates are two parallel unit squares, each two facets, a gap apart. The
// source is embedded for any positive gap.
func certPlates(gap float64) ([]r3.Vec, [][3]int) {
	verts := []r3.Vec{{}, {X: 1}, {X: 1, Y: 1}, {Y: 1}, {Z: gap}, {X: 1, Z: gap}, {X: 1, Y: 1, Z: gap}, {Y: 1, Z: gap}}
	return verts, [][3]int{{0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}}
}

// For random rigid placements of random embedded meshes, whenever the
// certificate accepts a copy, today's scan finds it clean, and a copy the scan
// finds folded is never accepted. Gaps from far above τ to far below one ulp of
// a distant copy cover the certified, thin and folded cases.
func TestCertificateAgreesWithTheScanOnRandomPlacements(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(31, 41))
	accepted, folded, thinRefused := 0, 0, 0
	sv, st := certIcosphere(2, 1, r3.Vec{})
	type source struct {
		verts []r3.Vec
		tris  [][3]int
	}
	var sources []source
	for _, gap := range []float64{0.5, 1e-6, 5e-9, 1e-9, 3e-10, 1e-12, 1e-13} {
		v, tr := certPlates(gap)
		sources = append(sources, source{v, tr})
	}
	for _, gap := range []float64{1e-3, 1e-8, 1e-12} {
		ov, ot := certIcosphere(1, 1, r3.Vec{X: 2 + gap})
		v, tr := certMerge(sv, st, ov, ot)
		sources = append(sources, source{v, tr})
	}
	for si, src := range sources {
		cert, err := BuildPlacementCertificate(t.Context(), src.verts, src.tris)
		require.NoError(t, err)
		require.NotNil(t, cert, "source %d", si)
		if len(cert.thin) > 0 {
			thinRefused++
		}
		for range 40 {
			axis := r3.Vec{X: rng.NormFloat64(), Y: rng.NormFloat64(), Z: rng.NormFloat64()}
			basis := certRotationBasis(t, axis, rng.Float64()*360)
			if rng.IntN(3) == 0 {
				basis.EX = basis.EX.Scale(-1)
			}
			reach := []float64{0, 1, 100, 1e4, 1e5}[rng.IntN(5)]
			tr := r3.Vec{X: reach * rng.NormFloat64(), Y: reach * rng.NormFloat64(), Z: reach * rng.NormFloat64()}
			held, tris, allow := certPlace(basis, tr, src.verts, src.tris)
			ok, err := cert.Check(t.Context(), basis, allow, held, tris)
			require.NoError(t, err)
			clean := certScanClean(t, held, tris)
			if ok {
				accepted++
				require.True(t, clean, "source %d: certificate accepts a copy the scan refuses", si)
			}
			if !clean {
				folded++
				require.False(t, ok, "source %d", si)
			}
		}
	}
	require.Positive(t, accepted)
	require.Positive(t, folded)
	require.Positive(t, thinRefused)
}

// A pair under τ apart is thin, and the copy's exact check of it is what decides
// the copy. With the pair listed neither the thin pair nor the copy is lost.
func TestCertificateThinPairsAreCheckedOnEveryCopy(t *testing.T) {
	t.Parallel()
	verts, tris := certPlates(1e-13)
	cert, err := BuildPlacementCertificate(t.Context(), verts, tris)
	require.NoError(t, err)
	require.NotNil(t, cert)
	require.NotEmpty(t, cert.thin)
	basis := certRotationBasis(t, r3.Vec{X: 1, Y: 2, Z: 5}, 77)
	for _, shift := range []float64{1e4, 1e5} {
		held, copyTris, allow := certPlace(basis, r3.Vec{X: shift, Y: shift / 7}, verts, tris)
		clean := certScanClean(t, held, copyTris)
		ok, err := cert.Check(t.Context(), basis, allow, held, copyTris)
		require.NoError(t, err)
		require.Equal(t, clean, ok, "shift %g", shift)
	}
}

func TestCertificateCheckStopsOnACanceledContext(t *testing.T) {
	t.Parallel()
	verts, tris := certIcosphere(3, 2, r3.Vec{})
	cert, err := BuildPlacementCertificate(t.Context(), verts, tris)
	require.NoError(t, err)
	basis := certRotationBasis(t, r3.Vec{Z: 1}, 20)
	held, copyTris, allow := certPlace(basis, r3.Vec{}, verts, tris)
	canceled, stop := context.WithCancel(t.Context())
	stop()
	_, err = cert.Check(canceled, basis, allow, held, copyTris)
	require.Error(t, err)
	_, err = BuildPlacementCertificate(canceled, verts, tris)
	require.Error(t, err)
}

// A mesh is certified on its second request, whichever slices carry it, and a
// mesh that differs in any vertex or facet is another mesh.
func TestCertificateCacheBuildsOnTheSecondRequestByContent(t *testing.T) {
	t.Parallel()
	verts, tris := certIcosphere(2, 2, r3.Vec{})
	ctx := WithPlacementCertificates(t.Context())
	require.Same(t, PlacementCertificatesFrom(ctx), PlacementCertificatesFrom(WithPlacementCertificates(ctx)))
	require.Nil(t, PlacementCertificatesFrom(t.Context()))
	cache := PlacementCertificatesFrom(ctx)

	cert, err := cache.Certificate(ctx, verts, tris)
	require.ErrorIs(t, err, ErrNoCertificate, "the first request pays no source pass")
	require.Nil(t, cert)
	require.Zero(t, cache.Built())

	cert, err = cache.Certificate(ctx, append([]r3.Vec(nil), verts...), append([][3]int(nil), tris...))
	require.NoError(t, err)
	require.NotNil(t, cert, "the second request, from other slices of equal content, builds")
	require.Equal(t, 1, cache.Built())

	again, err := cache.Certificate(ctx, append([]r3.Vec(nil), verts...), append([][3]int(nil), tris...))
	require.NoError(t, err)
	require.Same(t, cert, again, "equal content, other slices, same certificate")
	require.Equal(t, 1, cache.Built())

	// Another vertex value: a different mesh, so one request is not enough.
	moved := append([]r3.Vec(nil), verts...)
	moved[9].X = math.Nextafter(moved[9].X, 10)
	other, err := cache.Certificate(ctx, moved, tris)
	require.ErrorIs(t, err, ErrNoCertificate)
	require.Nil(t, other)
	require.Equal(t, 1, cache.Built())

	// The same vertices under other facets: also a different mesh.
	swapped := append([][3]int(nil), tris...)
	swapped[2], swapped[3] = swapped[3], swapped[2]
	other, err = cache.Certificate(ctx, verts, swapped)
	require.ErrorIs(t, err, ErrNoCertificate, "the facet list is part of the identity")
	require.Nil(t, other)
	other, err = cache.Certificate(ctx, verts, swapped)
	require.NoError(t, err)
	require.NotNil(t, other)
	require.NotSame(t, cert, other)
	require.Equal(t, 2, cache.Built())

	// A mesh that cannot be certified is built once and then left to the scan.
	badVerts := append([]r3.Vec(nil), verts...)
	badVerts[0].X = math.NaN()
	for range 4 {
		none, err := cache.Certificate(ctx, badVerts, tris)
		require.ErrorIs(t, err, ErrNoCertificate)
		require.Nil(t, none)
	}
	require.Equal(t, 3, cache.Built())
}

// Placements on several goroutines share one cache without racing, and each
// certified copy matches the scan.
func TestCertificateCacheIsSafeForConcurrentUse(t *testing.T) {
	t.Parallel()
	verts, tris := certIcosphere(3, 2, r3.Vec{X: 1})
	ctx := WithPlacementCertificates(t.Context())
	cache := PlacementCertificatesFrom(ctx)
	var group sync.WaitGroup
	checked := make([]int, 12)
	errs := make([]error, 12)
	for w := range checked {
		group.Go(func() {
			basis := certRotationBasis(t, r3.Vec{X: 1, Y: float64(w + 1), Z: 2}, float64(10+w))
			held, copyTris, allow := certPlace(basis, r3.Vec{X: float64(w)}, verts, tris)
			for range 3 {
				cert, err := cache.Certificate(ctx, append([]r3.Vec(nil), verts...), append([][3]int(nil), tris...))
				if errors.Is(err, ErrNoCertificate) {
					continue
				}
				if err != nil {
					errs[w] = err
					return
				}
				ok, err := cert.Check(ctx, basis, allow, held, copyTris)
				if err != nil || !ok {
					errs[w] = errors.Join(err, errors.New("a copy of an embedded source was refused"))
					return
				}
				checked[w]++
			}
		})
	}
	group.Wait()
	for _, err := range errs {
		require.NoError(t, err)
	}
	cert, err := cache.Certificate(ctx, verts, tris)
	require.NoError(t, err)
	require.NotNil(t, cert)
	require.Equal(t, 1, cache.Built())
}
