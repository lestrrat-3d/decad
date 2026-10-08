package decad

import (
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// mitredTreeBranch builds the mtilt-shaped fixture: a 16-gon of radius 1.5
// tapering to 1 along a five-point polyline leaning up to 45°.
func mitredTreeBranch(t *testing.T) (*Body, mitredSweepPayload) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	poly, err := s.CreatePolygon(0, 0, 16, 1.5)
	require.NoError(t, err)
	s.Fix(poly.Center)
	for _, v := range poly.Vertices {
		s.Fix(v)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	path, err := NewPath(r3.NewVec(0, 0, 0),
		LineTo{End: r3.NewVec(0, 0, 4)},
		LineTo{End: r3.NewVec(1, 0, 8)},
		LineTo{End: r3.NewVec(3, 0.5, 11.5)},
		LineTo{End: r3.NewVec(6, 0.5, 14.5)},
	)
	require.NoError(t, err)
	body, err := New().Sweep(t.Context(), s, s.Profiles()[0], path, WithMitredJoins(), WithSectionScale(
		units.Scalar(1.375/1.5), units.Scalar(1.25/1.5), units.Scalar(1.125/1.5), units.Scalar(1/1.5)))
	require.NoError(t, err)
	mp, ok := body.payload.(mitredSweepPayload)
	require.True(t, ok)
	return body, mp
}

func testRatVec(v r3.Vec) [3]*big.Rat {
	return [3]*big.Rat{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
}

func testRatSub(a, b [3]*big.Rat) [3]*big.Rat {
	return [3]*big.Rat{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1]), new(big.Rat).Sub(a[2], b[2])}
}

func testRatDot(a, b [3]*big.Rat) *big.Rat {
	out := new(big.Rat)
	for i := range 3 {
		out.Add(out, new(big.Rat).Mul(a[i], b[i]))
	}
	return out
}

func testRatCross(a, b [3]*big.Rat) [3]*big.Rat {
	c := func(i, j int) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(a[i], b[j]), new(big.Rat).Mul(a[j], b[i]))
	}
	return [3]*big.Rat{c(1, 2), c(2, 0), c(0, 1)}
}

func testRatOf(v sweepRatVec) [3]*big.Rat { return [3]*big.Rat{v[0], v[1], v[2]} }

// testLengthLower is the largest float whose square is at most q, found by
// walking from math.Sqrt rather than through the build's own helper.
func testLengthLower(q *big.Rat) *big.Rat {
	f, _ := q.Float64()
	f = math.Sqrt(f)
	square := func(f float64) *big.Rat { r := new(big.Rat).SetFloat64(f); return r.Mul(r, r) }
	for square(f).Cmp(q) > 0 {
		f = math.Nextafter(f, 0)
	}
	for square(math.Nextafter(f, math.Inf(1))).Cmp(q) <= 0 {
		f = math.Nextafter(f, math.Inf(1))
	}
	return new(big.Rat).SetFloat64(f)
}

// TestMitredSweepJoinPlanesAndWallsAreExact checks §16.3 over the rational
// vertex set: section 0 on the profile plane, every internal section on the
// plane through V_k with normal λ_k·d_{k−1} + λ_{k−1}·d_k (λ recomputed
// here independently), the last on the plane normal to the last span, and
// every wall quad exactly planar.
func TestMitredSweepJoinPlanesAndWallsAreExact(t *testing.T) {
	t.Parallel()
	_, mp := mitredTreeBranch(t)
	points := []r3.Vec{mp.path.Start()}
	for _, r := range mp.path.records {
		points = append(points, r.end)
	}
	n := len(points) - 1
	stride := len(mp.exact) / (n + 1)
	require.Equal(t, 16, stride)
	dirs := make([][3]*big.Rat, n)
	lambdas := make([]*big.Rat, n)
	for k := range n {
		dirs[k] = testRatSub(testRatVec(points[k+1]), testRatVec(points[k]))
		lambdas[k] = testLengthLower(testRatDot(dirs[k], dirs[k]))
	}
	for k := 0; k <= n; k++ {
		var normal [3]*big.Rat
		switch k {
		case 0:
			normal = [3]*big.Rat{big.NewRat(0, 1), big.NewRat(0, 1), big.NewRat(1, 1)}
		case n:
			normal = dirs[n-1]
		default:
			a := dirs[k-1]
			b := dirs[k]
			for i := range 3 {
				normal[i] = new(big.Rat).Add(new(big.Rat).Mul(lambdas[k], a[i]), new(big.Rat).Mul(lambdas[k-1], b[i]))
			}
		}
		for v := range stride {
			p := testRatOf(mp.exact[k*stride+v])
			require.Zero(t, testRatDot(normal, testRatSub(p, testRatVec(points[k]))).Sign(),
				`section %d vertex %d lies exactly on its section plane`, k, v)
		}
	}
	for k := range n {
		for v := range stride {
			next := (v + 1) % stride
			p, q := testRatOf(mp.exact[k*stride+v]), testRatOf(mp.exact[k*stride+next])
			pn, qn := testRatOf(mp.exact[(k+1)*stride+v]), testRatOf(mp.exact[(k+1)*stride+next])
			orient := testRatDot(testRatCross(testRatSub(q, p), testRatSub(qn, p)), testRatSub(pn, p))
			require.Zero(t, orient.Sign(), `wall (%d, 0, %d) is exactly planar`, k, v)
		}
	}
}

// testExactVolumeMoments recomputes six times the volume and the first
// moments about the origin over the rational vertices, splitting every wall
// along its OTHER diagonal: a planar quad's volume does not depend on it.
func testExactVolumeMoments(mp mitredSweepPayload) (*big.Rat, [3]*big.Rat) {
	stride := len(mp.exact) / (len(mp.path.records) + 1)
	var tris [][3]int
	for k := range len(mp.path.records) {
		for v := range stride {
			next := (v + 1) % stride
			b0, b1 := k*stride+v, k*stride+next
			t0, t1 := (k+1)*stride+v, (k+1)*stride+next
			tris = append(tris, [3]int{b0, b1, t0}, [3]int{b1, t1, t0})
		}
	}
	walls := 2 * len(mp.path.records) * stride
	tris = append(tris, mp.tris[walls:]...)
	vol6 := new(big.Rat)
	moments := [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
	for _, t := range tris {
		a, b, c := testRatOf(mp.exact[t[0]]), testRatOf(mp.exact[t[1]]), testRatOf(mp.exact[t[2]])
		term := testRatDot(a, testRatCross(b, c))
		vol6.Add(vol6, term)
		for i := range 3 {
			sum := new(big.Rat).Add(a[i], b[i])
			sum.Add(sum, c[i])
			moments[i].Add(moments[i], sum.Mul(sum, term))
		}
	}
	return vol6, moments
}

func ratDistanceAtMost(r *big.Rat, f float64, bound float64) bool {
	gap := new(big.Rat).Sub(r, new(big.Rat).SetFloat64(f))
	return gap.Abs(gap).Cmp(new(big.Rat).SetFloat64(bound)) <= 0
}

// TestMitredSweepReadingsEncloseExactValues checks §16.6's published bounds
// against independent references: the volume and centroid against an
// origin-anchored tetrahedron sum over the other wall diagonal, the area
// against 256-bit square roots, and the box against every rational vertex.
// Each bound is positive on this fixture, so dropping one fails here.
func TestMitredSweepReadingsEncloseExactValues(t *testing.T) {
	t.Parallel()
	body, mp := mitredTreeBranch(t)
	vol6, moments := testExactVolumeMoments(mp)
	if vol6.Sign() < 0 {
		vol6.Neg(vol6)
		for i := range moments {
			moments[i].Neg(moments[i])
		}
	}
	vol := new(big.Rat).Quo(vol6, big.NewRat(6, 1))
	require.Positive(t, body.volume.Bound.Base())
	require.True(t, ratDistanceAtMost(vol, body.volume.Value.Base(), body.volume.Bound.Base()))

	centroid := []float64{body.centroid.Value.X, body.centroid.Value.Y, body.centroid.Value.Z}
	require.Positive(t, body.centroid.Bound.Base())
	for i := range 3 {
		c := new(big.Rat).Quo(moments[i], new(big.Rat).Mul(big.NewRat(4, 1), vol6))
		require.True(t, ratDistanceAtMost(c, centroid[i], body.centroid.Bound.Base()), `centroid axis %d`, i)
	}

	const prec = 256
	total := new(big.Float).SetPrec(prec)
	for _, tri := range mp.tris {
		a, b, c := testRatOf(mp.exact[tri[0]]), testRatOf(mp.exact[tri[1]]), testRatOf(mp.exact[tri[2]])
		n := testRatCross(testRatSub(b, a), testRatSub(c, a))
		q := new(big.Float).SetPrec(prec).SetRat(testRatDot(n, n))
		total.Add(total, new(big.Float).SetPrec(prec).Quo(new(big.Float).SetPrec(prec).Sqrt(q), big.NewFloat(2)))
	}
	ref, _ := total.Float64()
	gap := math.Abs(ref - body.area.Value.Base())
	require.Positive(t, body.area.Bound.Base())
	require.LessOrEqual(t, gap, body.area.Bound.Base()+ref*1e-30)

	lo := []float64{body.bounds.Min.X, body.bounds.Min.Y, body.bounds.Min.Z}
	hi := []float64{body.bounds.Max.X, body.bounds.Max.Y, body.bounds.Max.Z}
	var exactLo, exactHi [3]*big.Rat
	for _, p := range mp.exact {
		for i := range 3 {
			require.LessOrEqual(t, new(big.Rat).SetFloat64(lo[i]).Cmp(p[i]), 0, `the box holds every exact vertex`)
			require.GreaterOrEqual(t, new(big.Rat).SetFloat64(hi[i]).Cmp(p[i]), 0, `the box holds every exact vertex`)
			if exactLo[i] == nil || p[i].Cmp(exactLo[i]) < 0 {
				exactLo[i] = p[i]
			}
			if exactHi[i] == nil || p[i].Cmp(exactHi[i]) > 0 {
				exactHi[i] = p[i]
			}
		}
	}
	rounded := false
	for i := range 3 {
		require.True(t, ratDistanceAtMost(exactLo[i], lo[i], body.bounds.Bound.Base()), `min axis %d`, i)
		require.True(t, ratDistanceAtMost(exactHi[i], hi[i], body.bounds.Bound.Base()), `max axis %d`, i)
		rounded = rounded || !ratDistanceAtMost(exactLo[i], lo[i], 0) || !ratDistanceAtMost(exactHi[i], hi[i], 0)
	}
	require.True(t, rounded, `this fixture has an extreme that is not a float`)
}

// TestMitredSweepGateDiameterShrinks checks the Verify gate reads the held
// vertex set's diameter shrunk by twice delta, never the held one itself.
func TestMitredSweepGateDiameterShrinks(t *testing.T) {
	t.Parallel()
	body, mp := mitredTreeBranch(t)
	held, ok := pointSetDiameter(mp.verts)
	require.True(t, ok)
	got, ok, err := bodyGateDiameter(t.Context(), body)
	require.NoError(t, err)
	require.True(t, ok)
	want, ok := lowerDiameterForDisplacement(held, mp.delta)
	require.True(t, ok)
	require.Equal(t, want, got)
	require.Less(t, got, held)
}

// TestMitredSweepDeltaAndMeshProofs checks the single rounding: delta covers
// every vertex's exact 3D gap and is within a factor two of the largest, the
// mesh publishes it as its Bound, and the mesh's held volume and area sit
// within volSymDiff and areaSlack of the exact ones.
func TestMitredSweepDeltaAndMeshProofs(t *testing.T) {
	t.Parallel()
	body, mp := mitredTreeBranch(t)
	worst := new(big.Rat)
	for v, p := range mp.exact {
		gap := testRatSub(testRatOf(p), testRatVec(mp.verts[v]))
		d2 := testRatDot(gap, gap)
		require.LessOrEqual(t, d2.Cmp(new(big.Rat).Mul(new(big.Rat).SetFloat64(mp.delta), new(big.Rat).SetFloat64(mp.delta))), 0)
		if d2.Cmp(worst) > 0 {
			worst = d2
		}
	}
	require.Positive(t, worst.Sign())
	worstF, _ := worst.Float64()
	require.LessOrEqual(t, mp.delta, 2*math.Sqrt(worstF))

	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, mp.delta, mesh.Bound().Base())
	require.True(t, mesh.symDiffOK)

	vol6, _ := testExactVolumeMoments(mp)
	vol6.Abs(vol6)
	exactVol := new(big.Rat).Quo(vol6, big.NewRat(6, 1))
	held := new(big.Rat)
	for _, tri := range mesh.triangles {
		a, b, c := testRatVec(mesh.vertices[tri[0]]), testRatVec(mesh.vertices[tri[1]]), testRatVec(mesh.vertices[tri[2]])
		held.Add(held, testRatDot(a, testRatCross(b, c)))
	}
	held.Quo(held, big.NewRat(6, 1))
	diff := new(big.Rat).Sub(held, exactVol)
	require.Positive(t, diff.Abs(diff).Sign(), `the rounded mesh's volume differs from the exact one`)
	require.LessOrEqual(t, diff.Cmp(new(big.Rat).SetFloat64(mesh.volSymDiff)), 0)

	heldArea := 0.0
	for _, tri := range mesh.triangles {
		a, b, c := mesh.vertices[tri[0]], mesh.vertices[tri[1]], mesh.vertices[tri[2]]
		heldArea += b.Sub(a).Cross(c.Sub(a)).Len() / 2
	}
	require.Positive(t, mesh.areaSlack)
	require.LessOrEqual(t, math.Abs(heldArea-body.area.Value.Base()), mesh.areaSlack+body.area.Bound.Base()+1e-12)
}

// TestMitredSweepPublishesPerVertexBounds is docs/faceted-vertex-bounds-
// design.md §2.1's mitred row. The fixture's path starts at the origin with an
// axis-aligned first span, so every start-cap vertex is the profile's own
// float vertex and rounds with no gap: each publishes β = 0, and so does the
// start cap's face. Every other vertex publishes its own gap, which covers
// the exact distance to its rational, is at most delta, and reaches delta at
// the vertex delta was read from. Every face bound is the largest corner β
// over that face's own triangles.
func TestMitredSweepPublishesPerVertexBounds(t *testing.T) {
	t.Parallel()
	body, mp := mitredTreeBranch(t)
	mesh, err := tessellateContext(t.Context(), body, units.Millimeters(0.01), VerifyAll)
	require.NoError(t, err)
	beta, err := mesh.vertexBounds()
	require.NoError(t, err)
	require.Len(t, beta, len(mp.exact))
	require.Positive(t, mp.delta)

	faceOfRole := map[string]*Face{}
	for _, f := range body.Faces() {
		for _, o := range f.Origins() {
			faceOfRole[o.Role] = f
		}
	}
	capCorners := 0
	for k, tri := range mp.tris {
		if mp.faceRoles[mp.triFace[k]] != roleCapStart {
			continue
		}
		for _, v := range tri {
			require.Zero(t, beta[v], `start-cap vertex %d rounds with no gap`, v)
			capCorners++
		}
	}
	require.Positive(t, capCorners)
	capBound, ok := mesh.sourceBound(faceOfRole[roleCapStart])
	require.True(t, ok)
	require.Zero(t, capBound)

	atDelta := 0
	for v, p := range mp.exact {
		gap := testRatSub(testRatOf(p), testRatVec(mp.verts[v]))
		b := new(big.Rat).SetFloat64(beta[v])
		require.LessOrEqual(t, testRatDot(gap, gap).Cmp(new(big.Rat).Mul(b, b)), 0, `vertex %d's β covers its exact gap`, v)
		require.LessOrEqual(t, beta[v], mp.delta)
		if beta[v] == mp.delta {
			atDelta++
		}
	}
	require.Positive(t, atDelta, `delta is some vertex's own β`)

	want := map[*Face]float64{}
	for k, tri := range mp.tris {
		f := faceOfRole[mp.faceRoles[mp.triFace[k]]]
		want[f] = max(want[f], beta[tri[0]], beta[tri[1]], beta[tri[2]])
	}
	for f, w := range want {
		got, ok := mesh.sourceBound(f)
		require.True(t, ok)
		require.Equal(t, w, got)
	}
	require.Equal(t, mp.delta, mesh.Bound().Base())
}

// TestMitredSweepRequireWallsRefuses is Table SM row SM7 over hand-built
// sections: §16.3's gates make both shapes unreachable through Sweep, so the
// row's own check is exercised directly.
func TestMitredSweepRequireWallsRefuses(t *testing.T) {
	t.Parallel()
	pt := func(x, y, z int64) sweepRatVec {
		return sweepRatVec{big.NewRat(x, 1), big.NewRat(y, 1), big.NewRat(z, 1)}
	}
	from := mitredSection{pt(0, 0, 0), pt(1, 0, 0), pt(1, 1, 0), pt(0, 1, 0)}
	loops := [][]int{{0, 1, 2, 3}}

	err := mitredRequireWalls(2, loops, from, from)
	require.ErrorIs(t, err, ErrDegenerate)
	require.ErrorContains(t, err, "span 2, loop 0, segment 0: its wall quad has zero area")

	to := mitredSection{pt(0, 0, 1), pt(1, 0, 1), pt(1, 0, 1), pt(0, 1, 1)}
	err = mitredRequireWalls(0, loops, from, to)
	require.ErrorIs(t, err, ErrDegenerate)
	require.ErrorContains(t, err, "two coincident vertices")

	require.NoError(t, mitredRequireWalls(0, loops, from, mitredSection{pt(0, 0, 1), pt(1, 0, 1), pt(1, 1, 1), pt(0, 1, 1)}))
}

// TestMitredSweepSpanLengthLower pins λ to the largest float whose square
// does not exceed the exact squared length.
func TestMitredSweepSpanLengthLower(t *testing.T) {
	t.Parallel()
	lambda, err := mitredSpanLengthLower(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 8))
	require.NoError(t, err)
	require.Equal(t, testLengthLower(big.NewRat(65, 1)), lambda)
	require.Equal(t, -1, new(big.Rat).Mul(lambda, lambda).Cmp(big.NewRat(65, 1)))

	lambda, err = mitredSpanLengthLower(r3.NewVec(0, 0, 0), r3.NewVec(3, 4, 0))
	require.NoError(t, err)
	require.Equal(t, big.NewRat(5, 1), lambda)
}

// mitredVolumeMomentsRat is mitredVolumeMoments as a term-by-term rational
// sum, every partial sum reduced: the reference the common-denominator sum
// must reproduce exactly.
func mitredVolumeMomentsRat(exact []sweepRatVec, tris [][3]int, anchor sweepRatVec) (*big.Rat, [3]*big.Rat) {
	vol6 := new(big.Rat)
	moments := [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
	for _, t := range tris {
		a := sweepRatSub(exact[t[0]], anchor)
		b := sweepRatSub(exact[t[1]], anchor)
		c := sweepRatSub(exact[t[2]], anchor)
		term := sweepRatDot(a, sweepRatCross(b, c))
		vol6.Add(vol6, term)
		for axis := range 3 {
			sum := new(big.Rat).Add(a[axis], b[axis])
			sum.Add(sum, c[axis])
			moments[axis].Add(moments[axis], sum.Mul(sum, term))
		}
	}
	return vol6, moments
}

// requireMitredSumsMatch asserts that the common-denominator sum and the
// rational reference agree on six times the volume and every moment, both
// as values and as canonical strings.
func requireMitredSumsMatch(t *testing.T, exact []sweepRatVec, tris [][3]int, anchor sweepRatVec, label string) {
	t.Helper()
	wantVol, wantMom := mitredVolumeMomentsRat(exact, tris, anchor)
	gotVol, gotMom := mitredVolumeMoments(exact, tris, anchor)
	require.Zero(t, wantVol.Cmp(gotVol), `%s: vol6`, label)
	require.Equal(t, wantVol.RatString(), gotVol.RatString(), `%s: vol6`, label)
	for axis := range 3 {
		require.Zero(t, wantMom[axis].Cmp(gotMom[axis]), `%s: moment %d`, label, axis)
		require.Equal(t, wantMom[axis].RatString(), gotMom[axis].RatString(), `%s: moment %d`, label, axis)
	}
}

// TestMitredSweepMomentsMatchRationalSum pins mitredVolumeMoments to the
// term-by-term rational sum over the built mitred fixtures (the tree branch,
// the clustered trunk and three placed branches, each with the anchor its
// own build uses) and over random triangle sets whose coordinates carry
// random int64 numerators and denominators, some shared and most not.
func TestMitredSweepMomentsMatchRationalSum(t *testing.T) {
	t.Parallel()
	t.Run("fixtures", func(t *testing.T) {
		t.Parallel()
		_, branch := mitredTreeBranch(t)
		cluster := newClusteredTree(t, 25, 1.1)
		bodies := map[string]*Body{"trunk": cluster.trunk}
		for k := range 3 {
			bodies[fmt.Sprintf("placed branch %d", k)] = cluster.branch(t, k)
		}
		payloads := map[string]mitredSweepPayload{"tree branch": branch}
		for name, body := range bodies {
			mp, ok := body.payload.(mitredSweepPayload)
			require.True(t, ok, name)
			payloads[name] = mp
		}
		for name, mp := range payloads {
			c, err := constructMitredSweep(t.Context(), mp)
			require.NoError(t, err, name)
			anchor := mitredPlace(mp.xform, c.anchor)
			requireMitredSumsMatch(t, mp.exact, mp.tris, anchor, name)
		}
	})
	t.Run("random", func(t *testing.T) {
		t.Parallel()
		rng := rand.New(rand.NewPCG(0x6d697472, 0x65640a))
		denominator := func() int64 {
			d := rng.Int64N(math.MaxInt64-2) + 3
			if d&(d-1) == 0 {
				d++
			}
			return d
		}
		for trial := range 200 {
			pool := []int64{denominator(), denominator(), denominator()}
			coord := func() *big.Rat {
				d := denominator()
				if rng.IntN(3) == 0 {
					d = pool[rng.IntN(len(pool))]
				}
				return big.NewRat(int64(rng.Uint64()), d)
			}
			vertex := func() sweepRatVec { return sweepRatVec{coord(), coord(), coord()} }
			exact := make([]sweepRatVec, 4+rng.IntN(8))
			for v := range exact {
				exact[v] = vertex()
			}
			tris := make([][3]int, 1+rng.IntN(20))
			for k := range tris {
				tris[k] = [3]int{rng.IntN(len(exact)), rng.IntN(len(exact)), rng.IntN(len(exact))}
			}
			requireMitredSumsMatch(t, exact, tris, vertex(), fmt.Sprintf("trial %d", trial))
		}
	})
	point := func(x, y, z int64) sweepRatVec {
		return sweepRatVec{big.NewRat(x, 2), big.NewRat(y, 3), big.NewRat(z, 5)}
	}
	local := []sweepRatVec{point(0, 0, 0), point(2, 0, 0), point(0, 3, 0), point(0, 0, 5)}
	anchor := sweepRatVec{big.NewRat(1, 3), big.NewRat(2, 7), big.NewRat(3, 11)}
	tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
	vol6, moments := mitredVolumeMoments(local, tris, anchor)
	frame, err := r3.NewFrame(r3.NewVec(7, 11, 13), r3.NewVec(0, 1, 0), r3.NewVec(-1, 0, 0))
	require.NoError(t, err)
	turn, err := r3.FromFrame(frame)
	require.NoError(t, err)
	reflection, err := r3.Reflection(identityFrame(t))
	require.NoError(t, err)
	for _, xform := range []r3.Transform{r3.Identity(), turn, reflection} {
		placed := make([]sweepRatVec, len(local))
		for i, p := range local {
			placed[i] = mitredPlace(xform, p)
		}
		wantVol, wantMoments := mitredVolumeMoments(placed, tris, mitredPlace(xform, anchor))
		gotVol, gotMoments := sweepmitre.PlacedVolumeMoments(vol6, moments, xform)
		require.Zero(t, wantVol.Cmp(gotVol))
		for axis := range 3 {
			require.Zero(t, wantMoments[axis].Cmp(gotMoments[axis]), "moment axis %d", axis)
		}
	}
}
