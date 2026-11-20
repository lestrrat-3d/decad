package meshbool

import (
	"math"
	"math/big"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// unprunedDepthBeyond is depthBeyond with the depth threshold prune turned
// off: the hint reading, then the block scan, then the certificate.
func unprunedDepthBeyond(p proof.Xpt, other *BoolMesh, b float64, hint *int) bool {
	pf := p.Vec()
	if hint != nil && *hint >= 0 && *hint < len(other.Tris) {
		h := *hint
		up := math.Max(PointBoxDist(pf, other.Boxes[h]), depthFacetDistance(pf, TriCorners(other, h), other.depthNormal(h)))
		if up*(1-1e-12) <= b {
			return false
		}
	}
	s := depthScan{pf: pf, other: other, b: b, best: math.Inf(1)}
	if settled := s.scan(); settled >= 0 {
		if hint != nil {
			*hint = settled
		}
		return false
	}
	return !(depthCertificate(s.best, p, pf, other) <= b)
}

// depthThresholdAt is the prune threshold depthBeyond computes for p and b,
// +Inf when the prune is off.
func depthThresholdAt(p proof.Xpt, m *BoolMesh, b float64) float64 {
	s := depthScan{pf: p.Vec(), other: m, b: b}
	s.setThreshold()
	return s.td
}

// depthBoundsAcross returns the bounds b on either side of the point where
// the prune threshold for p first exceeds x: the least non-negative b whose
// threshold is above x, and the float below it.
func depthBoundsAcross(p proof.Xpt, m *BoolMesh, x float64) []float64 {
	if !(x > 0) || math.IsInf(x, 1) {
		return nil
	}
	lo, hi := uint64(0), math.Float64bits(x)
	if !(depthThresholdAt(p, m, x) > x) {
		return nil
	}
	for lo < hi {
		mid := lo + (hi-lo)/2
		if depthThresholdAt(p, m, math.Float64frombits(mid)) > x {
			hi = mid
		} else {
			lo = mid + 1
		}
	}
	out := []float64{math.Float64frombits(lo)}
	if lo > 0 {
		out = append(out, math.Float64frombits(lo-1))
	}
	return out
}

// depthCriticalReadings are the values the prune compares its threshold with
// for the point pf: every box reading, every facet distance and every box's
// prune value fl(fl(P·(1 − 2^-46)) − s).
func depthCriticalReadings(pf r3.Vec, m *BoolMesh) []float64 {
	reach := max(math.Abs(pf.X), math.Abs(pf.Y), math.Abs(pf.Z), m.depthCoordReach())
	a1, a2 := depthSlackFactors(reach)
	value := func(d, kappa float64) float64 {
		return float64(float64(d*depthPruneShrink) - float64(float64(kappa*a1)+a2))
	}
	var out []float64
	for i := range m.Tris {
		d := PointBoxDist(pf, m.Boxes[i])
		out = append(out, d, depthFacetDistance(pf, TriCorners(m, i), m.depthNormal(i)))
		if len(m.depthKappa) > 0 {
			out = append(out, value(d, m.depthKappa[0][i]))
		}
	}
	for k, level := range m.depthLevels {
		for j, box := range level {
			d := PointBoxDist(pf, box)
			out = append(out, d)
			if len(m.depthKappa) > k+1 {
				out = append(out, value(d, m.depthKappa[k+1][j]))
			}
		}
	}
	slices.Sort(out)
	return slices.Compact(out)
}

// depthZeroNormalMesh is m with one more facet whose corners are collinear
// and whose exact normal is zero, prepared by hand because PrepContext
// refuses such a facet.
func depthZeroNormalMesh(t testing.TB, m *BoolMesh) *BoolMesh {
	t.Helper()
	bm := depthMesh(t, append([]r3.Vec(nil), m.Verts...), append([][3]int(nil), m.Tris...))
	a, d := r3.NewVec(0.25, 0.5, 0.75), r3.NewVec(0.5, -0.25, 1)
	base := len(bm.Verts)
	bm.Verts = append(bm.Verts, a, a.Add(d), a.Add(d.Scale(2)))
	for _, v := range bm.Verts[base:] {
		bm.Xverts = append(bm.Xverts, proof.XptOf(v))
	}
	tri := [3]int{base, base + 1, base + 2}
	bm.Tris = append(bm.Tris, tri)
	bm.Norms = append(bm.Norms, proof.Xpt{X: big.NewInt(0), Y: big.NewInt(0), Z: big.NewInt(0), W: big.NewInt(1)})
	bm.Boxes = append(bm.Boxes, TriBox(bm.Verts, tri))
	bm.Fnorms = append(bm.Fnorms, r3.Vec{})
	bm.FnormsReady = append(bm.FnormsReady, false)
	bm.setDepthNormals()
	bm.setDepthBlocks()
	return bm
}

// TestDepthPruneMatchesUnprunedScan compares depthBeyond against the block
// scan without the threshold prune (unprunedDepthBeyond) and against the
// facet-order definition (refDepthBeyond). For each mesh and query it places
// the bound so that the prune threshold falls just below and just above every
// box reading, facet distance and prune value of the query, and around the
// query's certified depth by a few ulps and a few u·M. Queries include exact
// non-float points just inside a face, whose PointRoundBound the threshold
// must cover. Meshes: cylinders near the origin and far from it, cube grids,
// sliver soups, sliver tetrahedra, a mesh with a zero-normal facet and one
// with odd boxes. Every comparison
// runs with no hint, a sampled hint and a hint carried from bound to bound, and
// must give the same answer and leave the same hint. The sweep must pass over
// boxes on deep answers. TestDepthPruneRestartsOnMisreadSliver covers the
// start-over path.
func TestDepthPruneMatchesUnprunedScan(t *testing.T) {
	t.Parallel()
	const u = 0x1p-53
	rng := rand.New(rand.NewPCG(41, 7))
	frame, err := r3.NewFrame(r3.NewVec(0.3, -0.2, 0.1), r3.NewVec(0.6, 0.64, 0.48), r3.NewVec(-0.8, 0.48, 0.36))
	require.NoError(t, err)
	tet7, _, _ := sliverTetrahedron(t, 1e-7, r3.Vec{})
	tet4, face4, apex4 := sliverTetrahedron(t, 1e-4, r3.Vec{})
	meshes := []struct {
		name string
		m    *BoolMesh
	}{
		{"cylinder24", depthCylinder(t, 24, r3.NewVec(0.375, -1.25, 0.5))},
		{"farCylinder", depthCylinder(t, 24, r3.NewVec(1e6, -3e5, 2.5e5))},
		{"turnedCubes", depthCubeGrid(t, 2, &frame)},
		{"slivers", depthSlivers(t, rng, 40)},
		{"tet7", tet7},
		{"tet4", tet4},
		{"zeroNormal", depthZeroNormalMesh(t, depthCylinder(t, 12, r3.Vec{}))},
		{"oddBoxes", depthOddBoxes(t, depthCylinder(t, 24, r3.Vec{}))},
	}
	queries, readingStride := 6, 1
	if raceDetector {
		queries, readingStride = 2, 7
	}
	compared, passes, restarts := 0, 0, 0
	for _, mc := range meshes {
		name, m := mc.name, mc.m
		qs := depthQueries(rng, m, queries)
		// Points inside the solid near a face, as the witness search asks.
		for _, p := range depthSamplePoints(m, max(1, len(m.Tris)/queries)) {
			off := r3.NewVec(rng.Float64()-0.5, rng.Float64()-0.5, rng.Float64()-0.5).Scale(1e-3)
			qs = append(qs, proof.XptOf(p.Vec().Add(off)))
		}
		if name == "tet4" {
			centroid := face4[0].Add(face4[1]).Add(face4[2]).Add(apex4).Scale(0.25)
			qs = append(qs, proof.XptOf(centroid))
		}
		if len(qs) > 4*queries {
			rng.Shuffle(len(qs), func(i, j int) { qs[i], qs[j] = qs[j], qs[i] })
			qs = qs[:4*queries]
		}
		// Exact points about 1e-7 of the way from a face point towards the
		// vertex centroid. PointRoundBound is positive for them and the
		// nearest reading is small, so the threshold's pad alone does not
		// cover R and the threshold must carry R̄.
		var centroid r3.Vec
		for _, v := range m.Verts {
			centroid = centroid.Add(v)
		}
		centroid = centroid.Scale(1 / float64(len(m.Verts)))
		near := depthSamplePoints(m, max(1, len(m.Tris)/queries))
		for _, sp := range near[:min(queries, len(near))] {
			qs = append(qs, proof.Xlerp(sp, proof.XptOf(centroid), big.NewInt(int64(1+rng.IntN(6))), big.NewInt(7_000_001)))
		}
		for qi, p := range qs {
			pf := p.Vec()
			reach := max(math.Abs(pf.X), math.Abs(pf.Y), math.Abs(pf.Z), m.depthCoordReach())
			var bounds []float64
			readings := depthCriticalReadings(pf, m)
			for k := rng.IntN(readingStride); k < len(readings); k += readingStride {
				bounds = append(bounds, depthBoundsAcross(p, m, readings[k])...)
			}
			if c := refCertifiedInteriorDepth(p, m); c > 0 {
				bounds = append(bounds, c, math.Nextafter(c, 0), math.Nextafter(c, math.Inf(1)))
				for _, k := range []float64{-300, -30, -3, 3, 30, 300} {
					bounds = append(bounds, c+k*u*reach)
				}
			}
			carriedRef, carriedGot := -1, -1
			for _, b := range bounds {
				want := unprunedDepthBeyond(p, m, b, nil)
				require.Equal(t, want, depthBeyond(p, m, b, nil), "%s query %d bound %g, no hint", name, qi, b)
				h := rng.IntN(len(m.Tris))
				hRef, hGot := h, h
				require.Equal(t, want, unprunedDepthBeyond(p, m, b, &hRef))
				require.Equal(t, want, depthBeyond(p, m, b, &hGot), "%s query %d bound %g hint %d", name, qi, b, h)
				require.Equal(t, hRef, hGot, "%s query %d bound %g hint %d", name, qi, b, h)
				require.Equal(t, unprunedDepthBeyond(p, m, b, &carriedRef), depthBeyond(p, m, b, &carriedGot),
					"%s query %d bound %g carried", name, qi, b)
				require.Equal(t, carriedRef, carriedGot, "%s query %d bound %g carried hint", name, qi, b)
				if compared%16 == 0 {
					require.Equal(t, refDepthBeyond(p, m, b, nil), want,
						"%s query %d bound %g: block scan against definition", name, qi, b)
				}
				compared++
				if len(m.depthLevels) == 0 || !m.depthKappaFit() {
					continue
				}
				s := depthScan{pf: pf, other: m, b: b, best: math.Inf(1), prune: true}
				switch s.prunedBlock(len(m.depthLevels)-1, 0) {
				case depthScanRestart:
					restarts++
				case -1:
					if s.passed {
						require.True(t, want, "%s query %d bound %g: a scan that passed over a box answers deep", name, qi, b)
						passes++
					}
				}
			}
		}
	}
	t.Logf("compared %d bounds: %d scans passed over boxes, %d started over", compared, passes, restarts)
	require.Positive(t, passes)
}

// TestDepthPruneRestartsOnMisreadSliver covers a facet whose reading falls
// far below its box reading. The sliver's perpendicular foot from a point
// just beyond its acute corner reads inside, so depthFacetDistance returns
// the plane distance, about 6e-4 below the box distance. The mesh lists four
// small facets far away, three small facets inside the sliver's box near its
// far corner, and the sliver last. With b at the sliver's reading the
// unpruned scan settles on the sliver. The pruned scan passes over the far
// block, then finds the sliver's block in the band, which depthPrunable does
// not prove, and must start over rather than pass over it, so it settles on
// the same facet. A second mesh puts three far facets and the sliver in one
// block, so the scan starts over on reading the sliver itself.
func TestDepthPruneRestartsOnMisreadSliver(t *testing.T) {
	t.Parallel()
	p := r3.Vec{X: -0.3809082442329791, Y: -0.3389216007473093, Z: -0.1640515350442391}
	sliver := [3]r3.Vec{
		{X: 0.3046059597787405, Y: 0.27964357325307587, Z: 0.9030040584780721},
		{X: -0.03800906965838711, Y: -0.029510816340430773, Z: 0.3696974085994357},
		{X: -0.38062409909553746, Y: -0.33866520593396426, Z: -0.16360924127920123},
	}
	small := func(c r3.Vec) []r3.Vec {
		return []r3.Vec{c, c.Add(r3.NewVec(1e-3, 0, 0)), c.Add(r3.NewVec(0, 1e-3, 1e-4))}
	}
	var verts []r3.Vec
	for k := range 4 {
		verts = append(verts, small(r3.NewVec(10+float64(k), 10, 10))...)
	}
	for k := range 3 {
		verts = append(verts, small(sliver[0].Add(r3.NewVec(-0.01*float64(k+1), -0.01, -0.02)))...)
	}
	verts = append(verts, sliver[:]...)
	tris := make([][3]int, 8)
	for i := range tris {
		tris[i] = [3]int{3 * i, 3*i + 1, 3*i + 2}
	}
	m := depthMesh(t, verts, tris)
	require.True(t, m.depthKappaFit())
	pf := p
	d := depthFacetDistance(pf, TriCorners(m, 7), m.depthNormal(7))
	require.Greater(t, PointBoxDist(pf, m.depthLevels[0][1]), d+1e-4, "the sliver's block reads well above the sliver")
	xp := proof.XptOf(p)
	for _, b := range []float64{d, math.Nextafter(d, math.Inf(1)), d + 1e-9, d + 1e-6} {
		hRef, hGot := -1, -1
		require.False(t, unprunedDepthBeyond(xp, m, b, &hRef))
		require.Equal(t, 7, hRef)
		require.False(t, depthBeyond(xp, m, b, &hGot), "bound %g", b)
		require.Equal(t, 7, hGot, "bound %g", b)
		s := depthScan{pf: pf, other: m, b: b, best: math.Inf(1), prune: true}
		require.Equal(t, depthScanRestart, s.prunedBlock(len(m.depthLevels)-1, 0), "bound %g", b)
	}

	// One block holding three far facets and the sliver. The block reads
	// the sliver's box distance, so the scan enters it, passes over the far
	// facets one by one, and then reads the sliver itself in the band: the
	// reading is at or below the threshold, so it must start over there.
	one := depthMesh(t, append(append([]r3.Vec(nil), verts[:9]...), sliver[:]...), tris[:4])
	require.Len(t, one.depthLevels, 1)
	for _, b := range []float64{d, math.Nextafter(d, math.Inf(1)), d + 1e-9, d + 1e-6} {
		hRef, hGot := -1, -1
		require.False(t, unprunedDepthBeyond(xp, one, b, &hRef))
		require.Equal(t, 3, hRef)
		require.False(t, depthBeyond(xp, one, b, &hGot), "one block, bound %g", b)
		require.Equal(t, 3, hGot, "one block, bound %g", b)
		s := depthScan{pf: pf, other: one, b: b, best: math.Inf(1), prune: true}
		require.Equal(t, depthScanRestart, s.prunedBlock(0, 0), "one block, bound %g", b)
	}
}

// depthVertexCase builds a sliver facet at scale s and a float point just
// beyond one of its acute corners, along the extension of an edge, where a
// perpendicular foot that PointInTriangle misreads as inside reads furthest
// below the exact distance. The point sits up to s·2^-53·10^14 beyond the
// corner and off the plane by up to about s·2^-51 or, one time in four, by up
// to s.
func depthVertexCase(rng *rand.Rand, s float64) (r3.Vec, [3]r3.Vec) {
	const u = 0x1p-53
	full := func() r3.Vec {
		return r3.Vec{X: s * (2*rng.Float64() - 1), Y: s * (2*rng.Float64() - 1), Z: s * (2*rng.Float64() - 1)}
	}
	a, b := full(), full()
	mid := a.Add(b).Scale(0.5)
	c := mid.Add(full().Sub(mid).Scale(math.Pow(10, -2-12*rng.Float64())))
	tri := [3]r3.Vec{a, b, c}
	nf := b.Sub(a).Cross(c.Sub(a))
	if l := nf.Len(); l > 0 {
		nf = nf.Scale(1 / l)
	}
	out := a.Sub(b)
	out = out.Scale(1 / out.Len())
	side := nf.Cross(out)
	rho := s * u * math.Pow(10, 14*rng.Float64())
	p := a.Add(out.Scale(rho)).Add(side.Scale(rho * math.Pow(10, -1-15*rng.Float64()) * (2*rng.Float64() - 1)))
	delta := s * u * math.Pow(10, -3+4*rng.Float64())
	if rng.IntN(4) == 0 {
		delta = s * math.Pow(10, -12+12*rng.Float64())
	}
	if rng.IntN(2) == 0 {
		delta = -delta
	}
	rot := rng.IntN(3)
	return p.Add(nf.Scale(delta)), [3]r3.Vec{tri[rot], tri[(rot+1)%3], tri[(rot+2)%3]}
}

// TestDepthPruneValueBelowFacetDistance searches for float inputs on which a
// facet box's prune value fl(fl(P·(1 − 2^-46)) − s) exceeds the facet's
// depthFacetDistance D, which would let the prune pass over a facet reading
// at or below the threshold. It draws the shapes of
// TestDepthFacetDistanceWithinBound and points just beyond an acute corner of
// a sliver, at scales from 2^-300 to 2^200, then climbs from the worst case
// of each shape and scale by moving the point and a corner by a few ulps. It
// checks two things against the exact distance d: the lower bound D >= (1 −
// 20u)·d − S of depth_prune.go, with S = 2σκ + 20u·M and κ the facet's
// depthKappaOf, and the prune value staying at or below D. Under the race
// detector it searches the 100 mm scale only.
func TestDepthPruneValueBelowFacetDistance(t *testing.T) {
	t.Parallel()
	const u = 0x1p-53
	const samples, climbs = 300, 150
	scales := []float64{1, 100, 1e6, 0x1p200, 0x1p-300}
	if raceDetector {
		scales = []float64{100}
	}
	finite := func(v r3.Vec) bool {
		return !proofbound.IsNonFinite(v.X) && !proofbound.IsNonFinite(v.Y) && !proofbound.IsNonFinite(v.Z)
	}
	prec := uint(400)
	bf := func(x float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(x) }
	worstLemma, worstValue := math.Inf(-1), math.Inf(-1)
	for _, s := range scales {
		for shape := range 6 {
			rng := rand.New(rand.NewPCG(uint64(shape)+101, math.Float64bits(s)))
			// score checks one case and returns how much of the slack S the
			// reading's shortfall uses, ((1 − 20u)·d − D)/S, and the prune
			// value's excess over D in units of u·M.
			score := func(p r3.Vec, tri [3]r3.Vec) (float64, float64) {
				xt := [3]proof.Xpt{proof.XptOf(tri[0]), proof.XptOf(tri[1]), proof.XptOf(tri[2])}
				xn := proof.XtriangleNormal(xt[0], xt[1], xt[2])
				n := depthNormalOf(xn)
				shift := 0
				if n != (r3.Vec{}) {
					shift = depthNormalShift(xn)
				}
				box := TriBox(tri[:], [3]int{0, 1, 2})
				kappa := depthKappaOf(tri, box, n, shift)
				d := depthFacetDistance(p, tri, n)
				m := vertexReach([]r3.Vec{p, tri[0], tri[1], tri[2]})
				a1, a2 := depthSlackFactors(m)
				// With the threshold at the reading itself, the prune must not
				// pass over the box; its test is monotone in the threshold, so
				// no threshold at or above D passes over it either.
				box1 := depthScan{td: d, a1: a1, a2: a2}
				require.False(t, box1.depthPrunable(PointBoxDist(p, box), kappa),
					"scale %g shape %d: the prune passes over a reading %.17g at the threshold", s, shape, d)
				value := float64(float64(PointBoxDist(p, box)*depthPruneShrink) - float64(float64(kappa*a1)+a2))
				if math.IsInf(kappa, 1) {
					return math.Inf(-1), (value - d) / (u * m)
				}
				d2 := exactTriDist2(exactVecOf(p), [3]exactVec{exactVecOf(tri[0]), exactVecOf(tri[1]), exactVecOf(tri[2])})
				exact := new(big.Float).SetPrec(prec).SetRat(d2)
				exact.Sqrt(exact)
				short := new(big.Float).SetPrec(prec).Mul(exact, bf(1-20*u))
				short.Sub(short, bf(d))
				slack := new(big.Float).SetPrec(prec).Mul(bf(2*(83*u*m+0x1p-568)), bf(kappa))
				slack.Add(slack, bf(20*u*m+0x1p-1000))
				ratio, _ := short.Quo(short, slack).Float64()
				require.LessOrEqual(t, ratio, 1.0,
					"scale %g shape %d: reading %.17g falls below the exact distance %.17g by more than S",
					s, shape, d, floatAtLeastSqrt(d2))
				return ratio, (value - d) / (u * m)
			}
			worst := math.Inf(-1)
			var wp r3.Vec
			var wt [3]r3.Vec
			for range samples {
				var p r3.Vec
				var tri [3]r3.Vec
				if shape == 5 {
					p, tri = depthVertexCase(rng, s)
				} else {
					p, tri = depthSearchCase(rng, s, shape)
				}
				if !finite(p) || !finite(tri[0]) || !finite(tri[1]) || !finite(tri[2]) {
					continue
				}
				r, v := score(p, tri)
				worstValue = max(worstValue, v)
				if r > worst {
					worst, wp, wt = r, p, tri
				}
			}
			if math.IsInf(worst, -1) {
				continue
			}
			for it := range climbs {
				k := 1 + it%8
				p, tri := nudgeULPs(rng, wp, k), wt
				if rng.IntN(3) == 0 {
					i := rng.IntN(3)
					tri[i] = nudgeULPs(rng, tri[i], k)
				}
				r, v := score(p, tri)
				worstValue = max(worstValue, v)
				if r > worst {
					worst, wp, wt = r, p, tri
				}
			}
			worstLemma = max(worstLemma, worst)
		}
	}
	t.Logf("largest shortfall: %.3g of S; largest prune value over the reading: %.3g u·M", worstLemma, worstValue)
}

// TestDepthRoundBoundUpper checks that depthRoundBoundUpper(p.Vec()) is at
// least PointRoundBound(p, p.Vec()) for exact points between two floats: in
// general position, straddling a power of two (where the gap below a float is
// half the gap above it), next to zero and the smallest subnormal, and at
// large magnitudes. It also requires the bound to come within a factor 8 of
// the exact one somewhere, so a bound that only ever reads +Inf fails.
func TestDepthRoundBoundUpper(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(13, 29))
	pairs := [][2]r3.Vec{
		{{X: 1, Y: -2, Z: 4}, {X: math.Nextafter(1, 0), Y: math.Nextafter(-2, 0), Z: math.Nextafter(4, 0)}},
		{{X: 1, Y: 0.5, Z: 8}, {X: math.Nextafter(1, 2), Y: math.Nextafter(0.5, 1), Z: math.Nextafter(8, 9)}},
		{{}, {X: math.SmallestNonzeroFloat64, Y: -math.SmallestNonzeroFloat64, Z: 2 * math.SmallestNonzeroFloat64}},
		{{X: 0x1p400, Y: -0x1p300, Z: 3e100}, {X: math.Nextafter(0x1p400, 0), Y: math.Nextafter(-0x1p300, 0), Z: math.Nextafter(3e100, 0)}},
	}
	for range 200 {
		s := math.Pow(2, float64(rng.IntN(120)-60))
		a := r3.NewVec(s*(2*rng.Float64()-1), s*(2*rng.Float64()-1), s*(2*rng.Float64()-1))
		pairs = append(pairs, [2]r3.Vec{a, {X: math.Nextafter(a.X, 0), Y: math.Nextafter(a.Y, math.Inf(1)), Z: math.Nextafter(a.Z, 0)}})
	}
	tight := false
	for k, pr := range pairs {
		for num := int64(1); num < 13; num++ {
			p := proof.Xlerp(proof.XptOf(pr[0]), proof.XptOf(pr[1]), big.NewInt(num), big.NewInt(13))
			pf := p.Vec()
			exact, upper := PointRoundBound(p, pf), depthRoundBoundUpper(pf)
			require.GreaterOrEqual(t, upper, exact, "pair %d step %d/13: R̄ %g below PointRoundBound %g", k, num, upper, exact)
			if exact > 0 && upper <= 8*exact {
				tight = true
			}
		}
	}
	require.True(t, tight, "R̄ never came within a factor 8 of PointRoundBound")
	require.True(t, math.IsInf(depthRoundBoundUpper(r3.Vec{X: math.MaxFloat64}), 1))
}

// TestDepthKappaOf checks depthKappaOf: 0 for a zero normal, +Inf for a box
// that misses a corner, a NaN box coordinate or an edge too short, and
// otherwise at least the exact L²/|N| over random generic facets and slivers.
func TestDepthKappaOf(t *testing.T) {
	t.Parallel()
	tri := [3]r3.Vec{{X: 1, Y: 2, Z: 3}, {X: 4, Y: 2.5, Z: 1}, {X: 0.5, Y: -1, Z: 2}}
	box := TriBox(tri[:], [3]int{0, 1, 2})
	xn := proof.XtriangleNormal(proof.XptOf(tri[0]), proof.XptOf(tri[1]), proof.XptOf(tri[2]))
	n, shift := depthNormalOf(xn), depthNormalShift(xn)
	require.Zero(t, depthKappaOf(tri, box, r3.Vec{}, 0))
	require.True(t, math.IsInf(depthKappaOf(tri, [2]r3.Vec{box[0], tri[0]}, n, shift), 1))
	nan := box
	nan[1].Y = math.NaN()
	require.True(t, math.IsInf(depthKappaOf(tri, nan, n, shift), 1))
	short := [3]r3.Vec{{X: 1}, {X: 1, Y: 0x1p-520}, {Y: 1}}
	sx := proof.XtriangleNormal(proof.XptOf(short[0]), proof.XptOf(short[1]), proof.XptOf(short[2]))
	shortBox := TriBox(short[:], [3]int{0, 1, 2})
	require.True(t, math.IsInf(depthKappaOf(short, shortBox, depthNormalOf(sx), depthNormalShift(sx)), 1))

	rng := rand.New(rand.NewPCG(3, 17))
	for k := range 400 {
		s := []float64{1, 100, 0x1p200, 0x1p-300}[k%4]
		_, tri := depthSearchCase(rng, s, k%3)
		et := [3]exactVec{exactVecOf(tri[0]), exactVecOf(tri[1]), exactVecOf(tri[2])}
		en := et[1].sub(et[0]).cross(et[2].sub(et[0]))
		nn := en.dot(en)
		if nn.Sign() == 0 {
			continue
		}
		xn := proof.XtriangleNormal(proof.XptOf(tri[0]), proof.XptOf(tri[1]), proof.XptOf(tri[2]))
		kappa := depthKappaOf(tri, TriBox(tri[:], [3]int{0, 1, 2}), depthNormalOf(xn), depthNormalShift(xn))
		if math.IsInf(kappa, 1) {
			continue
		}
		var l2 *big.Rat
		for i := range 3 {
			e := et[(i+1)%3].sub(et[i])
			if d := e.dot(e); l2 == nil || d.Cmp(l2) > 0 {
				l2 = d
			}
		}
		// kappa >= L²/|N| exactly when kappa²·|N|² >= L⁴.
		lhs := new(big.Rat).Mul(new(big.Rat).Mul(ratOf(kappa), ratOf(kappa)), nn)
		require.GreaterOrEqual(t, lhs.Cmp(new(big.Rat).Mul(l2, l2)), 0, "case %d: κ %.17g below L²/|N|", k, kappa)
	}
}
