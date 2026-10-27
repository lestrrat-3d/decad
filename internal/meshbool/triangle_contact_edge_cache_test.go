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

// legacyClassify is TriTriClassifyCore as it was before the edge-plane cache,
// the line-order dot reuse and the unreduced sin²θ: every edge normal and its
// corner's dot product built per pair, every line comparison recomputing both
// dot products, and sin²θ reduced to a big.Rat. The contact it returns has a
// nil Sin2; the reduced sin²θ is the second result.
func legacyClassify(ta, tb [3]r3.Vec, xta, xtb [3]proof.Xpt, na, nb proof.Xpt, pa, pb *[3]Xp2, sa, sb *[3]int,
	useFilter bool, fna, fnb r3.Vec, normalsPrepared bool,
) (TriContact, *big.Rat, error) {
	span, crossing, err := legacyTriTriSpan(ta, tb, xta, xtb, na, nb, pa, pb, sa, sb, useFilter, fna, fnb, normalsPrepared)
	out := TriContact{EdgeA: -1, EdgeB: -1}
	if err != nil {
		return out, nil, err
	}
	out.Kind, out.P0, out.P1 = span.Kind, span.P0, span.P1
	if !crossing {
		return out, nil, nil
	}
	sin2 := legacySinSquared(na, nb)
	if out.Kind == ContactSegment {
		out.P0OnA, out.P1OnA, out.EdgeA = legacySegmentBoundaryEdges(out.P0, out.P1, xta, na)
		out.P0OnB, out.P1OnB, out.EdgeB = legacySegmentBoundaryEdges(out.P0, out.P1, xtb, nb)
	} else {
		out.P0OnA = OnTriBoundary(out.P0, xta, na)
		out.P0OnB = OnTriBoundary(out.P0, xtb, nb)
		out.P1OnA, out.P1OnB = out.P0OnA, out.P0OnB
	}
	return out, sin2, nil
}

func legacyTriTriSpan(ta, tb [3]r3.Vec, xta, xtb [3]proof.Xpt, na, nb proof.Xpt, pa, pb *[3]Xp2, sa, sb *[3]int,
	useFilter bool, fna, fnb r3.Vec, normalsPrepared bool,
) (TriContactSpan, bool, error) {
	var span TriContactSpan
	var signsB, signsA [3]int
	if sb != nil {
		signsB = *sb
	} else {
		for i := range 3 {
			signsB[i] = proof.OrientSignPrepared(ta[0], ta[1], ta[2], tb[i], xta[0], xtb[i], na)
		}
	}
	if AllOneSide(signsB) {
		return span, false, nil
	}
	if sa != nil {
		signsA = *sa
	} else {
		for i := range 3 {
			signsA[i] = proof.OrientSignPrepared(tb[0], tb[1], tb[2], ta[i], xtb[0], xta[i], nb)
		}
	}
	if AllOneSide(signsA) {
		return span, false, nil
	}
	if CountZero(signsA) == 3 || CountZero(signsB) == 3 {
		if useFilter && CoplanarFloatSeparated(ta, tb, na) {
			return span, false, nil
		}
		overlap := false
		if pa != nil && pb != nil {
			overlap = CoplanarOverlapProjected(*pa, *pb)
		} else {
			overlap = CoplanarOverlap(xta, xtb, na)
		}
		if overlap {
			span.Kind = ContactRegion
			return span, false, nil
		}
		if p, ok := CoplanarTouch(xta, xtb, na, nb); ok {
			span.Kind, span.P0, span.P1 = ContactPoint, p, p
		}
		return span, false, nil
	}
	if useFilter {
		if !normalsPrepared {
			fna, fnb = na.Vec(), nb.Vec()
		}
		if TriTriMissesFilter(ta, tb, fna, fnb, signsA, signsB) {
			return span, false, nil
		}
	}
	ptsA := planeCrossingsPrepared(xta, xtb[0], nb, signsA)
	ptsB := planeCrossingsPrepared(xtb, xta[0], na, signsB)
	if len(ptsA) == 0 || len(ptsB) == 0 {
		return span, false, nil
	}
	if len(ptsA) > 2 || len(ptsB) > 2 {
		return TriContactSpan{}, false, fmt.Errorf(`legacy: a facet crosses a plane more than twice`)
	}
	dir := proof.Xcross(na, nb)
	loA, hiA := OrderOnLine(ptsA, dir)
	loB, hiB := OrderOnLine(ptsB, dir)
	lo, hi := loA, hiA
	if CmpOnLine(loB, lo, dir) > 0 {
		lo = loB
	}
	if CmpOnLine(hiB, hi, dir) < 0 {
		hi = hiB
	}
	switch CmpOnLine(lo, hi, dir) {
	case 1:
		return span, false, nil
	case 0:
		span.Kind, span.P0, span.P1 = ContactPoint, lo, lo
	default:
		span.Kind, span.P0, span.P1 = ContactSegment, lo, hi
	}
	return span, true, nil
}

func legacySegmentBoundaryEdges(p0, p1 proof.Xpt, xt [3]proof.Xpt, n proof.Xpt) (bool, bool, int) {
	points := [2]proof.Xpt{p0, p1}
	var known, on [2]uint8
	var edgeKnown uint8
	var edgeNormals [3]proof.Xpt
	var sideScratch [3]big.Int
	onEdge := func(point, edge int) bool {
		bit := uint8(1 << edge)
		if known[point]&bit == 0 {
			known[point] |= bit
			a, b := xt[edge], xt[(edge+1)%3]
			if edgeKnown&bit == 0 {
				edgeNormals[edge] = proof.Xcross(n, proof.Xsub(b, a))
				edgeKnown |= bit
			}
			if proof.XdotDiffSignScratch(&sideScratch, edgeNormals[edge], points[point], a) == 0 &&
				PointOnSegment3D(a, b, points[point]) {
				on[point] |= bit
			}
		}
		return on[point]&bit != 0
	}
	var on0, on1 bool
	for edge := range 3 {
		if onEdge(0, edge) {
			on0 = true
			break
		}
	}
	for edge := range 3 {
		if onEdge(1, edge) {
			on1 = true
			break
		}
	}
	for edge := range 3 {
		if onEdge(0, edge) && onEdge(1, edge) {
			return on0, on1, edge
		}
	}
	return on0, on1, -1
}

func legacySinSquared(na, nb proof.Xpt) *big.Rat {
	c := proof.XhpCross(proof.Xhp(na), proof.Xhp(nb))
	num := proof.XhpDotNum(c, c)
	den := new(big.Int).Mul(proof.XdotNum(na, na), proof.XdotNum(nb, nb))
	if den.Sign() == 0 {
		return new(big.Rat)
	}
	return new(big.Rat).SetFrac(num, den)
}

func legacySinLowerBound(sin2 *big.Rat) float64 {
	f, _ := sin2.Float64()
	if f <= 0 {
		return 0
	}
	s := math.Sqrt(f)
	s = math.Nextafter(s, 0)
	return math.Nextafter(s, 0)
}

func legacyRimBound(deltaA, deltaB float64, sin2 *big.Rat) float64 {
	d := proofbound.AbsSumUpper(deltaA, deltaB)
	if d <= 0 {
		return 0
	}
	if sin2 == nil {
		return math.Inf(1)
	}
	return proofbound.DivUpper(d, legacySinLowerBound(sin2))
}

// contactCacheStats counts what a differential sweep compared, so a test can
// require that it reached every case it is meant to.
type contactCacheStats struct {
	kinds      [4]int
	alongEdge  int
	onBoundary int
}

// requireSameContact compares got with the legacy answer field by field. The
// points are compared integer by integer, and sin²θ by value, by its float
// bits and by the rim bounds it gives.
func requireSameContact(t *testing.T, stats *contactCacheStats, want TriContact, wantSin2 *big.Rat, wantErr error,
	got TriContact, gotErr error, label string,
) {
	t.Helper()
	require.Equal(t, wantErr, gotErr, label)
	require.Equal(t, want.Kind, got.Kind, label)
	require.Equal(t, [4]bool{want.P0OnA, want.P1OnA, want.P0OnB, want.P1OnB},
		[4]bool{got.P0OnA, got.P1OnA, got.P0OnB, got.P1OnB}, label)
	require.Equal(t, [2]int{want.EdgeA, want.EdgeB}, [2]int{got.EdgeA, got.EdgeB}, label)
	for k, pair := range [][2]proof.Xpt{{want.P0, got.P0}, {want.P1, got.P1}} {
		w, g := pair[0], pair[1]
		require.Equal(t, w.X == nil, g.X == nil, "%s point %d", label, k)
		if w.X == nil {
			continue
		}
		require.Zero(t, w.X.Cmp(g.X), "%s point %d", label, k)
		require.Zero(t, w.Y.Cmp(g.Y), "%s point %d", label, k)
		require.Zero(t, w.Z.Cmp(g.Z), "%s point %d", label, k)
		require.Zero(t, w.W.Cmp(g.W), "%s point %d", label, k)
	}
	require.Equal(t, wantSin2 == nil, got.Sin2 == nil, label)
	if wantSin2 != nil {
		require.Zero(t, wantSin2.Cmp(got.Sin2.Rat()), label)
		wantFloat, _ := wantSin2.Float64()
		require.Equal(t, math.Float64bits(wantFloat), math.Float64bits(got.Sin2.Float64()), label)
		require.Equal(t, math.Float64bits(legacySinLowerBound(wantSin2)), math.Float64bits(SinLowerBound(got.Sin2)), label)
		for _, delta := range [][2]float64{{0, 0}, {1e-3, 0}, {1e-6, 2e-6}, {0.25, 0.5}} {
			require.Equal(t, math.Float64bits(legacyRimBound(delta[0], delta[1], wantSin2)),
				math.Float64bits(RimBound(delta[0], delta[1], got.Sin2)), label)
		}
	}
	if stats == nil {
		return
	}
	stats.kinds[got.Kind]++
	if got.EdgeA >= 0 || got.EdgeB >= 0 {
		stats.alongEdge++
	}
	if got.P0OnA || got.P1OnA || got.P0OnB || got.P1OnB {
		stats.onBoundary++
	}
}

// legacyPair is the legacy answer for facet pair (i, j) of two prepared meshes.
func legacyPair(ma, mb *BoolMesh, pair ContactPair) (TriContact, *big.Rat, error) {
	ma.PrepareFloatNormal(pair.I)
	mb.PrepareFloatNormal(pair.J)
	return legacyClassify(TriCorners(ma, pair.I), TriCorners(mb, pair.J), XtriCorners(ma, pair.I), XtriCorners(mb, pair.J),
		ma.Norms[pair.I], mb.Norms[pair.J], nil, nil, nil, nil, true, ma.Fnorms[pair.I], mb.Fnorms[pair.J], true)
}

// scaledMesh is m with every exact vertex rewritten over a weight that is not
// a power of two, so every exact predicate meets general homogeneous weights.
// Each vertex gets its own factor, so neighbouring weights differ.
func scaledMesh(t *testing.T, m *BoolMesh) *BoolMesh {
	t.Helper()
	out := *m
	out.Xverts = make([]proof.Xpt, len(m.Xverts))
	for i, x := range m.Xverts {
		out.Xverts[i] = scaleContactPoint(x, 3+2*(i%7))
	}
	out.Norms = make([]proof.Xpt, len(m.Tris))
	out.Fnorms = make([]r3.Vec, len(m.Tris))
	out.FnormsReady = make([]bool, len(m.Tris))
	for f, tri := range m.Tris {
		a, b, c := out.Xverts[tri[0]], out.Xverts[tri[1]], out.Xverts[tri[2]]
		out.Norms[f] = proof.Xcross(proof.Xsub(b, a), proof.Xsub(c, a))
	}
	return &out
}

// randomCrossingMeshes builds two soups of small triangles around common
// centres, so most pairs cross at random angles, some share a corner, and
// some lie along each other's edges.
func randomCrossingMeshes(t *testing.T, seed uint64, n int) (*BoolMesh, *BoolMesh) {
	t.Helper()
	rng := rand.New(rand.NewPCG(seed, seed^0x9e3779b97f4a7c15))
	// jitter is a random point of the unit cube centred on c.
	jitter := func(c r3.Vec) r3.Vec {
		return r3.NewVec(c.X+rng.Float64()-0.5, c.Y+rng.Float64()-0.5, c.Z+rng.Float64()-0.5)
	}
	var va, vb []r3.Vec
	var ta, tb [][3]int
	for k := range n {
		c := r3.NewVec(3*math.Sin(float64(k)), 3*math.Cos(1.3*float64(k)), 0.7*float64(k%5))
		base := len(va)
		va = append(va, jitter(c), jitter(c), jitter(c))
		ta = append(ta, [3]int{base, base + 1, base + 2})
		// Several partner triangles per first-operand triangle, as a coarse
		// facet meets several finer ones.
		for m := range 4 {
			bb := len(vb)
			switch m {
			case 0:
				// Shares the first corner exactly.
				vb = append(vb, va[base], jitter(c), jitter(c))
			case 1:
				// Shares the first edge exactly, so a segment can run along it.
				vb = append(vb, va[base+1], va[base], jitter(c))
			default:
				vb = append(vb, jitter(c), jitter(c), jitter(c))
			}
			tb = append(tb, [3]int{bb, bb + 1, bb + 2})
		}
	}
	ma, err := PrepContext(t.Context(), va, ta, nil)
	require.NoError(t, err)
	mb, err := PrepContext(t.Context(), vb, tb, nil)
	require.NoError(t, err)
	return ma, mb
}

// contactCacheFixtures returns the mesh pairs the differential sweep reads,
// each with its candidate pairs.
func contactCacheFixtures(t *testing.T) map[string]struct {
	ma, mb *BoolMesh
	pairs  []ContactPair
} {
	t.Helper()
	out := map[string]struct {
		ma, mb *BoolMesh
		pairs  []ContactPair
	}{}
	add := func(name string, ma, mb *BoolMesh, pairs []ContactPair) {
		out[name] = struct {
			ma, mb *BoolMesh
			pairs  []ContactPair
		}{ma, mb, pairs}
	}
	rod, thread, pairs := threadContactPairs(t, 12, 4, 64, 24)
	add("thread", rod, thread, pairs)
	add("thread scaled", scaledMesh(t, rod), scaledMesh(t, thread), pairs)
	for name, pair := range nearGatePairs(t, 5) {
		add("grid "+name, pair[0], pair[1], boxPairs(t, pair[0], pair[1]))
	}
	for seed := range uint64(3) {
		ma, mb := randomCrossingMeshes(t, seed+1, 40)
		var all []ContactPair
		for i := range ma.Tris {
			for j := range mb.Tris {
				all = append(all, ContactPair{I: i, J: j})
			}
		}
		add(fmt.Sprintf("random %d", seed), ma, mb, all)
		add(fmt.Sprintf("random %d scaled", seed), scaledMesh(t, ma), scaledMesh(t, mb), all)
	}
	return out
}

// TestContactBatchMatchesLegacyClassify holds every classification path the
// cache, the line-order reuse and the unreduced sin²θ touch to the legacy
// classifier, pair by pair: the contact batch runner on one and on four
// workers, the near-contact runner with its distances, the memo and the
// unprepared entry point.
func TestContactBatchMatchesLegacyClassify(t *testing.T) {
	t.Parallel()
	var stats contactCacheStats
	for name, fx := range contactCacheFixtures(t) {
		want := make([]TriContact, len(fx.pairs))
		wantSin2 := make([]*big.Rat, len(fx.pairs))
		wantErr := make([]error, len(fx.pairs))
		for k, pair := range fx.pairs {
			want[k], wantSin2[k], wantErr[k] = legacyPair(fx.ma, fx.mb, pair)
		}
		for _, workers := range []int{1, 4} {
			for _, near := range []bool{false, true} {
				run := RunContactBatch
				if near {
					run = NearContactRunner()
				}
				results := make([]ContactBatchResult, len(fx.pairs))
				for start := 0; start < len(fx.pairs); start += ContactBatchSize {
					end := min(start+ContactBatchSize, len(fx.pairs))
					require.NoError(t, run(t.Context(), fx.ma, fx.mb, fx.pairs[start:end], results[start:end], workers))
				}
				for k, pair := range fx.pairs {
					label := fmt.Sprintf("%s workers=%d near=%v pair=%v", name, workers, near, pair)
					var s *contactCacheStats
					if workers == 1 && !near {
						s = &stats
					}
					requireSameContact(t, s, want[k], wantSin2[k], wantErr[k], results[k].Contact, results[k].Err, label)
					measured := near && wantErr[k] == nil && want[k].Kind == ContactNone
					require.Equal(t, measured, results[k].Measured, label)
					wantDist := 0.0
					if measured {
						wantDist = TriTriDistance(TriCorners(fx.ma, pair.I), TriCorners(fx.mb, pair.J))
					}
					require.Equal(t, math.Float64bits(wantDist), math.Float64bits(results[k].Dist), label)
				}
			}
		}
		memo := NewContactMemo(fx.ma, fx.mb)
		for k, pair := range fx.pairs {
			label := fmt.Sprintf("%s memo pair=%v", name, pair)
			got, err := memo.Classify(pair.I, pair.J)
			if wantErr[k] != nil {
				require.Equal(t, wantErr[k], err, label)
				continue
			}
			requireSameContact(t, nil, want[k], wantSin2[k], nil, got, err, label)
			xa, xb := XtriCorners(fx.ma, pair.I), XtriCorners(fx.mb, pair.J)
			got, err = TriTriClassify(TriCorners(fx.ma, pair.I), TriCorners(fx.mb, pair.J), xa, xb, fx.ma.Norms[pair.I], fx.mb.Norms[pair.J])
			lw, lsin2, lerr := legacyClassify(TriCorners(fx.ma, pair.I), TriCorners(fx.mb, pair.J), xa, xb,
				fx.ma.Norms[pair.I], fx.mb.Norms[pair.J], nil, nil, nil, nil, true, r3.Vec{}, r3.Vec{}, false)
			requireSameContact(t, nil, lw, lsin2, lerr, got, err, label+" unprepared")
		}
	}
	require.Positive(t, stats.kinds[ContactNone], "the sweep never saw a miss")
	require.Positive(t, stats.kinds[ContactPoint], "the sweep never saw a point contact")
	require.Positive(t, stats.kinds[ContactSegment], "the sweep never saw a segment")
	require.Positive(t, stats.kinds[ContactRegion], "the sweep never saw a coplanar overlap")
	require.Positive(t, stats.alongEdge, "the sweep never saw a segment along a facet edge")
	require.Positive(t, stats.onBoundary, "the sweep never saw an end on a facet boundary")
	t.Logf("kinds=%v alongEdge=%d onBoundary=%d", stats.kinds, stats.alongEdge, stats.onBoundary)
}

// TestEdgePlaneCacheSwitchesFacets asks one cache about pairs whose first
// facet alternates, so a cache that answered for the wrong facet would hand
// one facet's edge planes to another.
func TestEdgePlaneCacheSwitchesFacets(t *testing.T) {
	t.Parallel()
	rod, thread, pairs := threadContactPairs(t, 12, 4, 64, 24)
	var segments []ContactPair
	for _, pair := range pairs {
		if c, _, err := legacyPair(rod, thread, pair); err == nil && c.Kind == ContactSegment {
			segments = append(segments, pair)
		}
	}
	require.Greater(t, len(segments), 8)
	// Interleave the segment pairs of different first facets.
	order := make([]ContactPair, 0, len(segments))
	for k := range (len(segments) + 1) / 2 {
		order = append(order, segments[k])
		if m := len(segments) - 1 - k; m > k {
			order = append(order, segments[m])
		}
	}
	switches := 0
	cache := newEdgePlaneCache()
	for k, pair := range order {
		if k > 0 && order[k-1].I != pair.I {
			switches++
		}
		want, wantSin2, wantErr := legacyPair(rod, thread, pair)
		got := classifyContactPair(rod, thread, pair, &cache)
		requireSameContact(t, nil, want, wantSin2, wantErr, got.Contact, got.Err, fmt.Sprintf("pair=%v", pair))
	}
	require.Greater(t, switches, len(order)/2)
}

// TestEdgePlanesMatchEdgePredicates holds each edge plane to the predicates it
// replaces: the side sign to XdotDiffSignScratch with a freshly built edge
// normal, and onSegment to PointOnSegment3D, for points on each edge's line
// inside and outside the edge, and points off it. The facets include edges
// along a single axis, where a wrong axis choice compares equal coordinates,
// and corners over general weights.
func TestEdgePlanesMatchEdgePredicates(t *testing.T) {
	t.Parallel()
	facets := [][3]r3.Vec{
		{{X: 0.1, Y: 0.2, Z: 0.3}, {X: 0.1, Y: 0.2, Z: 2.7}, {X: 1.9, Y: 0.2, Z: 0.3}},
		{{X: 0.5, Y: -1}, {X: 0.5, Y: 3}, {X: 0.5, Y: 1, Z: 2.25}},
		{{X: math.Sin(1), Y: math.Cos(2), Z: math.Sin(3)}, {X: math.Cos(4), Y: math.Sin(5), Z: math.Cos(6)},
			{X: math.Sin(7), Y: math.Cos(8), Z: math.Sin(9)}},
		{{X: 5, Y: 0, Z: 0}, {X: 5 * math.Cos(0.3), Y: 5 * math.Sin(0.3), Z: 0}, {X: 5, Y: 0, Z: 0.625}},
	}
	ts := [][2]int64{{-1, 1}, {-1, 4}, {0, 1}, {1, 3}, {1, 2}, {1, 1}, {5, 4}, {3, 1}}
	checked := 0
	for f, corners := range facets {
		for _, scale := range []int{1, 3} {
			var xt [3]proof.Xpt
			for k, c := range corners {
				xt[k] = scaleContactPoint(proof.XptOf(c), scale+k)
			}
			n := proof.Xcross(proof.Xsub(xt[1], xt[0]), proof.Xsub(xt[2], xt[0]))
			planes := newEdgePlanes(xt, n)
			var scratch [3]big.Int
			for k := range 3 {
				a, b := xt[k], xt[(k+1)%3]
				normal := proof.Xcross(n, proof.Xsub(b, a))
				probes := []proof.Xpt{xt[(k+2)%3], proof.XptOf(r3.NewVec(0.3, -0.7, 1.1))}
				for _, tt := range ts {
					probes = append(probes, proof.Xlerp(a, b, big.NewInt(tt[0]), big.NewInt(tt[1])))
				}
				for m, p := range probes {
					label := fmt.Sprintf("facet=%d scale=%d edge=%d probe=%d", f, scale, k, m)
					wantSide := proof.XdotDiffSignScratch(&scratch, normal, p, a)
					require.Equal(t, wantSide, proof.XdotDiffSignOffset(&scratch, planes.Normal[k], p, a, planes.Offset[k]), label)
					if wantSide != 0 {
						continue
					}
					require.Equal(t, PointOnSegment3D(a, b, p), planes.onSegment(&scratch[0], &scratch[1], k, p), label)
					checked++
				}
			}
		}
	}
	require.Greater(t, checked, 100)
}
