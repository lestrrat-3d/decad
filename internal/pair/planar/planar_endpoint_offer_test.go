package planar

import (
	"math"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/proof"
)

func TestEndpointOfferPrunedUsesStrictOutwardBound(t *testing.T) {
	point := floatBox{lo: [3]float64{0, 0, 0}, hi: [3]float64{0, 0, 0}}
	segment := floatBox{lo: [3]float64{2, 0, 0}, hi: [3]float64{3, 0, 0}}

	tests := []struct {
		name    string
		hasBest bool
		bestUp  float64
		segment floatBox
		want    bool
	}{
		{name: "no best", segment: segment},
		{name: "strictly beyond best", hasBest: true, bestUp: 3, segment: segment, want: true},
		{name: "equal to best", hasBest: true, bestUp: 4, segment: segment},
		{name: "overlapping boxes", hasBest: true, bestUp: 0, segment: point},
		{name: "infinite best", hasBest: true, bestUp: math.Inf(1), segment: segment},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			k := planarKernel{hasBest: tt.hasBest, bestUp: tt.bestUp}
			if got := k.endpointOfferPruned(&point, &tt.segment); got != tt.want {
				t.Fatalf("endpointOfferPruned() = %v, want %v", got, tt.want)
			}
		})
	}
}

// edgeEdgeUnfiltered preserves the exact offer path used by scanPlain as an
// independent reference for the endpoint filter.
func (k *planarKernel) edgeEdgeUnfiltered(edgeAI, edgeBI int) {
	edgeA, edgeB := k.a.edges[edgeAI], k.b.edges[edgeBI]
	p0, p1 := k.a.s.Verts[edgeA[0]], k.a.s.Verts[edgeA[1]]
	q0, q1 := k.b.s.Verts[edgeB[0]], k.b.s.Verts[edgeB[1]]
	d1, d2 := proof.DvSub(p1, p0), proof.DvSub(q1, q0)
	r := proof.DvSub(p0, q0)
	a, c := proof.DvDot(d1, d1), proof.DvDot(d2, d2)
	k.offer(pointSegmentRelative(r, d2, c))
	k.offer(pointSegmentRelative(proof.DvSub(p1, q0), d2, c))
	k.offer(pointSegmentRelative(proof.DvSub(q0, p0), d1, a))
	k.offer(pointSegmentRelative(proof.DvSub(q1, p0), d1, a))
	cross := proof.DvCross(d1, d2)
	contact := PlanarContact{A: PlanarFeature{Kind: FeatureEdge, Edge: edgeA},
		B: PlanarFeature{Kind: FeatureEdge, Edge: edgeB}}
	if proof.DvIsZero(cross) {
		if !proof.DvIsZero(proof.DvCross(r, d1)) {
			return
		}
		t0 := proof.DvDot(proof.DvSub(q0, p0), d1)
		t1 := proof.DvDot(proof.DvSub(q1, p0), d1)
		lo := proof.DyMax(proof.DyZero(), proof.DyMin(t0, t1))
		hi := proof.DyMin(a, proof.DyMax(t0, t1))
		if proof.DyCmp(lo, hi) >= 0 {
			return
		}
		w := proof.DyMul(proof.DyInt(2), a)
		mid := hpoint{x: proof.DvAdd(dvScale(p0, w), dvScale(d1, proof.DyAdd(lo, hi))), w: w}
		k.addSite(contactSite{contact: contact, at: mid, dir: d1, cell: true})
		return
	}
	b := proof.DvDot(d1, d2)
	d, e := proof.DvDot(d1, r), proof.DvDot(d2, r)
	den := proof.DvDot(cross, cross)
	s := proof.DySubScalar(proof.DyMul(b, e), proof.DyMul(c, d))
	t := proof.DySubScalar(proof.DyMul(a, e), proof.DyMul(b, d))
	if s.Sign() <= 0 || proof.DyCmp(s, den) >= 0 || t.Sign() <= 0 || proof.DyCmp(t, den) >= 0 {
		return
	}
	height := proof.DvDot(r, cross)
	k.offer(frac{num: proof.DyMul(height, height), den: den})
	if height.Sign() != 0 {
		return
	}
	at := hpoint{x: proof.DvAdd(dvScale(p0, den), dvScale(d1, s)), w: den}
	k.addSite(contactSite{contact: contact, at: at})
}

var endpointOfferPruneSink bool

func BenchmarkEndpointOfferPruneNoSkip(b *testing.B) {
	point := floatBox{lo: [3]float64{0, 0, 0}, hi: [3]float64{1, 1, 1}, tight: true}
	segment := point
	k := planarKernel{hasBest: true, bestUp: 1}

	b.Run("single-axis-pointer", func(b *testing.B) {
		for b.Loop() {
			endpointOfferPruneSink = k.endpointOfferPruned(&point, &segment)
		}
	})
	b.Run("generic-box-copy", func(b *testing.B) {
		for b.Loop() {
			endpointOfferPruneSink = gapSquaredBelowLimit(point, segment, k.bestUp) > k.bestUp
		}
	})
}

func BenchmarkEdgeOfferFilterNoSkip(b *testing.B) {
	filtered, exact := benchmarkEdgeOfferKernels()
	b.Run("filtered", func(b *testing.B) {
		for b.Loop() {
			filtered.edgeEdge(0, 0)
		}
	})
	b.Run("exact", func(b *testing.B) {
		for b.Loop() {
			exact.edgeEdgeUnfiltered(0, 0)
		}
	})
}

func BenchmarkEdgeOfferFilterNoSkipPaired(b *testing.B) {
	filtered, exact := benchmarkEdgeOfferKernels()
	var filteredTime, exactTime time.Duration
	for i := 0; b.Loop(); i++ {
		if i&1 == 0 {
			exactTime += benchmarkEdgeEdgeBatch(exact, false)
			filteredTime += benchmarkEdgeEdgeBatch(filtered, true)
		} else {
			filteredTime += benchmarkEdgeEdgeBatch(filtered, true)
			exactTime += benchmarkEdgeEdgeBatch(exact, false)
		}
	}
	b.StopTimer()
	b.ReportMetric(float64(filteredTime.Nanoseconds())/float64(b.N*256), "filtered-ns/op")
	b.ReportMetric(float64(exactTime.Nanoseconds())/float64(b.N*256), "exact-ns/op")
}

func benchmarkEdgeEdgeBatch(k *planarKernel, filtered bool) time.Duration {
	start := time.Now()
	if filtered {
		for range 256 {
			k.edgeEdge(0, 0)
		}
	} else {
		for range 256 {
			k.edgeEdgeUnfiltered(0, 0)
		}
	}
	return time.Since(start)
}

func benchmarkEdgeOfferKernels() (*planarKernel, *planarKernel) {
	point := func(x, y, z float64) proof.DyV3 {
		return proof.DyV3{proof.MustDyOf(x), proof.MustDyOf(y), proof.MustDyOf(z)}
	}
	a := &PlanarSolid{Verts: []proof.DyV3{point(0, 0, 0), point(4, 4, 4)}}
	b := &PlanarSolid{Verts: []proof.DyV3{point(0, 0, 0), point(4, 4, 4)}}
	pa := &planarPrep{s: a, edges: [][2]int{{0, 1}}}
	pb := &planarPrep{s: b, edges: [][2]int{{0, 1}}}
	pa.floatBoxes()
	pb.floatBoxes()
	best := frac{num: proof.DyZero(), den: proof.DyInt(1)}
	contact := PlanarContact{A: PlanarFeature{Kind: FeatureEdge, Edge: [2]int{0, 1}},
		B: PlanarFeature{Kind: FeatureEdge, Edge: [2]int{0, 1}}}
	filtered := &planarKernel{a: pa, b: pb, best: best, bestUp: 0, hasBest: true,
		seen: map[PlanarContact]struct{}{contact: {}}}
	exact := &planarKernel{a: pa, b: pb, best: best, bestUp: 0, hasBest: true,
		seen: map[PlanarContact]struct{}{contact: {}}}
	return filtered, exact
}
