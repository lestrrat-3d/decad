package meshbool

import (
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// threadContactMesh is an open tube of nTheta x nZ quads, two triangles each,
// whose radius at angle θ and height z is radius(θ, z). Its corners are
// trigonometric values, so their exact lifts carry the long mantissas a
// tessellated curved face has.
func threadContactMesh(tb testing.TB, nTheta, nZ int, length float64, radius func(theta, z float64) float64) *BoolMesh {
	tb.Helper()
	verts := make([]r3.Vec, 0, nTheta*(nZ+1))
	for iz := 0; iz <= nZ; iz++ {
		z := length * float64(iz) / float64(nZ)
		for it := range nTheta {
			theta := 2 * math.Pi * float64(it) / float64(nTheta)
			r := radius(theta, z)
			verts = append(verts, r3.NewVec(r*math.Cos(theta), r*math.Sin(theta), z))
		}
	}
	tris := make([][3]int, 0, 2*nTheta*nZ)
	for iz := range nZ {
		for it := range nTheta {
			v00 := iz*nTheta + it
			v10 := iz*nTheta + (it+1)%nTheta
			v01, v11 := v00+nTheta, v10+nTheta
			tris = append(tris, [3]int{v00, v10, v11}, [3]int{v00, v11, v01})
		}
	}
	bm, err := PrepContext(tb.Context(), verts, tris, nil)
	require.NoError(tb, err)
	return bm
}

// threadContactPairs is a coarse rod crossed by a finer threaded tube, with
// the candidate pairs in the order meshBoolean adds them: each rod facet in
// turn, its box-overlapping thread facets in ascending order.
func threadContactPairs(tb testing.TB, rodTheta, rodZ, threadTheta, threadZ int) (*BoolMesh, *BoolMesh, []ContactPair) {
	tb.Helper()
	const pitch = 1.25
	rod := threadContactMesh(tb, rodTheta, rodZ, 10, func(_, _ float64) float64 { return 5 })
	thread := threadContactMesh(tb, threadTheta, threadZ, 10, func(theta, z float64) float64 {
		return 4.85 + 0.4*math.Sin(2*math.Pi*(z/pitch-theta/(2*math.Pi)))
	})
	return rod, thread, boxPairs(tb, rod, thread)
}

// boxPairs lists the box-overlapping facet pairs of ma and mb in the order
// meshBoolean adds them.
func boxPairs(tb testing.TB, ma, mb *BoolMesh) []ContactPair {
	tb.Helper()
	tree := newBoxTree(mb.Boxes)
	var pairs []ContactPair
	var candidates []int
	for i := range ma.Tris {
		candidates = candidates[:0]
		require.NoError(tb, tree.visitOverlaps(ma.Boxes[i], func() error { return nil }, func(j int) error {
			candidates = append(candidates, j)
			return nil
		}))
		sort.Ints(candidates)
		for _, j := range candidates {
			pairs = append(pairs, ContactPair{I: i, J: j})
		}
	}
	return pairs
}

// BenchmarkNearContactBatch runs the near-contact runner over every candidate
// pair of the rod and thread in ContactBatchSize batches, as
// GatherNearContacts does, on one worker and on four.
func BenchmarkNearContactBatch(b *testing.B) {
	rod, thread, pairs := threadContactPairs(b, 24, 8, 160, 64)
	run := NearContactRunner(0.01)
	for _, workers := range []int{1, 4} {
		b.Run(map[int]string{1: "workers=1", 4: "workers=4"}[workers], func(b *testing.B) {
			results := make([]ContactBatchResult, ContactBatchSize)
			segments := 0
			b.ReportAllocs()
			for b.Loop() {
				segments = 0
				for start := 0; start < len(pairs); start += ContactBatchSize {
					batch := pairs[start:min(start+ContactBatchSize, len(pairs))]
					if err := run(b.Context(), rod, thread, batch, results[:len(batch)], workers); err != nil {
						b.Fatal(err)
					}
					for _, r := range results[:len(batch)] {
						if r.Contact.Kind == ContactSegment {
							segments++
						}
					}
				}
			}
			b.ReportMetric(float64(len(pairs)), "pairs/op")
			b.ReportMetric(float64(segments), "segments/op")
		})
	}
}

// BenchmarkGatherNearContactsApart gathers the near contacts of a rod inside a
// coaxial tube 0.4 mm farther out, at a 0.01 mm slack. Their oblique facets'
// boxes overlap, so the gate classifies and measures those pairs, and none
// comes within the slack. "fresh" measures on the batch workers; "memo"
// repeats the gather with a filled memo, so the producer measures each pair.
func BenchmarkGatherNearContactsApart(b *testing.B) {
	rod := threadContactMesh(b, 24, 8, 10, func(_, _ float64) float64 { return 5 })
	tube := threadContactMesh(b, 160, 64, 12, func(_, _ float64) float64 { return 5.4 })
	fa, fb := AllFacets(rod), AllFacets(tube)
	const slack = 0.01
	gather := func(b *testing.B, memo *ContactMemo) {
		nc, deferred, err := GatherNearContacts(b.Context(), rod, fa, tube, fb, slack, memo)
		require.NoError(b, err)
		require.False(b, deferred)
		require.Empty(b, nc.CloseA)
	}
	b.Run("fresh", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			gather(b, NewContactMemo(rod, tube))
		}
	})
	b.Run("memo", func(b *testing.B) {
		memo := NewContactMemo(rod, tube)
		gather(b, memo)
		b.ReportAllocs()
		for b.Loop() {
			gather(b, memo)
		}
	})
}
