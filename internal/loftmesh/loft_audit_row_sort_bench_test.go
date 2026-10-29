package loftmesh

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// BenchmarkLongRibbonRowSort measures the real tree row producer and parallel
// row sorting over a lobed tube at the twisted loft route's ceilings: 763
// sections of 328 points are 499,872 triangles, just under twistMaxTriangles
// (500_000), and the ceiling is twistMaxScans (4096 * twistMaxTriangles). Both
// constants live in internal/smoothloft/twisted_mesh.go, which this package
// cannot import. The size comes from those limits, not from any consumer model.
func BenchmarkLongRibbonRowSort(b *testing.B) {
	boxes := lobedTubeBoxes(763, 328, 24, 0.26, 0.004)
	members := allMembers(len(boxes))
	tree := newTreeScan(boxes, members)
	const ceiling = 4096 * 500_000
	counts, err := sweepCandidateCountsWorkers(proofbound.NewWorkBudget(b.Context()), tree,
		len(boxes), keepEveryPair, ceiling, 1)
	if err != nil || counts.exceeded {
		b.Fatalf("count long ribbon rows: %v (exceeded=%t)", err, counts.exceeded)
	}
	var chunks [][2]int
	for lo := 0; lo < len(members); {
		hi := loftRowChunkEnd(members, counts.starts, lo)
		chunks = append(chunks, [2]int{lo, hi})
		lo = hi
	}
	rows := make([]loftRowResult, min(loftRowChunkRows, len(members)))
	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		budget := proofbound.NewWorkBudget(b.Context())
		for _, chunk := range chunks {
			if err := walkLoftRows(budget, tree, members, chunk[0], chunk[1], ceiling, true, 4, rows); err != nil {
				b.Fatal(err)
			}
		}
	}
}
