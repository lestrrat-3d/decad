package meshbool

// CI: isolated-tests

import (
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestParityIndexAnswersEqualAcrossTheDeferThreshold requires every answer,
// error text included, to equal the unindexed scan whether the query took the
// plain scan (before ParityIndexDeferScans completed scans on its axis) or the
// parity index (after). Each mesh is queried 1, 15, 16, 17 and 40 times so the
// threshold falls inside one run, and the index state is checked per query.
func TestParityIndexAnswersEqualAcrossTheDeferThreshold(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0xdef3, 0x16))
	for _, fx := range parityIndexFixtures() {
		queries := parityIndexQueries(rng, fx)
		t.Run(fx.name, func(t *testing.T) {
			t.Parallel()
			ctx := t.Context()
			all := parityAllFacets(len(fx.tris))
			plain := NewParityMesh(fx.verts, fx.tris)
			plain.Unfiltered = true
			want := make([]parityAnswer, len(queries))
			for qi, p := range queries {
				want[qi] = answerOf(MeshParityPreparedContext(ctx, p, plain, all))
			}
			for _, count := range []int{1, 15, 16, 17, 40} {
				pm := NewParityMesh(fx.verts, fx.tris)
				for i := range count {
					qi := (i * 7) % len(queries)
					require.Equal(t, want[qi], answerOf(MeshParityAllContext(ctx, queries[qi], pm)),
						`count %d query %d (point %d)`, count, i, qi)
					for axis := range 3 {
						require.LessOrEqual(t, pm.plainScans[axis], ParityIndexDeferScans, `axis %d`, axis)
						if pm.indexBuilt[axis] {
							require.Equal(t, ParityIndexDeferScans, pm.plainScans[axis],
								`count %d query %d axis %d: built before the threshold`, count, i, axis)
						}
					}
				}
			}
		})
	}
}
