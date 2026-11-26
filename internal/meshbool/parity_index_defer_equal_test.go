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
					// A query completes a plain scan only when finite and box
					// filtered, so the threshold is read from the counter, not i.
					// Only axis 0 is certain to be queried: a later axis is
					// skipped once an earlier axis decides the answer.
					before := pm.plainScans
					require.Equal(t, want[qi], answerOf(MeshParityAllContext(ctx, queries[qi], pm)),
						`count %d query %d (point %d)`, count, i, qi)
					for axis := range 3 {
						require.LessOrEqual(t, pm.plainScans[axis], ParityIndexDeferScans, `axis %d`, axis)
						if pm.indexBuilt[axis] {
							require.Equal(t, ParityIndexDeferScans, pm.plainScans[axis],
								`count %d query %d axis %d: built before the threshold`, count, i, axis)
						}
						if axis == 0 && before[axis] >= ParityIndexDeferScans {
							require.True(t, pm.indexBuilt[axis],
								`count %d query %d axis %d: index not built after the threshold`, count, i, axis)
							if len(fx.tris) >= ParityIndexMinFacets {
								require.NotNil(t, pm.index[axis],
									`count %d query %d axis %d: no index after the threshold`, count, i, axis)
							}
						}
					}
				}
			}
		})
	}
}
