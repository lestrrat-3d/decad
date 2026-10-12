package loftmesh

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestStreamLoftPairsMatchesCompleteCandidateList(t *testing.T) {
	const sectors = 120
	verts := []r3.Vec{r3.NewVec(0, 0, 0)}
	tris := make([][3]int, 0, sectors+1)
	for i := range sectors {
		a := 2 * math.Pi * float64(i) / sectors
		b := a + math.Pi/sectors/2
		start := len(verts)
		verts = append(verts, r3.NewVec(math.Cos(a), math.Sin(a), 0),
			r3.NewVec(math.Cos(b), math.Sin(b), 0))
		tris = append(tris, [3]int{0, start, start + 1})
	}
	for _, fail := range []bool{false, true} {
		t.Run(map[bool]string{false: "clean", true: "first refusal"}[fail], func(t *testing.T) {
			input := tris
			if fail {
				start := len(verts)
				verts = append(verts, verts[1], verts[2])
				input = append(append([][3]int(nil), tris...), [3]int{0, start, start + 1})
			}
			want, wantErr := loftCrossingAuditLimited(proofbound.NewWorkBudget(t.Context()), verts,
				input, nil, loftGenericShortcuts, 8_000_000, 1_000, false)
			boxes := make([][2]r3.Vec, len(input))
			members := make([]int, len(input))
			for i, tri := range input {
				boxes[i] = meshbool.TriBox(verts, tri)
				members[i] = i
			}
			budget := proofbound.NewWorkBudget(t.Context())
			order := newPairScan(boxes, members, 8_000_000)
			sc, err := sweepCandidateCounts(budget, order, len(input), keepEveryPair, 8_000_000)
			require.NoError(t, err)
			require.Greater(t, sc.count, uint64(17))
			tree := newTreeScan(boxes, members)
			require.NoError(t, preflightLoftRows(budget, tree, members, sc.starts, 8_000_000))
			data := NewLoftAuditData(verts, input)
			got := LoftAuditWork{Candidates: int(sc.count), Scanned: int(sc.scanned)}
			testPair := func(i, j int) error {
				if err := budget.Step(); err != nil {
					return err
				}
				_, shared := tessellation.SharedVertexIndices(input[i], input[j])
				if shared == 0 && (!meshbool.BoxesOverlap(boxes[i], boxes[j]) ||
					loftPlaneSeparatedPrepared(data, i, j)) {
					got.Skips++
					return nil
				}
				outcome, err := AuditLoftPairData(data, input, i, j, loftGenericShortcuts)
				switch outcome {
				case LoftPairEdgeCertificate:
					got.EdgeCerts++
				case LoftPairVertexCertificate:
					got.VertexCerts++
				default:
					got.Classifications++
				}
				return err
			}
			_, _, gotErr := streamLoftPairs(budget, tree, members, sc.starts, sc.count,
				8_000_000, 17, input, boxes, data, loftGenericShortcuts, &got, testPair)
			require.Equal(t, want, got)
			if wantErr == nil {
				require.NoError(t, gotErr)
			} else {
				require.EqualError(t, gotErr, wantErr.Error())
			}
		})
	}
}

func TestParallelLoftPairsUsesBatchBaseRow(t *testing.T) {
	verts := []r3.Vec{
		r3.NewVec(10, 0, 0), r3.NewVec(11, 0, 0), r3.NewVec(10, 1, 0),
		r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0), r3.NewVec(0, 2, 0),
		r3.NewVec(0.25, 0.25, 0), r3.NewVec(1, 0.25, 0), r3.NewVec(0.25, 1, 0),
	}
	tris := [][3]int{{0, 1, 2}, {3, 4, 5}, {6, 7, 8}}
	boxes := make([][2]r3.Vec, len(tris))
	for i, tri := range tris {
		boxes[i] = meshbool.TriBox(verts, tri)
	}
	data := NewLoftAuditData(verts, tris)
	var work LoftAuditWork
	i, j, err := parallelLoftPairs(proofbound.NewWorkBudget(t.Context()), tris, boxes, data,
		loftGenericShortcuts, loftCandidates{baseRow: 1, starts: []int{0, 1}, js: []int32{2}}, &work)
	require.Error(t, err)
	require.Equal(t, 1, i)
	require.Equal(t, 2, j)
	require.Equal(t, 1, work.Classifications)
}

func TestLoftAuditStreamsLargeUnstructuredCandidateSet(t *testing.T) {
	const triangles = 1_500
	verts := make([]r3.Vec, 0, 3*triangles)
	tris := make([][3]int, 0, triangles)
	for range triangles {
		start := len(verts)
		verts = append(verts, r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		tris = append(tris, [3]int{start, start + 1, start + 2})
	}
	serial, serialErr := loftCrossingAuditLimited(proofbound.NewWorkBudget(t.Context()), verts,
		tris, nil, loftGenericShortcuts, 4_000_000, triangles, false)
	streamed, streamedErr := loftCrossingAuditLimited(proofbound.NewWorkBudget(t.Context()), verts,
		tris, nil, loftGenericShortcuts, 4_000_000, triangles, true)
	require.Greater(t, streamed.Candidates, loftAuditCandidateBatchSize)
	require.Equal(t, serial, streamed)
	require.Error(t, serialErr)
	require.EqualError(t, streamedErr, serialErr.Error())
}
