package loftmesh

import (
	"context"
	"math"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestTiledAuditMatchesGeneric(t *testing.T) {
	verts := []r3.Vec{
		{X: 0, Y: 0}, {X: 1, Y: 0}, {X: 0, Y: 1},
		{X: 1, Y: 1}, {X: 2, Y: 0}, {X: 2, Y: 1},
		{X: 0.2, Y: 0.2, Z: -1}, {X: 0.2, Y: 0.2, Z: 1}, {X: 0.8, Y: 0.2},
	}
	for _, tris := range [][][3]int{
		{{0, 1, 2}, {1, 3, 2}, {1, 4, 3}, {4, 5, 3}},
		{{0, 1, 2}, {1, 3, 2}, {1, 4, 3}, {6, 7, 8}},
		{{6, 7, 8}, {1, 4, 3}, {1, 3, 2}, {0, 1, 2}},
	} {
		want := LoftCrossingAudit(proofbound.NewWorkBudget(t.Context()), verts, tris)
		got := loftCrossingAuditTiled(proofbound.NewWorkBudget(t.Context()), verts, tris, 100, 2)
		require.Equal(t, want == nil, got == nil)
		if want != nil {
			var contact *LoftContactError
			require.ErrorAs(t, got, &contact)
		}
	}
}

func TestTiledAuditMatchesGenericAfterRotationAndReflection(t *testing.T) {
	base := []r3.Vec{{}, {X: 1}, {Y: 1}, {X: 1, Y: 1}, {X: 1, Y: 1, Z: 1}}
	tris := [][3]int{{0, 1, 2}, {1, 3, 2}, {1, 4, 3}}
	for _, sign := range []float64{1, -1} {
		verts := make([]r3.Vec, len(base))
		for i, p := range base {
			verts[i] = r3.Vec{X: sign * p.Z, Y: p.X, Z: p.Y}
		}
		want := LoftCrossingAudit(proofbound.NewWorkBudget(t.Context()), verts, tris)
		got := loftCrossingAuditTiled(proofbound.NewWorkBudget(t.Context()), verts, tris, 100, 1)
		require.Equal(t, want == nil, got == nil)
	}
}

func TestTiledCrossVisitCoversAllBoxPairs(t *testing.T) {
	rng := rand.New(rand.NewSource(42))
	for range 30 {
		boxes := make([][2]r3.Vec, 20)
		for i := range boxes {
			x, y, z := float64(rng.Intn(5)), float64(rng.Intn(5)), float64(rng.Intn(5))
			boxes[i] = [2]r3.Vec{{X: x, Y: y, Z: z}, {X: x + 1, Y: y + 1, Z: z + 1}}
		}
		var a, b []int
		for i := range boxes {
			if i < 10 {
				a = append(a, i)
			} else {
				b = append(b, i)
			}
		}
		ta, tb := newTreeScan(boxes, a), newTreeScan(boxes, b)
		got := make(map[[2]int]int)
		_, exceeded, err := tiledCrossVisit(proofbound.NewWorkBudget(t.Context()), &ta, &tb, 10000, func(i, j int) { got[[2]int{i, j}]++ })
		require.NoError(t, err)
		require.False(t, exceeded)
		for _, i := range a {
			for _, j := range b {
				want := 0
				if meshbool.BoxesOverlap(boxes[i], boxes[j]) {
					want = 1
				}
				require.Equal(t, want, got[[2]int{i, j}], "pair %d,%d", i, j)
			}
		}
	}
}

func TestTiledAuditMatchesGenericRandomTriangles(t *testing.T) {
	rng := rand.New(rand.NewSource(1))
	for range 100 {
		var verts []r3.Vec
		var tris [][3]int
		for range 8 {
			start := len(verts)
			for range 3 {
				verts = append(verts, r3.Vec{X: float64(rng.Intn(5)), Y: float64(rng.Intn(5)), Z: float64(rng.Intn(5))})
			}
			tris = append(tris, [3]int{start, start + 1, start + 2})
		}
		want := LoftCrossingAudit(proofbound.NewWorkBudget(t.Context()), verts, tris)
		got := loftCrossingAuditTiled(proofbound.NewWorkBudget(t.Context()), verts, tris, 100, 2)
		require.Equal(t, want == nil, got == nil)
	}
}

func TestTiledAuditRejectsInvalidInputAndCancellation(t *testing.T) {
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}}
	tris := [][3]int{{0, 1, 2}}
	require.Error(t, loftCrossingAuditTiled(proofbound.NewWorkBudget(t.Context()), verts, tris, 0, 2))
	require.Error(t, loftCrossingAuditTiled(proofbound.NewWorkBudget(t.Context()), verts, [][3]int{{0, 1, 3}}, 2, 2))
	require.Error(t, loftCrossingAuditTiled(proofbound.NewWorkBudget(t.Context()), []r3.Vec{{X: math.NaN()}, {X: 1}, {Y: 1}}, tris, 2, 2))
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	require.ErrorIs(t, loftCrossingAuditTiled(proofbound.NewWorkBudget(ctx), verts, tris, 2, 2), context.Canceled)
	var detail *proofbound.WorkLimitError
	err := loftCrossingAuditTiledLimits(proofbound.NewWorkBudget(t.Context()), verts, tris, 2, 1, 0, 100, 100)
	require.ErrorAs(t, err, &detail)
	require.Equal(t, "box_scans", detail.Resource)
	require.Greater(t, detail.Observed, detail.Maximum)
	tris = append(tris, tris[0])
	err = loftCrossingAuditTiledLimits(proofbound.NewWorkBudget(t.Context()), verts, tris, 2, 1, 100, 0, 100)
	require.ErrorAs(t, err, &detail)
	require.Equal(t, "facet_pairs", detail.Resource)
	require.Greater(t, detail.Observed, detail.Maximum)
	err = loftCrossingAuditTiledLimits(proofbound.NewWorkBudget(t.Context()), verts, tris, 2, 1, 100, 100, 1)
	require.ErrorAs(t, err, &detail)
	require.Equal(t, "table_facets", detail.Resource)
	require.Greater(t, detail.Observed, detail.Maximum)
}

func TestTiledAuditReportsRemoteCrossing(t *testing.T) {
	verts := []r3.Vec{
		{X: 20}, {X: 21}, {X: 20, Y: 1},
		{}, {X: 2}, {Y: 2},
		{X: 0.5, Y: 0.5, Z: -1}, {X: 0.5, Y: 0.5, Z: 1}, {X: 1, Y: 0.5},
	}
	tris := [][3]int{{0, 1, 2}, {3, 4, 5}, {0, 1, 2}, {6, 7, 8}}
	// Keep the remote pair in distinct blocks without introducing an earlier contact.
	verts = append(verts, r3.Vec{X: 30}, r3.Vec{X: 31}, r3.Vec{X: 30, Y: 1})
	tris[2] = [3]int{9, 10, 11}
	err := loftCrossingAuditTiled(proofbound.NewWorkBudget(t.Context()), verts, tris, 10, 1)
	var contact *LoftContactError
	require.ErrorAs(t, err, &contact)
	require.Equal(t, 1, contact.I)
	require.Equal(t, 3, contact.J)
}

func TestTiledAuditDoesNotWeldCoincidentVertices(t *testing.T) {
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {}, {X: 1}, {Y: 1}}
	tris := [][3]int{{0, 1, 2}, {3, 4, 5}}
	ref := LoftCrossingAudit(proofbound.NewWorkBudget(t.Context()), verts, tris)
	var want, got *LoftContactError
	require.ErrorAs(t, ref, &want)
	err := loftCrossingAuditTiled(proofbound.NewWorkBudget(t.Context()), verts, tris, 10, 1)
	require.ErrorAs(t, err, &got)
	require.Equal(t, want.Reason, got.Reason)
}

func TestTiledAuditReportsFirstPairInGlobalOrder(t *testing.T) {
	verts := []r3.Vec{
		{}, {X: 1}, {Y: 1},
		{X: 10}, {X: 11}, {X: 10, Y: 1},
		{X: 10}, {X: 11}, {X: 10, Y: 1},
		{}, {X: 1}, {Y: 1},
	}
	tris := [][3]int{{0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {9, 10, 11}}
	err := loftCrossingAuditTiled(proofbound.NewWorkBudget(t.Context()), verts, tris, 10, 2)
	var contact *LoftContactError
	require.ErrorAs(t, err, &contact)
	require.Equal(t, 0, contact.I)
	require.Equal(t, 3, contact.J)
}

func TestTiledAuditBudgetInterruptsBothPasses(t *testing.T) {
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {X: 1, Y: 1}}
	tris := [][3]int{{0, 1, 2}, {1, 3, 2}}
	for _, limit := range []int{1, 9, 14, 16} {
		steps := 0
		budget := &proofbound.WorkBudget{
			StepFn: func() error {
				steps++
				if steps >= limit {
					return context.Canceled
				}
				return nil
			},
			ErrFn: func() error { return nil },
		}
		err := loftCrossingAuditTiled(budget, verts, tris, 10, 1)
		require.ErrorIs(t, err, context.Canceled, "step %d", limit)
	}
}
