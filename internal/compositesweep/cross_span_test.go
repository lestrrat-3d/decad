package compositesweep_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestCrossSpanContactAudit(t *testing.T) {
	triangles := [][3]int{{0, 1, 2}, {0, 2, 3}}
	t.Run("inconsistent records", func(t *testing.T) {
		err := compositesweep.CrossSpanContactAudit(proofbound.NewWorkBudget(t.Context()),
			make([]revolvemesh.RevolveAuditTri, 1), triangles, []int{0, 0}, 0)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "the composite sweep's facet audit has inconsistent triangle records")
	})
	t.Run("one span audits no pair", func(t *testing.T) {
		err := compositesweep.CrossSpanContactAudit(proofbound.NewWorkBudget(t.Context()),
			make([]revolvemesh.RevolveAuditTri, 2), triangles, []int{0, 0}, 0)
		require.NoError(t, err)
	})
}

// overlapSoup is count slanted unit triangles shifted along X by 1e-4 each, so
// every pair's boxes overlap and the candidate sweep lists every pair. spans
// assigns each triangle its span.
func overlapSoup(t *testing.T, count int, spans func(int) int) spanMesh {
	t.Helper()
	verts := make([]r3.Vec, 0, 3*count)
	tris := make([][3]int, count)
	spanOf := make([]int, count)
	for i := range tris {
		o := r3.NewVec(1e-4*float64(i), 0, 0)
		verts = append(verts, o, o.Add(r3.NewVec(1, 0, 1)), o.Add(r3.NewVec(0, 1, 1)))
		tris[i] = [3]int{3 * i, 3*i + 1, 3*i + 2}
		spanOf[i] = spans(i)
	}
	return newSpanMesh(t, verts, tris, spanOf)
}

// TestCrossSpanContactAuditCountsCandidates requires the fixed facet-pair
// ceiling to count the candidate sweep's visits and the cross-span pairs it
// keeps, not the triangle count's total pair count.
func TestCrossSpanContactAuditCountsCandidates(t *testing.T) {
	ceiling := uint64(proofbound.MaxFacetPairTestsPerCall)
	t.Run("many facets, few candidates", func(t *testing.T) {
		// 601 thin cells: 4812 triangles, past 8,000,000 total pairs, but
		// each cell's boxes reach only its neighbours. The verdict must
		// match the all-pairs walk with no pair-count gate, for a clean mesh
		// and for one with a station corner pushed through the far wall.
		const cells = 601
		m := twistedPrismMesh(t, 7.7, 2.375, 87*math.Pi/180, 6, cells)
		pairs, ok := proofbound.WallChoose2(uint64(len(m.tris)))
		require.True(t, ok)
		require.Greater(t, pairs, ceiling)
		require.NoError(t, ungatedCrossSpanContactAudit(t, m, 1e-9))
		require.NoError(t, compositesweep.CrossSpanContactAudit(proofbound.NewWorkBudget(t.Context()),
			m.data, m.tris, m.spanOf, 1e-9))

		dented := twistedPrismMesh(t, 7.7, 2.375, 87*math.Pi/180, 6, cells)
		corner := (cells / 2) * 4
		dented.verts[corner] = dented.verts[corner].Scale(-0.6)
		dented = newSpanMesh(t, dented.verts, dented.tris, dented.spanOf)
		want := ungatedCrossSpanContactAudit(t, dented, 1e-9)
		require.Error(t, want)
		got := compositesweep.CrossSpanContactAudit(proofbound.NewWorkBudget(t.Context()),
			dented.data, dented.tris, dented.spanOf, 1e-9)
		require.Error(t, got)
		require.Equal(t, want.Error(), got.Error())
	})
	t.Run("same-span candidates cost no pair test", func(t *testing.T) {
		// One span's 3300 mutually overlapping triangles: the sweep visits
		// about 5.4 million pairs and keeps none, so the audit fits the
		// ceiling even though listing every candidate pair would not.
		m := overlapSoup(t, 3300, func(int) int { return 0 })
		require.NoError(t, compositesweep.CrossSpanContactAudit(proofbound.NewWorkBudget(t.Context()),
			m.data, m.tris, m.spanOf, 1e-9))
	})
	for _, tc := range []struct {
		name  string
		count int
		exact bool
	}{
		// Alternating spans keep half the 5.4 million visited pairs: the
		// visits fit, the visits plus kept pairs do not.
		{name: "kept pairs pass the ceiling", count: 3300, exact: true},
		// 4100 triangles need 8.4 million visits: the sweep itself stops.
		{name: "sweep visits pass the ceiling", count: 4100},
	} {
		t.Run(tc.name, func(t *testing.T) {
			m := overlapSoup(t, tc.count, func(i int) int { return i % 2 })
			err := compositesweep.CrossSpanContactAudit(proofbound.NewWorkBudget(t.Context()),
				m.data, m.tris, m.spanOf, 1e-9)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, "the composite sweep's facet-pair audit exceeds its fixed work ceiling")
			var detail *proofbound.WorkLimitError
			require.ErrorAs(t, err, &detail)
			require.Equal(t, "facet_pairs", detail.Resource)
			require.Equal(t, ceiling, detail.Maximum)
			require.Greater(t, detail.Observed, ceiling)
			if tc.exact {
				// The sweep visits every pair once and each triangle's three
				// vertex incidences once; the kept pairs are the odd-even ones.
				n := uint64(tc.count)
				require.Equal(t, n*(n-1)/2+3*n+(n/2)*(n-n/2), detail.Observed)
			}
		})
	}
	t.Run("unordered margin keeps the pair-count gate", func(t *testing.T) {
		m := twistedPrismMesh(t, 7.7, 2.375, 87*math.Pi/180, 6, 601)
		err := compositesweep.CrossSpanContactAudit(proofbound.NewWorkBudget(t.Context()),
			m.data, m.tris, m.spanOf, math.NaN())
		var detail *proofbound.WorkLimitError
		require.ErrorAs(t, err, &detail)
		require.Equal(t, uint64(len(m.tris))*uint64(len(m.tris)-1)/2, detail.Observed)
	})
}
