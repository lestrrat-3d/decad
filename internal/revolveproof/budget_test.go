package revolveproof_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/stretchr/testify/require"
)

type facetBudgetLoop struct{}

func (facetBudgetLoop) Len() int { return 1 }
func (facetBudgetLoop) Samples(int) []revolvemesh.RevMeridian {
	return []revolvemesh.RevMeridian{{Walk: 0}, {Walk: 1}}
}
func (facetBudgetLoop) AxisWalk(int, int) bool { return false }

func TestPreflightFacetLimitDetails(t *testing.T) {
	for _, tc := range []struct {
		name     string
		steps    int
		audit    bool
		work     revolveproof.FacetWork
		resource string
		observed uint64
		maximum  uint64
	}{
		{"mesh", 16_385, false, revolveproof.FacetWork{}, "mesh_facets", 65_540, 65_536},
		{"allocation", 2, false, revolveproof.FacetWork{Facets: 262_140}, "facet_allocations", 262_148, 262_144},
		{"pairs", 1001, true, revolveproof.FacetWork{}, "facet_pairs", 8_014_006, 8_000_000},
	} {
		t.Run(tc.name, func(t *testing.T) {
			before := tc.work
			err := revolveproof.PreflightFacets(facetBudgetLoop{}, tc.steps, true, false, tc.audit, &tc.work)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			var detail *proofbound.WorkLimitError
			require.ErrorAs(t, err, &detail)
			require.Equal(t, tc.resource, detail.Resource)
			require.Equal(t, tc.observed, detail.Observed)
			require.Equal(t, tc.maximum, detail.Maximum)
			require.Equal(t, before.Facets, tc.work.Facets)
			require.Equal(t, before.Pairs, tc.work.Pairs)
		})
	}
	work := revolveproof.FacetWork{}
	require.NoError(t, revolveproof.PreflightFacets(facetBudgetLoop{}, 16_384, true, false, false, &work))
	require.Equal(t, uint64(65_536), work.Facets)
	work = revolveproof.FacetWork{Facets: ^uint64(0)}
	err := revolveproof.PreflightFacets(facetBudgetLoop{}, 2, true, false, false, &work)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	var detail *proofbound.WorkLimitError
	require.NotErrorAs(t, err, &detail)
}
