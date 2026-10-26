package meshbool

import (
	"context"
	"runtime"

	"github.com/lestrrat-3d/decad/internal/orderedwork"
)

// cutFacetResult stores subdivision failures with their facet, so the caller
// can still return an earlier region-classification failure first.
type cutFacetResult struct {
	regions []CutRegion
	err     error
}

// subdivideCutFacets reads immutable operand data. Every subdivision owns its
// cutter and work budget. Classification, point-key caching and emission stay
// on the caller, in facet order. The caller bounds the batch's retained output.
// Projection storage follows cutProjectionScratch's ownership contract.
func subdivideCutFacets(ctx context.Context, m *BoolMesh, cuts map[int][]Xseg, facets []int) ([]cutFacetResult, error) {
	workers := 1
	if len(facets) >= 32 {
		workers = orderedwork.Workers(ctx, min(runtime.GOMAXPROCS(0), 4))
	}
	return orderedwork.RunWithWorker(ctx, ctx, facets, workers, func() func(context.Context, int) (cutFacetResult, error) {
		var projections cutProjectionScratch
		return func(ctx context.Context, i int) (cutFacetResult, error) {
			regions, err := cutTriangle(ctx, XtriCorners(m, i), m.Norms[i], cuts[i], &projections)
			return cutFacetResult{regions: regions, err: err}, nil
		}
	})
}
