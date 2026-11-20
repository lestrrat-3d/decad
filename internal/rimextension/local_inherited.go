package rimextension

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// localInheritedBound consumes the same closed chart and two-sided facet
// correspondence as source admission. Any old true point in the edit has
// a held counterpart within its own facet radius, so that inflated facet
// reaches the chart. In the other direction every held edit lies in the
// chart. Subdivision can also split a neighbor outside the chart; include
// the complete original facet whenever its triangle tuple disappeared.
// Apply keeps the old vertices at their original coordinates and indices.
func localInheritedBound(ctx context.Context, old facetedrecord.Payload, current map[[3]int]bool, support SourceWindow,
	wing int, meter *proofbound.ReplayBudget) (float64, bool, error) {
	if meter == nil || len(current) == 0 || len(old.Tris) == 0 || len(old.Verts) != len(old.VertexBound) || wing < -1 || wing > 1 {
		return 0, false, nil
	}
	if !windowFinite(support.Side0) || !windowFinite(support.Side1) || support.Side0.X == support.Side1.X ||
		!windowFinite(r3.NewVec(support.XMin, support.YMin, support.OffsetMin)) ||
		!windowFinite(r3.NewVec(support.XMax, support.YMax, support.OffsetMax)) ||
		support.XMin >= support.XMax || support.YMin >= support.YMax || support.OffsetMin >= support.OffsetMax {
		return 0, false, nil
	}
	if err := meter.ReserveBundle(ctx, uint64(len(old.Tris))*20, 1, 1024); err != nil {
		return 0, false, err
	}
	var bound float64
	localTri := [1][3]int{{0, 1, 2}}
	source := [1]int{0}
	for i, t := range old.Tris {
		if i%128 == 0 {
			if err := ctx.Err(); err != nil {
				return 0, false, err
			}
		}
		var p [3]r3.Vec
		var b [3]float64
		for k, id := range t {
			if id < 0 || id >= len(old.Verts) {
				return 0, false, nil
			}
			p[k], b[k] = old.Verts[id], old.VertexBound[id]
			if !windowFinite(p[k]) || proofbound.IsNonFinite(b[k]) || b[k] < 0 {
				return 0, false, nil
			}
			if wing != 0 {
				p[k] = ShoulderPoint(p[k], wing)
			}
		}
		r := max(b[0], b[1], b[2])
		if r <= bound {
			continue
		}
		if !current[t] {
			bound = r
			continue
		}
		if math.Nextafter(max(p[0].X, p[1].X, p[2].X)+r, math.Inf(1)) < support.XMin ||
			math.Nextafter(min(p[0].X, p[1].X, p[2].X)-r, math.Inf(-1)) > support.XMax ||
			math.Nextafter(max(p[0].Y, p[1].Y, p[2].Y)+r, math.Inf(1)) < support.YMin ||
			math.Nextafter(min(p[0].Y, p[1].Y, p[2].Y)-r, math.Inf(-1)) > support.YMax {
			continue
		}
		// With no allowed source, the admission predicate succeeds exactly
		// when it can prove this inflated triangle is strictly separated.
		separate, err := WindowSourcesCovered(ctx, p[:], localTri[:], source[:], b[:], nil, support, meter)
		if err != nil {
			return 0, false, err
		}
		if !separate {
			bound = r
		}
	}
	return bound, true, nil
}
