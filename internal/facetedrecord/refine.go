package facetedrecord

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/occupancy"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// RefinedSurfaceBounds certifies a surface bound per held facet no larger
// than target (agent-docs/body-refine-design.md §5) from the payload's
// occupancy record, within limit units of exact work. A facet whose crossing
// bound already sits within a quarter of target keeps it; every other facet is
// certified on the ladder from the record's own floor, and keeps the smaller
// of the two. A payload without a record, or a bound past target, is
// decaderr.decaderr.ErrUnsupported.
func (fp Payload) RefinedSurfaceBounds(ctx context.Context, target float64, limit int64) ([]float64, error) {
	rec := fp.OccupancyRecord()
	if rec == nil {
		return nil, fmt.Errorf(`%w: Refine takes mesh Unions of extruded, revolved, swept or coiled circles and extruded polygons, then Cuts by extruded circles or polygons, and this body's history is not one`, decaderr.ErrUnsupported)
	}
	cross := make([]float64, len(fp.Tris))
	for t, tri := range fp.Tris {
		cross[t] = max(fp.VertexBound[tri[0]], fp.VertexBound[tri[1]], fp.VertexBound[tri[2]])
	}
	bounds := rec.MeshBounds(rec.BulgeParts(fp.Verts, fp.Tris, cross))
	floor := max(bounds.Excess, bounds.TrueToHeld)
	if !(floor < target) {
		return nil, fmt.Errorf(`%w: the true-to-held bound %s already reaches the requested %s`,
			decaderr.ErrUnsupported, units.Millimeters(floor), units.Millimeters(target))
	}
	sigma := make([]float64, len(fp.Tris))
	var want []int
	for t := range fp.Tris {
		sigma[t] = cross[t]
		if cross[t] > target/4 {
			want = append(want, t)
		}
	}
	queries := make([][2]r3.Vec, len(want))
	for i, t := range want {
		tri := fp.Tris[t]
		lo, hi := fp.Verts[tri[0]], fp.Verts[tri[0]]
		for _, v := range []r3.Vec{fp.Verts[tri[1]], fp.Verts[tri[2]]} {
			lo = r3.Vec{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
			hi = r3.Vec{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
		}
		queries[i] = [2]r3.Vec{lo, hi}
	}
	// Only pieces within the ladder's reach of a certified facet can meet one
	// of its translations; the margin covers the float boxes' rounding.
	pieces, err := rec.PiecesNear(queries, target*(1+1e-9)+1e-9)
	if err != nil {
		return nil, fmt.Errorf(`%w: %w`, decaderr.ErrUnsupported, err)
	}
	inner, err := rec.Inner()
	if err != nil {
		return nil, fmt.Errorf(`%w: %w`, decaderr.ErrUnsupported, err)
	}
	h, err := occupancy.Certify(ctx, fp.Verts, fp.Tris, want, pieces,
		occupancy.Search{H0: max(floor, target/64), HMax: target, Step: 1.25, Inner: inner, MaxDepth: 24},
		&occupancy.Budget{Limit: limit})
	if err != nil {
		return nil, err
	}
	for i, t := range want {
		sigma[t] = min(sigma[t], proofbound.UpRound(max(h[i], floor)))
	}
	worst := 0.0
	for _, s := range sigma {
		worst = max(worst, s)
	}
	if worst > target {
		return nil, fmt.Errorf(`%w: the held mesh's surface bound %s exceeds the requested %s`,
			decaderr.ErrUnsupported, units.Millimeters(worst), units.Millimeters(target))
	}
	return sigma, nil
}
