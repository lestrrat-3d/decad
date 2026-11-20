package rimextension

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type facetBox struct{ lo, hi r3.Vec }

func boxFor(v []r3.Vec, t [3]int) facetBox {
	a, b, c := v[t[0]], v[t[1]], v[t[2]]
	return facetBox{r3.NewVec(min(a.X, b.X, c.X), min(a.Y, b.Y, c.Y), min(a.Z, b.Z, c.Z)), r3.NewVec(max(a.X, b.X, c.X), max(a.Y, b.Y, c.Y), max(a.Z, b.Z, c.Z))}
}
func (a facetBox) touches(b facetBox) bool {
	return a.lo.X <= b.hi.X && b.lo.X <= a.hi.X && a.lo.Y <= b.hi.Y && b.lo.Y <= a.hi.Y && a.lo.Z <= b.hi.Z && b.lo.Z <= a.hi.Z
}

// ChangedNeighbors keeps every facet that might touch any changed triangle.
// Disjoint closed coordinate intervals certify that omitted pairs do not meet.
func ChangedNeighbors(ctx context.Context, v []r3.Vec, candidates [][3]int, old map[[3]int]bool, m *proofbound.ReplayBudget) ([][3]int, error) {
	var boxes []facetBox
	for _, t := range candidates {
		if !old[t] {
			boxes = append(boxes, boxFor(v, t))
		}
	}
	if e := m.ReserveBundle(ctx, uint64(len(candidates))*(1+uint64(len(boxes))), uint64(len(candidates)), uint64(len(candidates))*96); e != nil {
		return nil, e
	}
	local := make([][3]int, 0, len(candidates))
	for i, t := range candidates {
		if i%64 == 0 && ctx.Err() != nil {
			return nil, ctx.Err()
		}
		b := boxFor(v, t)
		if slices.ContainsFunc(boxes, b.touches) {
			local = append(local, t)
		}
	}
	return local, nil
}
