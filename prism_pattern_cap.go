package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Every omitted root chord must stay inside the exact cap polygon, touching
// its outline only at the two shared root endpoints. The source root arcs
// lie strictly inside the blank circle; the source mesh bound later charges
// the small difference between each arc and these held chords.
func prismPatternRootChordsInsideCap(ctx context.Context, mesh *joinedMesh,
	outline []int, parts []prismPatternPart, cap int) bool {
	if cap < 0 || cap > 1 || len(outline) < 3 {
		return false
	}
	points := make([]loftCylinderXY, len(outline))
	poly := make([]meshbool.Xp2, len(outline))
	for i, index := range outline {
		if index < 0 || index >= len(mesh.verts) {
			return false
		}
		points[i] = loftCylinderExactXY(mesh.verts[index])
		poly[i] = meshbool.NewXP2(points[i].x, points[i].y)
	}
	same := func(a, b loftCylinderXY) bool {
		return a.x.Cmp(b.x) == 0 && a.y.Cmp(b.y) == 0
	}
	budget := proofbound.NewWorkBudget(ctx)
	for _, part := range parts {
		n := len(part.mesh.vertices) / 2
		rootA := loftCylinderExactXY(part.mesh.vertices[2*part.rootStart+cap])
		rootB := loftCylinderExactXY(part.mesh.vertices[cap])
		for j := part.rootStart; j < n; j++ {
			if ctx.Err() != nil {
				return false
			}
			a := loftCylinderExactXY(part.mesh.vertices[2*j+cap])
			b := loftCylinderExactXY(part.mesh.vertices[2*((j+1)%n)+cap])
			if same(a, b) {
				return false
			}
			mid := meshbool.NewXP2(
				new(big.Rat).Quo(new(big.Rat).Add(a.x, b.x), big.NewRat(2, 1)),
				new(big.Rat).Quo(new(big.Rat).Add(a.y, b.y), big.NewRat(2, 1)))
			inside, on, err := meshbool.PointInPoly2(budget, poly, mid)
			if err != nil || on || !inside {
				return false
			}
			for k, c := range points {
				d := points[(k+1)%len(points)]
				if !loftCylinderEdgesMeet(a, b, c, d) {
					continue
				}
				shared := (same(a, rootA) || same(a, rootB)) && (same(a, c) || same(a, d)) ||
					(same(b, rootA) || same(b, rootB)) && (same(b, c) || same(b, d))
				if !shared || loftCylinderOrient(a, b, c).Sign() == 0 &&
					loftCylinderOrient(a, b, d).Sign() == 0 {
					return false
				}
			}
		}
	}
	return true
}
