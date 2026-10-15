package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Every omitted root chord must stay inside the exact cap polygon, touching
// its outline only at the two shared root endpoints. The source root arcs
// lie strictly inside the blank circle; the source mesh bound later charges
// the small difference between each arc and these held chords.
func prismPatternRootChordsInsideCap(ctx context.Context, mesh *joinedMesh,
	outline []int, parts []prismPatternPart, capIndex int) bool {
	if capIndex < 0 || capIndex > 1 || len(outline) < 3 {
		return false
	}
	points := make([]loftcylinder.XY, len(outline))
	poly := make([]meshbool.Xp2, len(outline))
	for i, index := range outline {
		if index < 0 || index >= len(mesh.Verts) {
			return false
		}
		points[i] = loftcylinder.ExactXY(mesh.Verts[index])
		poly[i] = meshbool.NewXP2(points[i].X, points[i].Y)
	}
	same := func(a, b loftcylinder.XY) bool {
		return a.X.Cmp(b.X) == 0 && a.Y.Cmp(b.Y) == 0
	}
	budget := proofbound.NewWorkBudget(ctx)
	for _, part := range parts {
		n := len(part.Mesh.Vertices) / 2
		rootA := loftcylinder.ExactXY(part.Mesh.Vertices[2*part.RootStart+capIndex])
		rootB := loftcylinder.ExactXY(part.Mesh.Vertices[capIndex])
		for j := part.RootStart; j < n; j++ {
			if ctx.Err() != nil {
				return false
			}
			a := loftcylinder.ExactXY(part.Mesh.Vertices[2*j+capIndex])
			b := loftcylinder.ExactXY(part.Mesh.Vertices[2*((j+1)%n)+capIndex])
			if same(a, b) {
				return false
			}
			mid := meshbool.NewXP2(
				new(big.Rat).Quo(new(big.Rat).Add(a.X, b.X), big.NewRat(2, 1)),
				new(big.Rat).Quo(new(big.Rat).Add(a.Y, b.Y), big.NewRat(2, 1)))
			inside, on, err := meshbool.PointInPoly2(budget, poly, mid)
			if err != nil || on || !inside {
				return false
			}
			for k, c := range points {
				d := points[(k+1)%len(points)]
				if !loftcylinder.EdgesMeet(a, b, c, d) {
					continue
				}
				shared := (same(a, rootA) || same(a, rootB)) && (same(a, c) || same(a, d)) ||
					(same(b, rootA) || same(b, rootB)) && (same(b, c) || same(b, d))
				if !shared || loftcylinder.Orient(a, b, c).Sign() == 0 &&
					loftcylinder.Orient(a, b, d).Sign() == 0 {
					return false
				}
			}
		}
	}
	return true
}
