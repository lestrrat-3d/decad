package pointquery

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// obliqueDirections are deterministic exact rays for a point whose six axis
// rays all graze facet edges. Exhausting this ladder leaves the query undecided.
var obliqueDirections = [][3]int64{
	{1, 2, 3}, {2, 3, 5}, {3, 5, 7}, {5, 7, 11},
	{-1, 3, 5}, {2, -5, 7}, {3, 7, -11}, {-5, -7, 13},
}

func directionOf(values [3]int64) ratVec {
	return ratVec{
		new(big.Rat).SetInt64(values[0]),
		new(big.Rat).SetInt64(values[1]),
		new(big.Rat).SetInt64(values[2]),
	}
}

// ObliqueParity counts transversal intersections in the interior of held
// facets. A ray meeting an edge or running in a facet plane is discarded.
// The caller separately checks whether p lies on the mesh boundary.
func ObliqueParity(ctx context.Context, p r3.Vec, verts []r3.Vec, tris [][3]int) (bool, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	point := exactVec(p)
	converted := make([]ratVec, len(verts))
	for i, v := range verts {
		if err := budget.Step(); err != nil {
			return false, false, err
		}
		converted[i] = exactVec(v)
	}
	for _, values := range obliqueDirections {
		dir := directionOf(values)
		crossings := 0
		ambiguous := false
		for _, tri := range tris {
			if err := budget.Step(); err != nil {
				return false, false, err
			}
			a, b, c := converted[tri[0]], converted[tri[1]], converted[tri[2]]
			normal := cross(subtract(b, a), subtract(c, a))
			denom := dot(normal, dir)
			numerator := dot(normal, subtract(a, point))
			if denom.Sign() == 0 {
				if numerator.Sign() == 0 {
					ambiguous = true
					break
				}
				continue
			}
			parameter := new(big.Rat).Quo(numerator, denom)
			if parameter.Sign() <= 0 {
				continue
			}
			intersection := ratVec{}
			for i := range intersection {
				intersection[i] = new(big.Rat).Add(point[i], new(big.Rat).Mul(parameter, dir[i]))
			}
			vertices := [3]ratVec{a, b, c}
			inside := true
			grazes := false
			for i := range vertices {
				sign := dot(cross(subtract(vertices[(i+1)%3], vertices[i]),
					subtract(intersection, vertices[i])), normal).Sign()
				if sign < 0 {
					inside = false
					break
				}
				if sign == 0 {
					grazes = true
				}
			}
			if !inside {
				continue
			}
			if grazes {
				ambiguous = true
				break
			}
			crossings++
		}
		if !ambiguous {
			return crossings%2 == 1, true, nil
		}
	}
	return false, false, nil
}
