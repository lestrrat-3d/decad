package pointquery

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// obliqueDirections are deterministic exact rays for a point whose six axis
// rays all graze facet edges. Exhausting this ladder leaves the query undecided.
var obliqueDirections = [][3]int64{
	{1, 2, 3}, {2, 3, 5}, {3, 5, 7}, {5, 7, 11},
	{-1, 3, 5}, {2, -5, 7}, {3, 7, -11}, {-5, -7, 13},
}

// obliqueTriangle holds one triangle's exact data relative to the query
// point p, none of which depends on the ray direction: the normal
// n = (b−a)×(c−a), the sign of n·(a−p), and corners[i] = (v_i−p)×(v_{i+1}−p).
type obliqueTriangle struct {
	ready   bool
	normal  proof.DyV3
	offset  int
	corners [3]proof.DyV3
}

// ObliqueParity counts transversal intersections in the interior of held
// facets. A ray meeting an edge or running in a facet plane is discarded.
// The caller separately checks whether p lies on the mesh boundary.
func ObliqueParity(ctx context.Context, p r3.Vec, verts []r3.Vec, tris [][3]int) (bool, bool, error) {
	return obliqueParity(proofbound.NewWorkBudget(ctx), p, verts, tris)
}

// obliqueParity decides each ray p + t·d, t > 0, against each triangle in
// exact dyadic arithmetic. With n·d ≠ 0 the ray meets the facet plane at
// t = n·(a−p) / n·d, so it crosses forward exactly when n·(a−p) and n·d share
// a strict sign. The meeting point x lies on the side of edge v_i v_{i+1}
// given by the sign of ((v_{i+1}−v_i)×(x−v_i))·n. That cross product is
// α·n for some α, and det(v_i−p, v_{i+1}−p, d) = det(v_i−x, v_{i+1}−x, d),
// since v−p = (v−x) + t·d, which equals α·(n·d). The side sign is therefore
// sign(corners[i]·d)·sign(n·d), with no division. Every vertex and every
// (direction, triangle) pair spends one budget step, in the same order the
// scan visits them; a non-finite vertex or point panics when a triangle
// first uses it.
func obliqueParity(budget *proofbound.WorkBudget, p r3.Vec, verts []r3.Vec, tris [][3]int) (bool, bool, error) {
	for range verts {
		if err := budget.Step(); err != nil {
			return false, false, err
		}
	}
	held := make([]obliqueTriangle, len(tris))
	for _, values := range obliqueDirections {
		dir := proof.DyV3{proof.DyInt(values[0]), proof.DyInt(values[1]), proof.DyInt(values[2])}
		crossings := 0
		ambiguous := false
		for k, tri := range tris {
			if err := budget.Step(); err != nil {
				return false, false, err
			}
			facet := &held[k]
			if !facet.ready {
				*facet = prepareObliqueTriangle(p, verts[tri[0]], verts[tri[1]], verts[tri[2]])
			}
			denom := proof.DvDot(facet.normal, dir).Sign()
			if denom == 0 {
				if facet.offset == 0 {
					ambiguous = true
					break
				}
				continue
			}
			if facet.offset*denom <= 0 {
				continue
			}
			inside := true
			grazes := false
			for i := range facet.corners {
				sign := proof.DvDot(facet.corners[i], dir).Sign() * denom
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

// prepareObliqueTriangle lifts the triangle abc and the point p exactly and
// forms their direction-independent data. Every value must be finite.
func prepareObliqueTriangle(p, a, b, c r3.Vec) obliqueTriangle {
	point := proof.DyVec(p)
	rel := [3]proof.DyV3{
		proof.DvSub(proof.DyVec(a), point),
		proof.DvSub(proof.DyVec(b), point),
		proof.DvSub(proof.DyVec(c), point),
	}
	normal := proof.DvCross(proof.DvSub(rel[1], rel[0]), proof.DvSub(rel[2], rel[0]))
	return obliqueTriangle{
		ready:  true,
		normal: normal,
		offset: proof.DvDot(normal, rel[0]).Sign(),
		corners: [3]proof.DyV3{
			proof.DvCross(rel[0], rel[1]),
			proof.DvCross(rel[1], rel[2]),
			proof.DvCross(rel[2], rel[0]),
		},
	}
}
