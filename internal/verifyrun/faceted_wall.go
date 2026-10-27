package verifyrun

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshthick"
	"github.com/lestrrat-3d/decad/internal/orthoshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// facetedWallSlack is the relative allowance, of the held mesh's diameter, for
// the float rounding of the ray casts.
const facetedWallSlack = 1e-9

// facetedWall reads the wall of a faceted body from rays cast through its held
// mesh (agent-docs/mesh-wall-survey-design.md). The rays find walls and cannot
// exclude them: the thinnest ray is an upper bound on the thinnest wall, since
// the true minimum is at most the minimum over the sampled points. The reading
// is therefore the interval from zero to that upper bound, widened by twice the
// mesh's displacement bound and the rounding allowance. An interval that ends
// below the tool proves a wall too thin; one that reaches the tool leaves the
// question undecided, as the survey never proves a wall thick enough.
func facetedWall(budget *proofbound.WorkBudget, pl facetedrecord.Payload) (wallOutcome, error) {
	if out, ok := orthogonalWall(budget, pl); ok {
		return out, nil
	}
	found, err := meshthick.Min(budget, pl.Verts, pl.Tris)
	if err != nil {
		return wallOutcome{}, err
	}
	if !found.Found {
		return wallOutcome{reason: surveyFacetedUnsupported}, nil
	}
	upper := found.Thickness + 2*pl.MeshBound + facetedWallSlack*math.Max(pl.Diameter, found.Thickness)
	if proofbound.IsNonFinite(upper) {
		return wallOutcome{reason: surveyFacetedUnsupported}, nil
	}
	mid := upper / 2
	return wallOutcome{reading: &mid, bound: mid, ok: true}, nil
}

// orthogonalWall reads the wall of a faceted body every face of which squares to a world axis, with no vertex bound,
// exactly (agent-docs/orthogonal-wall-design.md): the thinnest wall is the shortest run of material along an axis on
// the solid's grid, a reading with a lower end as well as an upper one, so a wall thick enough is proven as well as one
// too thin. ok is false when the body is outside that class and the rays take it.
func orthogonalWall(budget *proofbound.WorkBudget, pl facetedrecord.Payload) (wallOutcome, bool) {
	for _, beta := range pl.VertexBound {
		if beta != 0 {
			return wallOutcome{}, false
		}
	}
	if budget.Err() != nil {
		return wallOutcome{}, false
	}
	thinnest, slack, err := orthoshell.MinWall(pl.Verts, pl.Tris)
	if err != nil || proofbound.IsNonFinite(thinnest) {
		return wallOutcome{}, false
	}
	return wallOutcome{reading: &thinnest, bound: slack, ok: true}, true
}
