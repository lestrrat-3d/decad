package verifyrun

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshthick"
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
