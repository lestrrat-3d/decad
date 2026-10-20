package shellsurvey

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/units"
)

// RequireSectionCavity is S10's section limit (agent-docs/modify-design.md §8): P ⊖ t
// is non-empty exactly when t is strictly less than the section's inradius.
// enough reports that a contained disk already proved the thickness fits.
// The accept boundary sits tol*max(1, inradius) below the limit — a
// SCALE-RELATIVE rounding tolerance that grows with the part's scale, not a
// fixed sub-nanometre margin — so the refusal reports the computed accepted
// maximum thickness rather than the bare inradius (at a large scale the two
// differ by far more than a noise floor).
func RequireSectionCavity(t units.Value, tmm, inradius float64, enough bool, tol float64) error {
	maxT := inradius - tol*math.Max(1, inradius)
	if enough || tmm < maxT {
		return nil
	}
	if maxT <= 0 {
		// The inradius itself is at or below the rounding tolerance, so the
		// accepted maximum is non-positive — no positive thickness leaves a
		// cavity. A smaller thickness cannot help; the section must be enlarged.
		return fmt.Errorf(`%w: the section's inradius %s is at or below the evaluator's rounding tolerance, so this evaluator accepts no positive shell thickness here (the tolerance-adjusted maximum is non-positive); enlarge the section`, decaderr.ErrDegenerate, units.Millimeters(inradius))
	}
	return fmt.Errorf(`%w: the shell thickness %s meets or exceeds the accepted maximum %s (the section's inradius %s less the evaluator's rounding tolerance); use a thickness strictly below the accepted maximum`, decaderr.ErrDegenerate, t, units.Millimeters(maxT), units.Millimeters(inradius))
}
