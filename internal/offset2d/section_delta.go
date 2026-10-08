package offset2d

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// SectionDeltaFromWalks bounds the gap between a recorded section offset and
// the exact offset denoted by a thickness within amount. Its closed-form
// carrier and join enclosures charge every held endpoint and miter.
//
// The denoted offset is §7's closed forms over the receiver's own walks taken
// exactly: a line walk is the segment between its recorded endpoints, a
// circular walk the circle about its recorded centre whose radius its walk
// brackets, each endpoint widened by the bound its walk states, and the
// corner rule (miter, arc, G1) is the construction's own (SectionJoinsBudget).
// The proof is an enclosure, the method capband.ContourDisplacement
// (capblend_contour.go) states: the same closed forms are re-evaluated over
// rational intervals with outward-rounded square roots, over the whole
// thickness interval at once, and each recorded join point is charged its
// enclosure's greatest reach from the held float. A G1 join is enclosed by the
// hull of its two shared-normal feet, so a join the dead zone classified G1
// with a residual turn is charged that spread too.
//
// The figure is three times the largest reach. A recorded line segment and
// the denoted one are within the larger of their two endpoint reaches at
// every matching parameter. A recorded arc shares its centre with the denoted
// one, and its radius is within one reach of the denoted radius; a point the
// recorded arc sweeps past the denoted end lies on a chord no longer than two
// reaches from the recorded end, so it is within three reaches of the denoted
// end. Every recorded boundary point is therefore within the figure of the
// denoted boundary, and every denoted point within it of the record, which is
// the reading the shell payload's sectionDelta states for a whole section.
func SectionDeltaFromWalks(budget *proofbound.WorkBudget, loops [][]survey2d.SideWalk,
	s, t float64, amount proofbound.RatInterval, tol float64,
) (float64, error) {
	reach := 0.0
	for _, walks := range loops {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, err
		}
		r, err := LoopReach(budget, walks, s, t, amount, tol)
		if err != nil {
			return 0, err
		}
		reach = math.Max(reach, r)
	}
	delta := proofbound.ProductUpper(3, reach)
	if proofbound.IsNonFinite(delta) {
		return 0, ErrUnbounded
	}
	return delta, nil
}
