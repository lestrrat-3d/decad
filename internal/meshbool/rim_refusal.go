package meshbool

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// RefuseRimPastPair refuses a boolean whose largest rim bound has stopped
// bounding anything. A rim vertex is not a point of either operand's surface:
// it is the exact crossing of an operand A facet's PLANE with an operand B
// facet's, and the true intersection curve lies anywhere within δ(t_A) of the
// one and δ(t_B) of the other — a tube of half-width (δ(t_A) + δ(t_B))/sin θ
// about the crossing line (meshbool.RimBound), which grows without limit as
// the two facets approach tangency. maxRim is the largest such bound any rim
// vertex of the operation takes. Once it is not finite, or reaches the pair's
// own diameter, the operation is refused (decaderr.ErrUnsupported) rather than
// reported with a bound nobody can use — decad never understates a bound, and
// never fakes one (agent-docs/faceted-vertex-bounds-design.md §3.2).
func RefuseRimPastPair(maxRim, dPair float64) error {
	if proofbound.IsNonFinite(maxRim) {
		return fmt.Errorf(`%w: the operands' facets meet at an angle this evaluator cannot bound`, decaderr.ErrUnsupported)
	}
	if maxRim > 0 && maxRim >= dPair {
		return fmt.Errorf(`%w: the operands cross too shallowly — the rim's proven displacement bound reaches the pair's own diameter, so no measurement of the result would be trustworthy`, decaderr.ErrUnsupported)
	}
	return nil
}
