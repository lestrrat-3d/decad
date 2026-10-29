package mirrorjoin

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// planeDeltaEps is the unit roundoff the arithmetic allowance counts in.
const planeDeltaEps = 0x1p-52

// PlaneDeltaAdmitted reports whether a mirror may reflect across a retained
// plane tag that states AxialDelta delta (topo.Face.AxialDelta), the proven
// distance of the true plane from the tag along its normal. A reflection across
// the tag puts the image up to 2·delta off the reflection across the true
// plane, and the placement carries no channel to charge that. The reflection
// is admitted only while 2·delta stays inside the arithmetic allowance the
// placement already leaves unstated, 64·ε·scale with scale the larger of 1 and
// the origin's coordinates; a larger or unusable delta declines. A zero delta
// is exact. The test only declines: a small delta proves nothing beyond that
// allowance.
func PlaneDeltaAdmitted(delta float64, origin r3.Vec) bool {
	if delta == 0 {
		return true
	}
	if !(delta > 0) || math.IsInf(delta, 0) {
		return false
	}
	scale := max(1, math.Abs(origin.X), math.Abs(origin.Y), math.Abs(origin.Z))
	return 2*delta <= 64*planeDeltaEps*scale
}
