package capband

import (
	"slices"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// PartialFilletContourDelta omits exact sphere walks from the offset proof.
// Their straight neighbors meet at the recorded pole, and unselected walks
// retain zero offset.
func PartialFilletContourDelta(walks []survey2d.SideWalk, joins []capcontour.Join,
	amounts []float64, sphere []bool, radiusDelta, tol float64,
) (float64, error) {
	if !slices.Contains(sphere, true) {
		return AmountsContourDisplacement(walks, joins, amounts, radiusDelta, tol)
	}
	n := len(walks)
	keptWalks := make([]survey2d.SideWalk, 0, n)
	keptJoins := make([]capcontour.Join, 0, n)
	keptAmounts := make([]float64, 0, n)
	for i, walk := range walks {
		if sphere[i] {
			continue
		}
		join := joins[i]
		prev := (i + n - 1) % n
		if sphere[prev] {
			pole := Point{U: walks[prev].CU, V: walks[prev].CV}
			join = capcontour.Join{M: pole, VU: pole.U, VV: pole.V}
		}
		keptWalks = append(keptWalks, walk)
		keptJoins = append(keptJoins, join)
		keptAmounts = append(keptAmounts, amounts[i])
	}
	if len(keptWalks) < 2 {
		return 0, offset2d.ErrDrop
	}
	return AmountsContourDisplacement(keptWalks, keptJoins, keptAmounts, radiusDelta, tol)
}
