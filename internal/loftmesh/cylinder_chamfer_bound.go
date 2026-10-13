package loftmesh

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// CylinderChamferBound composes a loft/cylinder cap chamfer's source bound
// from the joined mesh bound, the bridge reach, the inset and the clip. Exact
// containment C⊂H makes the original two-way source/held bound sufficient in
// the reverse direction. The complete source-occupied split rings and cap/side
// gates prove the forward direction separately.
func CylinderChamferBound(meshBound, inset, clip, bridge float64) (float64, error) {
	bound := proofbound.AbsSumUpper(max(meshBound, bridge), inset, clip)
	if math.IsNaN(bound) || math.IsInf(bound, 0) || bound > 0.1 {
		return 0, fmt.Errorf("%w: loft/cylinder cap chamfer source bound %g mm exceeds 0.1 mm "+
			"(joined %g, bridge %g, inset %g, clip %g)", decaderr.ErrUnsupported, bound,
			meshBound, bridge, inset, clip)
	}
	return bound, nil
}
