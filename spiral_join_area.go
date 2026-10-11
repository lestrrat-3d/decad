package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// spiralJoinAreaAllowance uses the whole operand boundary areas as a
// deliberately coarse ceiling on the true Union boundary area. This lets a
// source-specific radial patch proof state a smaller boundary displacement
// without pretending its facet areas follow the ordinary affine-corner
// construction. The Union boundary is a subset of the two operand boundaries.
func spiralJoinAreaAllowance(ctx context.Context, a, b *Body,
	held *meshbool.StitchedMesh) (float64, bool, error) {
	if ctx == nil || a == nil || b == nil || held == nil {
		return 0, false, nil
	}
	heldUpper, err := proofbound.PerturbedAreaUpperContext(ctx, held.Verts, held.Tris, 0)
	if err != nil {
		return 0, false, err
	}
	if proofbound.IsNonFinite(heldUpper) {
		return 0, false, nil
	}
	trueUpper := 0.0
	for _, body := range [...]*Body{a, b} {
		area, err := body.Area()
		if err != nil {
			return 0, false, err
		}
		value, err := area.Value.In(units.SquareMillimeter)
		if err != nil {
			return 0, false, err
		}
		bound, err := area.Bound.In(units.SquareMillimeter)
		if err != nil {
			return 0, false, err
		}
		if value < 0 || bound < 0 || proofbound.IsNonFinite(value) || proofbound.IsNonFinite(bound) {
			return 0, false, nil
		}
		trueUpper = proofbound.AbsSumUpper(trueUpper, value, bound)
	}
	allowance := proofbound.AbsSumUpper(heldUpper, trueUpper)
	return allowance, !proofbound.IsNonFinite(allowance), nil
}
