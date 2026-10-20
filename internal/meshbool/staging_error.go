package meshbool

import (
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/units"
)

// StagingError restates the local chain-depth gate's refusal
// (CoarseHeldContactError; agent-docs/faceted-vertex-bounds-design.md §5)
// in the boolean's own terms (docs/api-design.md §8, "The chain depth"): it
// names the operand, quotes the bound of the held facets the pair touches and
// the pair's chord tolerance, and says that a boolean takes no tolerance.
// Every other error passes through unchanged.
func StagingError(op OperationKind, err error) error {
	var held *CoarseHeldContactError
	if !errors.As(err, &held) {
		return err
	}
	return ExpectedBooleanForOperand(BooleanExpectedStaging, held.Operand,
		fmt.Errorf(`%w: %s's %s holds its mesh, where this pair meets it, only within a bound of %s, coarser than this pair's chord tolerance %s; a boolean takes no tolerance, so the chain cannot continue through that geometry: place the new contact away from the earlier booleans' rims, reshape the construction with fewer booleans over that region, or draw the pair so the analytic prism path admits it (docs/api-design.md §8, "The chain depth")`,
			decaderr.ErrUnsupported, op, operandRole(op, held.Operand), units.Millimeters(held.Bound), units.Millimeters(held.Tol)))
}

// operandRole names an operand the way the public signatures do: Cut's
// target and tool, otherwise first and second operand.
func operandRole(op OperationKind, operand int) string {
	switch {
	case op == OpCut && operand == 0:
		return "target"
	case op == OpCut:
		return "tool"
	case operand == 0:
		return "first operand"
	default:
		return "second operand"
	}
}
