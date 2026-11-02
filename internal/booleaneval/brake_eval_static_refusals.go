package booleaneval

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// These immutable evaluator refusals are initialized before replay starts.
var (
	errBrakeEvalAFacetSSourceIsNotAnOperandFace                      = fmt.Errorf("%w: a facet's source is not an operand face", decaderr.ErrBooleanFailed)
	errBrakeEvalBoundedMeshPreparationNeedsAMeshAndBudget            = fmt.Errorf("%w: bounded mesh preparation needs a mesh and budget", decaderr.ErrUnsupported)
	errBrakeEvalBoundedMesherDoesNotSupportThisOperand               = fmt.Errorf("%w: bounded mesher does not support this operand", decaderr.ErrUnsupported)
	errBrakeEvalBoundedMesherNeedsAnOperandAndBudget                 = fmt.Errorf("%w: bounded mesher needs an operand and budget", decaderr.ErrUnsupported)
	errBrakeEvalBoundedOperandFaceMissing                            = fmt.Errorf("%w: bounded operand face missing", decaderr.ErrUnsupported)
	errBrakeEvalBoundedOperandIsMissing                              = fmt.Errorf("%w: bounded operand is missing", decaderr.ErrUnsupported)
	errBrakeEvalBoundedOperandRectangleWalkIsUnavailable             = fmt.Errorf("%w: bounded operand rectangle walk is unavailable", decaderr.ErrUnsupported)
	errBrakeEvalBoundedOperandSectionIsNotLinear                     = fmt.Errorf("%w: bounded operand section is not linear", decaderr.ErrUnsupported)
	errBrakeEvalBoundedOperandTagPlacementRequiresARecordedRectangle = fmt.Errorf("%w: bounded operand tag placement requires a recorded rectangle", decaderr.ErrUnsupported)
	errBrakeEvalBoundedOperandWalkIsNotLinear                        = fmt.Errorf("%w: bounded operand walk is not linear", decaderr.ErrUnsupported)
	errBrakeEvalBoundedOperandWalksChanged                           = fmt.Errorf("%w: bounded operand walks changed", decaderr.ErrUnsupported)

	errBrakeEvalNilLump                    = fmt.Errorf("%w: nil lump", decaderr.ErrUnsupported)
	errBrakeEvalNilShell                   = fmt.Errorf("%w: nil shell", decaderr.ErrUnsupported)
	errBrakeEvalNonfiniteReplayContact     = fmt.Errorf("%w: nonfinite replay contact", decaderr.ErrUnsupported)
	errBrakeEvalReplayContactCount         = fmt.Errorf("%w: replay contact count", decaderr.ErrUnsupported)
	errBrakeEvalReplayContactPieceMismatch = fmt.Errorf("%w: replay contact piece mismatch", decaderr.ErrUnsupported)
	errBrakeEvalReplayContactSize          = fmt.Errorf("%w: replay contact size", decaderr.ErrUnsupported)
)
