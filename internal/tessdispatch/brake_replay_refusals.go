package tessdispatch

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// These immutable refusals exist before the replay budget starts.
var (
	errBrakeReplayBoundedTessellationRequiresADocumentBodyAndBudget = fmt.Errorf("%w: bounded tessellation requires a document body and budget", decaderr.ErrBooleanFailed)
	errBrakeReplayBoundedTessellationRequiresAFacetedSolid          = fmt.Errorf("%w: bounded tessellation requires a faceted solid", decaderr.ErrUnsupported)
	errBrakeReplayBoundedTessellationRequiresAPrismSolid            = fmt.Errorf("%w: bounded tessellation requires a prism solid", decaderr.ErrUnsupported)
	errBrakeReplayInvalidBoundedTessellationChord                   = fmt.Errorf("%w: invalid bounded tessellation chord", decaderr.ErrUnsupported)
)
