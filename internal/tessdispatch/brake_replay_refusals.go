package tessdispatch

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// These immutable refusals exist before the replay budget starts.
var (
	errBrakeReplayBoundedCircleRequiresABrakeRouteBodyAndBudget     = fmt.Errorf("%w: bounded circle requires a brake route body and budget", decaderr.ErrUnsupported)
	errBrakeReplayBoundedCircleRequiresAPrismSolid                  = fmt.Errorf("%w: bounded circle requires a prism solid", decaderr.ErrUnsupported)
	errBrakeReplayBoundedTessellationRequiresADocumentBodyAndBudget = fmt.Errorf("%w: bounded tessellation requires a document body and budget", decaderr.ErrBooleanFailed)
	errBrakeReplayBoundedTessellationRequiresAFacetedSolid          = fmt.Errorf("%w: bounded tessellation requires a faceted solid", decaderr.ErrUnsupported)
	errBrakeReplayBoundedTessellationRequiresAPrismSolid            = fmt.Errorf("%w: bounded tessellation requires a prism solid", decaderr.ErrUnsupported)
	errBrakeReplayInvalidBoundedCircleChord                         = fmt.Errorf("%w: invalid bounded circle chord", decaderr.ErrUnsupported)
	errBrakeReplayInvalidBoundedTessellationChord                   = fmt.Errorf("%w: invalid bounded tessellation chord", decaderr.ErrUnsupported)
)
