package booleaneval

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// These immutable capture seal refusals exist before replay begins.
var (
	errCaptureSealCaptureAncestryChanged       = fmt.Errorf("%w: capture ancestry changed", decaderr.ErrUnsupported)
	errCaptureSealCaptureProvenanceTooLarge    = fmt.Errorf("%w: capture provenance too large", decaderr.ErrUnsupported)
	errCaptureSealCaptureSourceAncestryTooDeep = fmt.Errorf("%w: capture source ancestry too deep", decaderr.ErrUnsupported)
	errCaptureSealCapturedCapChanged           = fmt.Errorf("%w: captured cap changed", decaderr.ErrUnsupported)
	errCaptureSealCapturedDenotationChanged    = fmt.Errorf("%w: captured denotation changed", decaderr.ErrUnsupported)
	errCaptureSealCapturedDenotationTooLarge   = fmt.Errorf("%w: captured denotation too large", decaderr.ErrUnsupported)
	errCaptureSealCapturedGroupChanged         = fmt.Errorf("%w: captured group changed", decaderr.ErrUnsupported)
	errCaptureSealCapturedGroupPointerChanged  = fmt.Errorf("%w: captured group pointer changed", decaderr.ErrUnsupported)
	errCaptureSealCapturedGroupsChanged        = fmt.Errorf("%w: captured groups changed", decaderr.ErrUnsupported)
	errCaptureSealCapturedProvenanceChanged    = fmt.Errorf("%w: captured provenance changed", decaderr.ErrUnsupported)
	errCaptureSealCapturedSourceChanged        = fmt.Errorf("%w: captured source changed", decaderr.ErrUnsupported)
	errCaptureSealMissingCaptureBudget         = fmt.Errorf("%w: missing capture budget", decaderr.ErrUnsupported)
	errCaptureSealMissingCaptureSeal           = fmt.Errorf("%w: missing capture seal", decaderr.ErrUnsupported)
)
