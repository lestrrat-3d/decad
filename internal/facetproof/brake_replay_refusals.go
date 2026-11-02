package facetproof

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// These immutable refusals exist before the replay budget starts.
var (
	errBrakeReplayInvalidReplayFaceIndex   = fmt.Errorf("%w: invalid replay face index", decaderr.ErrUnsupported)
	errBrakeReplayInvalidReplayFaceProofs  = fmt.Errorf("%w: invalid replay face proofs", decaderr.ErrUnsupported)
	errBrakeReplayInvalidReplayMomentIndex = fmt.Errorf("%w: invalid replay moment index", decaderr.ErrUnsupported)
	errBrakeReplayMissingReplayMomentInput = fmt.Errorf("%w: missing replay moment input", decaderr.ErrUnsupported)
)
