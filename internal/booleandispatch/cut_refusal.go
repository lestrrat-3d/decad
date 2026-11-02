package booleandispatch

import (
	"context"
	"errors"

	"github.com/lestrrat-3d/decad/internal/meshbool"
)

// CoarseTargetCut admits only a Cut without matching-faces intent whose refusal
// carries target-side coarse-held-contact staging tags. Every nested tag must
// agree. Joined refusals, cancellation and chains exceeding the bounded tag
// walk decline. This classifies the trigger in boolean-chain-regroup-design's
// Paired Cut continuation; it certifies no geometry or history.
func CoarseTargetCut(op meshbool.OperationKind, matchingFaces bool, cause error) bool {
	if op != meshbool.OpCut || matchingFaces || cause == nil ||
		errors.Is(cause, context.Canceled) || errors.Is(cause, context.DeadlineExceeded) {
		return false
	}
	expected, ok := errors.AsType[*meshbool.BooleanExpectedError](cause)
	if !ok || expected.Kind != meshbool.BooleanExpectedStaging || expected.Operand != 0 {
		return false
	}
	held, ok := errors.AsType[*meshbool.CoarseHeldContactError](expected.Err)
	if !ok || held.Operand != 0 {
		return false
	}
	return consistentCoarseTags(cause, 0)
}

func consistentCoarseTags(err error, depth int) bool {
	if err == nil {
		return true
	}
	if depth >= 128 {
		return false
	}
	switch e := err.(type) {
	case *meshbool.BooleanExpectedError:
		if e.Kind != meshbool.BooleanExpectedStaging || e.Operand != 0 {
			return false
		}
	case *meshbool.CoarseHeldContactError:
		if e.Operand != 0 {
			return false
		}
	}
	if _, ok := err.(interface{ Unwrap() []error }); ok {
		return false
	}
	if e, ok := err.(interface{ Unwrap() error }); ok {
		return consistentCoarseTags(e.Unwrap(), depth+1)
	}
	return true
}
