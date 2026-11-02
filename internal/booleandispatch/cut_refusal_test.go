package booleandispatch_test

import (
	"context"
	"errors"
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleandispatch"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/stretchr/testify/require"
)

func TestCoarseTargetCutRefusal(t *testing.T) {
	t.Parallel()
	held := &meshbool.CoarseHeldContactError{Operand: 0}
	staging := meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedStaging, 0, held)
	nested := func(kind meshbool.BooleanExpectedKind, operand int) error {
		return &meshbool.BooleanExpectedError{Kind: meshbool.BooleanExpectedStaging, Operand: 0,
			Err: &meshbool.BooleanExpectedError{Kind: kind, Operand: operand, Err: held}}
	}
	wrap := func(n int) error {
		err := staging
		for range n {
			err = fmt.Errorf("wrapped: %w", err)
		}
		return err
	}
	for _, tc := range []struct {
		name          string
		op            meshbool.OperationKind
		matchingFaces bool
		cause         error
		want          bool
	}{
		{name: "target staging", op: meshbool.OpCut, cause: staging, want: true},
		{name: "wrapped target staging", op: meshbool.OpCut, cause: fmt.Errorf("wrapped: %w", staging), want: true},
		{name: "matching nested tags", op: meshbool.OpCut, cause: nested(meshbool.BooleanExpectedStaging, 0), want: true},
		{name: "nil cause", op: meshbool.OpCut},
		{name: "plain unsupported", op: meshbool.OpCut, cause: decaderr.ErrUnsupported},
		{name: "bare held contact", op: meshbool.OpCut, cause: held},
		{name: "staging without held contact", op: meshbool.OpCut,
			cause: meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedStaging, 0, decaderr.ErrUnsupported)},
		{name: "tool held contact", op: meshbool.OpCut,
			cause: meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedStaging, 0, &meshbool.CoarseHeldContactError{Operand: 1})},
		{name: "nested operand conflict", op: meshbool.OpCut, cause: nested(meshbool.BooleanExpectedStaging, 1)},
		{name: "nested kind conflict", op: meshbool.OpCut, cause: nested(meshbool.BooleanExpectedContact, 0)},
		{name: "matching faces", op: meshbool.OpCut, matchingFaces: true, cause: staging},
		{name: "union", op: meshbool.OpUnion, cause: staging},
		{name: "intersect", op: meshbool.OpIntersect, cause: staging},
		{name: "joined cancellation", op: meshbool.OpCut, cause: errors.Join(staging, context.Canceled)},
		{name: "joined deadline", op: meshbool.OpCut, cause: errors.Join(staging, context.DeadlineExceeded)},
		{name: "joined single cause", op: meshbool.OpCut, cause: errors.Join(staging)},
		{name: "joined matching causes", op: meshbool.OpCut, cause: errors.Join(staging, staging)},
		{name: "last accepted tag depth", op: meshbool.OpCut, cause: wrap(125), want: true},
		{name: "tag depth exhausted", op: meshbool.OpCut, cause: wrap(126)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			require.Equal(t, tc.want, booleandispatch.CoarseTargetCut(tc.op, tc.matchingFaces, tc.cause))
		})
	}
}
