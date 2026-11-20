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

func TestCoarseHeldOperand(t *testing.T) {
	t.Parallel()
	staging := func(operand int) error {
		return meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedStaging, operand,
			&meshbool.CoarseHeldContactError{Operand: operand})
	}
	for _, tc := range []struct {
		name    string
		cause   error
		operand int
		want    bool
	}{
		{name: "first operand", cause: staging(0), operand: 0, want: true},
		{name: "second operand", cause: fmt.Errorf("wrapped: %w", staging(1)), operand: 1, want: true},
		{name: "nil cause"},
		{name: "plain unsupported", cause: decaderr.ErrUnsupported},
		{name: "operand out of range", cause: &meshbool.CoarseHeldContactError{Operand: 2}},
		{name: "canceled", cause: errors.Join(staging(0), context.Canceled)},
		{name: "deadline", cause: errors.Join(staging(1), context.DeadlineExceeded)},
	} {
		operand, ok := booleandispatch.CoarseHeldOperand(tc.cause)
		require.Equal(t, tc.want, ok, tc.name)
		require.Equal(t, tc.operand, operand, tc.name)
	}
}
