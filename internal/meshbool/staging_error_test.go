package meshbool_test

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/stretchr/testify/require"
)

func TestStagingError(t *testing.T) {
	plain := errors.New("plain")
	require.Same(t, plain, meshbool.StagingError(meshbool.OpCut, plain))

	for _, tc := range []struct {
		op      meshbool.OperationKind
		operand int
		role    string
	}{
		{meshbool.OpCut, 0, "cut's target"},
		{meshbool.OpCut, 1, "cut's tool"},
		{meshbool.OpUnion, 0, "union's first operand"},
		{meshbool.OpIntersect, 1, "intersect's second operand"},
	} {
		held := &meshbool.CoarseHeldContactError{Operand: tc.operand, Bound: 0.5, Tol: 0.25}
		err := meshbool.StagingError(tc.op, held)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, tc.role+" holds its mesh")
		var expected *meshbool.BooleanExpectedError
		require.ErrorAs(t, err, &expected)
		require.Equal(t, meshbool.BooleanExpectedStaging, expected.Kind)
		require.Equal(t, tc.operand, expected.Operand)
	}
}

func TestRefuseRimPastPair(t *testing.T) {
	for _, maxRim := range []float64{math.NaN(), math.Inf(1)} {
		err := meshbool.RefuseRimPastPair(maxRim, 1)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "cannot bound")
	}
	for _, maxRim := range []float64{1, 2} {
		err := meshbool.RefuseRimPastPair(maxRim, 1)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "cross too shallowly")
	}
	require.NoError(t, meshbool.RefuseRimPastPair(0.5, 1))
	require.NoError(t, meshbool.RefuseRimPastPair(0, 0))
}
