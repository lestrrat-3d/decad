package booleaneval

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestPreparedTagShiftRecheckRejectsMissingReplay(t *testing.T) {
	p := &preparedTagShift{}
	require.ErrorIs(t, p.recheck(t.Context(), nil, nil, nil, nil, pieces{}, nil, nil, proofbound.NewBrakeReplayBudget()), errBrakeEvalPreparedTagShiftChanged)
	require.ErrorIs(t, p.recheck(t.Context(), nil, nil, nil, nil, pieces{}, nil, &ReplayShifts{}, nil), errBrakeEvalPreparedTagShiftChanged)
}
