package booleaneval_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/stretchr/testify/require"
)

func TestEvaluationDiagnosticsCopiesAcceptedSummaries(t *testing.T) {
	t.Parallel()
	require.Empty(t, (booleaneval.Evaluation{}).Diagnostics(meshbool.OpCut))
	eval := booleaneval.Evaluation{
		Shift: &booleaneval.ShiftSummary{DisplacementBound: 0.001, AllowedReach: 0.01},
		Chain: &booleaneval.ChainAllowanceSummary{
			Operand: 0, TouchedBound: 0.03, PairChordLimit: 0.02, ChainAllowance: 0.1,
		},
		EmbeddingReach: &meshbool.EmbeddingReach{DisplacementBound: 0.002, AllowedReach: 0.01},
	}
	records := eval.Diagnostics(meshbool.OpCut)
	require.Len(t, records, 3)
	require.Equal(t, opdiagnostic.CodeGapShiftApplied, records[0].Code)
	require.Equal(t, int(meshbool.OpCut), records[0].Operation)
	require.Equal(t, 1, records[0].Operand)
	require.Equal(t, 0.001, records[0].DisplacementBound)
	require.Equal(t, 0.01, records[0].AllowedReach)
	require.Equal(t, opdiagnostic.CodeChainAllowanceApplied, records[1].Code)
	require.Equal(t, int(meshbool.OpCut), records[1].Operation)
	require.Equal(t, 0, records[1].Operand)
	require.Equal(t, 0.03, records[1].TouchedBound)
	require.Equal(t, 0.02, records[1].PairChordLimit)
	require.NotNil(t, records[1].ChainAllowance)
	require.Equal(t, 0.1, *records[1].ChainAllowance)
	require.Equal(t, opdiagnostic.CodeEmbeddingReachApplied, records[2].Code)
	require.Equal(t, -1, records[2].Operand)
	require.Equal(t, 0.002, records[2].DisplacementBound)
	require.Equal(t, 0.01, records[2].AllowedReach)

	*records[1].ChainAllowance = 9
	records[0].DisplacementBound = 9
	again := eval.Diagnostics(meshbool.OpCut)
	require.Equal(t, 0.1, eval.Chain.ChainAllowance)
	require.Equal(t, 0.1, *again[1].ChainAllowance)
	require.Equal(t, 0.001, again[0].DisplacementBound)
}
