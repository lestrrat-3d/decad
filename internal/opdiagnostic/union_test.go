package opdiagnostic_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/stretchr/testify/require"
)

func TestUnionSummariesOriginalOperandsAndOrder(t *testing.T) {
	var summaries opdiagnostic.UnionSummaries
	allowance := 0.01
	summaries.AddPair([]opdiagnostic.Record{
		{Code: opdiagnostic.CodeEmbeddingReachApplied, Operand: -1, DisplacementBound: 0.001, AllowedReach: 0.01},
		{Code: opdiagnostic.CodeGapShiftApplied, Operand: 1, DisplacementBound: 0.002, AllowedReach: 0.01},
		{Code: opdiagnostic.CodeChainAllowanceApplied, Operand: 0, TouchedBound: 0.003,
			PairChordLimit: 0.001, ChainAllowance: &allowance},
	}, 1)
	summaries.AddPair([]opdiagnostic.Record{
		{Code: opdiagnostic.CodeGapShiftApplied, Operand: 1, DisplacementBound: 0.003, AllowedReach: 0.01},
		{Code: opdiagnostic.CodeChainAllowanceApplied, Operand: 0, TouchedBound: 0.004,
			PairChordLimit: 0.002, ChainAllowance: &allowance},
		{Code: opdiagnostic.CodeEmbeddingReachApplied, Operand: -1, DisplacementBound: 0.002, AllowedReach: 0.01},
	}, 2)
	allowance = 99

	got := summaries.Records()
	require.Len(t, got, 5)
	wantOperands := []int{0, 1, 2, -1, -1}
	wantCodes := []opdiagnostic.Code{opdiagnostic.CodeChainAllowanceApplied,
		opdiagnostic.CodeGapShiftApplied, opdiagnostic.CodeGapShiftApplied,
		opdiagnostic.CodeChainAllowanceApplied, opdiagnostic.CodeEmbeddingReachApplied}
	for i, record := range got {
		require.Equal(t, wantOperands[i], record.Operand, "record %d", i)
		require.Equal(t, wantCodes[i], record.Code, "record %d", i)
	}
	require.NotNil(t, got[0].ChainAllowance)
	require.Equal(t, 0.01, *got[0].ChainAllowance)
	*got[0].ChainAllowance = 25
	require.Equal(t, 0.01, *summaries.Records()[0].ChainAllowance)
	require.GreaterOrEqual(t, got[4].DisplacementBound, 0.003)
	require.GreaterOrEqual(t, got[4].AllowedReach, 0.02)
}

func TestUnionSummariesBoundRepeatedMechanisms(t *testing.T) {
	var summaries opdiagnostic.UnionSummaries
	for i := range 1000 {
		allowance := 2.0
		summaries.AddPair([]opdiagnostic.Record{
			{Code: opdiagnostic.CodeEmbeddingReachApplied, Operand: -1,
				DisplacementBound: 0.0001, AllowedReach: 0.01},
			{Code: opdiagnostic.CodeChainAllowanceApplied, Operand: 0,
				TouchedBound: float64(i+1) / 1000, PairChordLimit: float64(i+1) / 2000,
				ChainAllowance: &allowance},
		}, i+2)
	}
	got := summaries.Records()
	require.Len(t, got, 2, "step count must not increase the inventory at one location")
	require.Equal(t, opdiagnostic.CodeChainAllowanceApplied, got[0].Code)
	require.Equal(t, -1, got[0].Operand)
	require.Equal(t, 1.0, got[0].TouchedBound)
	require.Equal(t, 0.5, got[0].PairChordLimit, "the summary preserves the selected gate's actual tuple")
	require.Equal(t, 2.0, *got[0].ChainAllowance)
	require.GreaterOrEqual(t, got[1].DisplacementBound, 0.1)
	require.GreaterOrEqual(t, got[1].AllowedReach, 10.0)
}
