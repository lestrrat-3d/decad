package opdiagnostic_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/stretchr/testify/require"
)

func TestCompositeSummariesBoundTransientLocations(t *testing.T) {
	var summaries opdiagnostic.CompositeSummaries
	for range 1000 {
		allowance := 0.02
		summaries.Add([]opdiagnostic.Record{
			{Code: opdiagnostic.CodeGapShiftApplied, Operand: 1, DisplacementBound: 0.001, AllowedReach: 0.01},
			{Code: opdiagnostic.CodeChainAllowanceApplied, Operand: 0, TouchedBound: 0.003,
				PairChordLimit: 0.001, ChainAllowance: &allowance},
		}, 91)
	}
	got := summaries.Records()
	require.Len(t, got, 2)
	require.Equal(t, opdiagnostic.CodeChainAllowanceApplied, got[0].Code)
	require.Equal(t, -1, got[0].Operand)
	require.Equal(t, 91, got[0].PublicOperation)
	require.Equal(t, 0.003, got[0].TouchedBound)
	require.Equal(t, opdiagnostic.CodeGapShiftApplied, got[1].Code)
	require.Equal(t, -1, got[1].Operand)
	require.GreaterOrEqual(t, got[1].DisplacementBound, 1.0)
	require.GreaterOrEqual(t, got[1].AllowedReach, 10.0)
}
