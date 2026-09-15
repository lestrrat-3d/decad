package decad

import (
	"testing"

	"github.com/stretchr/testify/require"
)

// This file tests verify_result.go's private vocabulary directly, in-package,
// because every declaration under test is unexported (task-list §PR-1).

// TestVerifyResultEnumTokens pins every private enum's stable lower-snake
// String() token, in constant order, plus its unknown-value rendering
// (proposal §3, task-list §4 item 1's report-vocabulary precedent).
func TestVerifyResultEnumTokens(t *testing.T) {
	t.Parallel()

	require.Equal(t, "not_evaluated", scalarNotEvaluated.String())
	require.Equal(t, "not_requested", scalarNotRequested.String())
	require.Equal(t, "unavailable", scalarUnavailable.String())
	require.Equal(t, "undecided", scalarUndecided.String())
	require.Equal(t, "absent", scalarAbsent.String())
	require.Equal(t, "measured", scalarMeasured.String())
	require.Equal(t, "scalar_outcome(7)", scalarOutcome(7).String())

	require.Equal(t, "not_evaluated", coverageNotEvaluated.String())
	require.Equal(t, "not_requested", coverageNotRequested.String())
	require.Equal(t, "unavailable", coverageUnavailable.String())
	require.Equal(t, "undecided", coverageUndecided.String())
	require.Equal(t, "partial", coveragePartial.String())
	require.Equal(t, "complete", coverageComplete.String())
	require.Equal(t, "coverage(7)", coverageState(7).String())

	require.Equal(t, "not_evaluated", assessmentNotEvaluated.String())
	require.Equal(t, "met", assessmentMet.String())
	require.Equal(t, "violated", assessmentViolated.String())
	require.Equal(t, "undecided", assessmentUndecided.String())
	require.Equal(t, "assessment(7)", assessmentState(7).String())

	require.Equal(t, "not_evaluated", toleranceNotEvaluated.String())
	require.Equal(t, "satisfied", toleranceSatisfied.String())
	require.Equal(t, "exceeded", toleranceExceeded.String())
	require.Equal(t, "undecided", toleranceUndecided.String())
	require.Equal(t, "tolerance_state(7)", toleranceState(7).String())

	require.Equal(t, "not_evaluated", validityNotEvaluated.String())
	require.Equal(t, "valid", validityValid.String())
	require.Equal(t, "invalid", validityInvalid.String())
	require.Equal(t, "undecided", validityUndecided.String())
	require.Equal(t, "validity_outcome(7)", validityOutcome(7).String())
}

// TestVerifyResultZeroValueIsUnverified pins proposal §4's zero-value
// guarantee at the private layer, ahead of PR 5 renaming these records to
// Report/BodyReport: the zero verifyResult and the zero bodyResult both
// retain Unverified, never a fabricated verdict.
func TestVerifyResultZeroValueIsUnverified(t *testing.T) {
	t.Parallel()
	var res verifyResult
	require.Equal(t, Unverified, res.Status)
	var body bodyResult
	require.Equal(t, Unverified, body.Status)
}
