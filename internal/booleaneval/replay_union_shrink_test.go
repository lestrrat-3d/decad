package booleaneval

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
)

func TestStoredUnionShrinkWarningMatchesLiveResult(t *testing.T) {
	cert, result, _, _ := shiftSnapshotFixture()
	cert.shift = pieceShift{Grow: -0.000625}
	cert.displacement, cert.reach = 0.0006250000003938229, 0.01
	warning := opdiagnostic.Record{
		Operation: int(meshbool.OpUnion), PublicOperation: 11,
		Code: opdiagnostic.CodeGapShiftApplied, Operand: 1, Change: opdiagnostic.ChangeGapShift,
		DisplacementBound: cert.displacement, AllowedReach: cert.reach,
	}
	cert.publicWarning = warning
	result.OperationWarnings = []opdiagnostic.Record{warning}
	if !storedUnionShrinkWarningMatches(result, cert, warning) {
		t.Fatal("captured warning did not match the live result")
	}

	result.OperationWarnings[0].AllowedReach++
	if storedUnionShrinkWarningMatches(result, cert, warning) {
		t.Fatal("mutated live result warning matched the captured warning")
	}
}
