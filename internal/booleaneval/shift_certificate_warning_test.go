package booleaneval

import (
	"errors"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestStoredShiftWarningRevalidated(t *testing.T) {
	cert, result, target, tool := shiftSnapshotFixture()
	warning := opdiagnostic.Record{}
	result.OperationWarnings = []opdiagnostic.Record{warning}
	budget := proofbound.NewReplayTrialBudget()
	snapshot, err := SnapshotShiftCertificate(t.Context(), budget, cert)
	if err != nil {
		t.Fatal(err)
	}
	if err := RevalidateStoredWarning(t.Context(), snapshot, result, target, tool, warning, budget); err != nil {
		t.Fatalf("original warning: %v", err)
	}
	result.OperationWarnings[0].AllowedReach = 1
	err = RevalidateStoredWarning(t.Context(), snapshot, result, target, tool, warning, budget)
	if !errors.Is(err, decaderr.ErrUnsupported) || !strings.Contains(err.Error(), "warning changed") {
		t.Fatalf("changed warning: %v", err)
	}
	result.OperationWarnings[0] = warning
	cert.selected.Vertices[0].X = 7
	err = RevalidateStoredWarning(t.Context(), snapshot, result, target, tool, warning, budget)
	if !errors.Is(err, decaderr.ErrUnsupported) || !strings.Contains(err.Error(), "stored mesh changed") {
		t.Fatalf("changed certificate: %v", err)
	}
}
