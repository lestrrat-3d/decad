package decad

import (
	"context"
	"errors"
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestOperationDiagnosticEnumsAndBooleanError(t *testing.T) {
	require.Equal(t, "gap_shift_applied", OpDiagGapShiftApplied.String())
	require.Equal(t, "result", OperationStageResult.String())
	require.Equal(t, "OperationStage(99)", OperationStage(99).String())

	original := errors.New("empty mesh result")
	err := newBooleanError(meshbool.OpCut, BooleanEmpty, ErrBooleanFailed, original)
	wrapped := fmt.Errorf("caller context: %w", err)
	require.ErrorIs(t, wrapped, ErrBooleanFailed)
	var diagnosticError OperationDiagnosticError
	require.ErrorAs(t, wrapped, &diagnosticError)
	diagnostics := diagnosticError.OperationDiagnostics()
	require.Len(t, diagnostics, 1)
	require.Equal(t, OpDiagResultEmpty, diagnostics[0].Code)
	require.Equal(t, OperationDiagnosticFailure, diagnostics[0].Severity)
	require.Equal(t, OperationCut, diagnostics[0].Operation)
	require.Equal(t, OperationStageResult, diagnostics[0].Stage)
}

func TestEmptyBooleanDiagnosticsKeepAnAcceptedShift(t *testing.T) {
	original := fmt.Errorf("%w: Intersect is empty", ErrBooleanFailed)
	expected := meshbool.ExpectedBooleanAfterShift(original, 0.0002, 0.001)
	err := newBooleanError(meshbool.OpIntersect, BooleanEmpty, ErrBooleanFailed, expected)
	diagnostics := err.(*BooleanError).OperationDiagnostics()
	require.Len(t, diagnostics, 2)
	require.Equal(t, OpDiagResultEmpty, diagnostics[0].Code)
	require.Equal(t, OpDiagGapShiftApplied, diagnostics[1].Code)
	require.Equal(t, OperationIntersect, diagnostics[1].Operation)
	require.Equal(t, "b", diagnostics[1].Subject.Name)
	require.NotNil(t, diagnostics[1].Change)
	require.Equal(t, 0.0002, diagnostics[1].Change.DisplacementBound.Base())
	require.Equal(t, 0.001, diagnostics[1].Change.AllowedReach.Base())
}

func TestOperationWorkLimitHasTypedArrangementCounts(t *testing.T) {
	original := prismcells.WithWorkLimit(fmt.Errorf("%w: arrangement cap reached", ErrUnsupported), 4097)
	err := operationError(OperationUnion, original)
	var diagnosticError OperationDiagnosticError
	require.ErrorAs(t, err, &diagnosticError)
	diagnostics := diagnosticError.OperationDiagnostics()
	require.Len(t, diagnostics, 1)
	require.Equal(t, OpDiagWorkLimitReached, diagnostics[0].Code)
	require.Equal(t, OperationUnion, diagnostics[0].Operation)
	require.Equal(t, "arrangement_segments", diagnostics[0].WorkLimit.Resource)
	require.Equal(t, 4097, diagnostics[0].WorkLimit.Observed)
	require.Equal(t, 4096, diagnostics[0].WorkLimit.Maximum)
	require.ErrorIs(t, err, ErrUnsupported)

	proofFailure := proofbound.WithWorkLimit(ErrUnsupported, "proof_steps", 257, 256)
	err = operationError(OperationFillet, proofFailure)
	require.ErrorAs(t, err, &diagnosticError)
	diagnostics = diagnosticError.OperationDiagnostics()
	require.Len(t, diagnostics, 1)
	require.Equal(t, OperationFillet, diagnostics[0].Operation)
	require.Equal(t, "proof_steps", diagnostics[0].WorkLimit.Resource)
	require.Equal(t, 257, diagnostics[0].WorkLimit.Observed)
	require.Equal(t, 256, diagnostics[0].WorkLimit.Maximum)
}

func TestOperationErrorCopiesOptionalDetailsAndUnwrapsOriginal(t *testing.T) {
	original := fmt.Errorf("wrapped input: %w", ErrUnsupported)
	ordinal := 2
	allowance := units.Millimeters(0.5)
	err := &OperationError{
		err: original,
		diagnostics: []OperationDiagnostic{{
			Code:      OpDiagOperandBoundExceeded,
			Severity:  OperationDiagnosticFailure,
			Operation: OperationCut,
			Stage:     OperationStageEvaluation,
			Subject:   OperationSubject{Name: operationRoleTool, Ordinal: &ordinal},
			Chain:     &OperationChainDetail{TouchedBound: units.Millimeters(0.7), PairChordLimit: units.Millimeters(0.1), ChainAllowance: &allowance},
			Change:    &OperationChange{Mechanism: OperationChangeGapShift, DisplacementBound: units.Millimeters(0.2), AllowedReach: units.Millimeters(1)},
			WorkLimit: &OperationWorkLimit{Resource: "cells", Observed: 8, Maximum: 8},
		}},
	}
	require.Equal(t, original.Error(), err.Error())
	require.ErrorIs(t, err, ErrUnsupported)

	first := err.OperationDiagnostics()
	*first[0].Subject.Ordinal = 99
	*first[0].Chain.ChainAllowance = units.Millimeters(99)
	first[0].Change.DisplacementBound = units.Millimeters(99)
	first[0].WorkLimit.Observed = 99
	second := err.OperationDiagnostics()
	require.Equal(t, 2, *second[0].Subject.Ordinal)
	require.Equal(t, 0.5, second[0].Chain.ChainAllowance.Base())
	require.Equal(t, 0.2, second[0].Change.DisplacementBound.Base())
	require.Equal(t, 8, second[0].WorkLimit.Observed)
}

func TestOperationErrorPreservesKnownFailureAndSkipsTypedErrors(t *testing.T) {
	original := fmt.Errorf("invalid tool: %w", ErrForeignBody)
	wrapped := operationError(OperationCut, original)
	require.NotSame(t, original, wrapped)
	require.Equal(t, original.Error(), wrapped.Error())
	require.ErrorIs(t, wrapped, ErrForeignBody)
	var structured OperationDiagnosticError
	require.ErrorAs(t, wrapped, &structured)
	diagnostics := structured.OperationDiagnostics()
	require.Len(t, diagnostics, 1)
	require.Equal(t, OpDiagInputRejected, diagnostics[0].Code)
	require.Equal(t, OperationCut, diagnostics[0].Operation)
	nested := operationError(OperationExtrude, fmt.Errorf("tool build: %w", wrapped))
	var nestedDiagnostics OperationDiagnosticError
	require.ErrorAs(t, nested, &nestedDiagnostics)
	require.Equal(t, OperationExtrude, nestedDiagnostics.OperationDiagnostics()[0].Operation)

	boolean := newBooleanError(meshbool.OpCut, BooleanEmpty, ErrBooleanFailed, errors.New("empty"))
	require.Same(t, boolean, operationError(OperationCut, boolean))
	selection := newSelectionError(EdgeSelectorKind, "edges(all)", nil, "1", 0, nil, 1, ErrCardinality)
	require.Same(t, selection, operationError(OperationCut, selection))
	proofUnavailable := meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedVolumeProof, 1, ErrUnsupported)
	proofDiagnostic := operationError(OperationCut, proofUnavailable).(OperationDiagnosticError).OperationDiagnostics()
	require.Equal(t, OpDiagOperandProofUnavailable, proofDiagnostic[0].Code)
	require.Equal(t, "tool", proofDiagnostic[0].Subject.Name)
	tessellation := meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedCoarseTessellation, 0, ErrUnsupported)
	tessellationDiagnostic := operationError(OperationCut, tessellation).(OperationDiagnosticError).OperationDiagnostics()
	require.Equal(t, OpDiagTessellationUnresolved, tessellationDiagnostic[0].Code)
	cancelled := fmt.Errorf("context canceled: %w", context.Canceled)
	require.Same(t, cancelled, operationError(OperationCut, cancelled))
}

func TestSelectionErrorHasSelectorDiagnostic(t *testing.T) {
	err := newSelectionError(EdgeSelectorKind, "edges(convex)", nil, "exactly 1", 0, nil, 1, ErrCardinality)
	diagnostics := err.OperationDiagnostics()
	require.Len(t, diagnostics, 1)
	require.Equal(t, OpDiagSelectionFailed, diagnostics[0].Code)
	require.Equal(t, OperationSelectEdges, diagnostics[0].Operation)
	require.Equal(t, "selector", diagnostics[0].Subject.Name)
	require.ErrorIs(t, err, ErrCardinality)
}
