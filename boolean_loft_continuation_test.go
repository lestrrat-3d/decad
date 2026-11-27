package decad

// CI: isolated-tests

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/stretchr/testify/require"
)

func TestLoftContinuationHistoryAndCommitGuards(t *testing.T) {
	d := New()
	stock := internalBoxBody(t, d, -5, -5, 5, 5, 6)
	loft := axialLineLoft(t, d, -1, -1, 1, 1, 2)
	target, err := Cut(stock, loft).Do(t.Context())
	require.NoError(t, err)
	tool := internalBoxBody(t, d, -1, -1, 1, 1, 3)
	// Isolate the history guard from the analytic route chosen for this box.
	target.n.Payload = facetedrecord.Payload{}
	ancestry, ok := loftContinuationSnapshot(d.historyAt(t.Context()), target, tool)
	require.True(t, ok)
	require.Same(t, stock, ancestry.stock)
	require.Same(t, loft, ancestry.loft)
	target.n.OperationWarnings = []opdiagnostic.Record{{Code: opdiagnostic.CodeGapShiftApplied, Change: opdiagnostic.ChangeGapShift}}
	_, ok = loftContinuationSnapshot(d.historyAt(t.Context()), target, tool)
	require.False(t, ok)
	target.n.OperationWarnings = nil
	beforeID := d.producerAt(t.Context())
	beforeHistory := len(d.OperationHistory().Operations())
	beforeBodies := len(d.Bodies())
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	err = commitLoftContinuation(canceled, d, target, tool, target, beforeID, ancestry)
	require.ErrorIs(t, err, context.Canceled)
	ancestry.prior.inputs[0].Role = operationRoleTool
	_, ok = loftContinuationSnapshot(d.historyAt(t.Context()), target, tool)
	require.False(t, ok)
	err = commitLoftContinuation(t.Context(), d, target, tool, target, beforeID, ancestry)
	require.ErrorIs(t, err, ErrBooleanFailed)
	require.Equal(t, beforeID, d.producerAt(t.Context()))
	require.Len(t, d.OperationHistory().Operations(), beforeHistory)
	require.Len(t, d.Bodies(), beforeBodies)
}
