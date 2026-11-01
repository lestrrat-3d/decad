package booleaneval

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// DiagnosticsReplay copies exactly the accepted warning fields under the meter.
func (e Evaluation) DiagnosticsReplay(ctx context.Context, op meshbool.OperationKind, b *proofbound.ReplayBudget) ([]opdiagnostic.Record, error) {
	chains := e.Chains
	count := len(chains)
	if chains == nil && e.Chain != nil {
		count = 1
	}
	n := count
	if e.Shift != nil {
		n++
	}
	if e.EmbeddingReach != nil {
		n++
	}
	if n == 0 {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		return nil, nil
	}
	if err := b.ReserveRecords(ctx, uint64(n+count)); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(opdiagnostic.Record{}))); err != nil {
		return nil, err
	}
	for range count {
		if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(float64(0)))); err != nil {
			return nil, err
		}
	}
	if err := b.SpendWork(ctx, uint64(n)); err != nil {
		return nil, err
	}
	out := make([]opdiagnostic.Record, 0, n)
	if e.Shift != nil {
		out = append(out, opdiagnostic.Record{Operation: int(op), Code: opdiagnostic.CodeGapShiftApplied, Operand: 1, Change: opdiagnostic.ChangeGapShift, DisplacementBound: e.Shift.DisplacementBound, AllowedReach: e.Shift.AllowedReach})
	}
	appendChain := func(c ChainAllowanceSummary) {
		v := c.ChainAllowance
		out = append(out, opdiagnostic.Record{Operation: int(op), Code: opdiagnostic.CodeChainAllowanceApplied, Operand: c.Operand, TouchedBound: c.TouchedBound, PairChordLimit: c.PairChordLimit, ChainAllowance: &v})
	}
	if chains == nil && e.Chain != nil {
		appendChain(*e.Chain)
	} else {
		for _, c := range chains {
			appendChain(c)
		}
	}
	if e.EmbeddingReach != nil {
		out = append(out, opdiagnostic.Record{Code: opdiagnostic.CodeEmbeddingReachApplied, Operand: -1, Change: opdiagnostic.ChangeEmbeddingReach, DisplacementBound: e.EmbeddingReach.DisplacementBound, AllowedReach: e.EmbeddingReach.AllowedReach})
	}
	return out, nil
}
