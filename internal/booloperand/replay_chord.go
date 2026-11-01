package booloperand

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// ChordShareReplay reads the supported operands without an unmetered revolve resolution.
func ChordShareReplay(ctx context.Context, b *topo.Body, budget *proofbound.ReplayBudget) (meshbool.ChordOperand, error) {
	if b == nil || budget == nil {
		return meshbool.ChordOperand{}, fmt.Errorf("%w: bounded chord requires an operand and budget", decaderr.ErrUnsupported)
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return meshbool.ChordOperand{}, err
	}
	section := 0.0
	switch p := b.Payload.(type) {
	case prismrecord.Payload:
		section = p.SectionDelta
	case facetedrecord.Payload:
	default:
		return meshbool.ChordOperand{}, fmt.Errorf("%w: bounded chord does not support this payload", decaderr.ErrUnsupported)
	}
	box := b.Bounds
	pad := r3.NewVec(box.Bound.Base(), box.Bound.Base(), box.Bound.Base())
	return meshbool.ChordOperand{Lo: box.Min.Sub(pad), Hi: box.Max.Add(pad), Floor: max(proofbound.ProductUpper(2, section), heldPrimitiveFloor(b.Payload))}, nil
}

// PairChordReplay derives the ordinary pair tolerance on the shared counter.
func PairChordReplay(ctx context.Context, a, b *topo.Body, budget *proofbound.ReplayBudget) (float64, float64, error) {
	x, err := ChordShareReplay(ctx, a, budget)
	if err != nil {
		return 0, 0, err
	}
	y, err := ChordShareReplay(ctx, b, budget)
	if err != nil {
		return 0, 0, err
	}
	tol, diameter, ok := meshbool.PairChordFrom(x, y)
	if !ok {
		return 0, 0, fmt.Errorf("%w: the operand pair has no extent to derive a chord tolerance from", decaderr.ErrDegenerate)
	}
	return tol, diameter, nil
}
