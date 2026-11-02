package booleaneval

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/boolshift"
	"github.com/lestrrat-3d/decad/internal/chaintol"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// EvaluateReplay runs the private mesh-only attempt and its shifted retries
// on one counter. Unsupported operand and placement branches refuse.
func EvaluateReplay(ctx context.Context, op meshbool.OperationKind, a, b *topo.Body, chordScale float64,
	budget *proofbound.ReplayBudget) (Evaluation, error) {
	return evaluateReplay(ctx, op, a, b, chordScale, 0, 0, nil, budget)
}

// EvaluateReplayClamped binds every attempt to the authenticated pair chord.
func EvaluateReplayClamped(ctx context.Context, op meshbool.OperationKind, a, b *topo.Body, chordScale, candidate, original float64, budget *proofbound.ReplayBudget) (Evaluation, error) {
	if op != meshbool.OpUnion || !(candidate > 0) || !(original > 0) || !(chordScale > 0) || chordScale > 1 || proofbound.IsNonFinite(candidate) || proofbound.IsNonFinite(original) || proofbound.IsNonFinite(chordScale) || candidate*chordScale > original {
		return Evaluation{}, fmt.Errorf("%w: invalid replay chord clamp", decaderr.ErrUnsupported)
	}
	return evaluateReplay(ctx, op, a, b, chordScale, candidate, original, nil, budget)
}

// EvaluateReplayRetainedClamped uses the authenticated original target mesh.
func EvaluateReplayRetainedClamped(ctx context.Context, a, b *topo.Body, chordScale, candidate, original float64, retained *RetainedTarget, budget *proofbound.ReplayBudget) (Evaluation, error) {
	if retained == nil || retained.body != a || !(candidate > 0) || !(original > 0) || !(chordScale > 0) || chordScale > 1 || proofbound.IsNonFinite(candidate) || proofbound.IsNonFinite(original) || proofbound.IsNonFinite(chordScale) || candidate*chordScale > original {
		return Evaluation{}, fmt.Errorf("%w: invalid retained replay", decaderr.ErrUnsupported)
	}
	return evaluateReplay(ctx, meshbool.OpUnion, a, b, chordScale, candidate, original, retained, budget)
}

func evaluateReplay(ctx context.Context, op meshbool.OperationKind, a, b *topo.Body, chordScale, candidate, original float64, retained *RetainedTarget, budget *proofbound.ReplayBudget) (Evaluation, error) {
	if budget == nil || a == nil || b == nil {
		return Evaluation{}, fmt.Errorf("%w: bounded evaluation needs two operands and a budget", decaderr.ErrUnsupported)
	}
	check := func() error {
		if cert, ok := ctx.Value(preparedTagShiftKey{}).(*BrakeCandidateCertificate); ok {
			if err := cert.validate(ctx, a, b, budget); err != nil {
				return err
			}
		}
		if candidate == 0 {
			return nil
		}
		got, _, err := booloperand.PairChordReplay(ctx, a, b, budget)
		if err != nil {
			return err
		}
		if math.Float64bits(got) != math.Float64bits(candidate) || got*chordScale > original {
			return fmt.Errorf("%w: replay pair chord changed", decaderr.ErrUnsupported)
		}
		return nil
	}
	if err := check(); err != nil {
		return Evaluation{}, err
	}
	meshes := &retainedReplayMesher{replayMesher: replayMesher{budget: budget}, target: retained} //nolint:modernize // Named fields keep the embedded budget explicit.
	twins := &operandTwins{}
	ev, err := evaluate(ctx, op, a, b, meshes, chordScale, nil, nil, twins, nil, nil, budget, nil, 0)
	if err == nil {
		return ev, nil
	}
	var touch *meshbool.TouchContactError
	if !errors.As(err, &touch) || !(gaptol.Reach(ctx) > 0) {
		return ev, err
	}
	tol, dPair, perr := booloperand.PairChordReplay(ctx, a, b, budget)
	if perr != nil {
		return Evaluation{}, perr
	}
	chord, perr := operandChordReplay(ctx, b, tol*chordScale, dPair, chordScale, budget)
	if perr != nil {
		return Evaluation{}, perr
	}
	mb, _, perr := meshes.TessellateOperand(ctx, b, chord)
	if perr != nil {
		return Evaluation{}, perr
	}
	pc, perr := boolshift.PiecesOfReplay(ctx, mb, budget)
	if perr != nil {
		return Evaluation{}, perr
	}
	return replayRetryShifted(ctx, err, op, pc, func(shifts *ReplayShifts) (Evaluation, error) {
		if err := check(); err != nil {
			return Evaluation{}, err
		}
		return evaluate(ctx, op, a, b, meshes, chordScale, nil, nil, twins, nil, shifts, budget, nil, 0)
	}, budget)
}

// EvaluateReplayFixedCut uses the authenticated moved tool exactly once.
func EvaluateReplayFixedCut(ctx context.Context, a, b *topo.Body, tool *FixedShiftedTool, targetAllowance float64, budget *proofbound.ReplayBudget) (Evaluation, error) {
	if budget == nil || a == nil || b == nil || tool == nil || tool.tool != b || targetAllowance < 0 || proofbound.IsNonFinite(targetAllowance) || chaintol.Of(ctx) != 0 {
		return Evaluation{}, fmt.Errorf("%w: fixed shifted Cut lacks authenticated operands or allowance", decaderr.ErrUnsupported)
	}
	base, _, err := booloperand.PairChordReplay(ctx, a, b, budget)
	if err != nil {
		return Evaluation{}, err
	}
	ev, err := evaluate(ctx, meshbool.OpCut, a, b, replayMesher{budget: budget}, 1, nil, nil, &operandTwins{}, nil, nil, budget, tool, targetAllowance)
	if err != nil {
		return ev, err
	}
	after, _, err := booloperand.PairChordReplay(ctx, a, b, budget)
	if err != nil {
		return Evaluation{}, err
	}
	if !(base > 0) || proofbound.IsNonFinite(base) || math.Float64bits(base) != math.Float64bits(after) {
		return Evaluation{}, fmt.Errorf("%w: final Cut chord changed", decaderr.ErrUnsupported)
	}
	return ev, nil
}
