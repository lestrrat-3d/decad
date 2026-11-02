package booleaneval

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessdispatch"
	"github.com/lestrrat-3d/decad/internal/topo"
)

type brakeCircleCandidateMesher struct {
	replayMesher
	pin   *topo.Body
	chord float64
}

func (m brakeCircleCandidateMesher) TessellateOperand(ctx context.Context, body *topo.Body, chord float64) (*topo.Mesh, bool, error) {
	if body != m.pin {
		return m.replayMesher.TessellateOperand(ctx, body, chord)
	}
	if math.Float64bits(chord) != math.Float64bits(m.chord) {
		return nil, false, errBrakeEvalCircleCandidateChordChanged
	}
	mesh, err := tessdispatch.BrakeCirclePrismReplay(ctx, body, chord, m.budget)
	return mesh, false, err
}

// EvaluateBrakeCircleCandidate runs one fixed-quarter slot and pin Union attempt.
func EvaluateBrakeCircleCandidate(ctx context.Context, slot, pin *topo.Body,
	b *proofbound.ReplayBudget) (Evaluation, error) {
	if slot == nil || pin == nil || slot == pin || b == nil || !b.IsBrakeReplay() {
		return Evaluation{}, errBrakeEvalInvalidBrakeCircleCandidate
	}
	tol, diameter, err := booloperand.PairChordReplay(ctx, slot, pin, b)
	if err != nil {
		return Evaluation{}, err
	}
	chord, err := operandChordReplay(ctx, pin, tol*.25, diameter, .25, b)
	if err != nil {
		return Evaluation{}, err
	}
	mesher := brakeCircleCandidateMesher{replayMesher: replayMesher{budget: b}, pin: pin, chord: chord} //nolint:modernize // Named fields keep the embedded budget explicit.
	ev, err := evaluate(ctx, meshbool.OpUnion, slot, pin, mesher, .25, nil, nil, &operandTwins{}, nil, nil, b, nil, 0)
	if err != nil {
		return ev, err
	}
	snapshot, err := snapshotCandidatePayload(ctx, ev.Payload, b)
	if err != nil {
		return Evaluation{}, err
	}
	if err := b.ReserveRecords(ctx, 1); err != nil {
		return Evaluation{}, err
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(circleCandidateSeal{}))); err != nil {
		return Evaluation{}, err
	}
	ev.circleSeal = &circleCandidateSeal{slot: slot, pin: pin, slotID: slot.Origin.Producer,
		pinID: pin.Origin.Producer, budget: b, chord: chord, payload: snapshot,
		volume: ev.Volume, volumeBound: ev.VolumeBound}
	return ev, nil
}
