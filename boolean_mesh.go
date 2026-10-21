package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/units"
)

// evaluateBooleanMeshes runs the mesh-boolean pipeline
// (internal/booleaneval, agent-docs/evaluator-design.md §9) over a and b, with the
// operand meshes drawn from meshes, and states its occupied volume as a public
// measurement. Verify passes its per-call cache (verifyMeshCache,
// internal/verifyrun/pairs.go), which meshes each body once at one chord for every pair it
// takes part in (agent-docs/interference-design.md §5.3); chordScale lets one
// source-certified modify route ask for finer operand chords, and a
// power-of-two scale preserves the pair tolerance's represented value exactly.
func evaluateBooleanMeshes(ctx context.Context, op meshbool.OperationKind, a, b *Body,
	meshes booleaneval.OperandMesher, chordScale float64) (booleanEvaluation, error) {
	ev, err := booleaneval.Evaluate(ctx, op, bodyNode(a), bodyNode(b), meshes, chordScale, nil, nil)
	if err != nil {
		return booleanEvaluation{}, err
	}
	return booleanEvaluation{payload: ev.Payload, volume: volumeMeasurement(ev.Volume, ev.VolumeBound),
		volumeRat: ev.VolumeRat, audit: ev.Audit}, nil
}

func volumeMeasurement(value, bound float64) Measurement {
	return Measurement{
		Value:     units.CubicMillimeters(value),
		Exactness: exactnessOf(bound),
		Bound:     units.CubicMillimeters(bound),
	}
}

// exactnessOf keys a measurement's exactness off its proven bound: zero is the
// truth, anything else is Approximate.
//
// A zero bound is therefore a CLAIM that the reported value is exactly
// representable — and only a value proven so may reach it: analytic features
// with explicit rational/error evaluation, and boolean rational integrals
// carrying their own rounding term. A value a float loop COMPUTED is never among
// them: its bound comes from internal/proofbound/bounds.go, whose helpers are never zero for a
// nonzero float-computed quantity (proofbound.SumSlop, proofbound.ChainLengthBound), so it can never
// arrive here claiming Exact.
func exactnessOf(bound float64) Exactness {
	if bound == 0 {
		return Exact
	}
	return Approximate
}
