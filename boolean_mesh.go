package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshbool"
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
	return booleanEvaluation{payload: ev.Payload, volume: measurement.CubicMillimeters(ev.Volume, ev.VolumeBound),
		volumeRat: ev.VolumeRat, audit: ev.Audit}, nil
}
