package prismrecord

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Payload implements topo.Payload: Transform reads the accumulated placement
// and Placed rebuilds the record under a composed one through Build
// (docs/evaluator-design.md §8).

// AxialDelta is the larger of the two ends' displacements: the figure a reading
// that cannot attribute its error to one particular end takes.
func (pp Payload) AxialDelta() float64 { return math.Max(pp.Z0Delta, pp.Z1Delta) }

// Transform is the accumulated rigid placement.
func (pp Payload) Transform() r3.Transform { return pp.Xform }

// Placed re-evaluates the same record under the composed motion. It is a
// re-evaluation path: no moments preflight has run on this record within the
// call, so the build opens the record's one counter itself.
//
// The motion is the ONLY thing it changes. Everything the payload carries about
// the section — the record, its displacement, the blend roles and the walk
// resolution — travels unchanged, which is what lets the build read the walks
// back instead of bracketing every free-form arc a second time. That counter is
// still charged what the resolution cost, so a record near its ceiling refuses
// here exactly as it did on the way in (docs/spline-design.md §5.2).
func (pp Payload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	pp.Xform = composed
	return Build(ctx, env, pp, freeform.NewFreeformWork())
}
