package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/topo"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
)

// This file is prismPayload — the record a straight extrude evaluates from —
// as the root wrapper of prismrecord.Payload. The coordinate readings taken
// directly off the record (Point, Dir, LiftedVertex, PointBound, LiftFactor,
// CircleCurveBound, CentroidGeometryBound) live in internal/prismrecord.

// prismPayload is the evaluator's own record of a prism body. Its plain data —
// section, frame, sweep levels and their displacements, placement, blend roles,
// surface flag and walk cache — is prismrecord.Payload, embedded by value so a
// copy of the wrapper copies the record exactly as before. The wrapper adds what
// needs the root package: Transform and Placed (topo.Payload), and the
// readings that satisfy root interfaces (AxialDelta, ExtentAlong). See
// docs/evaluator-design.md §8.
type prismPayload struct {
	prismrecord.Payload
}

// AxialDelta is the larger of the two ends' displacements: the figure a reading
// that cannot attribute its error to one particular end takes.
func (pp prismPayload) AxialDelta() float64 { return math.Max(pp.Z0Delta, pp.Z1Delta) }

func vecL1(v r3.Vec) float64 {
	return proofbound.AbsSumUpper(v.X, v.Y, v.Z)
}

// Transform is the accumulated rigid placement.
func (pp prismPayload) Transform() r3.Transform { return pp.Xform }

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
func (pp prismPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	d, ref := documentHandle(env.Doc), env.Ref
	pp.Xform = composed
	return placedNode(evalPrismContext(ctx, d, ref, pp, freeform.NewFreeformWork()))
}
