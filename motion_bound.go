package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/motionbound"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// This file converts the requested motion into motionbound.MotionFrame and
// reads a mover's payload-specific record radius. internal/motionbound owns
// the exact box corners, axis radius, swept box and interval calculations
// described in docs/motion-check-design.md §5–§6.

func newMotionFrame(spec motionSpec) (motionbound.MotionFrame, bool) {
	dirVec := spec.dir
	center := r3.Vec{}
	switch spec.kind {
	case motionbound.MotionRevolute:
		dirVec, center = spec.axis, spec.center
	case motionbound.MotionBetween:
		dirVec, center = spec.screw.Axis, spec.screw.Point
	}
	axis, okA := motionbound.RatVecOf(dirVec)
	pivot, okC := motionbound.RatVecOf(center)
	if !okA || !okC {
		return motionbound.MotionFrame{}, false
	}
	unit, ok := motionbound.UnitScaleInterval(axis)
	if !ok {
		return motionbound.MotionFrame{}, false
	}
	mf := motionbound.MotionFrame{Kind: spec.kind, Axis: axis, Unit: unit, Center: pivot}
	if spec.kind != motionbound.MotionBetween {
		return mf, true
	}
	theta, okT := motionbound.ExactMotionParam(spec.screw.Angle)
	slide := proofarith.FloatRat(spec.screw.Slide)
	fromRot, fromT, okF := motionbound.ExactTransform(spec.between.From)
	toRot, toT, okTo := motionbound.ExactTransform(spec.between.To)
	if !okT || slide == nil || !okF || !okTo {
		return motionbound.MotionFrame{}, false
	}
	mf.Theta, mf.Slide = theta, slide
	mf.FromRot, mf.FromT, mf.ToRot, mf.ToT = fromRot, fromT, toRot, toT
	return mf, true
}

// moverRecordRadius is R0 of docs/motion-check-design.md §5.1: a proven upper
// bound on |p| for every point p of the mover's record before its own
// placement applies, read off the payload's own envelopes. It is an L1 bound,
// which dominates the Euclidean one. A prism reads its profile coordinate
// envelope (prism_payload.go) widened by its section displacement, and its
// sweep levels widened by their axial displacement; a revolve reads the
// generator envelope and axis anchor revolveCentroidGeometryBound already
// bounds a rotated material point with. Any other payload states no record
// radius and answers +Inf, which motionbound.PoseDeviation charges only where the pose's
// linear part departs from the ideal one.
func moverRecordRadius(ctx context.Context, b *Body) float64 {
	if ctx.Err() != nil {
		return math.Inf(1)
	}
	switch pl := b.payload.(type) {
	case prismPayload:
		coordUpper, err := profileCoordinateEnvelope(pl.profile, freeform.NewFreeformWork(), pl.walks)
		if err != nil {
			return math.Inf(1)
		}
		coordUpper = proofbound.AbsSumUpper(coordUpper, pl.sectionDelta)
		zUpper := proofbound.AbsSumUpper(math.Max(math.Abs(pl.z0), math.Abs(pl.z1)), pl.axialDelta())
		return proofbound.AbsSumUpper(
			vecL1(pl.frame.Origin()),
			proofbound.ProductUpper(vecL1(pl.frame.U()), coordUpper),
			proofbound.ProductUpper(vecL1(pl.frame.V()), coordUpper),
			proofbound.ProductUpper(vecL1(pl.frame.N()), zUpper),
		)
	case revolvePayload:
		coordUpper, err := profileCoordinateUpper(pl.profile, freeform.NewFreeformWork(), nil)
		if err != nil {
			return math.Inf(1)
		}
		coordUpper = proofbound.AbsSumUpper(coordUpper, pl.sectionDelta)
		originUpper := vecL1(pl.frame.Origin())
		profileUpper := proofbound.AbsSumUpper(
			originUpper,
			proofbound.ProductUpper(vecL1(pl.frame.U()), coordUpper),
			proofbound.ProductUpper(vecL1(pl.frame.V()), coordUpper),
		)
		axisUpper := proofbound.AbsSumUpper(
			originUpper,
			proofbound.ProductUpper(vecL1(pl.frame.U()), proofbound.AbsSumUpper(pl.ax.aU, pl.ax.aUBound)),
			proofbound.ProductUpper(vecL1(pl.frame.V()), proofbound.AbsSumUpper(pl.ax.aV, pl.ax.aVBound)),
		)
		return proofbound.AbsSumUpper(proofbound.ProductUpper(3, profileUpper), proofbound.ProductUpper(4, axisUpper))
	default:
		return math.Inf(1)
	}
}
