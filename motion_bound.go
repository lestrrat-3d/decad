package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file reads a mover's payload-specific record radius. internal/motionbound
// builds motion frames, box corners, axis radii, swept boxes and interval
// calculations described in docs/motion-check-design.md §5–§6.

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
