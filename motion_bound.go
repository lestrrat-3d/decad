package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/motionbound"

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
// generator envelope and axis anchor revolvemass.Centroid already
// bounds a rotated material point with. Any other payload states no record
// radius and answers +Inf, which motionbound.PoseDeviation charges only where the pose's
// linear part departs from the ideal one.
func moverRecordRadius(ctx context.Context, b *Body) float64 {
	if ctx.Err() != nil {
		return math.Inf(1)
	}
	switch pl := b.payload.(type) {
	case prismPayload:
		coordUpper, err := momentinput.CoordinateEnvelope(pl.profile, freeform.NewFreeformWork(), pl.walks)
		if err != nil {
			return math.Inf(1)
		}
		coordUpper = proofbound.AbsSumUpper(coordUpper, pl.sectionDelta)
		zUpper := proofbound.AbsSumUpper(math.Max(math.Abs(pl.z0), math.Abs(pl.z1)), pl.axialDelta())
		return motionbound.LinearRecordRadius(pl.frame, coordUpper, zUpper)
	case draftPayload:
		return draftMoverRecordRadius(pl)
	case twoSidedDraftPayload:
		return math.Max(draftMoverRecordRadius(pl.negative), draftMoverRecordRadius(pl.positive))
	case revolvePayload:
		coordUpper, err := momentinput.CoordinateUpper(pl.profile, freeform.NewFreeformWork(), nil)
		if err != nil {
			return math.Inf(1)
		}
		coordUpper = proofbound.AbsSumUpper(coordUpper, pl.sectionDelta)
		return motionbound.RevolveRecordRadius(pl.frame, coordUpper, pl.ax.AU, pl.ax.AUBound, pl.ax.AV, pl.ax.AVBound)
	default:
		return math.Inf(1)
	}
}

// Every draft point lies on an end section or a ruling between them. Bound
// both section coordinate envelopes, then include the far contour's gap.
func draftMoverRecordRadius(pl draftPayload) float64 {
	nearUpper, err := motionbound.DraftSectionCoordinateUpper(pl.profile)
	if err != nil {
		return math.Inf(1)
	}
	farUpper, err := motionbound.DraftSectionCoordinateUpper(pl.far)
	if err != nil {
		return math.Inf(1)
	}
	coordUpper := math.Max(nearUpper, proofbound.AbsSumUpper(farUpper, pl.farDelta))
	zUpper := proofbound.AbsSumUpper(math.Max(math.Abs(pl.z0), math.Abs(pl.z1)), pl.axialDelta())
	return motionbound.LinearRecordRadius(pl.frame, coordUpper, zUpper)
}
