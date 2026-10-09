package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"

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
		return proofbound.AbsSumUpper(
			vecL1(pl.frame.Origin()),
			proofbound.ProductUpper(vecL1(pl.frame.U()), coordUpper),
			proofbound.ProductUpper(vecL1(pl.frame.V()), coordUpper),
			proofbound.ProductUpper(vecL1(pl.frame.N()), zUpper),
		)
	case draftPayload:
		// Every point of a draft body lies on the near section, the far
		// section, or a straight ruling or cone generator between matching
		// points of the two, so each plane coordinate is bounded by the larger
		// of the two sections' bounds, the far one widened by its contour
		// displacement (docs/draft-design.md Table DD row DD17). Each section
		// is read segment by segment (draftSectionCoordinateUpper), not from
		// the walks' L1 envelopes.
		nearUpper, err := draftSectionCoordinateUpper(pl.profile)
		if err != nil {
			return math.Inf(1)
		}
		farUpper, err := draftSectionCoordinateUpper(pl.far)
		if err != nil {
			return math.Inf(1)
		}
		coordUpper := math.Max(nearUpper, proofbound.AbsSumUpper(farUpper, pl.farDelta))
		zUpper := proofbound.AbsSumUpper(math.Max(math.Abs(pl.z0), math.Abs(pl.z1)), pl.axialDelta())
		return proofbound.AbsSumUpper(
			vecL1(pl.frame.Origin()),
			proofbound.ProductUpper(vecL1(pl.frame.U()), coordUpper),
			proofbound.ProductUpper(vecL1(pl.frame.V()), coordUpper),
			proofbound.ProductUpper(vecL1(pl.frame.N()), zUpper),
		)
	case revolvePayload:
		coordUpper, err := momentinput.CoordinateUpper(pl.profile, freeform.NewFreeformWork(), nil)
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

// draftSectionCoordinateUpper bounds max(|u|, |v|) over every point of a
// recorded section, segment by segment: a line by its recorded ends and an
// arc by its recorded extent (capband.SegmentCoordinateUpper, the reading
// capband.CoordUpper takes), a circle by its centre's larger coordinate plus
// its radius with the radius's own bound, and anything else by its walk's
// envelope, whichever is smallest.
func draftSectionCoordinateUpper(profile ProfileRecord) (float64, error) {
	work := freeform.NewFreeformWork()
	upper := 0.0
	for _, loop := range append([]LoopRecord{profile.Outer}, profile.Holes...) {
		for _, seg := range loop.Segments {
			w, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				return 0, err
			}
			segUpper := w.CoordUpper
			if local, ok := capband.SegmentCoordinateUpper(seg); ok {
				segUpper = math.Min(segUpper, local)
			}
			if w.IsCircular() && !proofbound.IsNonFinite(w.RadiusBound) {
				segUpper = math.Min(segUpper, proofbound.AbsSumUpper(math.Max(math.Abs(w.CU), math.Abs(w.CV)), w.Radius, w.RadiusBound))
			}
			upper = math.Max(upper, segUpper)
		}
	}
	return upper, nil
}
