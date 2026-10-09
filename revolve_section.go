package decad

import (
	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemass"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file is the section displacement's reach into a revolve's readings
// (docs/surface-intersection-design.md §7.2): how far a reading taken over the
// RECORDED meridian can sit from the same reading over the meridian the record
// denotes, when every recorded coordinate sits within revolvePayload.sectionDelta
// of it. The region readings — volume, the centroid's moments and a cap's area —
// take one term for any construction. The per-walk readings — a wall's area,
// a vertex, a wall's denoted normal — take the whole-section terms only where
// revolvePayload.sectionWhole says every coordinate moved; a cut construction
// charges its own cut ends through trimRevolveSegmentCharges instead.

// revolveSectionCharge is the region readings' share of a section
// displacement: Band encloses the area of the symmetric difference between the
// recorded region and the denoted one, and Envelope bounds |z| and |ρ| over that
// difference, so each axis-frame region integral moves by at most its
// integrand's envelope times band. The zero value charges nothing.
type revolveSectionCharge = revolveaxis.SectionCharge

// revolveSectionChargeOf reads rp's region charge. The symmetric difference
// lies inside the sectionDelta-neighbourhood of the recorded boundary, which
// proofbound.SectionDisplacementArea encloses from the boundary's proven
// perimeter and its segment count. Every point of that neighbourhood moves
// each plane-local coordinate by at most sectionDelta, so
// revolveaxis.SectionCoordUpper widens the recorded coordinate envelope over
// it, and axisFrame.radialUpper turns that into a bound on the distance from
// the axis anchor, which bounds |z| and |ρ| alike for a unit axis direction.
func revolveSectionChargeOf(rp revolvePayload, work *freeform.FreeformWork) (revolveSectionCharge, error) {
	delta := rp.sectionDelta
	if delta == 0 {
		return revolveSectionCharge{}, nil
	}
	coordUpper, err := momentinput.CoordinateUpper(rp.profile, work, nil)
	if err != nil {
		return revolveSectionCharge{}, err
	}
	perimeter := 0.0
	segments := 0
	for _, loop := range append([]loopRecord{rp.profile.Outer}, rp.profile.Holes...) {
		for _, seg := range loop.Segments {
			w, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				return revolveSectionCharge{}, err
			}
			perimeter = proofbound.AbsSumUpper(perimeter, w.Length, w.LengthBound)
			segments++
		}
	}
	return revolveSectionCharge{
		Band:     proofbound.SectionDisplacementArea(delta, segments, perimeter),
		Envelope: rp.ax.radialUpper(revolveaxis.SectionCoordUpper(coordUpper, delta)),
	}, nil
}

// denotedPoint is a recorded swept point widened to the point it denotes.
func (rp revolvePayload) denotedPoint(p sweptPoint) sweptPoint {
	charge := revolveaxis.SectionWholeCharges(rp.sectionWhole, rp.sectionDelta)
	p.bound = revolveaxis.DenotedBound(p.bound, charge)
	return p
}

// chargedWalk re-expresses one recorded segment's plane-local walk w in axis
// coordinates with the payload's section displacement folded in: a cut
// construction's own cut ends (trimRevolveSegmentCharges), or every end and,
// for an arc, its centre, radius and length where the whole section moved.
func (rp revolvePayload) chargedWalk(seg curveSegment, w survey2d.SegmentWalk) (survey2d.SegmentWalk, error) {
	if c := revolveaxis.SectionWholeCharges(rp.sectionWhole, rp.sectionDelta); c.U != 0 {
		walk := rp.ax.walkCharged(w, c, c)
		return revolveaxis.ChargeWholeWalk(walk, rp.sectionDelta, rp.ax.dU, rp.ax.dV), nil
	}
	startCharge, endCharge, err := trimRevolveSegmentCharges(seg, rp.sectionDelta)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	return rp.ax.walkCharged(w, startCharge, endCharge), nil
}

// wallMoment is revolvemass.WallAxisMoment over one wall, carried to the denoted wall
// where the whole section moved (revolveaxis.WallMomentAllow).
func (rp revolvePayload) wallMoment(w survey2d.SegmentWalk, kind wallKind, segs []curveSegment) proofbound.BoundedScalar {
	m := revolvemass.WallAxisMoment(w, kind, segs, rp.ax.numeric())
	if revolveaxis.SectionWholeCharges(rp.sectionWhole, rp.sectionDelta).U != 0 && w.IsCircular() && kind != wallAxis {
		allow := revolveaxis.WallMomentAllow(true, rp.sectionDelta, w.LengthUpper, w.AxisRadiusUpper)
		m.Bound = proofbound.AbsSumUpper(m.Bound, allow)
	}
	return m
}
