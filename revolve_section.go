package decad

import (
	"github.com/lestrrat-3d/decad/internal/freeform"
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
// displacement: band encloses the area of the symmetric difference between the
// recorded region and the denoted one, and env bounds |z| and |ρ| over that
// difference, so each axis-frame region integral moves by at most its
// integrand's envelope times band. The zero value charges nothing.
type revolveSectionCharge struct {
	band float64
	env  float64
}

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
	coordUpper, err := profileCoordinateUpper(rp.profile, work, nil)
	if err != nil {
		return revolveSectionCharge{}, err
	}
	perimeter := 0.0
	segments := 0
	for _, loop := range append([]LoopRecord{rp.profile.Outer}, rp.profile.Holes...) {
		for _, seg := range loop.Segments {
			w, err := walkOf(seg, work)
			if err != nil {
				return revolveSectionCharge{}, err
			}
			perimeter = proofbound.AbsSumUpper(perimeter, w.Length, w.LengthBound)
			segments++
		}
	}
	return revolveSectionCharge{
		band: proofbound.SectionDisplacementArea(delta, segments, perimeter),
		env:  rp.ax.radialUpper(revolveaxis.SectionCoordUpper(coordUpper, delta)),
	}, nil
}

// first is the charge on ∫ρ dA, and so on the volume's integrand.
func (c revolveSectionCharge) first() float64 { return proofbound.ProductUpper(c.env, c.band) }

// second is the charge on ∫zρ dA and on ∫ρ² dA alike.
func (c revolveSectionCharge) second() float64 {
	return proofbound.ProductUpper(proofbound.ProductUpper(c.env, c.env), c.band)
}

// sectionWholeCharges is the plane-local displacement every recorded endpoint
// of a whole-section payload carries: sectionDelta in each component, the box
// that holds the disk of that radius. A cut construction charges its own cut
// ends (trimRevolveSegmentCharges) and every other payload charges nothing.
func (rp revolvePayload) sectionWholeCharges() proofbound.WalkEndBound {
	if !rp.sectionWhole || rp.sectionDelta == 0 {
		return proofbound.WalkEndBound{}
	}
	return proofbound.WalkEndBound{U: rp.sectionDelta, V: rp.sectionDelta}
}

// denotedBound widens a recorded point's own bound by the whole-section
// displacement, so a comparison against the recorded point (a swept vertex, a
// wall's denoted normal) encloses the point the record denotes.
func (rp revolvePayload) denotedBound(b proofbound.WalkEndBound) proofbound.WalkEndBound {
	c := rp.sectionWholeCharges()
	if c.U == 0 {
		return b
	}
	return proofbound.WalkEndBound{U: proofbound.AbsSumUpper(b.U, c.U), V: proofbound.AbsSumUpper(b.V, c.V)}
}

// chargeWholeWalk finishes a whole-section charge on one axis-coordinate walk
// beside the per-endpoint fold axisFrame.walkCharged already took. A circular
// walk moves more than its two ends: its centre moves by sectionDelta and its
// radius by up to twice it, so its length moves by up to 12·π·sectionDelta
// (proofbound.SectionDisplacementLength's per-walk figure) and its centre's
// radial coordinate by the centre's own axis charge.
func (rp revolvePayload) chargeWholeWalk(w survey2d.SegmentWalk) survey2d.SegmentWalk {
	c := rp.sectionWholeCharges()
	if c.U == 0 || !w.IsCircular() {
		return w
	}
	grow := proofbound.SectionDisplacementLength(rp.sectionDelta, 1)
	w.LengthBound = proofbound.AbsSumUpper(w.LengthBound, grow)
	w.LengthUpper = proofbound.AbsSumUpper(w.LengthUpper, grow)
	w.CVBound = proofbound.AbsSumUpper(w.CVBound, proofbound.ProductUpper(rp.sectionDelta, proofbound.AbsSumUpper(rp.ax.dU, rp.ax.dV)))
	w.RadiusBound = proofbound.AbsSumUpper(w.RadiusBound, proofbound.ProductUpper(2, rp.sectionDelta))
	w.AxisMomentUpper = proofbound.ProductUpper(w.LengthUpper, w.AxisRadiusUpper)
	return w
}

// wallMomentAllow bounds how far a wall's boundary moment ∫ρ ds over its
// recorded walk can sit from the same moment over the walk it denotes, when
// the walk's ends (and an arc's centre) each sit within delta of the denoted
// ones. Parameterise both walks at constant speed over [0, 1]; then
//
//	|M − M*| ≤ |L − L*|·max|ρ| + L*·max|γ(s) − γ*(s)|
//
// since ρ is 1-Lipschitz in the plane. A line's ends move by delta, so its
// length moves by at most 2·delta and its points by at most delta. An arc's
// length moves by at most 12·π·delta (proofbound.SectionDisplacementLength);
// its points move by at most the centre's delta, the radius's 2·delta and the
// angle's sweep at the denoted radius, which the ends' 4·delta chord bounds by
// 2·π·delta while the denoted radius is at least 4·delta and by 8·delta
// outright below it — 12·delta in every case. lengthUpper must bound both
// lengths and rhoUpper the recorded walk's radial envelope.
func wallMomentAllow(circular bool, delta, lengthUpper, rhoUpper float64) float64 {
	if delta <= 0 {
		return 0
	}
	move, point := proofbound.ProductUpper(2, delta), delta
	if circular {
		move, point = proofbound.SectionDisplacementLength(delta, 1), proofbound.ProductUpper(12, delta)
	}
	return proofbound.AbsSumUpper(
		proofbound.ProductUpper(move, rhoUpper),
		proofbound.ProductUpper(proofbound.AbsSumUpper(lengthUpper, move), point),
	)
}

// wholeArcMomentAllow is the circular wall's own whole-section charge on its
// area moment: walkAxisMoment's arc arm encloses the RECORDED arc's moment,
// both its envelope and its rational closed form read the recorded segment,
// and this term carries that enclosure to the denoted arc. A line wall needs
// none: its moment is composed from the endpoint bounds walkCharged already
// widened.
func (rp revolvePayload) wholeArcMomentAllow(w survey2d.SegmentWalk) float64 {
	if rp.sectionWholeCharges().U == 0 || !w.IsCircular() {
		return 0
	}
	return wallMomentAllow(true, rp.sectionDelta, w.LengthUpper, w.AxisRadiusUpper)
}

// revolveSectionAreaAllow is the section displacement's charge on a revolve
// mesh's area slack: every wall's moment displacement over the sweep, and
// each partial solid cap's band. Lines and arcs alike take wallMomentAllow
// here, since the mesh's slack is against the denoted surface and composes no
// endpoint bound of its own.
func revolveSectionAreaAllow(p *revolvePlan, charge revolveSectionCharge) float64 {
	rp := p.rp
	if rp.sectionDelta == 0 {
		return 0
	}
	sweepUpper := proofbound.AbsSumUpper(p.sweep, rp.sweep().Bound)
	total := 0.0
	for _, r := range p.resolved {
		for i, w := range r.walks {
			if r.kinds[i] == wallAxis {
				continue
			}
			total = proofbound.AbsSumUpper(total, wallMomentAllow(w.IsCircular(), rp.sectionDelta, w.LengthUpper, w.AxisRadiusUpper))
		}
	}
	total = proofbound.ProductUpper(total, sweepUpper)
	if !rp.full && !rp.surfaceResult {
		total = proofbound.AbsSumUpper(total, proofbound.ProductUpper(2, charge.band))
	}
	return total
}

// revolveSectionVolumeAllow is the section displacement's charge on a revolve
// solid's occupied-volume bound: the recorded and denoted regions differ by
// the band, swept over the sweep at a radius no greater than the band's own
// envelope.
func revolveSectionVolumeAllow(p *revolvePlan, charge revolveSectionCharge) float64 {
	if p.rp.sectionDelta == 0 {
		return 0
	}
	return proofbound.ProductUpper(proofbound.AbsSumUpper(p.sweep, p.rp.sweep().Bound), charge.first())
}

// denotedPoint is a recorded swept point widened to the point it denotes.
func (rp revolvePayload) denotedPoint(p sweptPoint) sweptPoint {
	p.bound = rp.denotedBound(p.bound)
	return p
}

// chargedWalk re-expresses one recorded segment's plane-local walk w in axis
// coordinates with the payload's section displacement folded in: a cut
// construction's own cut ends (trimRevolveSegmentCharges), or every end and,
// for an arc, its centre, radius and length where the whole section moved.
func (rp revolvePayload) chargedWalk(seg CurveSegment, w survey2d.SegmentWalk) (survey2d.SegmentWalk, error) {
	if c := rp.sectionWholeCharges(); c.U != 0 {
		return rp.chargeWholeWalk(rp.ax.walkCharged(w, c, c)), nil
	}
	startCharge, endCharge, err := trimRevolveSegmentCharges(seg, rp.sectionDelta)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	return rp.ax.walkCharged(w, startCharge, endCharge), nil
}

// wallMoment is revolvemass.WallAxisMoment over one wall, carried to the denoted wall
// where the whole section moved (wholeArcMomentAllow).
func (rp revolvePayload) wallMoment(w survey2d.SegmentWalk, kind wallKind, segs []CurveSegment) proofbound.BoundedScalar {
	m := revolvemass.WallAxisMoment(w, kind, segs, rp.ax.numeric())
	if allow := rp.wholeArcMomentAllow(w); allow > 0 && kind != wallAxis {
		m.Bound = proofbound.AbsSumUpper(m.Bound, allow)
	}
	return m
}

// denotedRadiusBound widens a recorded circle's radius bound by twice the
// whole-section displacement: its centre and its ends each move by at most
// sectionDelta, so its radius moves by at most twice it.
func (rp revolvePayload) denotedRadiusBound(b float64) float64 {
	if rp.sectionWholeCharges().U == 0 {
		return b
	}
	return proofbound.AbsSumUpper(b, proofbound.ProductUpper(2, rp.sectionDelta))
}
