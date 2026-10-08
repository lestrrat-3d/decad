package revolveaxis

import (
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// SectionCharge bounds the area and coordinate envelope of the region between
// a recorded meridian and the meridian it denotes.
type SectionCharge struct {
	Band     float64
	Envelope float64
}

// First bounds the change in the first radial moment of the section.
func (c SectionCharge) First() float64 { return proofbound.ProductUpper(c.Envelope, c.Band) }

// Second bounds the change in its mixed and second radial moments.
func (c SectionCharge) Second() float64 {
	return proofbound.ProductUpper(proofbound.ProductUpper(c.Envelope, c.Envelope), c.Band)
}

// SectionWholeCharges gives each recorded endpoint the displacement of a
// whole-section construction. A cut construction charges its own cut ends.
func SectionWholeCharges(whole bool, delta float64) proofbound.WalkEndBound {
	if !whole || delta == 0 {
		return proofbound.WalkEndBound{}
	}
	return proofbound.WalkEndBound{U: delta, V: delta}
}

// DenotedBound widens a recorded point bound by a whole-section displacement.
func DenotedBound(b, charge proofbound.WalkEndBound) proofbound.WalkEndBound {
	if charge.U == 0 {
		return b
	}
	return proofbound.WalkEndBound{
		U: proofbound.AbsSumUpper(b.U, charge.U),
		V: proofbound.AbsSumUpper(b.V, charge.V),
	}
}

// ChargeWholeWalk widens a circular walk's length, centre and radius after
// its endpoint charges have been folded into axis coordinates.
func ChargeWholeWalk(w survey2d.SegmentWalk, delta, dU, dV float64) survey2d.SegmentWalk {
	if delta == 0 || !w.IsCircular() {
		return w
	}
	grow := proofbound.SectionDisplacementLength(delta, 1)
	w.LengthBound = proofbound.AbsSumUpper(w.LengthBound, grow)
	w.LengthUpper = proofbound.AbsSumUpper(w.LengthUpper, grow)
	w.CVBound = proofbound.AbsSumUpper(w.CVBound,
		proofbound.ProductUpper(delta, proofbound.AbsSumUpper(dU, dV)))
	w.RadiusBound = proofbound.AbsSumUpper(w.RadiusBound, proofbound.ProductUpper(2, delta))
	w.AxisMomentUpper = proofbound.ProductUpper(w.LengthUpper, w.AxisRadiusUpper)
	return w
}

// WallMomentAllow bounds the change in a wall's boundary moment when its
// recorded endpoints, and a circular walk's centre, each move by delta.
func WallMomentAllow(circular bool, delta, lengthUpper, rhoUpper float64) float64 {
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

// SectionAreaAllow bounds the mesh-area slack from displaced wall moments and
// both caps of a partial solid.
func SectionAreaAllow(delta, sweep, sweepBound float64, resolved []ResolvedWalks,
	partialSolid bool, charge SectionCharge) float64 {
	if delta == 0 {
		return 0
	}
	sweepUpper := proofbound.AbsSumUpper(sweep, sweepBound)
	total := 0.0
	for _, r := range resolved {
		for i, w := range r.Walks {
			if r.Kinds[i] == WallAxis {
				continue
			}
			total = proofbound.AbsSumUpper(total,
				WallMomentAllow(w.IsCircular(), delta, w.LengthUpper, w.AxisRadiusUpper))
		}
	}
	total = proofbound.ProductUpper(total, sweepUpper)
	if partialSolid {
		total = proofbound.AbsSumUpper(total, proofbound.ProductUpper(2, charge.Band))
	}
	return total
}

// SectionVolumeAllow bounds the swept volume of the displaced section band.
func SectionVolumeAllow(delta, sweep, sweepBound float64, charge SectionCharge) float64 {
	if delta == 0 {
		return 0
	}
	return proofbound.ProductUpper(proofbound.AbsSumUpper(sweep, sweepBound), charge.First())
}

// DenotedRadiusBound widens a recorded circular radius bound by twice the
// whole-section displacement of its centre and endpoints.
func DenotedRadiusBound(b float64, charge proofbound.WalkEndBound) float64 {
	if charge.U == 0 {
		return b
	}
	return proofbound.AbsSumUpper(b, proofbound.ProductUpper(2, charge.U))
}
