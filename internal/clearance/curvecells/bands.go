package curvecells

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// bandAngles are the azimuths a band reading samples for its upper bound: a
// uniform set, plus an arc's window ends and midpoint.
func bandAngles(e *clearance.CEdge) []float64 {
	const uniform = 16
	angles := make([]float64, 0, uniform+3)
	for i := range uniform {
		angles = append(angles, 2*math.Pi*float64(i)/uniform)
	}
	if !e.Ang.Full {
		angles = append(angles, e.Ang.Lo, (e.Ang.Lo+e.Ang.Hi)/2, e.Ang.Hi)
	}
	return angles
}

// circleAxisBand bounds the distance from the points of a circle of radius
// rho to a line, given delta, the distance from the circle's centre to the
// line, and s, the tilt sine of the circle's axis and the line's direction.
// Every point is c + rho·u for a unit u ⊥ axis, so its distance from the line
// is |P(c − a₀) + rho·P(u)|, P the projection off the line, and |P(u)| lies in
// [√(1 − s²), 1]: the distance lies in [rho·√(1 − s²) − delta, rho + delta].
// Both ends are rounded outward; the caller charges the floats it read.
func circleAxisBand(rho, delta, s float64) (float64, float64) {
	lo := freeform.DownRound(freeform.DownRound(rho*clearance.TiltCosine(s)) - delta)
	return lo, proofbound.AbsSumUpper(rho, delta)
}

// windowedCircleEE is the circle pair band of
// agent-docs/clearance-band-design.md §4.1: two circular edges whose spine
// pair has no closed form. Every point q of edge b lies at least
// √(max(0, H)² + max(0, R)²) from the full circle of edge a, with
// H = |h₀| − ρ_b·s the least height of q along a's axis and
// R = max(ρ_a − hi, lo − ρ_a) the least radial gap, [lo, hi] being
// circleAxisBand over a's axis and b's centre. The reading takes the larger of
// the two role orders. The upper bound is the nearest admitted pair of 16
// uniform samples of b (and its window ends) with their radial images on a.
// ok is false when the band does not clear the tolerance.
func (k *Kernel) windowedCircleEE(ea, eb *clearance.CEdge) (float64, float64, bool) {
	charge := clearance.EnvelopeCharge([]r3.Vec{ea.Center, eb.Center}, ea.Radius, eb.Radius)
	s := clearance.TiltSine(ea.Axis, eb.Axis)
	lo := math.Max(k.circlePairLower(ea, eb, s, charge), k.circlePairLower(eb, ea, s, charge))
	if proofbound.IsNonFinite(lo) || !(lo > k.tol) {
		return 0, 0, false
	}
	best := math.Inf(1)
	for _, th := range bandAngles(eb) {
		if clearance.CircleAngleAdmit(eb, th, k.tol) != 1 {
			continue
		}
		q := eb.At(th)
		rel := q.Sub(ea.Center)
		u, ok := rel.Sub(ea.Axis.Scale(rel.Dot(ea.Axis))).Normalize()
		if !ok || clearance.CircleAngleAdmit(ea, clearance.AngleOf(ea, u), k.tol) != 1 {
			continue
		}
		best = math.Min(best, q.Sub(ea.Center.Add(u.Scale(ea.Radius))).Len())
	}
	if math.IsInf(best, 1) {
		return lo, best, true
	}
	return lo, proofbound.AbsSumUpper(best, charge), true
}

// circlePairLower is the lower bound of every point of b to the full circle of
// a (§4.1), each of its two terms charged before the square.
func (k *Kernel) circlePairLower(a, b *clearance.CEdge, s, charge float64) float64 {
	rel := b.Center.Sub(a.Center)
	h0 := rel.Dot(a.Axis)
	delta := rel.Sub(a.Axis.Scale(h0)).Len()
	height := freeform.DownRound(freeform.DownRound(math.Abs(h0)-proofbound.ProductUpper(b.Radius, s)) - charge)
	bandLo, bandHi := circleAxisBand(b.Radius, delta, s)
	radial := math.Max(
		freeform.DownRound(freeform.DownRound(a.Radius-bandHi)-charge),
		freeform.DownRound(freeform.DownRound(bandLo-a.Radius)-charge))
	return freeform.DownRound(math.Hypot(math.Max(0, height), math.Max(0, radial)))
}

// tiltedRimOverPlane is the rim height band of
// agent-docs/clearance-band-design.md §4.3: a circular edge of radius ρ, centre
// c and unit axis m over a plane face of unit normal n whose parallelism the
// oracle cannot decide. Every point of the circle is c + ρ·u with u ⊥ m, so
// |u·n| ≤ s and its height over the plane lies in [b − ρ·s, b + ρ·s], b the
// centre's height. When |b| − ρ·s clears the tolerance after its charge, the
// whole circle lies strictly on one side of the plane and every point pair is
// at least that far apart. The upper bound is the least height of an admitted
// sample whose foot lies in the face's trim. ok is false when the band does
// not clear.
func (k *Kernel) tiltedRimOverPlane(f *clearance.CFace, e *clearance.CEdge) (float64, float64, bool) {
	charge := clearance.EnvelopeCharge([]r3.Vec{e.Center, f.O}, e.Radius)
	s := clearance.TiltSine(e.Axis, f.N)
	base := clearance.Height(e.Center, f.O, f.N).Abs().Lo
	lo := freeform.DownRound(freeform.DownRound(base-proofbound.ProductUpper(e.Radius, s)) - charge)
	if proofbound.IsNonFinite(lo) || !(lo > k.tol) {
		return 0, 0, false
	}
	best := math.Inf(1)
	for _, th := range bandAngles(e) {
		if clearance.CircleAngleAdmit(e, th, k.tol) != 1 {
			continue
		}
		pe := e.At(th)
		height := pe.Sub(f.O).Dot(f.N)
		x, y := f.PlaneCoords(pe.Sub(f.N.Scale(height)))
		if f.Region.Classify(x, y, k.tol) != 1 {
			continue
		}
		best = math.Min(best, clearance.Height(pe, f.O, f.N).Abs().Hi)
	}
	if math.IsInf(best, 1) {
		return lo, best, true
	}
	return lo, proofbound.AbsSumUpper(best, charge), true
}
