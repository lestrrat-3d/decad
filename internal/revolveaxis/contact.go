package revolveaxis

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// SnapAllow is one profile's accumulated snap allowance, one field per
// integral the revolve publishes a measurement from: area bounds Σ|Δ∫dA| (the
// cap's own reading), first bounds Σ|Δ∫ρ dA| (Pappus's second theorem, so the
// volume), mixed bounds Σ|Δ∫zρ dA| (the solid centroid's axial term) and second
// bounds Σ|Δ∫ρ² dA| (a partial sweep's in-plane centroid term). ρ and z are the
// AXIS-frame coordinates, which is what keeps the last three tight: a profile
// far down the axis has a large |z| and a small ρ, and charging both at one
// frame-origin envelope would inflate the volume by the axial offset.
type SnapAllow struct {
	Area   float64
	First  float64
	Mixed  float64
	Second float64
}

// add composes another walk's allowance into this one, each order summed
// outward. The integrals are additive over the boundary, so the region's total
// displacement is at most its walks' displacements summed.
func (a SnapAllow) add(b SnapAllow) SnapAllow {
	return SnapAllow{
		Area:   proofbound.AbsSumUpper(a.Area, b.Area),
		First:  proofbound.AbsSumUpper(a.First, b.First),
		Mixed:  proofbound.AbsSumUpper(a.Mixed, b.Mixed),
		Second: proofbound.AbsSumUpper(a.Second, b.Second),
	}
}

// snapAllowOf is ONE walk's contribution to the region snap allowance, and the
// place the charge is derived.
//
// Frame.Walk assigns an endpoint exactly 0 when the arithmetic put it
// within SnapTol of the axis, discarding a magnitude δ ≤ SnapTol. The built
// wall follows the snapped walk, the region integrals follow the recorded one,
// and the two curves bound a ribbon between them. Every point of that ribbon
// sits within δ of the recorded walk measured radially, so the ribbon lies in a
// band of width δ along a curve no longer than the longer of the two walks:
//
//	area(ribbon) ≤ δ · max(L, L') ≤ δ · (w.length + w.lengthBound)
//
// — w.lengthBound already carries both discarded magnitudes by the time this
// reads it (axisFrame.walk's own doc comment), so proofbound.AbsSumUpper over the pair
// covers whichever of the two is longer.
//
// The symmetric difference between the recorded region and the snapped one is
// contained in the union of those ribbons, so for any integrand f,
// |Δ∫f dA| ≤ area(ribbon) · sup|f| over the ribbon, and each order's charge is
// that product against the matching envelope: nothing for ∫dA, one radial
// envelope for ∫ρ dA, a radial and an axial one for ∫zρ dA, and two radial ones
// for ∫ρ² dA.
//
// Every step widens. proofbound.ProductUpper and proofbound.AbsSumUpper each round outward, δ is the
// discarded magnitude itself rather than an estimate of what it costs, and
// every sup|f| is replaced by an envelope that dominates it. A walk with
// nothing discarded contributes exactly zero, which is what leaves an
// axis-incident profile's published volume, cap area and centroid as proven as
// they were.
//
// A CIRCULAR walk with a snapped endpoint takes the same charge although its
// built surface keeps the recorded circle's own center, radius and angles: the
// snap still displaces the endpoint its neighbouring walls meet it at, by the
// same δ over the same walk, so the same ribbon dominates the difference.
func snapAllowOf(walked survey2d.SegmentWalk, discarded float64) SnapAllow {
	if !(discarded > 0) {
		return SnapAllow{}
	}
	ribbon := proofbound.ProductUpper(proofbound.AbsSumUpper(walked.Length, walked.LengthBound), discarded)
	rhoUp := walkRadialUpper(walked, discarded)
	// |z| = |(p−a)·d| ≤ |p−a| ≤ |p| + |a|, which is exactly what
	// Frame.RadialUpper composes — it is the envelope of the whole axis-
	// local position, so it bounds the axial coordinate as well as the radial
	// one, and for a profile far down the axis it is the axial one that is
	// large.
	zUp := proofbound.AbsSumUpper(walked.AxisRadiusUpper, discarded)
	first := proofbound.ProductUpper(ribbon, rhoUp)
	return SnapAllow{
		Area:   ribbon,
		First:  first,
		Mixed:  proofbound.ProductUpper(first, zUp),
		Second: proofbound.ProductUpper(first, rhoUp),
	}
}

// walkRadialUpper is a proven upper bound on |ρ| over one walk already
// re-expressed in axis coordinates, widened by the snap magnitude discarded
// along it. A straight walk's ρ runs linearly between its two endpoints, so its
// extremes ARE those endpoints, each read through the radial bound
// Frame.Walk proved for it; a circular walk reaches at most its center's
// radial coordinate plus its radius, each read through its own bound. The
// answer is capped by the walk's own axis-radius envelope, which is proven
// independently, so this can only ever tighten and never widen it.
func walkRadialUpper(w survey2d.SegmentWalk, discarded float64) float64 {
	held := proofbound.AbsSumUpper(
		math.Max(math.Abs(w.StartV), math.Abs(w.EndV)),
		math.Max(w.StartVBound, w.EndVBound),
		discarded,
	)
	if w.IsCircular() {
		held = proofbound.AbsSumUpper(math.Abs(w.CV), w.CVBound, w.Radius, w.RadiusBound, discarded)
	}
	return math.Min(held, proofbound.AbsSumUpper(w.AxisRadiusUpper, discarded))
}

// snapDiscarded is the largest radial magnitude Frame.Walk's snap discards
// over one walk's two endpoints, read from the walk BEFORE it was re-expressed
// — the same ToAxis reading and the same SnapTol comparison Walk itself makes,
// so the two can never disagree about whether an endpoint snapped.
func (ax Frame) snapDiscarded(w survey2d.SegmentWalk) float64 {
	var discarded float64
	for _, end := range [][2]float64{{w.StartU, w.StartV}, {w.EndU, w.EndV}} {
		_, rho := ax.ToAxis(end[0], end[1])
		if m := math.Abs(rho); m <= ax.SnapTol && m > discarded {
			discarded = m
		}
	}
	return discarded
}

// AuditAxisContact makes ONE pass over the profile's recorded walks for two
// jobs that both need every walk re-expressed in axis coordinates.
//
// It rejects the circular boundary walks a revolve cannot sweep soundly: a
// walk tangent to the axis at a point interior to
// the walk — the horn-torus contact §6 forbids — and a walk whose circle
// center lies across the axis, whose swept surface is a spindle-branch
// torus the shipped Torus (non-negative Major) cannot represent; the solid
// is valid, so that one is the staged ErrUnsupported, never a wrong face.
// For the tangency, the circle's radial minimum sits at its lowest angle;
// when that angle is strictly inside the walked range and the minimum
// reaches the axis, the contact is neither of the two allowed forms.
//
// It also sums the region snap allowance (snapAllowOf) the same walks earn,
// here rather than in a second pass of its own: re-walking a profile costs a
// second free-form conversion and a second rational bracket per segment for an
// answer this loop already holds.
func AuditAxisContact(
	ax Frame,
	loops []sectionrecord.LoopRecord,
	resolve func(sectionrecord.CurveSegment) (survey2d.SegmentWalk, error),
) (SnapAllow, error) {
	const angEps = 1e-9
	var snap SnapAllow
	for _, loop := range loops {
		for _, seg := range loop.Segments {
			w, err := resolve(seg)
			if err != nil {
				return SnapAllow{}, err
			}
			discarded := ax.snapDiscarded(w)
			w = ax.Walk(w)
			snap = snap.add(snapAllowOf(w, discarded))
			if !w.IsCircular() {
				continue
			}
			if w.CV < -ax.SnapTol {
				return SnapAllow{}, fmt.Errorf(`%w: a boundary arc centered across the revolve axis sweeps a spindle torus this evaluator cannot represent`, decaderr.ErrUnsupported)
			}
			if w.CV-w.Radius > ax.SnapTol {
				continue
			}
			lo, hi := math.Min(w.Th0, w.Th1), math.Max(w.Th0, w.Th1)
			if w.Closed {
				return SnapAllow{}, fmt.Errorf(`%w: a closed curve touching the revolve axis sweeps a self-touching solid`, decaderr.ErrDegenerate)
			}
			// The minimum-ρ angle is −π/2 modulo a full turn.
			for th := -math.Pi/2 + 2*math.Pi*math.Floor((lo+math.Pi/2)/(2*math.Pi)); th <= hi+angEps; th += 2 * math.Pi {
				if th > lo+angEps && th < hi-angEps {
					return SnapAllow{}, fmt.Errorf(`%w: the boundary touches the revolve axis at an interior point`, decaderr.ErrDegenerate)
				}
			}
		}
	}
	return snap, nil
}
