package revolvemass

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// WallAxisMoment is the first moment ∫ρ ds of one boundary walk about the
// axis. Pappus's first theorem reads the side face's area from it: a straight
// walk uses length times mean radius, a circular walk uses its closed-form
// antiderivative, a free-form walk uses certified length and radial envelopes,
// and an on-axis walk sweeps nothing.
//
// The straight arm composes the walk's bounded length and radial coordinates.
// Its magnitude envelope can only shrink the published bound. The circular
// arm uses the held axis-frame closed form; a rational interval over the
// recorded segments can shrink its magnitude-envelope bound. An unbracketed
// segment leaves that envelope as the only proof.
func WallAxisMoment(w survey2d.SegmentWalk, kind revolveaxis.WallKind, segs []sectionrecord.CurveSegment, ax revolveaxis.Frame) proofbound.BoundedScalar {
	if kind == revolveaxis.WallAxis {
		return proofbound.BoundedScalar{}
	}
	if kind == revolveaxis.WallFreeform {
		lengthLower, _ := proofbound.BoundedEnds(proofbound.MeasuredScalar(w.Length, w.LengthBound))
		lower := 0.0
		if lengthLower > 0 && ax.RadialLower > 0 {
			lower = math.Max(0, math.Nextafter(lengthLower*ax.RadialLower, math.Inf(-1)))
		}
		upper := w.AxisMomentUpper
		value := lower + (upper-lower)/2
		bound := proofbound.UpRound(math.Max(value-lower, upper-value))
		return proofbound.MeasuredScalar(value, bound)
	}
	if !w.IsCircular() {
		meanRadius := proofbound.BoundedDiv(
			proofbound.BoundedAdd(proofbound.MeasuredScalar(w.StartV, w.StartVBound), proofbound.MeasuredScalar(w.EndV, w.EndVBound)),
			proofbound.ExactScalar(2),
		)
		result := proofbound.BoundedMul(proofbound.MeasuredScalar(w.Length, w.LengthBound), meanRadius)
		result.Bound = math.Min(result.Bound, proofbound.ConservativeValueError(result.Value, w.AxisMomentUpper))
		return result
	}
	lo, hi := math.Min(w.Th0, w.Th1), math.Max(w.Th0, w.Th1)
	dtheta := proofbound.BoundedSub(proofbound.ExactScalar(hi), proofbound.ExactScalar(lo))
	cosDelta := proofbound.BoundedSub(proofbound.BoundedCos(proofbound.ExactScalar(lo)), proofbound.BoundedCos(proofbound.ExactScalar(hi)))
	result := proofbound.BoundedMul(
		proofbound.ExactScalar(w.Radius),
		proofbound.BoundedAdd(
			proofbound.BoundedMul(proofbound.ExactScalar(w.CV), dtheta),
			proofbound.BoundedMul(proofbound.ExactScalar(w.Radius), cosDelta),
		),
	)
	result.Bound = proofbound.ConservativeValueError(result.Value, w.AxisMomentUpper)
	if enc, ok := circularAxisMomentTotal(segs, ax); ok {
		result.Bound = math.Min(result.Bound, proofbound.IntervalFloatError(enc, result.Value))
	}
	return result
}

// ChargedWallAxisMoment includes the displacement of a whole meridian from
// its recorded circular wall. Cut constructions charge their own endpoints.
func ChargedWallAxisMoment(w survey2d.SegmentWalk, kind revolveaxis.WallKind,
	segs []sectionrecord.CurveSegment, ax revolveaxis.Frame, whole bool, delta float64) proofbound.BoundedScalar {
	m := WallAxisMoment(w, kind, segs, ax)
	if revolveaxis.SectionWholeCharges(whole, delta).U != 0 && w.IsCircular() && kind != revolveaxis.WallAxis {
		allow := revolveaxis.WallMomentAllow(true, delta, w.LengthUpper, w.AxisRadiusUpper)
		m.Bound = proofbound.AbsSumUpper(m.Bound, allow)
	}
	return m
}

// circularAxisMomentTotal sums the bounds over the recorded segments of a
// circular wall. Every segment must have an enclosure to prove the total.
func circularAxisMomentTotal(segs []sectionrecord.CurveSegment, ax revolveaxis.Frame) (proofbound.RatInterval, bool) {
	if len(segs) == 0 {
		return proofbound.RatInterval{}, false
	}
	frame := circularbounds.NewAxisFrame(ax.AU, ax.AV, ax.AUBound, ax.AVBound, ax.DU, ax.DV, ax.DUBound, ax.DVBound)
	total, ok := circularbounds.AxisMomentInterval(circularbounds.RecordSegment(segs[0]), frame)
	if !ok {
		return proofbound.RatInterval{}, false
	}
	for _, seg := range segs[1:] {
		enc, ok := circularbounds.AxisMomentInterval(circularbounds.RecordSegment(seg), frame)
		if !ok {
			return proofbound.RatInterval{}, false
		}
		total = proofbound.IntervalAdd(total, enc)
	}
	return total, true
}
