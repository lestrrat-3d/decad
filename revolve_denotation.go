package decad

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// phi0Delta and phi1Delta are the proven per-end displacements this payload's
// held sweep carries — the angular twins of prismPayload.z0Delta/z1Delta,
// read off the payload's own denotation against its own held float rather
// than stored redundantly, since the payload already carries both.
func (rp revolvePayload) phi0Delta() float64 { return rp.den.Phi0.Delta(rp.phi0) }
func (rp revolvePayload) phi1Delta() float64 { return rp.den.Phi1.Delta(rp.phi1) }

// angularDelta is the larger of the two ends' displacements: the figure a
// reading that cannot attribute its error to one particular end takes,
// mirroring prismPayload.axialDelta().
func (rp revolvePayload) angularDelta() float64 { return math.Max(rp.phi0Delta(), rp.phi1Delta()) }

// sweep is the proven bound on the sweep width every mass and edge reading
// multiplies into its own quantity: the held float64 subtraction stays the
// published value exactly as it always has, and the bound is the smaller of
// the magnitude envelope proofbound.ConservativeValueError has always published here
// (internal/proofbound/bounded.go's own documented fallback, sound for any legal sweep since a
// legal sweep never exceeds a full turn) and the proven displacement between
// that held value and the sweep the record DENOTES (den.WidthInterval,
// above), wherever a denotation exists for both ends. math.Min follows
// internal/proofbound/bounded.go's own convention for a reading a certified bracket admits — it
// can only shrink the published bound, never widen it — so a ToFaceAngular
// sweep or any other denotation this file cannot state keeps exactly the
// envelope it always published.
func (rp revolvePayload) sweep() proofbound.BoundedScalar {
	held := proofbound.BoundedSub(proofbound.ExactScalar(rp.phi1), proofbound.ExactScalar(rp.phi0))
	fallback := proofbound.ConservativeValueError(held.Value, proofbound.TwoPiUpper())
	enc, ok := rp.den.WidthInterval()
	if !ok {
		return proofbound.MeasuredScalar(held.Value, fallback)
	}
	return proofbound.MeasuredScalar(held.Value, math.Min(fallback, proofbound.IntervalFloatError(enc, held.Value)))
}
