package decad

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"

	"github.com/lestrrat-3d/units"
)

// angleDenotation keeps the root payload's private field names while the
// internal angle proof owns its arithmetic.
type angleDenotation struct {
	rad, turn *big.Rat
	span      *proofbound.RatInterval
}

func (d angleDenotation) toAngle() revolveangle.Angle {
	return revolveangle.Angle{Rad: d.rad, Turn: d.turn, Span: d.span}
}

func angleFromInternal(a revolveangle.Angle) angleDenotation {
	return angleDenotation{rad: a.Rad, turn: a.Turn, span: a.Span}
}

func zeroAngleDenotation() angleDenotation     { return angleFromInternal(revolveangle.Zero()) }
func (d angleDenotation) valid() bool          { return d.toAngle().Valid() }
func (d angleDenotation) neg() angleDenotation { return angleFromInternal(d.toAngle().Neg()) }
func (d angleDenotation) scale(k *big.Rat) angleDenotation {
	return angleFromInternal(d.toAngle().Scale(k))
}
func (d angleDenotation) enclosure() (proofbound.RatInterval, bool) {
	return d.toAngle().Enclosure()
}
func (d angleDenotation) sinCosFor(held float64) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return d.toAngle().SinCosFor(held)
}
func (d angleDenotation) delta(held float64) float64 { return d.toAngle().Delta(held) }

type sweepDenotation struct{ phi0, phi1 angleDenotation }

func (sd sweepDenotation) toSweep() revolveangle.Sweep {
	return revolveangle.Sweep{Phi0: sd.phi0.toAngle(), Phi1: sd.phi1.toAngle()}
}

func angleDenotationFromValue(v units.Value) angleDenotation {
	return angleFromInternal(revolveangle.FromValue(v))
}

// phi0Delta and phi1Delta are the proven per-end displacements this payload's
// held sweep carries — the angular twins of prismPayload.z0Delta/z1Delta,
// read off the payload's own denotation against its own held float rather
// than stored redundantly, since the payload already carries both.
func (rp revolvePayload) phi0Delta() float64 { return rp.den.phi0.delta(rp.phi0) }
func (rp revolvePayload) phi1Delta() float64 { return rp.den.phi1.delta(rp.phi1) }

// angularDelta is the larger of the two ends' displacements: the figure a
// reading that cannot attribute its error to one particular end takes,
// mirroring prismPayload.axialDelta().
func (rp revolvePayload) angularDelta() float64 { return math.Max(rp.phi0Delta(), rp.phi1Delta()) }

func (sd sweepDenotation) widthInterval() (proofbound.RatInterval, bool) {
	return sd.toSweep().WidthInterval()
}

func (sd sweepDenotation) halfTurnExcessFor(phi0, phi1 float64) (proofbound.RatInterval, bool) {
	return sd.toSweep().HalfTurnExcessFor(phi0, phi1)
}

// sweep is the proven bound on the sweep width every mass and edge reading
// multiplies into its own quantity: the held float64 subtraction stays the
// published value exactly as it always has, and the bound is the smaller of
// the magnitude envelope proofbound.ConservativeValueError has always published here
// (internal/proofbound/bounded.go's own documented fallback, sound for any legal sweep since a
// legal sweep never exceeds a full turn) and the proven displacement between
// that held value and the sweep the record DENOTES (den.widthInterval,
// above), wherever a denotation exists for both ends. math.Min follows
// internal/proofbound/bounded.go's own convention for a reading a certified bracket admits — it
// can only shrink the published bound, never widen it — so a ToFaceAngular
// sweep or any other denotation this file cannot state keeps exactly the
// envelope it always published.
func (rp revolvePayload) sweep() proofbound.BoundedScalar {
	held := proofbound.BoundedSub(proofbound.ExactScalar(rp.phi1), proofbound.ExactScalar(rp.phi0))
	fallback := proofbound.ConservativeValueError(held.Value, proofbound.TwoPiUpper())
	enc, ok := rp.den.widthInterval()
	if !ok {
		return proofbound.MeasuredScalar(held.Value, fallback)
	}
	return proofbound.MeasuredScalar(held.Value, math.Min(fallback, proofbound.IntervalFloatError(enc, held.Value)))
}

func endSinCos(d angleDenotation, held float64) (proofbound.BoundedScalar, proofbound.BoundedScalar) {
	return revolveangle.EndSinCos(d.toAngle(), held)
}
