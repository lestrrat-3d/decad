package decad

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/units"
)

// This file is the revolve's angular twin of the axial displacement
// docs/evaluator-design.md §5 states for extrude's z0Delta/z1Delta: the
// proven per-end gap between the sweep angle the evaluator HOLDS as a
// float64 and the sweep angle the recorded AngularExtent DENOTES.
//
// The angle a recorded extent denotes is stated exactly by the record
// itself — units.Degrees(90) denotes exactly a quarter turn, units.Radians(1)
// denotes exactly the radian value 1 — never by the float64 the resolver
// computed to hold it. Every reading that folds a held sweep angle into a
// published measurement must charge the gap between the two, exactly as
// revolvePayload.z0Delta/z1Delta already does for extrude's axial extent
// (docs/evaluator-design.md §6).
//
// angleDenotation carries a stated angle as rad + 2π·turn, both exact
// big.Rat. A radian-stated extent uses rad with turn zero. A degree-stated one
// uses turn (mag/360) with rad zero because units.Degree's factor is a rounded
// π/180. An atan2-derived Sweep angle instead carries a rational interval in
// radians. Every other angle unit, and an extent this evaluator cannot
// enclose (ToFaceAngular), leaves all three forms nil. Readings then use their
// existing magnitude envelope.

// angleDenotation encloses the angle a feature record denotes. Stated Revolve
// angles use rad + 2π·turn. A derived Sweep arc uses span. A zero value makes
// each reading use its existing magnitude envelope.
type angleDenotation struct {
	rad, turn *big.Rat
	// span is an exact rational enclosure for an angle whose denotation is
	// transcendental, such as ArcThrough's atan2-derived directed angle. It is
	// disjoint from the rad+2π·turn form so stated Revolve angles retain their
	// exact quarter-turn fast paths.
	span *ratInterval
}

// zeroAngleDenotation is the exact angle zero: what the end of an extent the
// caller did not displace (the sketch-plane end of an Along/Against
// AngleExtent, or FullRevolution's start) denotes.
func zeroAngleDenotation() angleDenotation {
	return angleDenotation{rad: new(big.Rat), turn: new(big.Rat)}
}

// valid reports whether d encloses an angle.
func (d angleDenotation) valid() bool {
	return d.span != nil || (d.rad != nil && d.turn != nil)
}

// neg encloses the denoted angle's negation.
func (d angleDenotation) neg() angleDenotation {
	if !d.valid() {
		return angleDenotation{}
	}
	if d.span != nil {
		negated := intervalNeg(*d.span)
		return angleDenotation{span: &negated}
	}
	return angleDenotation{rad: new(big.Rat).Neg(d.rad), turn: new(big.Rat).Neg(d.turn)}
}

// scale encloses the denoted angle multiplied by k. Every caller scales by
// ±1 (a side's travel sign) or 1/2 (a symmetric extent's half-angle).
func (d angleDenotation) scale(k *big.Rat) angleDenotation {
	if !d.valid() {
		return angleDenotation{}
	}
	if d.span != nil {
		scaled := intervalScale(*d.span, k)
		return angleDenotation{span: &scaled}
	}
	return angleDenotation{rad: new(big.Rat).Mul(d.rad, k), turn: new(big.Rat).Mul(d.turn, k)}
}

// enclosure returns the rational interval the denoted angle is proven to lie
// in. A radian-stated angle produces a point interval. A degree-stated angle
// includes 2π's enclosure. A derived span is copied unchanged. ok is false
// for an invalid denotation.
func (d angleDenotation) enclosure() (ratInterval, bool) {
	if !d.valid() {
		return ratInterval{}, false
	}
	if d.span != nil {
		return interval(d.span.lo, d.span.hi), true
	}
	return intervalAdd(pointInterval(d.rad), intervalScale(twoPiInterval(), d.turn)), true
}

// enclosureFor is enclosure with a fallback: an invalid denotation encloses
// the HELD float exactly instead, which reproduces today's reading — the
// value the record's own float64 subtraction already trusted — for a sweep
// this file cannot yet denote exactly (ToFaceAngular, a payload literal with
// no denotation, or an angle unit outside Radian/Degree).
func (d angleDenotation) enclosureFor(held float64) (ratInterval, bool) {
	if enc, ok := d.enclosure(); ok {
		return enc, true
	}
	r := floatRat(held)
	if r == nil {
		return ratInterval{}, false
	}
	return pointInterval(r), true
}

// sinCosFor encloses sin/cos of the angle d denotes, falling back to the HELD
// float exactly as enclosureFor does. Every end this design resolves is
// either pure-turn, pure-radian, or an atan2-derived radian span. This method
// routes each form to its certified trigonometric enclosure. An invalid or
// future mixed form falls back to the held float, matching readings that have
// no denotation.
func (d angleDenotation) sinCosFor(held float64) (sin, cos ratInterval, ok bool) {
	switch {
	case d.span != nil:
		return radSinCosSpan(*d.span)
	case d.valid() && d.turn.Sign() == 0:
		// Pure radian, the exact angle zero included: radSinCosInterval
		// already answers sin=0, cos=1 exactly at zero, with no series
		// margin — the fast path a zero-turn end must take, since
		// turnSinCosInterval's own series carries a fixed per-call margin
		// even at t=0 and would turn an exact zero into a straddling
		// interval no tighter than any other angle.
		sin, cos, ok = radSinCosInterval(d.rad)
		return sin, cos, ok
	case d.valid() && d.rad.Sign() == 0:
		// quarterTurnSinCos (moments_circular.go) is zero-width at every
		// quarter-turn boundary (0/±1 exactly, no series at all) and falls
		// back to turnSinCosInterval otherwise, so a quarter, half or
		// three-quarter turn stays exact rather than carrying
		// turnSinCosInterval's own fixed per-call series margin.
		sin, cos = quarterTurnSinCos(d.turn)
		return sin, cos, true
	default:
		r := floatRat(held)
		if r == nil {
			return ratInterval{}, ratInterval{}, false
		}
		sin, cos, ok = radSinCosInterval(r)
		return sin, cos, ok
	}
}

// delta is the proven displacement between the held float and the angle d
// denotes: intervalFloatError's outward-rounded distance from held to the
// far end of d's own enclosure. An invalid denotation answers +Inf, which
// every consumer below turns into its existing envelope or refusal — the
// same shape a nil denotation takes throughout this design.
func (d angleDenotation) delta(held float64) float64 {
	enc, ok := d.enclosure()
	if !ok {
		return math.Inf(1)
	}
	return intervalFloatError(enc, held)
}

// sweepDenotation is a revolve's denoted sweep interval [phi0, phi1], one
// angleDenotation per end — the angular twin of prismPayload's
// z0Delta/z1Delta pair, carried as the exact angle each end denotes rather
// than as the displacement alone, since the displacement is read off it
// against whichever float64 the resolver actually held.
type sweepDenotation struct{ phi0, phi1 angleDenotation }

// angleDenotationFromValue resolves v's own denoted angle by UNIT IDENTITY,
// never by the unit's conversion factor: units.Degree's factor is a rounded
// float64 approximation of π/180, and multiplying by it would recover an
// approximation of the denoted angle rather than the angle itself. A radian
// value denotes its own magnitude exactly; a degree value denotes mag/360 of
// an exact turn; any other angle unit denotes nothing here.
func angleDenotationFromValue(v units.Value) angleDenotation {
	mag := floatRat(v.Mag())
	if mag == nil {
		return angleDenotation{}
	}
	switch v.Unit() {
	case units.Radian:
		return angleDenotation{rad: mag, turn: new(big.Rat)}
	case units.Degree:
		return angleDenotation{rad: new(big.Rat), turn: new(big.Rat).Quo(mag, big.NewRat(360, 1))}
	default:
		return angleDenotation{}
	}
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

// widthInterval is the denoted sweep's own width phi1 − phi0, exact: the
// rational-interval twin of the held float64 subtraction sweep (below) takes
// alone, built from each end's own enclosure. ok is false wherever either
// end's denotation cannot state one, which is the sound answer — a width
// built from only one certified end would publish a claim the other end
// never proved.
func (sd sweepDenotation) widthInterval() (ratInterval, bool) {
	enc0, ok0 := sd.phi0.enclosure()
	enc1, ok1 := sd.phi1.enclosure()
	if !ok0 || !ok1 {
		return ratInterval{}, false
	}
	return intervalSub(enc1, enc0), true
}

// halfTurnExcessFor encloses the denoted sweep's width MINUS a half turn,
// (phi1 − phi0) − π: the quantity whose certified sign decides whether the
// sweep is at least a half turn wide (sweepExtremeBounds' reflex arm,
// revolve_extent.go). It is built from each end's own rad/turn form rather
// than by subtracting π's enclosure from widthInterval's, so that a half
// turn stated in degrees — a turn difference of exactly 1/2 — answers the
// POINT interval [0, 0] and certifies the half turn exactly, where the
// subtraction would straddle zero. An end the denotation cannot state falls
// back to the HELD float exactly as enclosureFor does, against π's own
// enclosure; ok is false only where a held float is not finite.
func (sd sweepDenotation) halfTurnExcessFor(phi0, phi1 float64) (ratInterval, bool) {
	if sd.phi0.valid() && sd.phi1.valid() && sd.phi0.span == nil && sd.phi1.span == nil {
		rad := new(big.Rat).Sub(sd.phi1.rad, sd.phi0.rad)
		turn := new(big.Rat).Sub(sd.phi1.turn, sd.phi0.turn)
		turn.Sub(turn, big.NewRat(1, 2))
		return intervalAdd(pointInterval(rad), intervalScale(twoPiInterval(), turn)), true
	}
	enc0, ok0 := sd.phi0.enclosureFor(phi0)
	enc1, ok1 := sd.phi1.enclosureFor(phi1)
	if !ok0 || !ok1 {
		return ratInterval{}, false
	}
	return intervalSub(intervalSub(enc1, enc0), interval(piLower, piUpper)), true
}

// sweep is the proven bound on the sweep width every mass and edge reading
// multiplies into its own quantity: the held float64 subtraction stays the
// published value exactly as it always has, and the bound is the smaller of
// the magnitude envelope conservativeValueError has always published here
// (bounded.go's own documented fallback, sound for any legal sweep since a
// legal sweep never exceeds a full turn) and the proven displacement between
// that held value and the sweep the record DENOTES (den.widthInterval,
// above), wherever a denotation exists for both ends. math.Min follows
// bounded.go's own convention for a reading a certified bracket admits — it
// can only shrink the published bound, never widen it — so a ToFaceAngular
// sweep or any other denotation this file cannot state keeps exactly the
// envelope it always published.
func (rp revolvePayload) sweep() boundedScalar {
	held := boundedSub(exactScalar(rp.phi1), exactScalar(rp.phi0))
	fallback := conservativeValueError(held.value, twoPiUpper())
	enc, ok := rp.den.widthInterval()
	if !ok {
		return measuredScalar(held.value, fallback)
	}
	return measuredScalar(held.value, math.Min(fallback, intervalFloatError(enc, held.value)))
}

// endSinCos encloses sin(held)/cos(held) for one sweep endpoint: the
// published value is always math.Sincos(held), the twin the partial-sweep
// centroid used to compose from boundedSin/boundedCos alone (boundedCos
// survives in bounded.go for walkAxisMoment's circular arm, which this
// change does not touch). The bound is the smaller of the existing ≥1
// magnitude envelope and the denoted angle's own certified enclosure
// (angleDenotation.sinCosFor), gated on d.valid() rather than sinCosFor's own
// wider ok — sinCosFor's fallback branch encloses the HELD float exactly
// wherever d is invalid, which is the right answer for a reading
// (sweepExtremeBounds, revolve_extent.go) that already trusted the held
// float as the truth, but is a tighter claim than the centroid may take: a
// ToFaceAngular endpoint's true angle carries no proven relation to the held
// float here, so its trig keeps the envelope, per docs/evaluator-design.md
// §6's "every reading it feeds keeps the magnitude envelope it always has".
func endSinCos(d angleDenotation, held float64) (sin, cos boundedScalar) {
	sinValue, cosValue := math.Sincos(held)
	sinBound, cosBound := conservativeValueError(sinValue, 1), conservativeValueError(cosValue, 1)
	if d.valid() {
		if sinEnc, cosEnc, ok := d.sinCosFor(held); ok {
			sinBound = math.Min(sinBound, intervalFloatError(sinEnc, sinValue))
			cosBound = math.Min(cosBound, intervalFloatError(cosEnc, cosValue))
		}
	}
	return measuredScalar(sinValue, sinBound), measuredScalar(cosValue, cosBound)
}
