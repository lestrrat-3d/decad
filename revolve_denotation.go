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
// angleDenotation carries that exact angle as rad + 2π·turn, both exact
// big.Rat: a radian-stated extent lands entirely in rad with turn zero; a
// degree-stated one lands entirely in turn (mag/360) with rad zero, since
// units.Degree's own factor is a ROUNDED π/180 and multiplying by it would
// recover an approximation of the denoted angle rather than the angle
// itself (units' own value.go:629 warns that 180×factor and math.Pi are
// different quantities). Every other angle unit, and any extent this
// evaluator cannot yet enclose (ToFaceAngular), denotes no exact angle here:
// rad and turn are both nil, and every reading below reads that as an
// infinite displacement — the sound answer when nothing proves a tighter
// one — falling back to whatever it published before this file existed.

// angleDenotation is the angle an extent's own record denotes: rad + 2π·turn,
// both exact rationals. A nil pair means the resolver could not state one,
// and every reading falls back to the magnitude envelope it uses today.
type angleDenotation struct{ rad, turn *big.Rat }

// zeroAngleDenotation is the exact angle zero: what the end of an extent the
// caller did not displace (the sketch-plane end of an Along/Against
// AngleExtent, or FullRevolution's start) denotes.
func zeroAngleDenotation() angleDenotation {
	return angleDenotation{rad: new(big.Rat), turn: new(big.Rat)}
}

// valid reports whether d states an exact angle at all.
func (d angleDenotation) valid() bool { return d.rad != nil && d.turn != nil }

// neg is the denoted angle's own negation, exact: negating a rational is
// never a rounding.
func (d angleDenotation) neg() angleDenotation {
	if !d.valid() {
		return angleDenotation{}
	}
	return angleDenotation{rad: new(big.Rat).Neg(d.rad), turn: new(big.Rat).Neg(d.turn)}
}

// scale multiplies the denoted angle by an exact rational, exact: every
// caller here scales by ±1 (a side's travel sign) or 1/2 (a symmetric
// extent's half-angle), none of which round.
func (d angleDenotation) scale(k *big.Rat) angleDenotation {
	if !d.valid() {
		return angleDenotation{}
	}
	return angleDenotation{rad: new(big.Rat).Mul(d.rad, k), turn: new(big.Rat).Mul(d.turn, k)}
}

// enclosure returns the rational interval the denoted angle is proven to lie
// in: a point interval for a radian-stated angle (turn is exactly zero, so
// intervalScale contributes [0,0]), and an interval no narrower than 2π's own
// enclosure for a degree-stated one, since 2π itself is only ever bracketed,
// never exact. ok is false for an invalid denotation.
func (d angleDenotation) enclosure() (ratInterval, bool) {
	if !d.valid() {
		return ratInterval{}, false
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
// either pure-turn (rad exactly zero — a degree-stated end, or the exact
// zero of an end the caller did not displace) or pure-radian (turn exactly
// zero), never both, so this dispatches to whichever certified primitive
// needs no detour through the other: turnSinCosInterval for a pure turn,
// which never compares against π and is EXACT at every eighth-turn boundary
// (a quarter, half or three-quarter turn's sin/cos is a zero-width
// enclosure) — the fact enclosure()'s own 2π-multiplication would otherwise
// lose, since 2π itself is only ever bracketed; radSinCosInterval for a pure
// radian, which is what enclosure() already reduces to when rad alone is
// nonzero. A denotation this design cannot yet resolve into one pure case —
// invalid, or a future mixed one — falls back through enclosureFor and
// radSinCosInterval over the HELD float, exactly like every other reading
// this file cannot denote exactly.
func (d angleDenotation) sinCosFor(held float64) (sin, cos ratInterval, ok bool) {
	switch {
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
