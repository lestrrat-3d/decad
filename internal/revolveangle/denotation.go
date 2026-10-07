package revolveangle

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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
// Angle carries a stated angle as rad + 2π·turn, both exact
// big.Rat. A radian-stated extent uses rad with turn zero. A degree-stated one
// uses turn (mag/360) with rad zero because units.Degree's factor is a rounded
// π/180. An atan2-derived Sweep angle instead carries a rational interval in
// radians. Every other angle unit, and an extent this evaluator cannot
// enclose (ToFaceAngular), leaves all three forms nil. Readings then use their
// existing magnitude envelope.

// Angle encloses the angle a feature record denotes. Stated Revolve
// angles use rad + 2π·turn. A derived Sweep arc uses span. A zero value makes
// each reading use its existing magnitude envelope.
type Angle struct {
	Rad, Turn *big.Rat
	// Span is an exact rational enclosure for an angle whose denotation is
	// transcendental, such as ArcThrough's atan2-derived directed angle. It is
	// disjoint from the rad+2π·turn form so stated Revolve angles retain their
	// exact quarter-turn fast paths.
	Span *proofbound.RatInterval
}

// Zero is the exact angle zero: what the end of an extent the
// caller did not displace (the sketch-plane end of an Along/Against
// AngleExtent, or FullRevolution's start) denotes.
func Zero() Angle {
	return Angle{Rad: new(big.Rat), Turn: new(big.Rat)}
}

// Valid reports whether d encloses an angle.
func (d Angle) Valid() bool {
	return d.Span != nil || (d.Rad != nil && d.Turn != nil)
}

// Neg encloses the denoted angle's negation.
func (d Angle) Neg() Angle {
	if !d.Valid() {
		return Angle{}
	}
	if d.Span != nil {
		negated := proofbound.IntervalNeg(*d.Span)
		return Angle{Span: &negated}
	}
	return Angle{Rad: new(big.Rat).Neg(d.Rad), Turn: new(big.Rat).Neg(d.Turn)}
}

// Scale encloses the denoted angle multiplied by k. Every caller scales by
// ±1 (a side's travel sign) or 1/2 (a symmetric extent's half-angle).
func (d Angle) Scale(k *big.Rat) Angle {
	if !d.Valid() {
		return Angle{}
	}
	if d.Span != nil {
		scaled := proofbound.IntervalScale(*d.Span, k)
		return Angle{Span: &scaled}
	}
	return Angle{Rad: new(big.Rat).Mul(d.Rad, k), Turn: new(big.Rat).Mul(d.Turn, k)}
}

// Enclosure returns the rational interval the denoted angle is proven to lie
// in. A radian-stated angle produces a point interval. A degree-stated angle
// includes 2π's enclosure. A derived span is copied unchanged. ok is false
// for an invalid denotation.
func (d Angle) Enclosure() (proofbound.RatInterval, bool) {
	if !d.Valid() {
		return proofbound.RatInterval{}, false
	}
	if d.Span != nil {
		return proofbound.Interval(d.Span.Lo, d.Span.Hi), true
	}
	return proofbound.IntervalAdd(proofbound.PointInterval(d.Rad), proofbound.IntervalScale(proofbound.TwoPiInterval(), d.Turn)), true
}

// EnclosureFor is Enclosure with a fallback: an invalid denotation encloses
// the HELD float exactly instead, which reproduces today's reading — the
// value the record's own float64 subtraction already trusted — for a sweep
// this file cannot yet denote exactly (ToFaceAngular, a payload literal with
// no denotation, or an angle unit outside Radian/Degree).
func (d Angle) EnclosureFor(held float64) (proofbound.RatInterval, bool) {
	if enc, ok := d.Enclosure(); ok {
		return enc, true
	}
	r := proofarith.FloatRat(held)
	if r == nil {
		return proofbound.RatInterval{}, false
	}
	return proofbound.PointInterval(r), true
}

// SinCosFor encloses sin/cos of the angle d denotes, falling back to the HELD
// float exactly as EnclosureFor does. Every end this design resolves is
// either pure-turn, pure-radian, or an atan2-derived radian span. This method
// routes each form to its certified trigonometric enclosure. An invalid or
// future mixed form falls back to the held float, matching readings that have
// no denotation.
func (d Angle) SinCosFor(held float64) (sin, cos proofbound.RatInterval, ok bool) {
	switch {
	case d.Span != nil:
		return proofbound.RadSinCosSpan(*d.Span)
	case d.Valid() && d.Turn.Sign() == 0:
		// Pure radian, the exact angle zero included: proofbound.RadSinCosInterval
		// already answers sin=0, cos=1 exactly at zero, with no series
		// margin — the fast path a zero-turn end must take, since
		// proofbound.TurnSinCosInterval's own series carries a fixed per-call margin
		// even at t=0 and would turn an exact zero into a straddling
		// interval no tighter than any other angle.
		sin, cos, ok = proofbound.RadSinCosInterval(d.Rad)
		return sin, cos, ok
	case d.Valid() && d.Rad.Sign() == 0:
		// circularbounds.QuarterTurnSinCos is zero-width at every
		// quarter-turn boundary (0/±1 exactly, no series at all) and falls
		// back to proofbound.TurnSinCosInterval otherwise, so a quarter, half or
		// three-quarter turn stays exact rather than carrying
		// proofbound.TurnSinCosInterval's own fixed per-call series margin.
		sin, cos = circularbounds.QuarterTurnSinCos(d.Turn)
		return sin, cos, true
	default:
		r := proofarith.FloatRat(held)
		if r == nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		sin, cos, ok = proofbound.RadSinCosInterval(r)
		return sin, cos, ok
	}
}

// Delta is the proven displacement between the held float and the angle d
// denotes: proofbound.IntervalFloatError's outward-rounded distance from held to the
// far end of d's own enclosure. An invalid denotation answers +Inf, which
// every consumer below turns into its existing envelope or refusal — the
// same shape a nil denotation takes throughout this design.
func (d Angle) Delta(held float64) float64 {
	enc, ok := d.Enclosure()
	if !ok {
		return math.Inf(1)
	}
	return proofbound.IntervalFloatError(enc, held)
}

// Sweep is a revolve's denoted sweep interval [phi0, phi1], one
// Angle per end — the angular twin of prismPayload's
// z0Delta/z1Delta pair, carried as the exact angle each end denotes rather
// than as the displacement alone, since the displacement is read off it
// against whichever float64 the resolver actually held.
type Sweep struct{ Phi0, Phi1 Angle }

// FromValue resolves v's own denoted angle by UNIT IDENTITY,
// never by the unit's conversion factor: units.Degree's factor is a rounded
// float64 approximation of π/180, and multiplying by it would recover an
// approximation of the denoted angle rather than the angle itself. A radian
// value denotes its own magnitude exactly; a degree value denotes mag/360 of
// an exact turn; any other angle unit denotes nothing here.
func FromValue(v units.Value) Angle {
	mag := proofarith.FloatRat(v.Mag())
	if mag == nil {
		return Angle{}
	}
	switch v.Unit() {
	case units.Radian:
		return Angle{Rad: mag, Turn: new(big.Rat)}
	case units.Degree:
		return Angle{Rad: new(big.Rat), Turn: new(big.Rat).Quo(mag, big.NewRat(360, 1))}
	default:
		return Angle{}
	}
}

// WidthInterval is the denoted sweep's own width phi1 − phi0, exact: the
// rational-interval twin of the held float64 subtraction sweep (below) takes
// alone, built from each end's own enclosure. ok is false wherever either
// end's denotation cannot state one, which is the sound answer — a width
// built from only one certified end would publish a claim the other end
// never proved.
func (sd Sweep) WidthInterval() (proofbound.RatInterval, bool) {
	enc0, ok0 := sd.Phi0.Enclosure()
	enc1, ok1 := sd.Phi1.Enclosure()
	if !ok0 || !ok1 {
		return proofbound.RatInterval{}, false
	}
	return proofbound.IntervalSub(enc1, enc0), true
}

// HalfTurnExcessFor encloses the denoted sweep's width MINUS a half turn,
// (phi1 − phi0) − π: the quantity whose certified sign decides whether the
// sweep is at least a half turn wide (ExtremeBounds' reflex arm,
// bounds.go). It is built from each end's own rad/turn form rather
// than by subtracting π's enclosure from WidthInterval's, so that a half
// turn stated in degrees — a turn difference of exactly 1/2 — answers the
// POINT interval [0, 0] and certifies the half turn exactly, where the
// subtraction would straddle zero. An end the denotation cannot state falls
// back to the HELD float exactly as EnclosureFor does, against π's own
// enclosure; ok is false only where a held float is not finite.
func (sd Sweep) HalfTurnExcessFor(phi0, phi1 float64) (proofbound.RatInterval, bool) {
	if sd.Phi0.Valid() && sd.Phi1.Valid() && sd.Phi0.Span == nil && sd.Phi1.Span == nil {
		rad := new(big.Rat).Sub(sd.Phi1.Rad, sd.Phi0.Rad)
		turn := new(big.Rat).Sub(sd.Phi1.Turn, sd.Phi0.Turn)
		turn.Sub(turn, big.NewRat(1, 2))
		return proofbound.IntervalAdd(proofbound.PointInterval(rad), proofbound.IntervalScale(proofbound.TwoPiInterval(), turn)), true
	}
	enc0, ok0 := sd.Phi0.EnclosureFor(phi0)
	enc1, ok1 := sd.Phi1.EnclosureFor(phi1)
	if !ok0 || !ok1 {
		return proofbound.RatInterval{}, false
	}
	return proofbound.IntervalSub(proofbound.IntervalSub(enc1, enc0), proofbound.Interval(proofbound.PiLower, proofbound.PiUpper)), true
}

// EndSinCos encloses sin(held)/cos(held) for one sweep endpoint: the
// published value is always math.Sincos(held), the twin the partial-sweep
// centroid used to compose from boundedSin/proofbound.BoundedCos alone (proofbound.BoundedCos
// survives in internal/proofbound/bounded.go for walkAxisMoment's circular arm).
// The bound is the smaller of the existing ≥1
// magnitude envelope and the denoted angle's own certified enclosure
// (Angle.SinCosFor), gated on d.Valid() rather than SinCosFor's own
// wider ok — SinCosFor's fallback branch encloses the HELD float exactly
// wherever d is invalid, which is the right answer for a reading
// (ExtremeBounds, bounds.go) that already trusted the held
// float as the truth, but is a tighter claim than the centroid may take: a
// ToFaceAngular endpoint's true angle carries no proven relation to the held
// float here, so its trig keeps the envelope, per docs/evaluator-design.md
// §6's "every reading it feeds keeps the magnitude envelope it always has".
func EndSinCos(d Angle, held float64) (sin, cos proofbound.BoundedScalar) {
	sinValue, cosValue := math.Sincos(held)
	sinBound, cosBound := proofbound.ConservativeValueError(sinValue, 1), proofbound.ConservativeValueError(cosValue, 1)
	if d.Valid() {
		if sinEnc, cosEnc, ok := d.SinCosFor(held); ok {
			sinBound = math.Min(sinBound, proofbound.IntervalFloatError(sinEnc, sinValue))
			cosBound = math.Min(cosBound, proofbound.IntervalFloatError(cosEnc, cosValue))
		}
	}
	return proofbound.MeasuredScalar(sinValue, sinBound), proofbound.MeasuredScalar(cosValue, cosBound)
}
