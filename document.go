package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// Document is the mutable root of a model: it owns the live body set and the
// producer identities used for topology provenance. It is immediate-mode —
// the caller's Go function is the feature tree (core §6) — and it is NOT
// safe for concurrent mutation (core §12);
// the bodies it hands out are immutable and safe to read from anywhere.
type Document struct {
	bodies       []*Body
	nextProducer producerID
	// nextLevel is denotation.go's own counter behind mintLevel: the LEVEL
	// half of the shared-denotation certificate.
	nextLevel levelID
	// nextCurve is denotation.go's own counter behind mintCurve: the CURVE
	// half of the shared-denotation certificate.
	nextCurve curveID
}

// DocumentOption configures New. No options are currently supported: the
// option group exists so options can be added without changing the signature.
type DocumentOption interface {
	option.Interface
	documentOption()
}

// New returns an empty document.
func New(_ ...DocumentOption) *Document {
	return &Document{}
}

// Bodies returns the live bodies — the model as it stands. The slice is a
// copy; the elements are the live (immutable) bodies.
func (d *Document) Bodies() []*Body {
	return append([]*Body(nil), d.bodies...)
}

// commit registers the produced body, advances its private producer identity,
// and retires the consumed inputs — the atomic tail of every feature call
// (docs/evaluator-design.md §8): a failed evaluation never reaches here, so
// a rejected operation leaves the document untouched.
func (d *Document) commit(produced *Body, consumed ...*Body) {
	d.nextProducer++
	for _, c := range consumed {
		d.retire(c)
	}
	d.bodies = append(d.bodies, produced)
}

// commitMany is commit's N-producer variant (docs/surface-design.md §6.5):
// Unstitch produces many bodies from one receiver, and registering them one
// commit at a time would leave the document in a partial state — the
// receiver retired but fewer than len(produced) results live — if a later
// one somehow failed between calls. Every produced body must already be
// fully built by the time this runs (evalUnstitchFaceContext's own errors
// surface before commitMany is ever called), so this function's own job is
// only to make the retirement and every registration one atomic step from
// [Document.Bodies]'s point of view, exactly as commit already is for one
// producer.
func (d *Document) commitMany(produced []*Body, consumed ...*Body) {
	d.nextProducer += producerID(len(produced))
	for _, c := range consumed {
		d.retire(c)
	}
	d.bodies = append(d.bodies, produced...)
}

// retire removes a body from the live set. The body itself is untouched —
// retiring is a change of document membership, not of the body (core §6).
func (d *Document) retire(b *Body) {
	for i, live := range d.bodies {
		if live != b {
			continue
		}
		d.bodies = append(d.bodies[:i], d.bodies[i+1:]...)
		return
	}
}

// isLive reports whether b is currently part of the model.
func (d *Document) isLive(b *Body) bool {
	return slices.Contains(d.bodies, b)
}

// nextProducerID is the identity the next successful operation will occupy.
func (d *Document) nextProducerID() producerID { return d.nextProducer }

// requireLive gates a body-consuming operation: the body must belong to this
// document (ErrForeignBody) and still be part of the model (ErrRetiredBody).
func (d *Document) requireLive(b *Body) error {
	if b == nil {
		return fmt.Errorf(`%w: a nil body names nothing to operate on`, ErrDegenerate)
	}
	if b.doc != d {
		return ErrForeignBody
	}
	if !d.isLive(b) {
		return ErrRetiredBody
	}
	return nil
}

// featurePayload is the evaluator's own record of how a body was built —
// what Placed re-evaluates under a composed motion (docs/evaluator-design.md
// §8). Each feature's payload carries its accumulated rigid placement and
// rebuilds the same record under a new one.
type featurePayload interface {
	// transform is the accumulated rigid placement.
	transform() r3.Transform
	// placed re-evaluates the same record under the composed motion, checking
	// ctx during any long rebuild.
	placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error)
}

// Placed calls [Body.PlacedContext] with [context.Background].
func (b *Body) Placed(t r3.Transform) (*Body, error) {
	return b.PlacedContext(context.Background(), t)
}

// PlacedContext returns a new body carrying the receiver's geometry under the
// rigid motion t, retiring the receiver (core §8). The zero transform is
// invalid and is ErrDegenerate. A canceled context stops the rebuild before
// the document changes. The motion composes onto the placement this body already carries,
// and it is that ACCUMULATED placement, never t alone, that the analytic
// interference reading compares against a coplanar partner's: where the two
// differ, an otherwise admitted crossing overlap reroutes away from that
// reading (docs/prism-boolean-design.md §3.4), while motions composing back to
// the partner's own placement leave it available. Seating both sections in one
// sketch clears that cause alone — a section displacement or a walk charge on
// either operand reroutes the pair however it was placed.
func (b *Body) PlacedContext(ctx context.Context, t r3.Transform) (*Body, error) {
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := b.doc
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if !t.IsValid() {
		return nil, fmt.Errorf(`%w: an invalid transform names no placement`, ErrDegenerate)
	}
	if b.payload == nil {
		return nil, fmt.Errorf(`%w: this evaluator cannot place a body it did not build`, ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	composed, err := b.payload.transform().Then(t)
	if err != nil {
		return nil, fmt.Errorf(`decad: composing the placement failed: %w`, err)
	}
	ref := d.nextProducerID()
	placed, err := b.payload.placed(ctx, d, ref, composed)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(placed, b)
	return placed, nil
}

// Duplicate calls [Body.DuplicateContext] with [context.Background].
func (b *Body) Duplicate() (*Body, error) {
	return b.DuplicateContext(context.Background())
}

// DuplicateContext returns a new live body carrying the receiver's geometry
// unchanged, leaving the receiver LIVE (core §8): the source is depended on,
// never consumed. It is PlacedCopy with no motion — the identity placement —
// so the copy is identical, independent geometry at a fresh body identity. A
// body this evaluator did not build is ErrUnsupported. A canceled context
// stops the rebuild before the document changes.
func (b *Body) DuplicateContext(ctx context.Context) (*Body, error) {
	return b.copyUnder(ctx, r3.Identity())
}

// PlacedCopy calls [Body.PlacedCopyContext] with [context.Background].
func (b *Body) PlacedCopy(t r3.Transform) (*Body, error) {
	return b.PlacedCopyContext(context.Background(), t)
}

// PlacedCopyContext returns a new live body carrying the receiver's geometry
// under the rigid motion t, leaving the receiver LIVE (core §8): the source is
// depended on, never consumed. The payload re-evaluates under the composed
// motion exactly as Placed does, so the centroid moves by t; the zero transform
// is invalid and is ErrDegenerate, and r3.Identity() is a valid no-op. A body
// this evaluator did not build is ErrUnsupported. A canceled context stops the
// rebuild before the document changes.
func (b *Body) PlacedCopyContext(ctx context.Context, t r3.Transform) (*Body, error) {
	return b.copyUnder(ctx, t)
}

// copyUnder is the shared non-consuming copy path behind Duplicate and
// PlacedCopy. It mirrors Placed's gate order and commit while leaving the source
// live, so a body can be modelled once and instanced many times
// (core §8, docs/api-design.md H4).
func (b *Body) copyUnder(ctx context.Context, motion r3.Transform) (*Body, error) {
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := b.doc
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if !motion.IsValid() {
		return nil, fmt.Errorf(`%w: an invalid transform names no placement`, ErrDegenerate)
	}
	if b.payload == nil {
		return nil, fmt.Errorf(`%w: this evaluator cannot copy a body it did not build`, ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	composed, err := b.payload.transform().Then(motion)
	if err != nil {
		return nil, fmt.Errorf(`decad: composing the placement failed: %w`, err)
	}
	ref := d.nextProducerID()
	copied, err := b.payload.placed(ctx, d, ref, composed)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	// The source is depended on, not consumed: commit with no consumed inputs,
	// so the retire rule never touches it.
	d.commit(copied)
	return copied, nil
}

// originProducer returns the private identity of the operation that produced this body.
func (b *Body) originProducer() producerID { return b.origin.producer }

// magnitudeIn validates a magnitude parameter (core §8.1/§12): the right
// Kind, finite, and non-negative — sense is enumerated, never a sign.
func magnitudeIn(v units.Value, kind units.Kind, unit units.Unit, what string) (float64, error) {
	if v.Kind() != kind {
		return 0, fmt.Errorf(`%w: %s must be a %s, got %s`, ErrUnitKind, what, kind, v.Kind())
	}
	m, err := v.In(unit)
	if err != nil {
		return 0, fmt.Errorf(`%w: %s is not representable: %s`, ErrNotFinite, what, err)
	}
	if m < 0 {
		return 0, fmt.Errorf(`%w: %s must be non-negative, got %s`, ErrNegativeMagnitude, what, v)
	}
	return m, nil
}

// magnitudeInBounded is magnitudeIn beside the rounding the conversion itself
// committed. A magnitude carried in a non-base unit reaches the evaluator as a
// RESCALED float — units multiplies by the source unit's factor and divides by
// the target's — so the millimetre figure returned is that rounding away from
// the quantity the caller stated, and a level built from it is a computed level
// like any other. A magnitude already in millimetres rescales by a factor of
// one and reports zero, which is what keeps every millimetre-stated extent
// exact.
func magnitudeInBounded(v units.Value, kind units.Kind, unit units.Unit, what string) (float64, float64, error) {
	m, err := magnitudeIn(v, kind, unit, what)
	if err != nil {
		return 0, 0, err
	}
	return m, conversionRound(v, unit, m), nil
}

// conversionRound measures that rounding rather than estimating it: the rescale
// is redone in exact rationals over the same magnitude and the same two unit
// factors, and compared with the float that was held. Every input is a float64
// and so an exact rational, so the comparison is the conversion's true error,
// whatever sequence of float operations produced the held value.
func conversionRound(v units.Value, unit units.Unit, held float64) float64 {
	mag, from, to := floatRat(v.Mag()), floatRat(v.Unit().Factor()), floatRat(unit.Factor())
	if mag == nil || from == nil || to == nil || to.Sign() == 0 {
		return math.Inf(1)
	}
	exact := new(big.Rat).Quo(new(big.Rat).Mul(mag, from), to)
	return rationalFloatError(exact, held)
}
