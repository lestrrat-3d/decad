package decad

import (
	"fmt"
	"slices"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// Document is the mutable root of a model: it owns the live body set and the
// step list. It is immediate-mode — the caller's Go function is the feature
// tree (core §6) — and it is NOT safe for concurrent mutation (core §12);
// the bodies it hands out are immutable and safe to read from anywhere.
type Document struct {
	bodies []*Body
	steps  []Step
}

// DocumentOption configures New. The set is empty today; the option group
// exists so options can be added without changing the signature.
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

// Recipe returns the exact record of intent: a value holding no pointer into
// the document (core §6.2). The steps slice is a copy.
func (d *Document) Recipe() Recipe {
	return Recipe{Steps: cloneSteps(d.steps)}
}

// commit appends the step and registers the produced body, retiring the
// consumed inputs — the atomic tail of every feature call
// (docs/evaluator-design.md §8): a failed evaluation never reaches here, so
// a rejected operation leaves the recipe and the document untouched.
func (d *Document) commit(step Step, produced *Body, consumed ...*Body) {
	d.steps = append(d.steps, step)
	for _, c := range consumed {
		d.retire(c)
	}
	d.bodies = append(d.bodies, produced)
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

// nextStepRef is the StepRef the next recorded step will occupy.
func (d *Document) nextStepRef() StepRef { return StepRef(len(d.steps)) }

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
	// placed re-evaluates the same record under the composed motion.
	placed(d *Document, ref StepRef, composed r3.Transform) (*Body, error)
}

// Placed returns a new body carrying the receiver's geometry under the rigid
// motion t, retiring the receiver (core §8). The zero transform is invalid
// and is ErrDegenerate; the step records the motion as a TransformRecord.
func (b *Body) Placed(t r3.Transform) (*Body, error) {
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
	rec, err := RecordTransform(t)
	if err != nil {
		return nil, err
	}
	if b.payload == nil {
		return nil, fmt.Errorf(`%w: this evaluator cannot place a body it did not build`, ErrUnsupported)
	}

	composed, err := b.payload.transform().Then(t)
	if err != nil {
		return nil, fmt.Errorf(`decad: composing the placement failed: %w`, err)
	}
	step := Step{
		Op:        OpPlaced,
		Inputs:    []StepRef{b.originStep()},
		Placement: rec,
	}
	ref := d.nextStepRef()
	placed, err := b.payload.placed(d, ref, composed)
	if err != nil {
		return nil, err
	}
	d.commit(step, placed, b)
	return placed, nil
}

// originStep returns the StepRef that produced this body.
func (b *Body) originStep() StepRef { return b.origin.Step }

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
