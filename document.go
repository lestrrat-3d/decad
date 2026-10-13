package decad

import (
	"context"
	"fmt"
	"slices"
	"sync"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-go/option/v3"
)

// Document is the mutable root of a model: it owns the live body set and the
// producer identities used for topology provenance. It is immediate-mode —
// the caller's Go function is the feature tree (core §6) — and it is NOT
// safe for concurrent mutation (core §12), with one guarded exception: the
// live body set sits behind mu, so Bodies, Remove and Verify may run
// concurrently with one another. The bodies it hands out are immutable and
// safe to read from anywhere.
type Document struct {
	// mu guards bodies alone. Every reader takes a snapshot under it
	// (liveBodies) and works on that copy, so a Remove racing a Verify never
	// shifts the slice a reader is walking.
	mu           sync.RWMutex
	bodies       []*Body
	nextProducer producerID
	// nextLevel is denotation.go's own counter behind mintLevel: the LEVEL
	// half of the shared-denotation certificate.
	nextLevel levelID
	// nextCurve is denotation.go's own counter behind mintCurve: the CURVE
	// half of the shared-denotation certificate.
	nextCurve curveID
}

// DocumentOption is a placeholder option tier for New. No options exist yet;
// it reserves the constructor signature for future options.
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
	return d.liveBodies()
}

// liveBodies is the snapshot every internal reader of the live set walks.
func (d *Document) liveBodies() []*Body {
	d.mu.RLock()
	defer d.mu.RUnlock()
	return append([]*Body(nil), d.bodies...)
}

// Remove takes b out of the live model by the caller's own hand: b leaves
// Bodies() and Verify stops reporting on it, exactly as a body a feature
// consumed (core §6). A removed body IS a retired body. It stays readable —
// its measurements, topology, selectors and Origin still answer — but no
// operation takes it (ErrRetiredBody), and nothing returns it to the model.
// Remove produces no body, so no producer identity advances, and every other
// body, including one built from b, keeps its geometry and provenance.
//
// b MUST NOT be nil (ErrDegenerate). A body another document owns is
// ErrForeignBody, and a body already retired — consumed by a feature or
// removed before — is ErrRetiredBody. A refused call leaves the document
// unchanged. Remove may run concurrently with Bodies and Verify; a Verify
// already under way reports on the set as it stood when the call began.
func (d *Document) Remove(b *Body) error {
	if d == nil {
		return fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if b == nil {
		return fmt.Errorf(`%w: a nil body names nothing to remove`, ErrDegenerate)
	}
	if b.doc != d {
		return ErrForeignBody
	}
	d.mu.Lock()
	defer d.mu.Unlock()
	if !slices.Contains(d.bodies, b) {
		return ErrRetiredBody
	}
	d.retire(b)
	return nil
}

// commit registers the produced body, advances its private producer identity,
// and retires the consumed inputs — the atomic tail of every feature call
// (docs/evaluator-design.md §8): a failed evaluation never reaches here, so
// a rejected operation leaves the document untouched.
func (d *Document) commit(produced *Body, consumed ...*Body) {
	d.nextProducer++
	d.mu.Lock()
	defer d.mu.Unlock()
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
	d.mu.Lock()
	defer d.mu.Unlock()
	for _, c := range consumed {
		d.retire(c)
	}
	d.bodies = append(d.bodies, produced...)
}

// commitSpan is commit for an operation that evaluated several bodies under
// span consecutive producer identities, from nextProducerID on, and keeps
// only produced: Patterned builds its instances and every intermediate Union
// before registering the last one. Advancing past the whole span keeps every
// identity those transient bodies minted roles under unique, so a later
// producer never repeats one a result's provenance can carry.
func (d *Document) commitSpan(produced *Body, span producerID, consumed ...*Body) {
	d.nextProducer += span
	d.mu.Lock()
	defer d.mu.Unlock()
	for _, c := range consumed {
		d.retire(c)
	}
	d.bodies = append(d.bodies, produced)
}

// retire removes a body from the live set. The body itself is untouched —
// retiring is a change of document membership, not of the body (core §6).
// The caller holds mu.
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
	d.mu.RLock()
	defer d.mu.RUnlock()
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
// what Transform re-evaluates under a composed motion (docs/evaluator-design.md
// §8). Each feature's payload carries its accumulated rigid placement and
// rebuilds the same record under a new one.
type featurePayload interface {
	// transform is the accumulated rigid placement.
	transform() r3.Transform
	// placed re-evaluates the same record under the composed motion, checking
	// ctx during any long rebuild.
	placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error)
}

// Transform returns a new body carrying body's geometry under the rigid
// motion t, retiring body (core §8). body MUST NOT be nil. The zero transform is
// invalid and is ErrDegenerate. A canceled context stops the rebuild before
// the document changes. The motion composes onto the placement body already carries,
// and it is that ACCUMULATED placement, never t alone, that the analytic
// booleans and interference reading compare against a coplanar partner's:
// where the two differ, the partner's coordinates are re-expressed and every
// crossing they cut is charged that rounding over the crossing angle
// (docs/prism-boolean-design.md §3.4), while motions composing back to the
// partner's own placement leave the re-expression the identity.
func Transform(ctx context.Context, body *Body, t r3.Transform) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a placement`, ErrDegenerate)
	}
	if body == nil || body.doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := body.doc
	if err := d.requireLive(body); err != nil {
		return nil, err
	}
	if !t.IsValid() {
		return nil, fmt.Errorf(`%w: an invalid transform names no placement`, ErrDegenerate)
	}
	if body.payload == nil {
		return nil, fmt.Errorf(`%w: this evaluator cannot place a body it did not build`, ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	composed, err := body.payload.transform().Then(t)
	if err != nil {
		return nil, fmt.Errorf(`decad: composing the placement failed: %w`, err)
	}
	ref := d.nextProducerID()
	placed, err := body.payload.placed(ctx, d, ref, composed)
	if err != nil {
		return nil, err
	}
	if source, ok := body.payload.(facetedPayload); ok && source.copyLineage != nil {
		bindFacetedCopyLineage(placed, source.copyLineage.source, composed)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(placed, body)
	return placed, nil
}

// CopyOption configures Copy. Only WithCopyPlacement changes the copy's
// placement. An option from another package cannot implement this sealed tier.
type CopyOption interface{ copyOption() }

type copyPlacementOption struct{ motion r3.Transform }

func (copyPlacementOption) copyOption() {}

// WithCopyPlacement places a new copy under t. A zero or otherwise invalid
// transform is ErrDegenerate when Copy evaluates the option.
func WithCopyPlacement(t r3.Transform) CopyOption {
	return copyPlacementOption{motion: t}
}

// Copy returns a new live body carrying source's geometry and leaves source
// live. With no options, the copy has source's placement. WithCopyPlacement
// composes a rigid motion onto it; repeated placement options use the last
// value. The copy gets a new body identity and reproduces source provenance.
// source MUST NOT be nil. A body this evaluator did not build is
// ErrUnsupported. Invalid or nil options are ErrDegenerate. A canceled context
// stops the rebuild before the document changes.
func Copy(ctx context.Context, source *Body, opts ...CopyOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a copy`, ErrDegenerate)
	}
	motion := r3.Identity()
	for _, opt := range opts {
		if opt == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		placement, ok := opt.(copyPlacementOption)
		if !ok {
			return nil, fmt.Errorf(`%w: the copy option is not a decad copy option (%T)`, ErrDegenerate, opt)
		}
		motion = placement.motion
	}
	return source.copyUnder(ctx, motion)
}

// copyUnder leaves its source live while re-evaluating at a new identity.
// It mirrors Transform's gate order and commit (core §8, docs/api-design.md H4).
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
	if faceted, ok := b.payload.(facetedPayload); ok {
		source := b
		if record := faceted.copyLineage; record != nil {
			source = record.source
		}
		bindFacetedCopyLineage(copied, source, composed)
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
