package decad

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// Document is the mutable root of a model: it owns the live body set and the
// producer identities used for topology provenance. It is immediate-mode —
// the caller's Go function is the feature tree (core §6) — and it is NOT
// safe for concurrent mutation (core §12), with one guarded exception: the
// live body set sits behind its node's LiveMu, so Bodies, Remove and Verify may run
// concurrently with one another. The bodies it hands out are immutable and
// safe to read from anywhere.
type Document struct {
	// n is the node data internal packages read and build (internal/topo).
	n topo.Document
}

// DocumentOption is a placeholder option tier for New. No options exist yet;
// it reserves the constructor signature for future options.
type DocumentOption interface {
	option.Interface
	documentOption()
}

// New returns an empty document.
func New(_ ...DocumentOption) *Document {
	return bornDocument(&Document{})
}

// Bodies returns the live bodies — the model as it stands. The slice is a
// copy; the elements are the live (immutable) bodies.
func (d *Document) Bodies() []*Body {
	return d.liveBodies()
}

// liveBodies is the handle snapshot every root reader of the live set walks:
// the node's Live copy, taken under LiveMu, mapped to its handles.
func (d *Document) liveBodies() []*Body {
	return bodyHandles(d.n.Live())
}

// Remove takes b out of the live model by the caller's own hand: b leaves
// Bodies() and Verify stops reporting on it, exactly as a body a feature
// consumed (core §6). A removed body IS a retired body. It stays readable —
// its measurements, topology, selectors and Origin still answer — but no
// operation takes it (ErrRetiredBody), and nothing returns it to the model.
// Remove produces no body, so no producer identity advances, and every other
// body, including one built from b, keeps its geometry and provenance.
//
// b MUST NOT be nil (ErrDegenerate), and a body that belongs to no document
// is ErrDegenerate. The document is the one b names; a body already retired
// — consumed by a feature or removed before — is ErrRetiredBody. A refused
// call leaves the document unchanged. Remove may run concurrently with Bodies and Verify; a Verify
// already under way reports on the set as it stood when the call began.
func Remove(b *Body) error {
	if b == nil {
		return fmt.Errorf(`%w: a nil body names nothing to remove`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if d == nil {
		return fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	if !slices.Contains(d.n.LiveSet, bodyNode(b)) {
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
	d.n.NextProducer++
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	for _, c := range consumed {
		d.retire(c)
	}
	d.n.LiveSet = append(d.n.LiveSet, bodyNode(produced))
}

// commitMany is commit's N-producer variant (docs/surface-design.md §6.5):
// Unstitch produces many bodies from one receiver, and registering them one
// commit at a time would leave the document in a partial state — the
// receiver retired but fewer than len(produced) results live — if a later
// one somehow failed between calls. Every produced body must already be
// fully built by the time this runs (stitchrecord.BuildUnstitchFace's own errors
// surface before commitMany is ever called), so this function's own job is
// only to make the retirement and every registration one atomic step from
// [Document.Bodies]'s point of view, exactly as commit already is for one
// producer.
func (d *Document) commitMany(produced []*Body, consumed ...*Body) {
	d.n.NextProducer += producerID(len(produced))
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	for _, c := range consumed {
		d.retire(c)
	}
	d.n.LiveSet = append(d.n.LiveSet, bodyNodes(produced)...)
}

// commitSpan is commit for an operation that evaluated several bodies under
// span consecutive producer identities, from nextProducerID on, and keeps
// only produced: Patterned builds its instances and every intermediate Union
// before registering the last one. Advancing past the whole span keeps every
// identity those transient bodies minted roles under unique, so a later
// producer never repeats one a result's provenance can carry.
func (d *Document) commitSpan(produced *Body, span producerID, consumed ...*Body) {
	d.n.NextProducer += span
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	for _, c := range consumed {
		d.retire(c)
	}
	d.n.LiveSet = append(d.n.LiveSet, bodyNode(produced))
}

// retire removes a body from the live set. The body itself is untouched —
// retiring is a change of document membership, not of the body (core §6).
// The caller holds LiveMu.
func (d *Document) retire(b *Body) {
	n := bodyNode(b)
	for i, live := range d.n.LiveSet {
		if live != n {
			continue
		}
		d.n.LiveSet = append(d.n.LiveSet[:i], d.n.LiveSet[i+1:]...)
		return
	}
}

// nextProducerID is the identity the next successful operation will occupy.
func (d *Document) nextProducerID() producerID { return d.n.NextProducer }

// requireLive gates a body-consuming operation: the body must belong to this
// document (ErrForeignBody) and still be part of the model (ErrRetiredBody).
func (d *Document) requireLive(b *Body) error {
	if b == nil {
		return fmt.Errorf(`%w: a nil body names nothing to operate on`, ErrDegenerate)
	}
	if b.n.Doc != documentNode(d) {
		return ErrForeignBody
	}
	if !d.n.IsLive(bodyNode(b)) {
		return ErrRetiredBody
	}
	return nil
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
	if body == nil || body.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(body.n.Doc)
	if err := d.requireLive(body); err != nil {
		return nil, err
	}
	if !t.IsValid() {
		return nil, fmt.Errorf(`%w: an invalid transform names no placement`, ErrDegenerate)
	}
	if body.n.Payload == nil {
		return nil, fmt.Errorf(`%w: this evaluator cannot place a body it did not build`, ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	composed, err := body.n.Payload.Transform().Then(t)
	if err != nil {
		return nil, fmt.Errorf(`decad: composing the placement failed: %w`, err)
	}
	ref := d.nextProducerID()
	placed, err := placePayload(ctx, body.n.Payload, d, ref, composed)
	if err != nil {
		return nil, err
	}
	if source, ok := body.n.Payload.(facetedPayload); ok && source.CopyLineage != nil {
		bindFacetedCopyLineage(placed, bodyHandle(source.CopyLineage.Source), composed, false, nil)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(placed, body)
	return placed, nil
}

// CopyOption configures Copy. Only WithCopyPlacement and WithCopyRotation
// change the copy's placement. An option from another package cannot implement
// this sealed tier.
type CopyOption interface {
	option.Interface
	copyOption()
}

type copyOption struct{ option.Interface }

func (copyOption) copyOption() {}

type identCopyPlacement struct{}
type identCopyRotation struct{}

// copyRotation is the value WithCopyRotation stores under identCopyRotation.
type copyRotation struct {
	center, axis r3.Vec
	angle        units.Value
}

// WithCopyPlacement places a new copy under t. A zero or otherwise invalid
// transform is ErrDegenerate when Copy evaluates the option.
func WithCopyPlacement(t r3.Transform) CopyOption {
	return copyOption{option.New(identCopyPlacement{}, t)}
}

// WithCopyRotation places a new copy by a rotation about the axis through
// center along axis, by angle. The rotation a faceted copy denotes is an
// exactly orthogonal rational rotation about exactly that axis, chosen near
// angle: it agrees with angle to within a few float steps, but it is not in
// general the rotation by exactly that transcendental angle. The held
// placement's rounding is charged against that exact rotation. Other payloads
// are placed by r3.RotationAround(center, axis, angle), exactly as
// WithCopyPlacement of that transform.
//
// A faceted copy made this way carries no analytic surface tags: every face
// reads as Faceted with its held bound, so Face.NormalAt refuses it and
// ToFace stops and MirrorFace refuse it. It also stops being a point-loft
// source. Use WithCopyPlacement for a copy that needs those readings.
//
// A zero or non-finite axis is ErrDegenerate, a non-finite center or angle is
// ErrNotFinite, and an angle that is not an angle is ErrUnitKind, all when
// Copy evaluates the option. A zero angle is the identity.
func WithCopyRotation(center, axis r3.Vec, angle units.Value) CopyOption {
	return copyOption{option.New(identCopyRotation{}, copyRotation{center: center, axis: axis, angle: angle})}
}

// Copy returns a new live body carrying source's geometry and leaves source
// live. With no options, the copy has source's placement. WithCopyPlacement
// and WithCopyRotation compose a rigid motion onto it; repeated placement
// options use the last value. The copy gets a new body identity and reproduces
// source provenance. source MUST NOT be nil. A body this evaluator did not
// build is ErrUnsupported. Invalid or nil options are ErrDegenerate. A
// canceled context stops the rebuild before the document changes.
func Copy(ctx context.Context, source *Body, opts ...CopyOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a copy`, ErrDegenerate)
	}
	motion := r3.Identity()
	var exact *exactrotation.Rotation
	for _, opt := range opts {
		if opt == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		if c, ok := opt.(copyOption); ok && c.Interface == nil {
			return nil, fmt.Errorf(`%w: a zero option names nothing to apply`, ErrDegenerate)
		}
		switch opt.Ident().(type) {
		case identCopyPlacement:
			t, ok := option.Get[r3.Transform](opt)
			if !ok {
				return nil, fmt.Errorf(`%w: WithCopyPlacement carries no transform`, ErrDegenerate)
			}
			motion, exact = t, nil
		case identCopyRotation:
			rot, ok := option.Get[copyRotation](opt)
			if !ok {
				return nil, fmt.Errorf(`%w: WithCopyRotation carries no rotation`, ErrDegenerate)
			}
			record, t, err := exactrotation.New(rot.center, rot.axis, rot.angle)
			if err != nil {
				return nil, err
			}
			motion, exact = t, record
		default:
			return nil, fmt.Errorf(`%w: the copy option is not a decad copy option (%T)`, ErrDegenerate, opt)
		}
	}
	return source.copyUnder(ctx, motion, exact)
}

// copyUnder leaves its source live while re-evaluating at a new identity.
// It mirrors Transform's gate order and commit (core §8, docs/api-design.md H4).
// A non-nil exact record asks a faceted payload for the exact-rotation
// placement; every other payload places by motion alone.
func (b *Body) copyUnder(ctx context.Context, motion r3.Transform, exact *exactrotation.Rotation) (*Body, error) {
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if !motion.IsValid() {
		return nil, fmt.Errorf(`%w: an invalid transform names no placement`, ErrDegenerate)
	}
	if b.n.Payload == nil {
		return nil, fmt.Errorf(`%w: this evaluator cannot copy a body it did not build`, ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	composed, err := b.n.Payload.Transform().Then(motion)
	if err != nil {
		return nil, fmt.Errorf(`decad: composing the placement failed: %w`, err)
	}
	ref := d.nextProducerID()
	var copied *Body
	faceted, isFaceted := b.n.Payload.(facetedPayload)
	if isFaceted && exact != nil {
		copied, err = bodyResult(faceted.PlacedWith(ctx, envOf(documentNode(d), ref), composed, exact))
	} else {
		copied, err = placePayload(ctx, b.n.Payload, d, ref, composed)
	}
	if err != nil {
		return nil, err
	}
	if isFaceted {
		source := b
		if record := faceted.CopyLineage; record != nil {
			source = bodyHandle(record.Source)
		}
		bindFacetedCopyLineage(copied, source, composed, faceted.CopyLineage == nil, exact)
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
func (b *Body) originProducer() producerID { return b.n.Origin.Producer }
