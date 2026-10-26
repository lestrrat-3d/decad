package decad

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/massdispatch"
	"github.com/lestrrat-3d/decad/internal/parambind"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch/param"
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
	// gapReach is the gap tolerance in millimetres its operations carry
	// (gap_tolerance.go), and gapErr the error an invalid option named.
	gapReach float64
	gapErr   error
	// chainTol is the chain tolerance in millimetres its booleans carry
	// (chain_tolerance.go), and chainErr the error an invalid option named.
	chainTol float64
	chainErr error
	// params is the parameter table Param and Expr read (params.go), and
	// ledger the reads they have made.
	params *param.Table
	ledger parambind.Ledger
	// groups is the container tree and the placement of its members
	// (container.go), guarded by the node's LiveMu.
	groups containerState
	// resultIn names the container commits that name none place their result
	// in, while an operation that cannot pass one down holds it (resultIn).
	resultC *Container
	// operations and their indexes are published under n.LiveMu with body
	// retirement and registration.
	operations []*Operation
	producers  map[*Body]*Operation
	consumers  map[*Body][]*Operation
}

// DocumentOption configures New. WithGapTolerance, ExactOnly,
// WithChainTolerance and WithParams are its options. An option from another package cannot implement this sealed tier.
type DocumentOption interface {
	option.Interface
	documentOption()
}

// New returns an empty document.
func New(opts ...DocumentOption) *Document {
	d := bornDocument(&Document{})
	d.producers = make(map[*Body]*Operation)
	d.consumers = make(map[*Body][]*Operation)
	d.gapReach, d.gapErr = gapReachOf(opts)
	d.chainTol, d.chainErr = chainToleranceOf(opts)
	d.params = paramsOf(opts)
	return d
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
	operation := d.operation(OperationRemove, []OperationInput{{Body: b, Role: "body", Use: InputConsumed}})
	operation.faceSuccessors = make(map[*Face]FaceHistory)
	for _, face := range b.Faces() {
		operation.faceSuccessors[face] = FaceHistory{coverage: HistoryComplete}
	}
	if err := d.validateOperation(operation, []*Body{b}); err != nil {
		return err
	}
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	if !slices.Contains(d.n.LiveSet, bodyNode(b)) {
		return ErrRetiredBody
	}
	d.retire(b)
	d.appendOperationLocked(operation)
	return nil
}

// commitOperationIn publishes one public operation with its output and input
// retirement under the same lock.
func (d *Document) commitOperationIn(ctx context.Context, c *Container, kind OperationKind, inputs []OperationInput,
	produced *Body, consumed ...*Body) error {
	return d.commitOperationSpanIn(ctx, c, kind, inputs, produced, 1, consumed...)
}

func (d *Document) commitOperationSpanIn(ctx context.Context, c *Container, kind OperationKind, inputs []OperationInput,
	produced *Body, span producerID, consumed ...*Body) error {
	return d.commitOperationSpanPrefixIn(ctx, c, kind, inputs, produced, 0, span, consumed...)
}

func (d *Document) commitOperationSpanPrefixIn(ctx context.Context, c *Container, kind OperationKind,
	inputs []OperationInput, produced *Body, prefix, span producerID, consumed ...*Body) error {
	if !operationHistoryInputs(ctx, inputs) {
		if err := ctx.Err(); err != nil {
			return err
		}
		capture, ok := ctx.Value(operationHistorySuppressedKey{}).(*operationHistoryCapture)
		if !ok {
			return fmt.Errorf("decad: private construction is missing its history capture")
		}
		capture.advanceProducerSpan(span)
		return nil
	}
	operation := d.operation(kind, inputs, produced)
	if err := d.validateOperation(operation, consumed); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	d.n.NextProducer += prefix + span
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	dest := d.destinationLocked(c, consumed)
	for _, gone := range consumed {
		d.retire(gone)
	}
	d.n.LiveSet = append(d.n.LiveSet, bodyNode(produced))
	d.placeLocked(bodyNode(produced), dest)
	stampOperationWarningKind(produced, kind)
	d.appendOperationLocked(operation)
	return nil
}

func (d *Document) commitOperationManyIn(ctx context.Context, c *Container, kind OperationKind, inputs []OperationInput,
	produced []*Body, consumed ...*Body) error {
	return d.commitOperationManyFacesIn(ctx, c, kind, inputs, produced, nil, consumed...)
}

func (d *Document) commitOperationManyFacesIn(ctx context.Context, c *Container, kind OperationKind,
	inputs []OperationInput, produced []*Body, faceSuccessors map[*Face]FaceHistory, consumed ...*Body) error {
	if !operationHistoryInputs(ctx, inputs) {
		if err := ctx.Err(); err != nil {
			return err
		}
		capture, ok := ctx.Value(operationHistorySuppressedKey{}).(*operationHistoryCapture)
		if !ok {
			return fmt.Errorf("decad: private construction is missing its history capture")
		}
		capture.advanceProducerSpan(producerID(len(produced)))
		return nil
	}
	operation := d.operation(kind, inputs, produced...)
	operation.faceSuccessors = cloneFaceSuccessors(faceSuccessors)
	if err := d.validateOperation(operation, consumed); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	d.n.NextProducer += producerID(len(produced))
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	dest := d.destinationLocked(c, consumed)
	for _, gone := range consumed {
		d.retire(gone)
	}
	d.n.LiveSet = append(d.n.LiveSet, bodyNodes(produced)...)
	for _, body := range produced {
		d.placeLocked(bodyNode(body), dest)
		stampOperationWarningKind(body, kind)
	}
	d.appendOperationLocked(operation)
	return nil
}

func (d *Document) validateOperation(operation *Operation, consumed []*Body) error {
	if operation.kind == OperationUnknown || len(operation.outputs) == 0 && operation.kind != OperationRemove {
		return fmt.Errorf("%w: operation record has no public kind or output", ErrBooleanFailed)
	}
	// Most modeling calls consume one body. Validate that common case without
	// allocating two temporary maps on every successful feature evaluation.
	if len(consumed) <= 1 {
		var consumedBody *Body
		if len(consumed) == 1 {
			consumedBody = consumed[0]
			if consumedBody == nil || consumedBody.n.Doc != &d.n {
				return fmt.Errorf("%w: operation record names a foreign consumed body", ErrForeignBody)
			}
		}
		matchedConsumedInput := false
		for _, input := range operation.inputs {
			if input.Body == nil || input.Body.n.Doc != &d.n || input.Role == "" {
				return fmt.Errorf("%w: operation record has an invalid body input", ErrForeignBody)
			}
			switch input.Use {
			case InputConsumed:
				if consumedBody == nil || input.Body != consumedBody {
					return fmt.Errorf("%w: operation record inputs do not match the consumed bodies", ErrBooleanFailed)
				}
				matchedConsumedInput = true
			case InputReferenced:
			default:
				return fmt.Errorf("%w: operation record has an unknown input use", ErrBooleanFailed)
			}
		}
		if consumedBody != nil && !matchedConsumedInput {
			return fmt.Errorf("%w: operation record omits a consumed body", ErrBooleanFailed)
		}
	} else {
		consumedSet := make(map[*Body]struct{}, len(consumed))
		for _, body := range consumed {
			if body == nil || body.n.Doc != &d.n {
				return fmt.Errorf("%w: operation record names a foreign consumed body", ErrForeignBody)
			}
			consumedSet[body] = struct{}{}
		}
		inputsConsumed := make(map[*Body]struct{}, len(consumedSet))
		for _, input := range operation.inputs {
			if input.Body == nil || input.Body.n.Doc != &d.n || input.Role == "" {
				return fmt.Errorf("%w: operation record has an invalid body input", ErrForeignBody)
			}
			switch input.Use {
			case InputConsumed:
				inputsConsumed[input.Body] = struct{}{}
			case InputReferenced:
			default:
				return fmt.Errorf("%w: operation record has an unknown input use", ErrBooleanFailed)
			}
		}
		if len(inputsConsumed) != len(consumedSet) {
			return fmt.Errorf("%w: operation record inputs do not match the consumed bodies", ErrBooleanFailed)
		}
		for body := range consumedSet {
			if _, ok := inputsConsumed[body]; !ok {
				return fmt.Errorf("%w: operation record omits a consumed body", ErrBooleanFailed)
			}
		}
	}
	for _, output := range operation.outputs {
		if output == nil || output.n.Doc != &d.n {
			return fmt.Errorf("%w: operation record has a foreign output", ErrBooleanFailed)
		}
	}
	if len(operation.faceSuccessors) > 0 {
		inputFaces := make(map[*topo.Face]struct{})
		for _, input := range operation.inputs {
			for _, lump := range input.Body.n.Lumps {
				for _, shell := range lump.Shells {
					for _, face := range shell.Faces {
						inputFaces[face] = struct{}{}
					}
				}
			}
		}
		outputFaces := make(map[*topo.Face]struct{})
		for _, output := range operation.outputs {
			for _, lump := range output.n.Lumps {
				for _, shell := range lump.Shells {
					for _, face := range shell.Faces {
						outputFaces[face] = struct{}{}
					}
				}
			}
		}
		for face, history := range operation.faceSuccessors {
			if face == nil {
				return fmt.Errorf("%w: operation face mapping names a face outside its inputs", ErrBooleanFailed)
			}
			if _, ok := inputFaces[&face.n]; !ok {
				return fmt.Errorf("%w: operation face mapping names a face outside its inputs", ErrBooleanFailed)
			}
			if history.coverage != HistoryUnknown && history.coverage != HistoryComplete {
				return fmt.Errorf("%w: operation face mapping has unknown coverage", ErrBooleanFailed)
			}
			var seen map[*Face]struct{}
			if len(history.faces) > 1 {
				seen = make(map[*Face]struct{}, len(history.faces))
			}
			for _, successor := range history.faces {
				if successor == nil {
					return fmt.Errorf("%w: operation face mapping names a face outside its outputs", ErrBooleanFailed)
				}
				if _, ok := outputFaces[&successor.n]; !ok {
					return fmt.Errorf("%w: operation face mapping names a face outside its outputs", ErrBooleanFailed)
				}
				if seen != nil {
					if _, duplicate := seen[successor]; duplicate {
						return fmt.Errorf("%w: operation face mapping repeats a successor", ErrBooleanFailed)
					}
					seen[successor] = struct{}{}
				}
			}
		}
	}
	return nil
}

// commit registers the produced body, advances its private producer identity,
// and retires the consumed inputs — the atomic tail of every feature call
// (agent-docs/evaluator-design.md §8): a failed evaluation never reaches here, so
// a rejected operation leaves the document untouched.
func (d *Document) commit(produced *Body, consumed ...*Body) {
	d.commitIn(nil, produced, consumed...)
}

// commitIn is commit placing the produced body in c. A nil c reads the container
// of the first consumed body, and the root when nothing is consumed
// (agent-docs/container-design.md §2).
func (d *Document) commitIn(c *Container, produced *Body, consumed ...*Body) {
	d.n.NextProducer++
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	dest := d.destinationLocked(c, consumed)
	for _, gone := range consumed {
		d.retire(gone)
	}
	d.n.LiveSet = append(d.n.LiveSet, bodyNode(produced))
	d.placeLocked(bodyNode(produced), dest)
}

// destinationLocked is the container a commit places its results in. The caller
// holds LiveMu, and has not yet retired the consumed bodies.
func (d *Document) destinationLocked(c *Container, consumed []*Body) *Container {
	if c != nil {
		return c
	}
	if d.resultC != nil {
		return d.resultC
	}
	if len(consumed) > 0 && consumed[0] != nil {
		return d.bodyContainerLocked(bodyNode(consumed[0]))
	}
	return d.rootLocked()
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
		delete(d.groups.ofBody, n)
		return
	}
}

// nextProducerID is the identity the next successful operation will occupy.
func (d *Document) nextProducerID() producerID { return d.n.NextProducer }

func (d *Document) producerID(ctx context.Context) producerID {
	if capture, ok := ctx.Value(operationHistorySuppressedKey{}).(*operationHistoryCapture); ok {
		return capture.nextProducerID(d)
	}
	return d.nextProducerID()
}

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
// (agent-docs/prism-boolean-design.md §3.4), while motions composing back to the
// partner's own placement leave the re-expression the identity.
func Transform(ctx context.Context, body *Body, t r3.Transform) (*Body, error) {
	placed, err := transform(ctx, body, t, OperationTransform)
	return placed, operationError(OperationTransform, err)
}

func transform(ctx context.Context, body *Body, t r3.Transform, kind OperationKind,
	additionalInputs ...OperationInput) (*Body, error) {
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
		facetedrecord.BindCopyLineage(bodyNode(placed), source.CopyLineage.Source, composed, false, nil)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	inputs := append([]OperationInput{{Body: body, Role: operationRoleSource, Use: InputConsumed}}, additionalInputs...)
	if err := d.commitOperationIn(ctx, nil, kind, inputs, placed, body); err != nil {
		return nil, err
	}
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
// A faceted copy made this way keeps the analytic tag of a face that a mesh
// Boolean retained from its operands (a plane, cylinder, sphere or torus), with
// a NormalAt bound that charges the float image of the tag against the exact
// rotation; a retained cone keeps the bound 2. Every other face reads as
// Faceted with its held bound, so Face.NormalAt refuses it and ToFace stops
// refuse it. A rotated plane tag is a float image, so MirrorFace refuses it.
// The copy also stops being a point-loft source. Use WithCopyPlacement for a
// copy that needs those readings.
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
func Copy(ctx context.Context, source *Body, opts ...CopyOption) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationCopy, retErr) }()
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
	return source.copyUnder(ctx, motion, exact, OperationCopy)
}

// copyUnder leaves its source live while re-evaluating at a new identity.
// It mirrors Transform's gate order and commit (core §8, agent-docs/api-design.md H4).
// A non-nil exact record asks a faceted payload for the exact-rotation
// placement; every other payload places by motion alone.
func (b *Body) copyUnder(ctx context.Context, motion r3.Transform, exact *exactrotation.Rotation,
	kind OperationKind, additionalInputs ...OperationInput) (*Body, error) {
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
		source := bodyNode(b)
		if record := faceted.CopyLineage; record != nil {
			source = record.Source
		}
		facetedrecord.BindCopyLineage(bodyNode(copied), source, composed, faceted.CopyLineage == nil, exact)
	}
	if kind == OperationCopy && isFaceted && exact != nil {
		evidence, err := massdispatch.CaptureExactCopyMoments(ctx, bodyNode(b), bodyNode(copied), exact)
		if err != nil {
			return nil, err
		}
		if evidence != nil {
			copiedPayload, ok := copied.n.Payload.(facetedPayload)
			if !ok {
				return nil, fmt.Errorf("faceted exact-copy rebuild returned %T", copied.n.Payload)
			}
			copiedPayload.ExactCopyMoments = evidence
			copied.n.Payload = copiedPayload
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	// The source is depended on, not consumed: commit with no consumed inputs,
	// so the retire rule never touches it.
	inputs := append([]OperationInput{{Body: b, Role: operationRoleSource, Use: InputReferenced}}, additionalInputs...)
	if err := d.commitOperationIn(ctx, d.bodyContainer(b), kind, inputs, copied); err != nil {
		return nil, err
	}
	return copied, nil
}

// originProducer returns the private identity of the operation that produced this body.
func (b *Body) originProducer() producerID { return b.n.Origin.Producer }

// producerID is the root spelling of provenance.Producer: the document-local
// identity assigned to one successful model operation. It supports provenance
// while keeping the model's execution bookkeeping private.
type producerID = provenance.Producer

func zeroVec(v r3.Vec) bool { return v == (r3.Vec{}) }
