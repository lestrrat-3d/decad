package decad

import (
	"context"
	"maps"
	"slices"
	"sync"

	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/topo"
)

const (
	operationRoleSource  = "source"
	operationRoleTarget  = "target"
	operationRoleTool    = "tool"
	operationRoleOperand = "operand"
	operationUnknownName = "unknown"
)

type operationHistoryCapture struct {
	mu          sync.Mutex
	inputs      []OperationInput
	base        producerID
	span        producerID
	initialized bool
}

type operationHistorySuppressedKey struct{}

// privateConstructionContext suppresses public history for temporary geometry
// that a composite operation creates and retires before it returns.
func privateConstructionContext(ctx context.Context) (context.Context, *operationHistoryCapture) {
	capture := &operationHistoryCapture{}
	return context.WithValue(ctx, operationHistorySuppressedKey{}, capture), capture
}

func operationHistoryInputs(ctx context.Context, inputs []OperationInput) bool {
	capture, suppressed := ctx.Value(operationHistorySuppressedKey{}).(*operationHistoryCapture)
	if !suppressed {
		return true
	}
	capture.mu.Lock()
	capture.inputs = append(capture.inputs, inputs...)
	capture.mu.Unlock()
	return false
}

func (capture *operationHistoryCapture) nextProducerID(d *Document) producerID {
	capture.mu.Lock()
	defer capture.mu.Unlock()
	if !capture.initialized {
		capture.base = d.n.NextProducer
		capture.initialized = true
	}
	return capture.base + capture.span
}

func (capture *operationHistoryCapture) producerSpan(d *Document) producerID {
	capture.mu.Lock()
	defer capture.mu.Unlock()
	if !capture.initialized {
		capture.base = d.n.NextProducer
		capture.initialized = true
	}
	return capture.span
}

func (capture *operationHistoryCapture) advanceProducerSpan(span producerID) {
	capture.mu.Lock()
	capture.span += span
	capture.mu.Unlock()
}

func (capture *operationHistoryCapture) snapshot() []OperationInput {
	capture.mu.Lock()
	defer capture.mu.Unlock()
	return slices.Clone(capture.inputs)
}

// InputUse describes whether a call retired or only read an input body.
type InputUse int

const (
	InputUseUnknown InputUse = iota
	InputConsumed
	InputReferenced
)

// OperationInput identifies a body argument and its public role.
type OperationInput struct {
	Body *Body
	Role string
	Use  InputUse
}

// HistoryCoverage reports whether a face correspondence is exhaustive.
type HistoryCoverage int

const (
	HistoryUnknown HistoryCoverage = iota
	HistoryComplete
)

// FaceHistory contains the known output faces derived from one input face.
type FaceHistory struct {
	coverage HistoryCoverage
	faces    []*Face
}

// Coverage reports whether Faces lists every successor for this input face.
func (h FaceHistory) Coverage() HistoryCoverage { return h.coverage }

// Faces returns a copy of the known successor faces.
func (h FaceHistory) Faces() []*Face { return slices.Clone(h.faces) }

// Operation is one successful public modeling call.
type Operation struct {
	kind           OperationKind
	inputs         []OperationInput
	outputs        []*Body
	faceSuccessors map[*Face]FaceHistory
	doc            *topo.Document
}

// Kind reports the public entry point used for this operation.
func (o *Operation) Kind() OperationKind { return o.kind }

// Inputs returns the ordered body inputs for this call.
func (o *Operation) Inputs() []OperationInput { return slices.Clone(o.inputs) }

// Outputs returns the bodies returned by this call.
func (o *Operation) Outputs() []*Body { return slices.Clone(o.outputs) }

// FaceSuccessors returns known face contributors and their coverage.
func (o *Operation) FaceSuccessors(face *Face) (FaceHistory, error) {
	if o == nil {
		return FaceHistory{}, ErrDegenerate
	}
	if face == nil || face.n.Owner == nil || face.n.Body == nil {
		return FaceHistory{}, ErrDegenerate
	}
	if face.n.Body.Doc != o.doc {
		return FaceHistory{}, ErrForeignBody
	}
	for _, input := range o.inputs {
		for _, candidate := range input.Body.Faces() {
			if candidate == face {
				if history, ok := o.faceSuccessors[face]; ok {
					history.faces = slices.Clone(history.faces)
					return history, nil
				}
				return FaceHistory{coverage: HistoryUnknown}, nil
			}
		}
	}
	return FaceHistory{}, ErrNoMatch
}

// OperationHistory is an immutable snapshot of a document's successful calls.
type OperationHistory struct {
	doc        *topo.Document
	operations []*Operation
	producers  map[*Body]*Operation
	consumers  map[*Body][]*Operation
}

// OperationHistory returns an immutable snapshot of all recorded calls.
func (d *Document) OperationHistory() *OperationHistory {
	if d == nil {
		return &OperationHistory{}
	}
	d.n.LiveMu.RLock()
	defer d.n.LiveMu.RUnlock()
	h := &OperationHistory{
		doc:        &d.n,
		operations: slices.Clone(d.operations),
		producers:  make(map[*Body]*Operation, len(d.producers)),
		consumers:  make(map[*Body][]*Operation, len(d.consumers)),
	}
	maps.Copy(h.producers, d.producers)
	for body, operations := range d.consumers {
		h.consumers[body] = slices.Clone(operations)
	}
	return h
}

// Operations returns successful public calls in commit order.
func (h *OperationHistory) Operations() []*Operation {
	if h == nil {
		return nil
	}
	return slices.Clone(h.operations)
}

// Producer returns the call that returned body, including when body is retired.
func (h *OperationHistory) Producer(body *Body) (*Operation, error) {
	if h == nil {
		return nil, ErrDegenerate
	}
	if body == nil || body.n.Doc == nil {
		return nil, ErrDegenerate
	}
	if body.n.Doc != h.doc {
		return nil, ErrForeignBody
	}
	operation, ok := h.producers[body]
	if !ok {
		return nil, ErrNoMatch
	}
	return operation, nil
}

// Consumers returns calls that consumed body within this snapshot.
func (h *OperationHistory) Consumers(body *Body) ([]*Operation, error) {
	if h == nil {
		return nil, ErrDegenerate
	}
	if body == nil || body.n.Doc == nil {
		return nil, ErrDegenerate
	}
	if body.n.Doc != h.doc {
		return nil, ErrForeignBody
	}
	if _, ok := h.producers[body]; !ok {
		return nil, ErrNoMatch
	}
	return slices.Clone(h.consumers[body]), nil
}

func (d *Document) appendOperationLocked(operation *Operation) {
	d.operations = append(d.operations, operation)
	for _, output := range operation.outputs {
		d.producers[output] = operation
	}
	seen := make(map[*Body]struct{}, len(operation.inputs))
	for _, input := range operation.inputs {
		if input.Use != InputConsumed {
			continue
		}
		if _, ok := seen[input.Body]; ok {
			continue
		}
		seen[input.Body] = struct{}{}
		d.consumers[input.Body] = append(d.consumers[input.Body], operation)
	}
}

func stampOperationWarningKind(body *Body, kind OperationKind) {
	for i := range body.n.OperationWarnings {
		code := body.n.OperationWarnings[i].Code
		if code == opdiagnostic.CodeEmbeddingReachApplied || code == opdiagnostic.CodeGapShiftApplied ||
			code == opdiagnostic.CodeChainAllowanceApplied {
			body.n.OperationWarnings[i].PublicOperation = int(kind)
		}
	}
}

func cloneFaceSuccessors(in map[*Face]FaceHistory) map[*Face]FaceHistory {
	if len(in) == 0 {
		return nil
	}
	out := make(map[*Face]FaceHistory, len(in))
	for face, history := range in {
		history.faces = slices.Clone(history.faces)
		out[face] = history
	}
	return out
}

func (d *Document) operation(kind OperationKind, inputs []OperationInput, outputs ...*Body) *Operation {
	return &Operation{kind: kind, inputs: slices.Clone(inputs), outputs: slices.Clone(outputs), doc: &d.n}
}

func referencedInputs(bodies []*Body, role string) []OperationInput {
	inputs := make([]OperationInput, len(bodies))
	for i, body := range bodies {
		inputs[i] = OperationInput{Body: body, Role: role, Use: InputReferenced}
	}
	return inputs
}
