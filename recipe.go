package decad

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"slices"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the Recipe IR of docs/api-design.md §6.2: the exact record of
// intent, a real value with no pointer into a live document or sketch.
// Every type reachable from a Recipe encodes and decodes — the sealed sets
// ship their tagged codecs — and a Step holds only values: bodies as
// StepRefs, the profile and plane as records, quantities as units.Value.

// Recipe is the exact record of intent: an ordered, immutable list of steps —
// the model, exactly as meant. It is the library's actual deliverable
// (core §2): declarative, kernel-independent, and serializable, which is what
// lets a second evaluator re-run it and makes emitting CAD code from it
// mechanical.
type Recipe struct {
	Steps []Step `json:"steps"`
}

// StepRef refers to the Step that produced a body. It is NOT a topology
// index (core §3 invariant #3): it names a step in this recipe, and the step
// list is the recipe's own content — step 2 is step 2 under every evaluator,
// forever.
type StepRef int

// bodyRef seals StepRef into BodyRef: a recorded — or decoded — Recipe names
// bodies by the step that produced them.
func (StepRef) bodyRef() {}

// BodyRef names the body a selector or extent resolves against. A live *Body
// is one (it seals in with the topology model), which is what a caller
// passes; a StepRef is one, which is what a recorded Recipe holds. A StepRef
// handed to a feature call, where a live body is required, is
// ErrUnresolvedBody (core §6.2).
type BodyRef interface{ bodyRef() }

// Selector is what a Step may carry: an unresolved edge or face query. The
// root is sealed so a Step never holds an `any`; the variants are *EdgeQuery
// and *FaceQuery (selector.go), which is what makes every selector a feature
// accepts a value a Recipe can record (core §9/§6.2).
type Selector interface{ selector() }

// StepOpts is the sealed per-op options record: one struct per OpKind that
// has options, never a stringly map (core §6.2). Every functional option a
// feature accepts MUST be representable here, or the recipe would be a lossy
// record of intent.
type StepOpts interface{ stepOpts() }

// ExtrudeOpts records an extrude's options. Taper is a SIGNED displacement
// angle — which way the wall leans — so it is outside ErrNegativeMagnitude
// (core §12); a nonzero taper is recorded exactly and is ErrUnsupported in
// evaluator v1 (docs/evaluator-design.md §5).
type ExtrudeOpts struct {
	Taper units.Value `json:"taper"`
}

func (ExtrudeOpts) stepOpts() {}

// ShellOpts records a shell's options (docs/modify-design.md §8). Sense is the
// wall sense — Inward (the default) or Outward — encoded as a named text token
// exactly as Direction is, so a renumbered constant could never silently
// reinterpret an old recipe. It is the StepOpts variant a shell step records.
type ShellOpts struct {
	Sense ShellSense `json:"sense"`
}

func (ShellOpts) stepOpts() {}

// OpKind names the operation a Step records.
type OpKind int

const (
	// OpExtrude sweeps a recorded profile along its plane normal.
	OpExtrude OpKind = iota
	// OpRevolve sweeps a recorded profile about an axis.
	OpRevolve
	// OpUnion combines two bodies.
	OpUnion
	// OpCut removes the tool from the target; Inputs order is [target, tool].
	OpCut
	// OpIntersect keeps the two bodies' common volume.
	OpIntersect
	// OpFillet rounds selected edges.
	OpFillet
	// OpChamfer bevels selected edges.
	OpChamfer
	// OpShell hollows a body through selected faces.
	OpShell
	// OpPlaced applies a recorded rigid placement.
	OpPlaced
	// OpDuplicate re-registers a body's geometry unchanged, without consuming
	// the source; it records no placement.
	OpDuplicate
	// OpPlacedCopy re-registers a body under a recorded rigid placement without
	// consuming the source.
	OpPlacedCopy
)

// opKindNames is the stable wire vocabulary; the constant order is never a
// serialization concern because the codec goes through these names.
var opKindNames = map[OpKind]string{
	OpExtrude:    "extrude",
	OpRevolve:    "revolve",
	OpUnion:      "union",
	OpCut:        "cut",
	OpIntersect:  "intersect",
	OpFillet:     "fillet",
	OpChamfer:    "chamfer",
	OpShell:      "shell",
	OpPlaced:     "placed",
	OpDuplicate:  "duplicate",
	OpPlacedCopy: "placed_copy",
}

// String renders the op kind for diagnostics.
func (k OpKind) String() string {
	if name, ok := opKindNames[k]; ok {
		return name
	}
	return fmt.Sprintf("OpKind(%d)", int(k))
}

// MarshalText encodes the op by name; an unknown value refuses to encode.
func (k OpKind) MarshalText() ([]byte, error) {
	name, ok := opKindNames[k]
	if !ok {
		return nil, fmt.Errorf(`decad: unknown op kind %d`, int(k))
	}
	return []byte(name), nil
}

// UnmarshalText decodes an op by name; an unknown name is an error.
func (k *OpKind) UnmarshalText(text []byte) error {
	for kind, name := range opKindNames {
		if name == string(text) {
			*k = kind
			return nil
		}
	}
	return fmt.Errorf(`decad: unknown op kind %q`, string(text))
}

// Step is one recorded operation: the exact statement of intent, complete
// enough to re-evaluate under any evaluator and to emit equivalent CAD code
// (core §6.2's completeness rule). Every Step produces exactly one body, so
// a StepRef names it without ambiguity.
type Step struct {
	// Op is the operation kind.
	Op OpKind
	// Inputs are the bodies this step depends on, as StepRefs — consumed
	// operands, and the bodies a body-relative extent resolves against
	// (extent order first, deduplicated). Cut's order is [target, tool].
	Inputs []StepRef
	// Profile and Plane are the recorded region and the sketch plane that
	// lifts it into world space (Extrude/Revolve).
	Profile ProfileRecord
	Plane   PlaneRecord
	// Extent is the linear extent (Extrude only; nil otherwise).
	Extent Extent
	// Angular is the angular extent (Revolve only; nil otherwise). At most
	// one of Extent and Angular is non-nil, keyed to Op (core §6.2).
	Angular AngularExtent
	// Axis is what the revolve spins about (Revolve only; nil otherwise,
	// keyed to Op like Angular).
	Axis Axis
	// Placement is the recorded rigid motion (Placed only; zero otherwise).
	Placement TransformRecord
	// Selectors are the unresolved edge/face queries (Fillet/Chamfer/Shell).
	Selectors []Selector
	// Opts are the per-op options; nil when the op takes none.
	Opts StepOpts
	// Values are the step's literal quantities — radii, distances,
	// thicknesses. There is no parameter sublanguage: the caller's Go
	// function is the feature tree (core §6.2).
	Values []units.Value
}

// stepOptsKind and the codec below: StepOpts is a closed set decad owns.
const optsKindExtrude = "extrude"
const optsKindShell = "shell"

type jsonExtrudeOpts struct {
	Taper *units.Value `json:"taper"`
}

type jsonShellOpts struct {
	Sense *ShellSense `json:"sense"`
}

// marshalStepOpts encodes one options record as its tagged object. Pointer
// forms normalize to values — the sealed set uses value receivers, so a
// *ExtrudeOpts satisfies StepOpts too — and a nil pointer is rejected.
func marshalStepOpts(o StepOpts) ([]byte, error) {
	if p, ok := o.(*ExtrudeOpts); ok {
		if p == nil {
			return nil, fmt.Errorf(`decad: nil step options`)
		}
		o = *p
	}
	if p, ok := o.(*ShellOpts); ok {
		if p == nil {
			return nil, fmt.Errorf(`decad: nil step options`)
		}
		o = *p
	}
	switch o := o.(type) {
	case ExtrudeOpts:
		return marshalTagged(optsKindExtrude, o)
	case ShellOpts:
		return marshalTagged(optsKindShell, o)
	default:
		return nil, fmt.Errorf(`decad: unencodable step options type %T`, o)
	}
}

// unmarshalStepOpts dispatches on the kind tag. Each variant decodes through
// pointer payload fields so a missing or explicit-null required value never
// becomes the value form's zero/default.
func unmarshalStepOpts(data []byte) (StepOpts, error) {
	var probe struct {
		Kind string `json:"kind"`
	}
	if err := json.Unmarshal(data, &probe); err != nil {
		return nil, codecJSONErrorAt(data, &probe, fmt.Errorf(`decad: failed to decode step options tag: %w`, err))
	}
	switch probe.Kind {
	case optsKindExtrude:
		var wire jsonExtrudeOpts
		if err := json.Unmarshal(data, &wire); err != nil {
			return nil, codecJSONErrorAt(data, &wire, fmt.Errorf(`decad: failed to decode extrude options: %w`, err))
		}
		if wire.Taper == nil {
			return nil, prependCodecPath(fmt.Errorf(`decad: extrude options are missing taper`), "taper")
		}
		return ExtrudeOpts{Taper: *wire.Taper}, nil
	case optsKindShell:
		var wire jsonShellOpts
		if err := json.Unmarshal(data, &wire); err != nil {
			return nil, codecJSONErrorAt(data, &wire, fmt.Errorf(`decad: failed to decode shell options: %w`, err))
		}
		if wire.Sense == nil {
			return nil, prependCodecPath(fmt.Errorf(`decad: shell options are missing sense`), "sense")
		}
		return ShellOpts{Sense: *wire.Sense}, nil
	case "":
		return nil, prependCodecPath(fmt.Errorf(`decad: step options are missing their kind tag`), "kind")
	default:
		return nil, prependCodecPath(fmt.Errorf(`decad: unknown step options kind %q`, probe.Kind), "kind")
	}
}

// jsonStep is Step's shallow wire shape. Every field except Op remains raw so
// the operation shape can be checked before typed payload decoding. Its field
// set is the single source of truth for the accepted step keys.
type jsonStep struct {
	// Op is a pointer so a missing "op" is distinguishable from the zero
	// kind: a step with no op is malformed, never silently an extrude.
	Op      *OpKind         `json:"op"`
	Inputs  json.RawMessage `json:"inputs,omitempty"`
	Profile json.RawMessage `json:"profile,omitempty"`
	Plane   json.RawMessage `json:"plane,omitempty"`
	Extent  json.RawMessage `json:"extent,omitempty"`
	Angular json.RawMessage `json:"angular,omitempty"`
	Axis    json.RawMessage `json:"axis,omitempty"`
	// Placement is a RawMessage, not a *TransformRecord, so a wire-present
	// field is distinguishable from an absent one: encoding/json decodes an
	// explicit null to a nil pointer — indistinguishable from a missing key —
	// but leaves the raw bytes "null" here, so the presence-aware keying below
	// can reject a placement keyed to a forbidding op whether it is written
	// null, {}, or a real motion.
	Placement json.RawMessage `json:"placement,omitempty"`
	Selectors json.RawMessage `json:"selectors,omitempty"`
	Opts      json.RawMessage `json:"opts,omitempty"`
	Values    json.RawMessage `json:"values,omitempty"`
}

// jsonStepDecode keeps fields raw until Step.UnmarshalJSON can attach paths.
type jsonStepDecode struct {
	Op        json.RawMessage `json:"op"`
	Inputs    json.RawMessage `json:"inputs"`
	Profile   json.RawMessage `json:"profile"`
	Plane     json.RawMessage `json:"plane"`
	Extent    json.RawMessage `json:"extent"`
	Angular   json.RawMessage `json:"angular"`
	Axis      json.RawMessage `json:"axis"`
	Placement json.RawMessage `json:"placement"`
	Selectors json.RawMessage `json:"selectors"`
	Opts      json.RawMessage `json:"opts"`
	Values    json.RawMessage `json:"values"`
}

// zeroVec reports whether v is the zero vector.
func zeroVec(v r3.Vec) bool { return v == r3.Vec{} }

// validateExtentKeying enforces the core §6.2 one-of contract on both wire
// directions: at most one of Extent and Angular is non-nil, and each is
// keyed to its op — Extent to extrude, Angular and Axis to revolve. A step
// violating it names no recordable intent, so it neither encodes nor
// decodes.
func validateExtentKeying(s Step) error {
	_, err := extentKeyingError(s)
	return err
}

func extentKeyingError(s Step) (string, error) {
	if s.Extent != nil && s.Angular != nil {
		return "extent", fmt.Errorf(`decad: a step carries at most one of extent and angular`)
	}
	if s.Extent != nil && s.Op != OpExtrude {
		return "extent", fmt.Errorf(`decad: a linear extent is keyed to the extrude op, got %q`, s.Op)
	}
	if s.Angular != nil && s.Op != OpRevolve {
		return "angular", fmt.Errorf(`decad: an angular extent is keyed to the revolve op, got %q`, s.Op)
	}
	if s.Axis != nil && s.Op != OpRevolve {
		return "axis", fmt.Errorf(`decad: a revolve axis is keyed to the revolve op, got %q`, s.Op)
	}
	return "", nil
}

type stepFieldPresence struct {
	profile   bool
	plane     bool
	extent    bool
	angular   bool
	axis      bool
	placement bool
	selectors bool
	opts      bool
	values    bool
}

type stepSelectorKind int

const (
	stepSelectorNone stepSelectorKind = iota
	stepSelectorEdge
	stepSelectorFace
)

type stepOptsKind int

const (
	stepOptsNone stepOptsKind = iota
	stepOptsExtrude
	stepOptsShell
)

type operationShape struct {
	minInputs int
	maxInputs int
	profile   bool
	plane     bool
	extent    bool
	angular   bool
	axis      bool
	placement bool
	selector  stepSelectorKind
	opts      stepOptsKind
	values    int
}

func shapeForOperation(op OpKind) (operationShape, bool) {
	switch op {
	case OpExtrude:
		return operationShape{
			minInputs: 0,
			maxInputs: -1,
			profile:   true,
			plane:     true,
			extent:    true,
			opts:      stepOptsExtrude,
		}, true
	case OpRevolve:
		return operationShape{
			minInputs: 0,
			maxInputs: -1,
			profile:   true,
			plane:     true,
			angular:   true,
			axis:      true,
		}, true
	case OpUnion, OpCut, OpIntersect:
		return operationShape{minInputs: 2, maxInputs: 2}, true
	case OpFillet, OpChamfer:
		return operationShape{
			minInputs: 1,
			maxInputs: 1,
			selector:  stepSelectorEdge,
			values:    1,
		}, true
	case OpShell:
		return operationShape{
			minInputs: 1,
			maxInputs: 1,
			selector:  stepSelectorFace,
			opts:      stepOptsShell,
			values:    1,
		}, true
	case OpPlaced:
		return operationShape{minInputs: 1, maxInputs: 1, placement: true}, true
	case OpDuplicate:
		return operationShape{minInputs: 1, maxInputs: 1}, true
	case OpPlacedCopy:
		return operationShape{minInputs: 1, maxInputs: 1, placement: true}, true
	default:
		return operationShape{}, false
	}
}

func nonzeroProfile(p ProfileRecord) bool {
	return len(p.Outer.Segments) > 0 || len(p.Holes) > 0
}

func nonzeroPlane(p PlaneRecord) bool {
	return !zeroVec(p.U) || !zeroVec(p.V) || !zeroVec(p.Origin)
}

// nonzeroPlacement reports whether a placement record carries a motion — the
// same test MarshalJSON uses to decide whether to emit the field. A placed copy
// records a motion here (OpPlacedCopy, r3.Identity() among them — its basis is
// nonzero) while a duplicate leaves it zero (OpDuplicate). It is the VALIDITY
// half of the placement keying: a nonzero record is present, the zero record
// is none. Full transform validation belongs to recipe record validation.
func nonzeroPlacement(p TransformRecord) bool {
	return !zeroVec(p.EX) || !zeroVec(p.EY) || !zeroVec(p.EZ) || !zeroVec(p.T)
}

// validatePlacementKeying enforces the Placement keying on both wire
// directions, presence-aware and bidirectional. OpPlaced and OpPlacedCopy
// REQUIRE a placement — the field must be present AND a valid (nonzero) motion,
// so an absent or zero-value placement for these ops names no recordable
// intent. Every OTHER op (OpDuplicate among them) FORBIDS the field entirely:
// a present placement, even a zero-value one, is rejected.
func validatePlacementKeying(op OpKind, present bool, p TransformRecord) error {
	if op == OpPlaced || op == OpPlacedCopy {
		if !present || !nonzeroPlacement(p) {
			return fmt.Errorf(`decad: the %q op requires a placement`, op)
		}
		return nil
	}
	if present {
		return fmt.Errorf(`decad: a placement is keyed to the placed and placed_copy ops, got %q`, op)
	}
	return nil
}

func marshalStepPresence(s Step) stepFieldPresence {
	return stepFieldPresence{
		profile:   nonzeroProfile(s.Profile),
		plane:     nonzeroPlane(s.Plane),
		extent:    s.Extent != nil,
		angular:   s.Angular != nil,
		axis:      s.Axis != nil,
		placement: nonzeroPlacement(s.Placement),
		selectors: len(s.Selectors) > 0,
		opts:      s.Opts != nil,
		values:    len(s.Values) > 0,
	}
}

func unmarshalStepPresence(raw jsonStepDecode) stepFieldPresence {
	return stepFieldPresence{
		profile:   raw.Profile != nil,
		plane:     raw.Plane != nil,
		extent:    raw.Extent != nil,
		angular:   raw.Angular != nil,
		axis:      raw.Axis != nil,
		placement: raw.Placement != nil,
		selectors: raw.Selectors != nil,
		opts:      raw.Opts != nil,
		values:    raw.Values != nil,
	}
}

type stepShapeFields struct {
	inputs           int
	inputsPresent    bool
	profile          bool
	plane            bool
	extent           bool
	angular          bool
	axis             bool
	placement        bool
	selectors        int
	selectorsPresent bool
	opts             bool
	values           int
	valuesPresent    bool
}

func shapeWireFieldPresent(data json.RawMessage, required bool) bool {
	if data == nil {
		return false
	}
	return !required || !isJSONNull(data)
}

func stepShapeFieldsFromJSON(raw jsonStepDecode, op OpKind) (stepShapeFields, error) {
	shape, ok := shapeForOperation(op)
	if !ok {
		return stepShapeFields{}, fmt.Errorf(`decad: unknown op kind %d`, int(op))
	}
	present := unmarshalStepPresence(raw)
	fields := stepShapeFields{
		inputsPresent:    raw.Inputs != nil,
		profile:          shapeWireFieldPresent(raw.Profile, shape.profile),
		plane:            shapeWireFieldPresent(raw.Plane, shape.plane),
		extent:           shapeWireFieldPresent(raw.Extent, shape.extent),
		angular:          shapeWireFieldPresent(raw.Angular, shape.angular),
		axis:             shapeWireFieldPresent(raw.Axis, shape.axis),
		placement:        shapeWireFieldPresent(raw.Placement, shape.placement),
		selectorsPresent: present.selectors,
		opts:             shapeWireFieldPresent(raw.Opts, shape.opts != stepOptsNone),
		valuesPresent:    present.values,
	}
	if err := validateStepShapePresence(op, fields); err != nil {
		return stepShapeFields{}, err
	}

	inputs, err := jsonArrayLength(raw.Inputs, "inputs")
	if err != nil {
		return stepShapeFields{}, err
	}
	selectors, err := jsonArrayLength(raw.Selectors, "selectors")
	if err != nil {
		return stepShapeFields{}, err
	}
	values, err := jsonArrayLength(raw.Values, "values")
	if err != nil {
		return stepShapeFields{}, err
	}
	fields.inputs = inputs
	fields.selectors = selectors
	fields.values = values
	return fields, nil
}

func jsonArrayLength(data json.RawMessage, field string) (int, error) {
	if data == nil || isJSONNull(data) {
		return 0, nil
	}
	var values []json.RawMessage
	if err := json.Unmarshal(data, &values); err != nil {
		return 0, fmt.Errorf(`decad: step field %q must be an array: %w`, field, err)
	}
	return len(values), nil
}

func unmarshalPresentJSONSlice[T any](data json.RawMessage, field string) ([]T, error) {
	if isJSONNull(data) {
		return nil, fmt.Errorf(`decad: step field %q must be an array`, field)
	}
	var raw []json.RawMessage
	if err := json.Unmarshal(data, &raw); err != nil {
		return nil, fmt.Errorf(`decad: step field %q must be an array: %w`, field, err)
	}
	values := make([]T, len(raw))
	for i, element := range raw {
		if isJSONNull(element) {
			return nil, fmt.Errorf(`decad: step field %q element %d must not be null`, field, i)
		}
		if err := json.Unmarshal(element, &values[i]); err != nil {
			return nil, fmt.Errorf(`decad: failed to decode %s[%d]: %w`, field, i, err)
		}
	}
	return values, nil
}

func validStepOptsKind(opts StepOpts, want stepOptsKind) bool {
	switch want {
	case stepOptsExtrude:
		switch o := opts.(type) {
		case ExtrudeOpts:
			return true
		case *ExtrudeOpts:
			return o != nil
		}
	case stepOptsShell:
		switch o := opts.(type) {
		case ShellOpts:
			return true
		case *ShellOpts:
			return o != nil
		}
	}
	return false
}

func validateStepShapePresence(op OpKind, fields stepShapeFields) error {
	shape, ok := shapeForOperation(op)
	if !ok {
		return fmt.Errorf(`decad: unknown op kind %d`, int(op))
	}

	if shape.minInputs > 0 && !fields.inputsPresent {
		if shape.maxInputs == shape.minInputs {
			return fmt.Errorf(`decad: the %q op requires exactly %d inputs`, op, shape.minInputs)
		}
		return fmt.Errorf(`decad: the %q op requires at least %d inputs`, op, shape.minInputs)
	}
	if shape.profile {
		if !fields.profile {
			return fmt.Errorf(`decad: the %q op requires a non-empty profile`, op)
		}
	} else if fields.profile {
		return fmt.Errorf(`decad: the %q op forbids a profile`, op)
	}
	if shape.plane {
		if !fields.plane {
			return fmt.Errorf(`decad: the %q op requires a plane`, op)
		}
	} else if fields.plane {
		return fmt.Errorf(`decad: the %q op forbids a plane`, op)
	}
	if fields.extent != shape.extent {
		if shape.extent {
			return fmt.Errorf(`decad: the %q op requires a linear extent`, op)
		}
		return fmt.Errorf(`decad: the %q op forbids a linear extent`, op)
	}
	if fields.angular != shape.angular {
		if shape.angular {
			return fmt.Errorf(`decad: the %q op requires an angular extent`, op)
		}
		return fmt.Errorf(`decad: the %q op forbids an angular extent`, op)
	}
	if fields.axis != shape.axis {
		if shape.axis {
			return fmt.Errorf(`decad: the %q op requires an axis`, op)
		}
		return fmt.Errorf(`decad: the %q op forbids an axis`, op)
	}
	if shape.placement {
		if !fields.placement {
			return fmt.Errorf(`decad: the %q op requires a placement`, op)
		}
	} else if fields.placement {
		return fmt.Errorf(`decad: the %q op forbids a placement`, op)
	}

	switch shape.selector {
	case stepSelectorNone:
		if fields.selectorsPresent {
			return fmt.Errorf(`decad: the %q op forbids selectors`, op)
		}
	case stepSelectorEdge, stepSelectorFace:
		if !fields.selectorsPresent {
			if shape.selector == stepSelectorEdge {
				return fmt.Errorf(`decad: the %q op requires exactly one edge selector`, op)
			}
			return fmt.Errorf(`decad: the %q op requires exactly one face selector`, op)
		}
	}

	if shape.opts == stepOptsNone {
		if fields.opts {
			return fmt.Errorf(`decad: the %q op forbids options`, op)
		}
	} else if !fields.opts {
		return fmt.Errorf(`decad: the %q op requires its matching options`, op)
	}

	if shape.values == 0 {
		if fields.valuesPresent {
			return fmt.Errorf(`decad: the %q op forbids values`, op)
		}
	} else if !fields.valuesPresent {
		return fmt.Errorf(`decad: the %q op requires exactly %d values`, op, shape.values)
	}
	return nil
}

func validateStepShapeFields(op OpKind, fields stepShapeFields) (operationShape, error) {
	shape, ok := shapeForOperation(op)
	if !ok {
		return operationShape{}, fmt.Errorf(`decad: unknown op kind %d`, int(op))
	}

	if fields.inputs < shape.minInputs || (shape.maxInputs >= 0 && fields.inputs > shape.maxInputs) {
		if shape.maxInputs == shape.minInputs {
			return operationShape{}, fmt.Errorf(`decad: the %q op requires exactly %d inputs`, op, shape.minInputs)
		}
		return operationShape{}, fmt.Errorf(`decad: the %q op requires at least %d inputs`, op, shape.minInputs)
	}

	if shape.profile {
		if !fields.profile {
			return operationShape{}, fmt.Errorf(`decad: the %q op requires a non-empty profile`, op)
		}
	} else if fields.profile {
		return operationShape{}, fmt.Errorf(`decad: the %q op forbids a profile`, op)
	}
	if shape.plane {
		if !fields.plane {
			return operationShape{}, fmt.Errorf(`decad: the %q op requires a plane`, op)
		}
	} else if fields.plane {
		return operationShape{}, fmt.Errorf(`decad: the %q op forbids a plane`, op)
	}
	if fields.extent != shape.extent {
		if shape.extent {
			return operationShape{}, fmt.Errorf(`decad: the %q op requires a linear extent`, op)
		}
		return operationShape{}, fmt.Errorf(`decad: the %q op forbids a linear extent`, op)
	}
	if fields.angular != shape.angular {
		if shape.angular {
			return operationShape{}, fmt.Errorf(`decad: the %q op requires an angular extent`, op)
		}
		return operationShape{}, fmt.Errorf(`decad: the %q op forbids an angular extent`, op)
	}
	if fields.axis != shape.axis {
		if shape.axis {
			return operationShape{}, fmt.Errorf(`decad: the %q op requires an axis`, op)
		}
		return operationShape{}, fmt.Errorf(`decad: the %q op forbids an axis`, op)
	}
	if shape.placement {
		if !fields.placement {
			return operationShape{}, fmt.Errorf(`decad: the %q op requires a placement`, op)
		}
	} else if fields.placement {
		return operationShape{}, fmt.Errorf(`decad: the %q op forbids a placement`, op)
	}

	switch shape.selector {
	case stepSelectorNone:
		if fields.selectorsPresent {
			return operationShape{}, fmt.Errorf(`decad: the %q op forbids selectors`, op)
		}
	case stepSelectorEdge:
		if fields.selectors != 1 {
			return operationShape{}, fmt.Errorf(`decad: the %q op requires exactly one edge selector`, op)
		}
	case stepSelectorFace:
		if fields.selectors != 1 {
			return operationShape{}, fmt.Errorf(`decad: the %q op requires exactly one face selector`, op)
		}
	}

	if shape.opts == stepOptsNone {
		if fields.opts {
			return operationShape{}, fmt.Errorf(`decad: the %q op forbids options`, op)
		}
	} else if !fields.opts {
		return operationShape{}, fmt.Errorf(`decad: the %q op requires its matching options`, op)
	}

	if shape.values == 0 {
		if fields.valuesPresent {
			return operationShape{}, fmt.Errorf(`decad: the %q op forbids values`, op)
		}
		return shape, nil
	}
	if fields.values != shape.values {
		return operationShape{}, fmt.Errorf(`decad: the %q op requires exactly %d values`, op, shape.values)
	}
	return shape, nil
}

func validateStepShapeBase(s Step, shape operationShape) error {
	if shape.profile && len(s.Profile.Outer.Segments) == 0 {
		return fmt.Errorf(`decad: the %q op requires a non-empty profile`, s.Op)
	}
	if shape.plane && !nonzeroPlane(s.Plane) {
		return fmt.Errorf(`decad: the %q op requires a plane`, s.Op)
	}
	if shape.placement && !nonzeroPlacement(s.Placement) {
		return fmt.Errorf(`decad: the %q op requires a placement`, s.Op)
	}
	if shape.placement {
		if _, err := s.Placement.Transform(); err != nil {
			return err
		}
	}
	return nil
}

// validateStepShape is the complete core §6.2 per-operation structural gate.
// It is shared by both codec directions so caller-built and decoded Step
// values admit exactly the shapes immediate feature calls record.
func validateStepShape(s Step, present stepFieldPresence) error {
	shape, err := validateStepShapeFields(s.Op, stepShapeFields{
		inputs:           len(s.Inputs),
		profile:          present.profile,
		plane:            present.plane,
		extent:           present.extent,
		angular:          present.angular,
		axis:             present.axis,
		placement:        present.placement,
		selectors:        len(s.Selectors),
		selectorsPresent: present.selectors,
		opts:             present.opts,
		values:           len(s.Values),
		valuesPresent:    present.values,
	})
	if err != nil {
		return err
	}
	for i, ref := range s.Inputs {
		if slices.Contains(s.Inputs[:i], ref) {
			return fmt.Errorf(`decad: the %q op requires unique inputs`, s.Op)
		}
	}
	if err := validateStepShapeBase(s, shape); err != nil {
		return err
	}

	switch shape.selector {
	case stepSelectorEdge:
		q, ok := s.Selectors[0].(*EdgeQuery)
		if !ok {
			return fmt.Errorf(`decad: the %q op requires an edge selector`, s.Op)
		}
		if q == nil {
			return errNilSelector
		}
	case stepSelectorFace:
		q, ok := s.Selectors[0].(*FaceQuery)
		if !ok {
			return fmt.Errorf(`decad: the %q op requires a face selector`, s.Op)
		}
		if q == nil {
			return errNilSelector
		}
	}
	if shape.opts != stepOptsNone && !validStepOptsKind(s.Opts, shape.opts) {
		return fmt.Errorf(`decad: the %q op requires its matching options`, s.Op)
	}
	return nil
}

// MarshalJSON encodes the step with every absent field omitted: a decoded
// step is the recorded one, field for field.
func (s Step) MarshalJSON() ([]byte, error) {
	if err := validateStepShape(s, marshalStepPresence(s)); err != nil {
		return nil, err
	}
	op := s.Op
	out := jsonStep{Op: &op}
	if len(s.Inputs) > 0 {
		raw, err := json.Marshal(s.Inputs)
		if err != nil {
			return nil, err
		}
		out.Inputs = raw
	}
	if nonzeroProfile(s.Profile) {
		raw, err := json.Marshal(s.Profile)
		if err != nil {
			return nil, err
		}
		out.Profile = raw
	}
	if nonzeroPlane(s.Plane) {
		raw, err := json.Marshal(s.Plane)
		if err != nil {
			return nil, err
		}
		out.Plane = raw
	}
	if s.Extent != nil {
		raw, err := marshalExtent(s.Extent)
		if err != nil {
			return nil, err
		}
		out.Extent = raw
	}
	if s.Angular != nil {
		raw, err := marshalAngularExtent(s.Angular)
		if err != nil {
			return nil, err
		}
		out.Angular = raw
	}
	if s.Axis != nil {
		raw, err := marshalAxis(s.Axis)
		if err != nil {
			return nil, err
		}
		out.Axis = raw
	}
	if nonzeroPlacement(s.Placement) {
		raw, err := json.Marshal(s.Placement)
		if err != nil {
			return nil, err
		}
		out.Placement = raw
	}
	if len(s.Selectors) > 0 {
		sels := make([]json.RawMessage, 0, len(s.Selectors))
		for _, sel := range s.Selectors {
			raw, err := marshalSelector(sel)
			if err != nil {
				return nil, err
			}
			sels = append(sels, raw)
		}
		raw, err := json.Marshal(sels)
		if err != nil {
			return nil, err
		}
		out.Selectors = raw
	}
	if s.Opts != nil {
		raw, err := marshalStepOpts(s.Opts)
		if err != nil {
			return nil, err
		}
		out.Opts = raw
	}
	if len(s.Values) > 0 {
		raw, err := json.Marshal(s.Values)
		if err != nil {
			return nil, err
		}
		out.Values = raw
	}
	return json.Marshal(out)
}

func decodeStrictJSONStep(data []byte, raw *jsonStepDecode) error {
	if err := rejectDuplicateJSONKeys(data); err != nil {
		return err
	}
	dec := json.NewDecoder(bytes.NewReader(data))
	dec.DisallowUnknownFields()
	if err := dec.Decode(raw); err != nil {
		return fmt.Errorf(`decad: failed to decode step: %w`, err)
	}
	var trailing json.RawMessage
	if err := dec.Decode(&trailing); err == nil {
		return fmt.Errorf(`decad: step contains more than one JSON value`)
	} else if err != io.EOF {
		return fmt.Errorf(`decad: failed to decode trailing step JSON: %w`, err)
	}
	return nil
}

// UnmarshalJSON decodes the wire shape, dispatching every tagged field
// through its own closed-set codec.
func (s *Step) UnmarshalJSON(data []byte) error {
	var raw jsonStepDecode
	if err := decodeStrictJSONStep(data, &raw); err != nil {
		return err
	}
	var op *OpKind
	if raw.Op != nil {
		if err := json.Unmarshal(raw.Op, &op); err != nil {
			return prependCodecPath(codecJSONError(err), "op")
		}
	}
	if op == nil {
		return prependCodecPath(fmt.Errorf(`decad: step is missing its op`), "op")
	}
	fields, err := stepShapeFieldsFromJSON(raw, *op)
	if err != nil {
		return err
	}
	shape, err := validateStepShapeFields(*op, fields)
	if err != nil {
		return err
	}
	present := unmarshalStepPresence(raw)
	out := Step{Op: *op}
	if raw.Profile != nil {
		var v *ProfileRecord
		if err := json.Unmarshal(raw.Profile, &v); err != nil {
			return prependCodecPath(codecJSONErrorAt(raw.Profile, &v, err), "profile")
		}
		if v != nil {
			out.Profile = *v
		}
	}
	if raw.Plane != nil {
		var v *PlaneRecord
		if err := json.Unmarshal(raw.Plane, &v); err != nil {
			return prependCodecPath(codecJSONErrorAt(raw.Plane, &v, err), "plane")
		}
		if v != nil {
			out.Plane = *v
		}
	}
	if present.placement {
		if err := json.Unmarshal(raw.Placement, &out.Placement); err != nil {
			return prependCodecPath(codecJSONErrorAt(raw.Placement, &out.Placement, fmt.Errorf(`decad: failed to decode placement: %w`, err)), "placement")
		}
	}
	if err := validateStepShapeBase(out, shape); err != nil {
		return err
	}
	if raw.Inputs != nil {
		inputs, err := unmarshalPresentJSONSlice[StepRef](raw.Inputs, "inputs")
		if err != nil {
			return fmt.Errorf(`decad: failed to decode inputs: %w`, err)
		}
		out.Inputs = inputs
	}
	if raw.Values != nil {
		values, err := unmarshalPresentJSONSlice[units.Value](raw.Values, "values")
		if err != nil {
			return fmt.Errorf(`decad: failed to decode values: %w`, err)
		}
		out.Values = values
	}
	if raw.Extent != nil {
		e, err := unmarshalExtent(raw.Extent)
		if err != nil {
			return prependCodecPath(err, "extent")
		}
		out.Extent = e
	}
	if raw.Angular != nil {
		a, err := unmarshalAngularExtent(raw.Angular)
		if err != nil {
			return prependCodecPath(err, "angular")
		}
		out.Angular = a
	}
	if raw.Axis != nil {
		a, err := unmarshalAxis(raw.Axis)
		if err != nil {
			return prependCodecPath(err, "axis")
		}
		out.Axis = a
	}
	if raw.Selectors != nil {
		var selectorData []json.RawMessage
		if err := json.Unmarshal(raw.Selectors, &selectorData); err != nil {
			return prependCodecPath(codecJSONError(err), "selectors")
		}
		sels := make([]Selector, 0, len(selectorData))
		for i, b := range selectorData {
			sel, err := unmarshalSelector(b)
			if err != nil {
				return prependCodecPath(err, fmt.Sprintf(`selectors[%d]`, i))
			}
			sels = append(sels, sel)
		}
		if len(selectorData) > 0 {
			out.Selectors = sels
		}
	}
	if raw.Opts != nil {
		o, err := unmarshalStepOpts(raw.Opts)
		if err != nil {
			return prependCodecPath(err, "opts")
		}
		out.Opts = o
	}
	if field, err := extentKeyingError(out); err != nil {
		return prependCodecPath(err, field)
	}
	// Presence is read from the wire, not the
	// flattened value: a present-but-zero placement ({} or an explicit null) on
	// a forbidding op must be rejected, and an absent one on a placing op must
	// be caught, neither of which the value alone reveals.
	if err := validatePlacementKeying(out.Op, present.placement, out.Placement); err != nil {
		return prependCodecPath(err, "placement")
	}
	if err := validateStepShape(out, present); err != nil {
		return err
	}
	*s = out
	return nil
}

// cloneSteps deep-copies recorded steps: a Recipe is a value (core §6.2), so
// nothing it hands out may alias document state — a caller mutating a
// returned recipe must never change future replay behavior.
func cloneSteps(steps []Step) []Step {
	out := make([]Step, len(steps))
	for i, s := range steps {
		out[i] = cloneStep(s)
	}
	return out
}

// cloneStep deep-copies one step's slice-bearing fields.
func cloneStep(s Step) Step {
	out := s
	out.Inputs = slices.Clone(s.Inputs)
	out.Values = slices.Clone(s.Values)
	// A selector variant is a pointer (the queries seal in with pointer
	// receivers), so cloning the slice alone would alias the caller's query;
	// each query is deep-copied instead.
	out.Selectors = cloneSelectors(s.Selectors)
	out.Profile = cloneProfileRecord(s.Profile)
	// An extent normalizes to pure values recursively (nested TwoSided sides
	// included) and a ToFace/ToFaceAngular's selector is deep-copied, so a
	// cloned step can never share a caller-visible pointer with the
	// document. A malformed nil pointer stays as-is — the codecs and the
	// feature calls reject it at their own gates.
	if s.Extent != nil {
		out.Extent = cloneExtent(s.Extent)
	}
	if s.Angular != nil {
		out.Angular = cloneAngularExtent(s.Angular)
	}
	// An axis normalizes to a value; an EdgeAxis's selector is deep-copied
	// so a recorded step never aliases a caller-owned query.
	if s.Axis != nil {
		out.Axis = cloneAxis(s.Axis)
	}
	return out
}

// cloneProfileRecord deep-copies a recorded region.
func cloneProfileRecord(r ProfileRecord) ProfileRecord {
	out := ProfileRecord{Outer: cloneLoopRecord(r.Outer)}
	if r.Holes != nil {
		out.Holes = make([]LoopRecord, len(r.Holes))
		for i, h := range r.Holes {
			out.Holes[i] = cloneLoopRecord(h)
		}
	}
	return out
}

// cloneLoopRecord deep-copies one loop's segments.
func cloneLoopRecord(l LoopRecord) LoopRecord {
	if l.Segments == nil {
		return LoopRecord{}
	}
	out := LoopRecord{Segments: make([]CurveSegment, len(l.Segments))}
	for i, seg := range l.Segments {
		out.Segments[i] = cloneSegment(seg)
	}
	return out
}

// cloneSegment deep-copies a segment's slice-bearing fields. Value variants
// with only scalar fields copy by assignment; pointer forms normalize to
// values first (a malformed nil pointer is kept as-is — the codecs and
// integrals reject it at their own gates).
func cloneSegment(seg CurveSegment) CurveSegment {
	normalized, err := normalizeSegment(seg)
	if err != nil {
		return seg
	}
	switch s := normalized.(type) {
	case SplineSeg:
		s.Control = slices.Clone(s.Control)
		return s
	case NURBSSeg:
		s.Control = slices.Clone(s.Control)
		s.Knots = slices.Clone(s.Knots)
		s.Weights = slices.Clone(s.Weights)
		return s
	case ClosedSplineSeg:
		s.Control = slices.Clone(s.Control)
		return s
	case FitSplineSeg:
		s.Fit = slices.Clone(s.Fit)
		return s
	default:
		return normalized
	}
}
