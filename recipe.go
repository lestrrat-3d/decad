package decad

import (
	"encoding/json"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the Recipe IR of docs/api-design.md §6.2: the exact record of
// intent, a real value with no pointer into a live document or sketch.
// Every type reachable from a Recipe encodes and decodes — the sealed sets
// ship their tagged codecs — and a Step holds only values: bodies as
// StepRefs, the profile and plane as records, quantities as units.Value.
//
// The Angular and Axis fields of a revolve Step join with the Revolve
// increment (docs/evaluator-design.md §6/§11); adding fields to Step is
// additive, and an extrude-only recipe round-trips identically before and
// after they land.

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
// query types seal in with the selector vocabulary
// (docs/evaluator-design.md §7/§11); the root exists from the start so a
// Step never holds an `any`.
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
)

// opKindNames is the stable wire vocabulary; the constant order is never a
// serialization concern because the codec goes through these names.
var opKindNames = map[OpKind]string{
	OpExtrude:   "extrude",
	OpRevolve:   "revolve",
	OpUnion:     "union",
	OpCut:       "cut",
	OpIntersect: "intersect",
	OpFillet:    "fillet",
	OpChamfer:   "chamfer",
	OpShell:     "shell",
	OpPlaced:    "placed",
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
	switch o := o.(type) {
	case ExtrudeOpts:
		return marshalTagged(optsKindExtrude, o)
	default:
		return nil, fmt.Errorf(`decad: unencodable step options type %T`, o)
	}
}

// unmarshalStepOpts dispatches on the kind tag.
func unmarshalStepOpts(data []byte) (StepOpts, error) {
	var probe struct {
		Kind string `json:"kind"`
	}
	if err := json.Unmarshal(data, &probe); err != nil {
		return nil, fmt.Errorf(`decad: failed to decode step options tag: %w`, err)
	}
	switch probe.Kind {
	case optsKindExtrude:
		var o ExtrudeOpts
		if err := json.Unmarshal(data, &o); err != nil {
			return nil, fmt.Errorf(`decad: failed to decode extrude options: %w`, err)
		}
		return o, nil
	case "":
		return nil, fmt.Errorf(`decad: step options are missing their kind tag`)
	default:
		return nil, fmt.Errorf(`decad: unknown step options kind %q`, probe.Kind)
	}
}

// jsonStep is Step's wire shape: interface-typed fields as tagged raw
// messages, absent fields omitted.
type jsonStep struct {
	// Op is a pointer so a missing "op" is distinguishable from the zero
	// kind: a step with no op is malformed, never silently an extrude.
	Op        *OpKind           `json:"op"`
	Inputs    []StepRef         `json:"inputs,omitempty"`
	Profile   *ProfileRecord    `json:"profile,omitempty"`
	Plane     *PlaneRecord      `json:"plane,omitempty"`
	Extent    json.RawMessage   `json:"extent,omitempty"`
	Placement *TransformRecord  `json:"placement,omitempty"`
	Selectors []json.RawMessage `json:"selectors,omitempty"`
	Opts      json.RawMessage   `json:"opts,omitempty"`
	Values    []units.Value     `json:"values,omitempty"`
}

// zeroVec reports whether v is the zero vector.
func zeroVec(v r3.Vec) bool { return v == r3.Vec{} }

// MarshalJSON encodes the step with every absent field omitted: a decoded
// step is the recorded one, field for field.
func (s Step) MarshalJSON() ([]byte, error) {
	op := s.Op
	out := jsonStep{Op: &op, Inputs: s.Inputs, Values: s.Values}
	if len(s.Profile.Outer.Segments) > 0 || len(s.Profile.Holes) > 0 {
		p := s.Profile
		out.Profile = &p
	}
	if !zeroVec(s.Plane.U) || !zeroVec(s.Plane.V) || !zeroVec(s.Plane.Origin) {
		p := s.Plane
		out.Plane = &p
	}
	if s.Extent != nil {
		raw, err := marshalExtent(s.Extent)
		if err != nil {
			return nil, err
		}
		out.Extent = raw
	}
	if !zeroVec(s.Placement.EX) || !zeroVec(s.Placement.EY) || !zeroVec(s.Placement.EZ) || !zeroVec(s.Placement.T) {
		p := s.Placement
		out.Placement = &p
	}
	// The selector query types land with the selector vocabulary
	// (docs/evaluator-design.md §7/§11); until then no Selector value is
	// encodable, and the codec says so rather than dropping one.
	if len(s.Selectors) > 0 {
		return nil, fmt.Errorf(`decad: unencodable selector type %T`, s.Selectors[0])
	}
	if s.Opts != nil {
		raw, err := marshalStepOpts(s.Opts)
		if err != nil {
			return nil, err
		}
		out.Opts = raw
	}
	return json.Marshal(out)
}

// UnmarshalJSON decodes the wire shape, dispatching every tagged field
// through its own closed-set codec.
func (s *Step) UnmarshalJSON(data []byte) error {
	var raw jsonStep
	if err := json.Unmarshal(data, &raw); err != nil {
		return fmt.Errorf(`decad: failed to decode step: %w`, err)
	}
	if raw.Op == nil {
		return fmt.Errorf(`decad: step is missing its op`)
	}
	out := Step{Op: *raw.Op, Inputs: raw.Inputs, Values: raw.Values}
	if raw.Profile != nil {
		out.Profile = *raw.Profile
	}
	if raw.Plane != nil {
		out.Plane = *raw.Plane
	}
	if raw.Extent != nil {
		e, err := unmarshalExtent(raw.Extent)
		if err != nil {
			return err
		}
		out.Extent = e
	}
	if raw.Placement != nil {
		out.Placement = *raw.Placement
	}
	if len(raw.Selectors) > 0 {
		var probe struct {
			Kind string `json:"kind"`
		}
		if err := json.Unmarshal(raw.Selectors[0], &probe); err != nil {
			return fmt.Errorf(`decad: failed to decode selector tag: %w`, err)
		}
		return fmt.Errorf(`decad: unknown selector kind %q`, probe.Kind)
	}
	if raw.Opts != nil {
		o, err := unmarshalStepOpts(raw.Opts)
		if err != nil {
			return err
		}
		out.Opts = o
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
	out.Selectors = slices.Clone(s.Selectors)
	out.Profile = cloneProfileRecord(s.Profile)
	// An extent normalizes to pure values recursively (nested TwoSided sides
	// included), so a cloned step can never share a caller-visible pointer
	// with the document. A malformed nil pointer stays as-is — the codecs
	// and the feature calls reject it at their own gates.
	if s.Extent != nil {
		if e, err := normalizeExtent(s.Extent); err == nil {
			out.Extent = e
		}
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
