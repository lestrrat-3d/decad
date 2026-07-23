package decad

import (
	"bytes"
	"encoding/json"
	"fmt"

	"github.com/lestrrat-3d/units"
)

// This file is the extent vocabulary of docs/api-design.md §8.1 — the linear
// set Extrude takes and the angular set Revolve takes, two deliberately
// disjoint sealed tiers: illegal states are unrepresentable structurally, not
// rejected at runtime. A magnitude is non-negative and sense is carried ONLY
// by the enumerated Direction (enforced at the feature call —
// ErrNegativeMagnitude); a side of a two-sided extent is a single direction
// of travel, so the side variants carry no Direction at all.
//
// ToFace and ToFaceAngular — the extents that stop at a face of a named
// body — are resolved at the feature call (stops.go) and recorded with the
// body as its producing StepRef, exactly like EdgeAxis (core §6.2).

// Direction is the enumerated sense a standalone one-sided extent carries.
// There is no "both": a sweep that runs both ways is Symmetric or TwoSided,
// stated structurally.
type Direction int

const (
	// Along sweeps with the sketch plane's normal (U × V — core §7) for a
	// linear extent, and right-handed about the revolve axis for an angular
	// one.
	Along Direction = iota
	// Against sweeps the opposite sense.
	Against
)

// String renders the direction for diagnostics.
func (d Direction) String() string {
	switch d {
	case Along:
		return "Along"
	case Against:
		return "Against"
	default:
		return fmt.Sprintf("Direction(%d)", int(d))
	}
}

// MarshalText encodes the direction by name, so a recorded step stays
// readable and a renumbered constant could never silently reinterpret an
// old recipe. An unknown value refuses to encode.
func (d Direction) MarshalText() ([]byte, error) {
	switch d {
	case Along:
		return []byte("along"), nil
	case Against:
		return []byte("against"), nil
	default:
		return nil, fmt.Errorf(`decad: unknown direction %d`, int(d))
	}
}

// UnmarshalText decodes a direction by name; an unknown name is an error,
// never a default.
func (d *Direction) UnmarshalText(text []byte) error {
	switch string(text) {
	case "along":
		*d = Along
	case "against":
		*d = Against
	default:
		return fmt.Errorf(`decad: unknown direction %q`, string(text))
	}
	return nil
}

// Extent is what Extrude takes: the sealed linear extent set. No angular
// extent satisfies it — "revolve 90mm" is unrepresentable, not a runtime
// error.
type Extent interface{ extent() }

// SideExtent is what ONE side of a TwoSided extent may be. Sealed, and
// narrower than Extent: implemented only by DistanceSide, ThroughAllSide and
// ToFace. A side already IS a single direction of travel, so no side variant
// carries a Direction — TwoSided{One: Distance{Dir: …}} does not compile.
type SideExtent interface{ sideExtent() }

// Distance sweeps a non-negative distance D in Direction Dir. Extent only.
type Distance struct {
	D   units.Value `json:"d"`
	Dir Direction   `json:"dir"`
}

// ThroughAll sweeps in Direction Dir to the farthest far side of every live
// body with material beyond the sketch plane in the travel sense — the lateral
// footprint is not consulted, so a body off to the side still counts. Extent
// only; each such body is resolved at the call and recorded as a StepRef in the
// step's Inputs (core §6.2, docs/evaluator-design.md §5).
type ThroughAll struct {
	Dir Direction `json:"dir"`
}

// Symmetric sweeps both ways from the sketch plane: D each way, or D total
// when FullLength is set. Structurally two-sided, so it carries no Direction.
type Symmetric struct {
	D          units.Value `json:"d"`
	FullLength bool        `json:"full_length,omitempty"`
}

// TwoSided sweeps each side independently. One is the Along side, Two the
// Against side — the side supplies the sense, which is why only the side
// variants are admissible here.
type TwoSided struct {
	One SideExtent `json:"-"`
	Two SideExtent `json:"-"`
}

// DistanceSide is one side of a TwoSided: a non-negative distance in the
// sense the side it occupies supplies. SideExtent only.
type DistanceSide struct {
	D units.Value `json:"d"`
}

// ThroughAllSide is one side of a TwoSided that runs to the farthest far side
// of every live body with material beyond the sketch plane on this side, its
// lateral footprint likewise not consulted. SideExtent only; like ThroughAll,
// each such body is resolved at the call and recorded as a StepRef in the
// step's Inputs (core §6.2).
type ThroughAllSide struct{}

// ToFace stops the sweep at a face of Body, displaced by the signed Offset —
// the one signed number in the extent vocabulary: a positive offset
// overshoots the face, a negative one stops short of it, so
// ErrNegativeMagnitude does not apply (core §8.1/§12). A zero-value Offset
// means no displacement. The sense of the sweep comes from the target face,
// so ToFace carries no Direction. Face is a FaceSelector, never a *Face
// (core §9), resolved as Face.SelectFaces(Body) under the implicit
// exactly-one of core §12; Body must be a live body of the same document at
// the call (a StepRef there is ErrUnresolvedBody), and its StepRef is
// recorded in the step's Inputs — the step depends on it; the body is not
// consumed and not retired. Both Extent and SideExtent: a ToFace is a single
// direction of travel, so it may also stand as one side of a TwoSided.
type ToFace struct {
	Body   BodyRef
	Face   FaceSelector
	Offset units.Value
}

// The sealed sets. The two tiers are deliberately disjoint: a standalone
// extent is never a side, and a side is never a standalone extent. ToFace is
// the deliberate exception — core §8.1 admits it in both roles.
func (Distance) extent()           {}
func (ThroughAll) extent()         {}
func (Symmetric) extent()          {}
func (TwoSided) extent()           {}
func (ToFace) extent()             {}
func (DistanceSide) sideExtent()   {}
func (ThroughAllSide) sideExtent() {}
func (ToFace) sideExtent()         {}

// Extent and SideExtent are closed variant sets decad owns, so decad ships
// their codec (core §6.2): tagged objects, dispatch on the tag, no fallback.

const (
	extKindDistance       = "distance"
	extKindThroughAll     = "through_all"
	extKindSymmetric      = "symmetric"
	extKindTwoSided       = "two_sided"
	extKindToFace         = "to_face"
	extKindDistanceSide   = "distance_side"
	extKindThroughAllSide = "through_all_side"
)

// emptyExtentWire is the complete payload shared by the two extent variants
// that carry no data. Its strict decoder admits the dispatch tag and nothing
// else, so a field from another variant cannot silently change intent.
type emptyExtentWire struct {
	Kind string `json:"kind"`
}

func unmarshalEmptyExtent(data []byte, name string) error {
	decoder := json.NewDecoder(bytes.NewReader(data))
	decoder.DisallowUnknownFields()
	var raw emptyExtentWire
	if err := decoder.Decode(&raw); err != nil {
		return fmt.Errorf(`decad: failed to decode %s: %w`, name, err)
	}
	return nil
}

// errNilExtent rejects a nil variant pointer: it names no extent to record.
// It wraps ErrDegenerate so a typed nil pointer is branchable exactly like an
// untyped nil extent.
var errNilExtent = fmt.Errorf(`%w: nil extent`, ErrDegenerate)

// normalizeExtent returns the value form of e, RECURSIVELY: the variants seal
// with value receivers, so a *Distance satisfies Extent as readily as a
// Distance does — the codec accepts both and records the value the pointer
// names — and a TwoSided's sides normalize with it, so no caller-owned
// pointer survives into a recorded step to alias document state. A nil
// pointer is rejected.
func normalizeExtent(e Extent) (Extent, error) {
	switch e := e.(type) {
	case *Distance:
		if e == nil {
			return nil, errNilExtent
		}
		return *e, nil
	case *ThroughAll:
		if e == nil {
			return nil, errNilExtent
		}
		return *e, nil
	case *Symmetric:
		if e == nil {
			return nil, errNilExtent
		}
		return *e, nil
	case *TwoSided:
		if e == nil {
			return nil, errNilExtent
		}
		return normalizeExtent(*e)
	case *ToFace:
		if e == nil {
			return nil, errNilExtent
		}
		return *e, nil
	case TwoSided:
		one, err := normalizeSideExtent(e.One)
		if err != nil {
			return nil, err
		}
		two, err := normalizeSideExtent(e.Two)
		if err != nil {
			return nil, err
		}
		return TwoSided{One: one, Two: two}, nil
	default:
		return e, nil
	}
}

// normalizeSideExtent is normalizeExtent's side-tier analog.
func normalizeSideExtent(s SideExtent) (SideExtent, error) {
	switch s := s.(type) {
	case *DistanceSide:
		if s == nil {
			return nil, errNilExtent
		}
		return *s, nil
	case *ThroughAllSide:
		if s == nil {
			return nil, errNilExtent
		}
		return *s, nil
	case *ToFace:
		if s == nil {
			return nil, errNilExtent
		}
		return *s, nil
	default:
		return s, nil
	}
}

// cloneExtent deep-copies a recorded extent so a Recipe never aliases a
// caller-owned face selector — the extent analog of cloneAxis. A malformed
// nil pointer stays as-is; the codecs and the feature call reject it at
// their own gates.
func cloneExtent(e Extent) Extent {
	n, err := normalizeExtent(e)
	if err != nil {
		return e
	}
	switch n := n.(type) {
	case ToFace:
		return cloneToFace(n)
	case TwoSided:
		n.One = cloneSideExtent(n.One)
		n.Two = cloneSideExtent(n.Two)
		return n
	default:
		return n
	}
}

// cloneSideExtent is cloneExtent's side-tier analog. The sides were
// normalized with the enclosing TwoSided already.
func cloneSideExtent(s SideExtent) SideExtent {
	if tf, ok := s.(ToFace); ok {
		return cloneToFace(tf)
	}
	return s
}

// cloneToFace deep-copies a ToFace's selector so no caller-owned query
// survives into a recorded step and none escapes Recipe().
func cloneToFace(tf ToFace) ToFace {
	if tf.Face != nil {
		if sel, ok := cloneSelector(tf.Face).(FaceSelector); ok {
			tf.Face = sel
		}
	}
	return tf
}

// marshalExtent encodes one extent as its tagged object.
func marshalExtent(e Extent) ([]byte, error) {
	e, err := normalizeExtent(e)
	if err != nil {
		return nil, err
	}
	switch e := e.(type) {
	case Distance:
		return marshalTagged(extKindDistance, e)
	case ThroughAll:
		return marshalTagged(extKindThroughAll, e)
	case Symmetric:
		return marshalTagged(extKindSymmetric, e)
	case TwoSided:
		one, err := marshalSideExtent(e.One)
		if err != nil {
			return nil, err
		}
		two, err := marshalSideExtent(e.Two)
		if err != nil {
			return nil, err
		}
		return json.Marshal(struct {
			Kind string          `json:"kind"`
			One  json.RawMessage `json:"one"`
			Two  json.RawMessage `json:"two"`
		}{Kind: extKindTwoSided, One: one, Two: two})
	case ToFace:
		return marshalToFace(e)
	default:
		return nil, fmt.Errorf(`decad: unencodable extent type %T`, e)
	}
}

// marshalToFace encodes a to-face stop as its tagged object, shared by the
// Extent and SideExtent codecs. Like an EdgeAxis, it records its body as the
// producing StepRef — a live *Body is a handle, not a record.
func marshalToFace(tf ToFace) ([]byte, error) {
	ref, ok := tf.Body.(StepRef)
	if !ok {
		return nil, fmt.Errorf(`decad: a to-face extent records its body as a StepRef, got %T`, tf.Body)
	}
	if tf.Face == nil {
		return nil, errNilSelector
	}
	face, err := marshalSelector(tf.Face)
	if err != nil {
		return nil, err
	}
	return json.Marshal(struct {
		Kind   string          `json:"kind"`
		Body   StepRef         `json:"body"`
		Face   json.RawMessage `json:"face"`
		Offset units.Value     `json:"offset"`
	}{Kind: extKindToFace, Body: ref, Face: face, Offset: normalizeStopOffset(tf.Offset)})
}

// normalizeStopOffset reads the zero Value as "no displacement": the record
// and the wire always carry an explicit length.
func normalizeStopOffset(v units.Value) units.Value {
	if v == (units.Value{}) {
		return units.Millimeters(0)
	}
	return v
}

// unmarshalToFace decodes a to-face stop, shared by the Extent and SideExtent
// codecs. Wire structs with pointer fields: an absent body, face or offset is
// malformed input, never silently step 0, a match-all query or a zero offset.
func unmarshalToFace(data []byte) (ToFace, error) {
	var raw struct {
		Body   *StepRef        `json:"body"`
		Face   json.RawMessage `json:"face"`
		Offset *units.Value    `json:"offset"`
	}
	if err := json.Unmarshal(data, &raw); err != nil {
		return ToFace{}, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode to-face extent: %w`, err))
	}
	if raw.Body == nil {
		return ToFace{}, prependCodecPath(fmt.Errorf(`decad: a to-face extent requires body, face and offset`), "body")
	}
	if raw.Face == nil {
		return ToFace{}, prependCodecPath(fmt.Errorf(`decad: a to-face extent requires body, face and offset`), "face")
	}
	if raw.Offset == nil {
		return ToFace{}, prependCodecPath(fmt.Errorf(`decad: a to-face extent requires body, face and offset`), "offset")
	}
	sel, err := unmarshalSelector(raw.Face)
	if err != nil {
		return ToFace{}, prependCodecPath(err, "face")
	}
	face, ok := sel.(FaceSelector)
	if !ok {
		return ToFace{}, prependCodecPath(fmt.Errorf(`decad: a to-face extent requires a face selector, got %T`, sel), "face")
	}
	return ToFace{Body: *raw.Body, Face: face, Offset: *raw.Offset}, nil
}

// unmarshalExtent dispatches on the kind tag; an unknown or missing tag is an
// error — the set is closed.
func unmarshalExtent(data []byte) (Extent, error) {
	var probe struct {
		Kind string `json:"kind"`
	}
	if err := json.Unmarshal(data, &probe); err != nil {
		return nil, codecJSONErrorAt(data, &probe, fmt.Errorf(`decad: failed to decode extent tag: %w`, err))
	}
	switch probe.Kind {
	case extKindDistance:
		// Wire structs with pointer fields: an absent magnitude or sense is
		// malformed input, never silently a zero distance or Along.
		var raw struct {
			D   *units.Value `json:"d"`
			Dir *Direction   `json:"dir"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode distance extent: %w`, err))
		}
		if raw.D == nil {
			return nil, prependCodecPath(fmt.Errorf(`decad: a distance extent requires both d and dir`), "d")
		}
		if raw.Dir == nil {
			return nil, prependCodecPath(fmt.Errorf(`decad: a distance extent requires both d and dir`), "dir")
		}
		return Distance{D: *raw.D, Dir: *raw.Dir}, nil
	case extKindThroughAll:
		var raw struct {
			Dir *Direction `json:"dir"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode through-all extent: %w`, err))
		}
		if raw.Dir == nil {
			return nil, prependCodecPath(fmt.Errorf(`decad: a through-all extent requires dir`), "dir")
		}
		return ThroughAll{Dir: *raw.Dir}, nil
	case extKindSymmetric:
		var raw struct {
			D          *units.Value `json:"d"`
			FullLength bool         `json:"full_length"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode symmetric extent: %w`, err))
		}
		if raw.D == nil {
			return nil, prependCodecPath(fmt.Errorf(`decad: a symmetric extent requires d`), "d")
		}
		return Symmetric{D: *raw.D, FullLength: raw.FullLength}, nil
	case extKindTwoSided:
		var raw struct {
			One json.RawMessage `json:"one"`
			Two json.RawMessage `json:"two"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode two-sided extent: %w`, err))
		}
		one, err := unmarshalSideExtent(raw.One)
		if err != nil {
			return nil, prependCodecPath(err, "one")
		}
		two, err := unmarshalSideExtent(raw.Two)
		if err != nil {
			return nil, prependCodecPath(err, "two")
		}
		return TwoSided{One: one, Two: two}, nil
	case extKindToFace:
		return unmarshalToFace(data)
	case "":
		return nil, prependCodecPath(fmt.Errorf(`decad: extent is missing its kind tag`), "kind")
	default:
		return nil, prependCodecPath(fmt.Errorf(`decad: unknown extent kind %q`, probe.Kind), "kind")
	}
}

// marshalSideExtent encodes one side of a TwoSided as its tagged object.
func marshalSideExtent(s SideExtent) ([]byte, error) {
	s, err := normalizeSideExtent(s)
	if err != nil {
		return nil, err
	}
	switch s := s.(type) {
	case DistanceSide:
		return marshalTagged(extKindDistanceSide, s)
	case ThroughAllSide:
		return marshalTagged(extKindThroughAllSide, s)
	case ToFace:
		return marshalToFace(s)
	default:
		return nil, fmt.Errorf(`decad: unencodable side extent type %T`, s)
	}
}

// unmarshalSideExtent dispatches one side on its kind tag.
func unmarshalSideExtent(data []byte) (SideExtent, error) {
	var probe struct {
		Kind string `json:"kind"`
	}
	if err := json.Unmarshal(data, &probe); err != nil {
		return nil, codecJSONErrorAt(data, &probe, fmt.Errorf(`decad: failed to decode side extent tag: %w`, err))
	}
	switch probe.Kind {
	case extKindDistanceSide:
		var raw struct {
			D *units.Value `json:"d"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode distance side: %w`, err))
		}
		if raw.D == nil {
			return nil, prependCodecPath(fmt.Errorf(`decad: a distance side requires d`), "d")
		}
		return DistanceSide{D: *raw.D}, nil
	case extKindThroughAllSide:
		if err := unmarshalEmptyExtent(data, "through-all side"); err != nil {
			return nil, err
		}
		return ThroughAllSide{}, nil
	case extKindToFace:
		return unmarshalToFace(data)
	case "":
		return nil, prependCodecPath(fmt.Errorf(`decad: side extent is missing its kind tag`), "kind")
	default:
		return nil, prependCodecPath(fmt.Errorf(`decad: unknown side extent kind %q`, probe.Kind), "kind")
	}
}

// AngularExtent is what Revolve takes: the sealed angular extent set. No
// linear extent satisfies it and no angular extent satisfies Extent — the
// two sets are deliberately disjoint, so "revolve 90mm" is unrepresentable,
// not a runtime error.
type AngularExtent interface{ angularExtent() }

// SideAngular is what ONE side of a TwoSidedAngle extent may be. Sealed, and
// narrower than AngularExtent: implemented only by AngleSide and ToFaceAngular.
// A side already IS a single direction of travel, so no side variant carries a
// Direction — TwoSidedAngle{One: AngleExtent{Dir: …}} does not compile.
type SideAngular interface{ sideAngular() }

// AngleExtent sweeps a non-negative angle A in Direction Dir about the
// revolve axis. AngularExtent only.
type AngleExtent struct {
	A   units.Value `json:"a"`
	Dir Direction   `json:"dir"`
}

// FullRevolution sweeps the full turn. Structurally two-sided — a full turn
// reaches the same solid either way — so it carries no Direction and no
// magnitude. AngularExtent only.
type FullRevolution struct{}

// SymmetricAngle sweeps both ways from the sketch plane: A each way, or A
// total when FullLength is set. Structurally two-sided, so it carries no
// Direction.
type SymmetricAngle struct {
	A          units.Value `json:"a"`
	FullLength bool        `json:"full_length,omitempty"`
}

// TwoSidedAngle sweeps each side independently. One is the Along side, Two
// the Against side — the side supplies the sense, which is why only the side
// variants are admissible here.
type TwoSidedAngle struct {
	One SideAngular `json:"-"`
	Two SideAngular `json:"-"`
}

// AngleSide is one side of a TwoSidedAngle: a non-negative angle in the
// sense the side it occupies supplies. SideAngular only.
type AngleSide struct {
	A units.Value `json:"a"`
}

// ToFaceAngular stops the revolve at a face of Body — the angular analog of
// ToFace, with no offset. The sense of the sweep comes from the target face
// (a standalone ToFaceAngular takes the nearer way around; as a side of a
// TwoSidedAngle the side supplies the sense), so it carries no Direction.
// Face is a FaceSelector, never a *Face (core §9), resolved as
// Face.SelectFaces(Body) under the implicit exactly-one of core §12; Body
// must be a live body of the same document at the call (a StepRef there is
// ErrUnresolvedBody), and its StepRef is recorded in the step's Inputs — the
// step depends on it; the body is not consumed and not retired. Both
// AngularExtent and SideAngular.
type ToFaceAngular struct {
	Body BodyRef
	Face FaceSelector
}

// The sealed sets. The two tiers are deliberately disjoint: a standalone
// angular extent is never a side, and a side is never a standalone extent.
// ToFaceAngular is the deliberate exception — core §8.1 admits it in both
// roles.
func (AngleExtent) angularExtent()    {}
func (FullRevolution) angularExtent() {}
func (SymmetricAngle) angularExtent() {}
func (TwoSidedAngle) angularExtent()  {}
func (ToFaceAngular) angularExtent()  {}
func (AngleSide) sideAngular()        {}
func (ToFaceAngular) sideAngular()    {}

// AngularExtent and SideAngular are closed variant sets decad owns, so decad
// ships their codec (core §6.2): tagged objects, dispatch on the tag, no
// fallback.

const (
	extKindAngleExtent    = "angle_extent"
	extKindFullRevolution = "full_revolution"
	extKindSymmetricAngle = "symmetric_angle"
	extKindTwoSidedAngle  = "two_sided_angle"
	extKindAngleSide      = "angle_side"
	extKindToFaceAngular  = "to_face_angular"
)

// normalizeAngularExtent is normalizeExtent's angular analog: it returns the
// value form of a, RECURSIVELY — a TwoSidedAngle's sides normalize with it,
// so no caller-owned pointer survives into a recorded step to alias document
// state. A nil pointer is rejected.
func normalizeAngularExtent(a AngularExtent) (AngularExtent, error) {
	switch a := a.(type) {
	case *AngleExtent:
		if a == nil {
			return nil, errNilExtent
		}
		return *a, nil
	case *FullRevolution:
		if a == nil {
			return nil, errNilExtent
		}
		return *a, nil
	case *SymmetricAngle:
		if a == nil {
			return nil, errNilExtent
		}
		return *a, nil
	case *TwoSidedAngle:
		if a == nil {
			return nil, errNilExtent
		}
		return normalizeAngularExtent(*a)
	case *ToFaceAngular:
		if a == nil {
			return nil, errNilExtent
		}
		return *a, nil
	case TwoSidedAngle:
		one, err := normalizeSideAngular(a.One)
		if err != nil {
			return nil, err
		}
		two, err := normalizeSideAngular(a.Two)
		if err != nil {
			return nil, err
		}
		return TwoSidedAngle{One: one, Two: two}, nil
	default:
		return a, nil
	}
}

// normalizeSideAngular is normalizeAngularExtent's side-tier analog.
func normalizeSideAngular(s SideAngular) (SideAngular, error) {
	switch s := s.(type) {
	case *AngleSide:
		if s == nil {
			return nil, errNilExtent
		}
		return *s, nil
	case *ToFaceAngular:
		if s == nil {
			return nil, errNilExtent
		}
		return *s, nil
	default:
		return s, nil
	}
}

// cloneAngularExtent is cloneExtent's angular analog: it deep-copies a
// recorded angular extent so a Recipe never aliases a caller-owned face
// selector.
func cloneAngularExtent(a AngularExtent) AngularExtent {
	n, err := normalizeAngularExtent(a)
	if err != nil {
		return a
	}
	switch n := n.(type) {
	case ToFaceAngular:
		return cloneToFaceAngular(n)
	case TwoSidedAngle:
		n.One = cloneSideAngular(n.One)
		n.Two = cloneSideAngular(n.Two)
		return n
	default:
		return n
	}
}

// cloneSideAngular is cloneAngularExtent's side-tier analog. The sides were
// normalized with the enclosing TwoSidedAngle already.
func cloneSideAngular(s SideAngular) SideAngular {
	if tfa, ok := s.(ToFaceAngular); ok {
		return cloneToFaceAngular(tfa)
	}
	return s
}

// cloneToFaceAngular deep-copies a ToFaceAngular's selector so no
// caller-owned query survives into a recorded step and none escapes
// Recipe().
func cloneToFaceAngular(tfa ToFaceAngular) ToFaceAngular {
	if tfa.Face != nil {
		if sel, ok := cloneSelector(tfa.Face).(FaceSelector); ok {
			tfa.Face = sel
		}
	}
	return tfa
}

// marshalAngularExtent encodes one angular extent as its tagged object.
func marshalAngularExtent(a AngularExtent) ([]byte, error) {
	a, err := normalizeAngularExtent(a)
	if err != nil {
		return nil, err
	}
	switch a := a.(type) {
	case AngleExtent:
		return marshalTagged(extKindAngleExtent, a)
	case FullRevolution:
		return marshalTagged(extKindFullRevolution, a)
	case SymmetricAngle:
		return marshalTagged(extKindSymmetricAngle, a)
	case TwoSidedAngle:
		one, err := marshalSideAngular(a.One)
		if err != nil {
			return nil, err
		}
		two, err := marshalSideAngular(a.Two)
		if err != nil {
			return nil, err
		}
		return json.Marshal(struct {
			Kind string          `json:"kind"`
			One  json.RawMessage `json:"one"`
			Two  json.RawMessage `json:"two"`
		}{Kind: extKindTwoSidedAngle, One: one, Two: two})
	case ToFaceAngular:
		return marshalToFaceAngular(a)
	default:
		return nil, fmt.Errorf(`decad: unencodable angular extent type %T`, a)
	}
}

// marshalToFaceAngular encodes an angular to-face stop as its tagged object,
// shared by the AngularExtent and SideAngular codecs. Like an EdgeAxis, it
// records its body as the producing StepRef.
func marshalToFaceAngular(tfa ToFaceAngular) ([]byte, error) {
	ref, ok := tfa.Body.(StepRef)
	if !ok {
		return nil, fmt.Errorf(`decad: a to-face angular extent records its body as a StepRef, got %T`, tfa.Body)
	}
	if tfa.Face == nil {
		return nil, errNilSelector
	}
	face, err := marshalSelector(tfa.Face)
	if err != nil {
		return nil, err
	}
	return json.Marshal(struct {
		Kind string          `json:"kind"`
		Body StepRef         `json:"body"`
		Face json.RawMessage `json:"face"`
	}{Kind: extKindToFaceAngular, Body: ref, Face: face})
}

// unmarshalToFaceAngular decodes an angular to-face stop, shared by the
// AngularExtent and SideAngular codecs. Wire structs with pointer fields: an
// absent body or face is malformed input, never silently step 0 or a
// match-all query.
func unmarshalToFaceAngular(data []byte) (ToFaceAngular, error) {
	var raw struct {
		Body *StepRef        `json:"body"`
		Face json.RawMessage `json:"face"`
	}
	if err := json.Unmarshal(data, &raw); err != nil {
		return ToFaceAngular{}, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode to-face angular extent: %w`, err))
	}
	if raw.Body == nil {
		return ToFaceAngular{}, prependCodecPath(fmt.Errorf(`decad: a to-face angular extent requires both body and face`), "body")
	}
	if raw.Face == nil {
		return ToFaceAngular{}, prependCodecPath(fmt.Errorf(`decad: a to-face angular extent requires both body and face`), "face")
	}
	sel, err := unmarshalSelector(raw.Face)
	if err != nil {
		return ToFaceAngular{}, prependCodecPath(err, "face")
	}
	face, ok := sel.(FaceSelector)
	if !ok {
		return ToFaceAngular{}, prependCodecPath(fmt.Errorf(`decad: a to-face angular extent requires a face selector, got %T`, sel), "face")
	}
	return ToFaceAngular{Body: *raw.Body, Face: face}, nil
}

// unmarshalAngularExtent dispatches on the kind tag; an unknown or missing
// tag is an error — the set is closed.
func unmarshalAngularExtent(data []byte) (AngularExtent, error) {
	var probe struct {
		Kind string `json:"kind"`
	}
	if err := json.Unmarshal(data, &probe); err != nil {
		return nil, codecJSONErrorAt(data, &probe, fmt.Errorf(`decad: failed to decode angular extent tag: %w`, err))
	}
	switch probe.Kind {
	case extKindAngleExtent:
		// Wire structs with pointer fields: an absent magnitude or sense is
		// malformed input, never silently a zero angle or Along.
		var raw struct {
			A   *units.Value `json:"a"`
			Dir *Direction   `json:"dir"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode angle extent: %w`, err))
		}
		if raw.A == nil {
			return nil, prependCodecPath(fmt.Errorf(`decad: an angle extent requires both a and dir`), "a")
		}
		if raw.Dir == nil {
			return nil, prependCodecPath(fmt.Errorf(`decad: an angle extent requires both a and dir`), "dir")
		}
		return AngleExtent{A: *raw.A, Dir: *raw.Dir}, nil
	case extKindFullRevolution:
		if err := unmarshalEmptyExtent(data, "full-revolution extent"); err != nil {
			return nil, err
		}
		return FullRevolution{}, nil
	case extKindSymmetricAngle:
		var raw struct {
			A          *units.Value `json:"a"`
			FullLength bool         `json:"full_length"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode symmetric angle extent: %w`, err))
		}
		if raw.A == nil {
			return nil, prependCodecPath(fmt.Errorf(`decad: a symmetric angle extent requires a`), "a")
		}
		return SymmetricAngle{A: *raw.A, FullLength: raw.FullLength}, nil
	case extKindTwoSidedAngle:
		var raw struct {
			One json.RawMessage `json:"one"`
			Two json.RawMessage `json:"two"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode two-sided angle extent: %w`, err))
		}
		one, err := unmarshalSideAngular(raw.One)
		if err != nil {
			return nil, prependCodecPath(err, "one")
		}
		two, err := unmarshalSideAngular(raw.Two)
		if err != nil {
			return nil, prependCodecPath(err, "two")
		}
		return TwoSidedAngle{One: one, Two: two}, nil
	case extKindToFaceAngular:
		return unmarshalToFaceAngular(data)
	case "":
		return nil, prependCodecPath(fmt.Errorf(`decad: angular extent is missing its kind tag`), "kind")
	default:
		return nil, prependCodecPath(fmt.Errorf(`decad: unknown angular extent kind %q`, probe.Kind), "kind")
	}
}

// marshalSideAngular encodes one side of a TwoSidedAngle as its tagged object.
func marshalSideAngular(s SideAngular) ([]byte, error) {
	s, err := normalizeSideAngular(s)
	if err != nil {
		return nil, err
	}
	switch s := s.(type) {
	case AngleSide:
		return marshalTagged(extKindAngleSide, s)
	case ToFaceAngular:
		return marshalToFaceAngular(s)
	default:
		return nil, fmt.Errorf(`decad: unencodable side angular type %T`, s)
	}
}

// unmarshalSideAngular dispatches one side on its kind tag.
func unmarshalSideAngular(data []byte) (SideAngular, error) {
	var probe struct {
		Kind string `json:"kind"`
	}
	if err := json.Unmarshal(data, &probe); err != nil {
		return nil, codecJSONErrorAt(data, &probe, fmt.Errorf(`decad: failed to decode side angular tag: %w`, err))
	}
	switch probe.Kind {
	case extKindAngleSide:
		var raw struct {
			A *units.Value `json:"a"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode angle side: %w`, err))
		}
		if raw.A == nil {
			return nil, prependCodecPath(fmt.Errorf(`decad: an angle side requires a`), "a")
		}
		return AngleSide{A: *raw.A}, nil
	case extKindToFaceAngular:
		return unmarshalToFaceAngular(data)
	case "":
		return nil, prependCodecPath(fmt.Errorf(`decad: side angular is missing its kind tag`), "kind")
	default:
		return nil, prependCodecPath(fmt.Errorf(`decad: unknown side angular kind %q`, probe.Kind), "kind")
	}
}
