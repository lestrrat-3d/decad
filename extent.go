package decad

import (
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
// body — need the selector vocabulary and land with it in evaluator
// increment 2 (docs/evaluator-design.md §5/§6/§11).

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
// (when it lands with the selector vocabulary) ToFace. A side already IS a
// single direction of travel, so no side variant carries a Direction —
// TwoSided{One: Distance{Dir: …}} does not compile.
type SideExtent interface{ sideExtent() }

// Distance sweeps a non-negative distance D in Direction Dir. Extent only.
type Distance struct {
	D   units.Value `json:"d"`
	Dir Direction   `json:"dir"`
}

// ThroughAll sweeps in Direction Dir through the far side of every body the
// sweep meets. Extent only; its stop bodies are resolved at the call and
// recorded as StepRefs in the step's Inputs (core §6.2). Evaluator support
// lands in increment 2 (docs/evaluator-design.md §5).
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

// ThroughAllSide is one side of a TwoSided that runs through every body on
// its side. SideExtent only; evaluator support lands in increment 2.
type ThroughAllSide struct{}

// The sealed sets. The two tiers are deliberately disjoint: a standalone
// extent is never a side, and a side is never a standalone extent.
func (Distance) extent()           {}
func (ThroughAll) extent()         {}
func (Symmetric) extent()          {}
func (TwoSided) extent()           {}
func (DistanceSide) sideExtent()   {}
func (ThroughAllSide) sideExtent() {}

// Extent and SideExtent are closed variant sets decad owns, so decad ships
// their codec (core §6.2): tagged objects, dispatch on the tag, no fallback.

const (
	extKindDistance       = "distance"
	extKindThroughAll     = "through_all"
	extKindSymmetric      = "symmetric"
	extKindTwoSided       = "two_sided"
	extKindDistanceSide   = "distance_side"
	extKindThroughAllSide = "through_all_side"
)

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
	default:
		return s, nil
	}
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
	default:
		return nil, fmt.Errorf(`decad: unencodable extent type %T`, e)
	}
}

// unmarshalExtent dispatches on the kind tag; an unknown or missing tag is an
// error — the set is closed.
func unmarshalExtent(data []byte) (Extent, error) {
	var probe struct {
		Kind string `json:"kind"`
	}
	if err := json.Unmarshal(data, &probe); err != nil {
		return nil, fmt.Errorf(`decad: failed to decode extent tag: %w`, err)
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
			return nil, fmt.Errorf(`decad: failed to decode distance extent: %w`, err)
		}
		if raw.D == nil || raw.Dir == nil {
			return nil, fmt.Errorf(`decad: a distance extent requires both d and dir`)
		}
		return Distance{D: *raw.D, Dir: *raw.Dir}, nil
	case extKindThroughAll:
		var raw struct {
			Dir *Direction `json:"dir"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, fmt.Errorf(`decad: failed to decode through-all extent: %w`, err)
		}
		if raw.Dir == nil {
			return nil, fmt.Errorf(`decad: a through-all extent requires dir`)
		}
		return ThroughAll{Dir: *raw.Dir}, nil
	case extKindSymmetric:
		var raw struct {
			D          *units.Value `json:"d"`
			FullLength bool         `json:"full_length"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, fmt.Errorf(`decad: failed to decode symmetric extent: %w`, err)
		}
		if raw.D == nil {
			return nil, fmt.Errorf(`decad: a symmetric extent requires d`)
		}
		return Symmetric{D: *raw.D, FullLength: raw.FullLength}, nil
	case extKindTwoSided:
		var raw struct {
			One json.RawMessage `json:"one"`
			Two json.RawMessage `json:"two"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, fmt.Errorf(`decad: failed to decode two-sided extent: %w`, err)
		}
		one, err := unmarshalSideExtent(raw.One)
		if err != nil {
			return nil, err
		}
		two, err := unmarshalSideExtent(raw.Two)
		if err != nil {
			return nil, err
		}
		return TwoSided{One: one, Two: two}, nil
	case "":
		return nil, fmt.Errorf(`decad: extent is missing its kind tag`)
	default:
		return nil, fmt.Errorf(`decad: unknown extent kind %q`, probe.Kind)
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
		return nil, fmt.Errorf(`decad: failed to decode side extent tag: %w`, err)
	}
	switch probe.Kind {
	case extKindDistanceSide:
		var raw struct {
			D *units.Value `json:"d"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, fmt.Errorf(`decad: failed to decode distance side: %w`, err)
		}
		if raw.D == nil {
			return nil, fmt.Errorf(`decad: a distance side requires d`)
		}
		return DistanceSide{D: *raw.D}, nil
	case extKindThroughAllSide:
		return ThroughAllSide{}, nil
	case "":
		return nil, fmt.Errorf(`decad: side extent is missing its kind tag`)
	default:
		return nil, fmt.Errorf(`decad: unknown side extent kind %q`, probe.Kind)
	}
}

// AngularExtent is what Revolve takes: the sealed angular extent set. No
// linear extent satisfies it and no angular extent satisfies Extent — the
// two sets are deliberately disjoint, so "revolve 90mm" is unrepresentable,
// not a runtime error.
type AngularExtent interface{ angularExtent() }

// SideAngular is what ONE side of a TwoSidedAngle extent may be. Sealed, and
// narrower than AngularExtent: implemented only by AngleSide and (when it
// lands with the selector vocabulary) ToFaceAngular. A side already IS a
// single direction of travel, so no side variant carries a Direction —
// TwoSidedAngle{One: AngleExtent{Dir: …}} does not compile.
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

// The sealed sets. The two tiers are deliberately disjoint: a standalone
// angular extent is never a side, and a side is never a standalone extent.
func (AngleExtent) angularExtent()    {}
func (FullRevolution) angularExtent() {}
func (SymmetricAngle) angularExtent() {}
func (TwoSidedAngle) angularExtent()  {}
func (AngleSide) sideAngular()        {}

// AngularExtent and SideAngular are closed variant sets decad owns, so decad
// ships their codec (core §6.2): tagged objects, dispatch on the tag, no
// fallback.

const (
	extKindAngleExtent    = "angle_extent"
	extKindFullRevolution = "full_revolution"
	extKindSymmetricAngle = "symmetric_angle"
	extKindTwoSidedAngle  = "two_sided_angle"
	extKindAngleSide      = "angle_side"
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
	default:
		return s, nil
	}
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
	default:
		return nil, fmt.Errorf(`decad: unencodable angular extent type %T`, a)
	}
}

// unmarshalAngularExtent dispatches on the kind tag; an unknown or missing
// tag is an error — the set is closed.
func unmarshalAngularExtent(data []byte) (AngularExtent, error) {
	var probe struct {
		Kind string `json:"kind"`
	}
	if err := json.Unmarshal(data, &probe); err != nil {
		return nil, fmt.Errorf(`decad: failed to decode angular extent tag: %w`, err)
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
			return nil, fmt.Errorf(`decad: failed to decode angle extent: %w`, err)
		}
		if raw.A == nil || raw.Dir == nil {
			return nil, fmt.Errorf(`decad: an angle extent requires both a and dir`)
		}
		return AngleExtent{A: *raw.A, Dir: *raw.Dir}, nil
	case extKindFullRevolution:
		return FullRevolution{}, nil
	case extKindSymmetricAngle:
		var raw struct {
			A          *units.Value `json:"a"`
			FullLength bool         `json:"full_length"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, fmt.Errorf(`decad: failed to decode symmetric angle extent: %w`, err)
		}
		if raw.A == nil {
			return nil, fmt.Errorf(`decad: a symmetric angle extent requires a`)
		}
		return SymmetricAngle{A: *raw.A, FullLength: raw.FullLength}, nil
	case extKindTwoSidedAngle:
		var raw struct {
			One json.RawMessage `json:"one"`
			Two json.RawMessage `json:"two"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, fmt.Errorf(`decad: failed to decode two-sided angle extent: %w`, err)
		}
		one, err := unmarshalSideAngular(raw.One)
		if err != nil {
			return nil, err
		}
		two, err := unmarshalSideAngular(raw.Two)
		if err != nil {
			return nil, err
		}
		return TwoSidedAngle{One: one, Two: two}, nil
	case "":
		return nil, fmt.Errorf(`decad: angular extent is missing its kind tag`)
	default:
		return nil, fmt.Errorf(`decad: unknown angular extent kind %q`, probe.Kind)
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
		return nil, fmt.Errorf(`decad: failed to decode side angular tag: %w`, err)
	}
	switch probe.Kind {
	case extKindAngleSide:
		var raw struct {
			A *units.Value `json:"a"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return nil, fmt.Errorf(`decad: failed to decode angle side: %w`, err)
		}
		if raw.A == nil {
			return nil, fmt.Errorf(`decad: an angle side requires a`)
		}
		return AngleSide{A: *raw.A}, nil
	case "":
		return nil, fmt.Errorf(`decad: side angular is missing its kind tag`)
	default:
		return nil, fmt.Errorf(`decad: unknown side angular kind %q`, probe.Kind)
	}
}
