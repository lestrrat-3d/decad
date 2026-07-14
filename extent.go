package decad

import (
	"encoding/json"
	"fmt"

	"github.com/lestrrat-3d/units"
)

// This file is the linear extent vocabulary of docs/api-design.md §8.1:
// illegal states are unrepresentable structurally, not rejected at runtime.
// A magnitude is non-negative and sense is carried ONLY by the enumerated
// Direction (enforced at the feature call — ErrNegativeMagnitude); a side of
// a two-sided extent is a single direction of travel, so the side variants
// carry no Direction at all.
//
// ToFace — the extent that stops at a face of a named body — needs the
// selector vocabulary and lands with it in evaluator increment 2
// (docs/evaluator-design.md §5/§11); the angular set lands with Revolve.

// Direction is the enumerated sense a standalone one-sided extent carries.
// There is no "both": a sweep that runs both ways is Symmetric or TwoSided,
// stated structurally.
type Direction int

const (
	// Along sweeps with the sketch plane's normal (U × V — core §7).
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
var errNilExtent = fmt.Errorf(`decad: nil extent`)

// normalizeExtent returns the value form of e: the variants seal with value
// receivers, so a *Distance satisfies Extent as readily as a Distance does —
// the codec accepts both and records the value the pointer names. A nil
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
		return *e, nil
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
