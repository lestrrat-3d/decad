package decad

import (
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
// body as a live dependency, exactly like EdgeAxis (core §8.1).

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
// only; each such body is resolved at the call and remains live
// (docs/evaluator-design.md §5).
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
// lateral footprint likewise not consulted. SideExtent only.
type ThroughAllSide struct{}

// ToFace stops the sweep at a face of Body, displaced by the signed Offset —
// the one signed number in the extent vocabulary: a positive offset
// overshoots the face, a negative one stops short of it, so
// ErrNegativeMagnitude does not apply (core §8.1/§12). A zero-value Offset
// means no displacement. The sense of the sweep comes from the target face,
// so ToFace carries no Direction. Face is a FaceSelector, never a *Face
// (core §9), resolved as Face.SelectFaces(Body) under the implicit
// exactly-one of core §12; Body must be a live body of the same document at
// the call and is not consumed or retired. Both Extent and SideExtent: a ToFace is a single
// direction of travel, so it may also stand as one side of a TwoSided.
type ToFace struct {
	Body   *Body
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

// errNilExtent rejects a nil variant pointer: it names no extent.
// It wraps ErrDegenerate so a typed nil pointer is branchable exactly like an
// untyped nil extent.
var errNilExtent = fmt.Errorf(`%w: nil extent`, ErrDegenerate)

// normalizeExtent returns the value form of e, RECURSIVELY: the variants seal
// with value receivers, so a *Distance satisfies Extent as readily as a
// Distance does — and a TwoSided's sides normalize with it, so no caller-owned
// pointer survives beyond the call boundary to alias document state. A nil
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
// must be a live body of the same document at the call and is not consumed or
// retired. Both
// AngularExtent and SideAngular.
type ToFaceAngular struct {
	Body *Body
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

// normalizeAngularExtent is normalizeExtent's angular analog: it returns the
// value form of a, RECURSIVELY — a TwoSidedAngle's sides normalize with it,
// so no caller-owned pointer survives beyond the call boundary to alias
// document state. A nil pointer is rejected.
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
