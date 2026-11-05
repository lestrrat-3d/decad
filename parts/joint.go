package parts

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// JointKind is the geometric form of a connection. The zero value is reserved.
type JointKind int

const (
	JointUnset  JointKind = iota
	PinInBore             // a convex cylinder of A inside a coaxial concave cylinder of B
	PinInSlot             // a convex cylinder of A between two parallel planar faces of B
	FaceContact           // planar faces of A and B rest on each other: a lap, a butt
	Contact               // A and B touch by design in an unspecified way: a snap at rest
	Keepout               // A and B must stay MinGap apart
	Tie                   // a declared connection with no geometric check: a band on a post
)

// String returns the lower-case words for the kind.
func (k JointKind) String() string {
	switch k {
	case PinInBore:
		return "pin in bore"
	case PinInSlot:
		return "pin in slot"
	case FaceContact:
		return "face contact"
	case Contact:
		return "contact"
	case Keepout:
		return "keepout"
	case Tie:
		return "tie"
	}
	return "unset"
}

// Motion is what the joint lets A do against B. Fixed is the zero value.
type Motion int

const (
	Fixed          Motion = iota
	Turns                 // about the joint axis
	Slides                // along Joint.Direction, or along a slot
	TurnsAndSlides        // about and along the joint axis
)

// String returns the lower-case words for the motion.
func (m Motion) String() string {
	switch m {
	case Turns:
		return "turns"
	case Slides:
		return "slides"
	case TurnsAndSlides:
		return "turns and slides"
	}
	return "fixed"
}

// Axis is a line: a point on it and a unit direction.
type Axis struct{ Point, Dir r3.Vec }

// Joint connects part A to part B. A is the inserted or moving member (the pin,
// the slider), B the receiver.
type Joint struct {
	Name   string // unique within the Set; empty names get "A~B"
	Kind   JointKind
	A, B   string // part names
	Motion Motion
	// Axis is the author's statement of a PinInBore's or PinInSlot's axis, in
	// assembled world coordinates, or nil. The faces' own axis is used when
	// they show one; Axis is the fallback where they do not, and Check reports
	// a declared axis the faces contradict.
	Axis *Axis
	// Clearance is the declared radial clearance of a PinInBore or the
	// declared side clearance of a PinInSlot, a signed Length: negative is a
	// press fit. The zero Value declares none, and Check reports the measured
	// clearance without judging it.
	Clearance units.Value
	// MinGap is a Keepout's least allowed gap, a Length > 0.
	MinGap units.Value
	// Direction is a Slides joint's direction for a FaceContact, in assembled
	// world coordinates. Zero elsewhere.
	Direction r3.Vec
}

func (j Joint) geometric() bool { return j.Kind != Tie }

// Join appends j. It refuses, with decad.ErrDegenerate, an unknown part,
// A == B, JointUnset, a repeated name, a Keepout without a positive MinGap, a
// Motion the kind cannot have, an Axis on a kind without one or with a zero
// direction, and a geometric kind naming a body-less part. A quantity of the
// wrong kind is decad.ErrUnitKind and a non-finite one decad.ErrNotFinite. Join
// normalises Axis.Dir.
func (s *Set) Join(j Joint) error {
	if s == nil || s.doc == nil {
		return fmt.Errorf("parts: %w: nil set or document", decad.ErrDegenerate)
	}
	if j.Kind < PinInBore || j.Kind > Tie {
		return fmt.Errorf("parts: %w: a joint needs a kind", decad.ErrDegenerate)
	}
	ia, oka := s.index[j.A]
	ib, okb := s.index[j.B]
	if !oka || !okb {
		return fmt.Errorf("parts: %w: joint names an unknown part (%q, %q)", decad.ErrDegenerate, j.A, j.B)
	}
	if j.A == j.B {
		return fmt.Errorf("parts: %w: joint joins part %q to itself", decad.ErrDegenerate, j.A)
	}
	if j.Name == "" {
		j.Name = j.A + "~" + j.B
	}
	if _, dup := s.jnames[j.Name]; dup {
		return fmt.Errorf("parts: %w: joint %q is already in the set", decad.ErrDegenerate, j.Name)
	}
	if err := checkLength(j.Clearance, "clearance", true); err != nil {
		return err
	}
	if err := checkLength(j.MinGap, "min gap", false); err != nil {
		return err
	}
	if !finiteVec(j.Direction) {
		return fmt.Errorf("parts: %w: joint %q direction", decad.ErrNotFinite, j.Name)
	}
	if j.Kind == Keepout && j.MinGap.Base() <= 0 {
		return fmt.Errorf("parts: %w: keepout %q needs a positive min gap", decad.ErrDegenerate, j.Name)
	}
	if err := checkMotion(j); err != nil {
		return err
	}
	if j.Axis != nil {
		if j.Kind != PinInBore && j.Kind != PinInSlot {
			return fmt.Errorf("parts: %w: joint %q: only a pin joint has an axis", decad.ErrDegenerate, j.Name)
		}
		if !finiteVec(j.Axis.Point) || !finiteVec(j.Axis.Dir) {
			return fmt.Errorf("parts: %w: joint %q axis", decad.ErrNotFinite, j.Name)
		}
		d, ok := j.Axis.Dir.Normalize()
		if !ok {
			return fmt.Errorf("parts: %w: joint %q axis has no direction", decad.ErrDegenerate, j.Name)
		}
		ax := *j.Axis
		ax.Dir = d
		j.Axis = &ax
	}
	if j.geometric() && (s.parts[ia].Body == nil || s.parts[ib].Body == nil) {
		return fmt.Errorf("parts: %w: joint %q names a part without a body", decad.ErrDegenerate, j.Name)
	}
	s.jnames[j.Name] = struct{}{}
	s.joints = append(s.joints, j)
	return nil
}

func checkMotion(j Joint) error {
	switch j.Kind {
	case Keepout, Contact, Tie:
		if j.Motion != Fixed {
			return fmt.Errorf("parts: %w: a %s joint is fixed", decad.ErrDegenerate, j.Kind)
		}
	case FaceContact:
		if j.Motion == Turns || j.Motion == TurnsAndSlides {
			return fmt.Errorf("parts: %w: a face contact cannot turn", decad.ErrDegenerate)
		}
		if j.Motion == Slides && j.Direction == (r3.Vec{}) {
			return fmt.Errorf("parts: %w: a sliding face contact needs a direction", decad.ErrDegenerate)
		}
	}
	if j.Motion < Fixed || j.Motion > TurnsAndSlides {
		return fmt.Errorf("parts: %w: unknown motion", decad.ErrDegenerate)
	}
	return nil
}

// Joints returns the joints in Join order.
func (s *Set) Joints() []Joint { return slices.Clone(s.joints) }

func finiteVec(v r3.Vec) bool {
	return !math.IsNaN(v.X+v.Y+v.Z) && !math.IsInf(v.X+v.Y+v.Z, 0)
}

// checkLength validates a Length. A zero Value is "none" and passes.
func checkLength(v units.Value, what string, signed bool) error {
	if v == (units.Value{}) {
		return nil
	}
	if v.Kind() != units.Length {
		return fmt.Errorf("parts: %s: %w", what, decad.ErrUnitKind)
	}
	b := v.Base()
	if math.IsNaN(b) || math.IsInf(b, 0) {
		return fmt.Errorf("parts: %s: %w", what, decad.ErrNotFinite)
	}
	if !signed && b < 0 {
		return fmt.Errorf("parts: %s: %w", what, decad.ErrNegativeMagnitude)
	}
	return nil
}
