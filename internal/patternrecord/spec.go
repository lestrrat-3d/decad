package patternrecord

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// PatternSpec lays out instances through a sealed linear or circular variant.
type PatternSpec interface{ patternSpec() }

// LinearPattern places instance i at i·Step along Dir, i = 0..Count-1.
// Instance 0 is the receiver. Dir is a nonzero direction; Step is a length.
type LinearPattern struct {
	Dir   r3.Vec
	Step  units.Value
	Count int
}

// CircularPattern places instance i at i/Count of a turn about the axis
// through Center along Axis, i = 0..Count-1.
type CircularPattern struct {
	Center, Axis r3.Vec
	Count        int
}

func (LinearPattern) patternSpec()   {}
func (CircularPattern) patternSpec() {}

func finiteVec(v r3.Vec) bool {
	return !math.IsNaN(v.X) && !math.IsNaN(v.Y) && !math.IsNaN(v.Z) &&
		!math.IsInf(v.X, 0) && !math.IsInf(v.Y, 0) && !math.IsInf(v.Z, 0)
}

// Resolve validates a pattern spec. Both value and pointer variants are
// accepted; a nil pointer is a degenerate input.
func Resolve(spec PatternSpec) (Spec, error) {
	errNil := fmt.Errorf(`%w: a nil pattern spec names no pattern`, decaderr.ErrDegenerate)
	switch s := spec.(type) {
	case *LinearPattern:
		if s == nil {
			return Spec{}, errNil
		}
		return Resolve(*s)
	case *CircularPattern:
		if s == nil {
			return Spec{}, errNil
		}
		return Resolve(*s)
	case LinearPattern:
		if s.Count < 2 {
			return Spec{}, fmt.Errorf(`%w: a pattern needs a Count of at least 2, got %d (a pattern of one is the receiver)`,
				decaderr.ErrDegenerate, s.Count)
		}
		if !finiteVec(s.Dir) {
			return Spec{}, fmt.Errorf(`%w: the pattern direction %v is not finite`, decaderr.ErrNotFinite, s.Dir)
		}
		if s.Dir == (r3.Vec{}) {
			return Spec{}, fmt.Errorf(`%w: a zero pattern direction names no line`, decaderr.ErrDegenerate)
		}
		step, err := sectionrecord.MagnitudeIn(s.Step, units.Length, units.Millimeter, "the pattern step")
		if err != nil {
			return Spec{}, err
		}
		if step == 0 {
			return Spec{}, fmt.Errorf(`%w: a zero pattern step stacks every instance on the receiver`, decaderr.ErrDegenerate)
		}
		exact := extent.ExactConversion(s.Step, units.Millimeter)
		if exact == nil {
			return Spec{}, fmt.Errorf(`%w: the pattern step is not representable`, decaderr.ErrNotFinite)
		}
		return Spec{Count: s.Count, Dir: s.Dir, StepMM: step, StepRat: exact}, nil
	case CircularPattern:
		if s.Count < 2 {
			return Spec{}, fmt.Errorf(`%w: a pattern needs a Count of at least 2, got %d (a pattern of one is the receiver)`,
				decaderr.ErrDegenerate, s.Count)
		}
		if !finiteVec(s.Center) || !finiteVec(s.Axis) {
			return Spec{}, fmt.Errorf(`%w: the pattern axis (%v, %v) is not finite`,
				decaderr.ErrNotFinite, s.Center, s.Axis)
		}
		if s.Axis == (r3.Vec{}) {
			return Spec{}, fmt.Errorf(`%w: a zero pattern axis names no rotation`, decaderr.ErrDegenerate)
		}
		return Spec{Count: s.Count, Circular: true, Center: s.Center, Axis: s.Axis}, nil
	default:
		return Spec{}, errNil
	}
}

// WorldMotion is instance i's rigid motion in world space.
func (rp Spec) WorldMotion(i int) (r3.Transform, error) {
	if rp.Circular {
		return r3.RotationAround(rp.Center, rp.Axis, units.Degrees(360*float64(i)/float64(rp.Count)))
	}
	k := float64(i) * rp.StepMM / rp.Dir.Len()
	return r3.Translation(rp.Dir.Scale(k))
}
