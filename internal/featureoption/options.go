// Package featureoption owns the validation of the optional inputs the
// profile-fed feature commands share, and the configurations those inputs fold
// into (agent-docs/command-api-design.md §4, rule A9). A command method stores
// a value with the error its validation here returned, and the operation's gate
// refuses that error where the old option decode did.
package featureoption

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/units"
)

// LoftSplit names one recorded segment of one loop and the fractions of its
// parameter range at which a loft cuts it.
type LoftSplit struct {
	Loop, Segment int
	At            []float64
}

// CloneSplits deep-copies splits, so a caller's later write to a slice reaches
// neither the copy nor its fractions.
func CloneSplits(in []LoftSplit) []LoftSplit {
	out := make([]LoftSplit, len(in))
	for i, s := range in {
		out[i] = LoftSplit{Loop: s.Loop, Segment: s.Segment, At: slices.Clone(s.At)}
	}
	return out
}

// ExtrudeConfig is the folded extrusion input set.
type ExtrudeConfig struct {
	Taper         units.Value
	SurfaceResult bool
}

// ValidateTaper checks that a taper is a representable angle. It reads
// nothing but the value.
func ValidateTaper(a units.Value) error {
	if a.Kind() != units.Angle {
		return fmt.Errorf(`%w: a taper must be an angle, got %s`, decaderr.ErrUnitKind, a.Kind())
	}
	if _, err := a.In(units.Radian); err != nil {
		return fmt.Errorf(`%w: the taper is not representable: %s`, decaderr.ErrNotFinite, err)
	}
	return nil
}

// SweepConfig is the folded sweep input set.
type SweepConfig struct {
	SurfaceResult bool
	Mitred        bool
	Scaled        bool
	Factors       []float64
	Twisted       bool
	Twist         units.Value
}

// SweepInput is what the sweep methods stored: the flags, the copied section
// scale factors with the error of validating them, and the twist angle with
// the error of validating it.
type SweepInput struct {
	SurfaceResult bool
	Mitred        bool
	ScaleSet      bool
	Scale         []float64
	ScaleErr      error
	TwistSet      bool
	Twist         units.Value
	TwistErr      error
}

// ValidateSectionScale checks every factor of a section scale and returns
// their base values. The factor count against the path is CheckScaleCount's.
func ValidateSectionScale(raw []units.Value) ([]float64, error) {
	out := make([]float64, len(raw))
	for k, f := range raw {
		if f.Kind() != units.Dimensionless {
			return nil, fmt.Errorf(`%w: section scale factor %d must be dimensionless, got %s`, decaderr.ErrUnitKind, k, f.Kind())
		}
		v := f.Base()
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return nil, fmt.Errorf(`%w: section scale factor %d is not finite`, decaderr.ErrNotFinite, k)
		}
		if v <= 0 {
			return nil, fmt.Errorf(`%w: section scale factor %d must be positive, got %g`, decaderr.ErrDegenerate, k, v)
		}
		out[k] = v
	}
	return out, nil
}

// ValidateTwist checks that a sweep twist is a representable angle.
func ValidateTwist(angle units.Value) error {
	if angle.Kind() != units.Angle {
		return fmt.Errorf(`%w: sweep twist must be an angle, got %s`, decaderr.ErrUnitKind, angle.Kind())
	}
	if _, err := angle.In(units.Radian); err != nil {
		return fmt.Errorf(`%w: the sweep twist is not representable: %s`, decaderr.ErrNotFinite, err)
	}
	return nil
}

// SweepFaultError is the fault of a stored sweep method. Method names the method
// whose value is refused, for the diagnostic's subject.
type SweepFaultError struct {
	Method string
	Err    error
}

// Error returns the text of the underlying fault.
func (f *SweepFaultError) Error() string { return f.Err.Error() }

// Unwrap returns the underlying fault, so errors.Is sees its sentinel.
func (f *SweepFaultError) Unwrap() error { return f.Err }

// FoldSweep folds the stored sweep inputs for a path of segments spans. A
// stored method fault is returned first, as a *SweepFaultError, in the order the
// section scale, the twist and then the conflicts were refused before.
func FoldSweep(in SweepInput, segments int) (SweepConfig, error) {
	cfg := SweepConfig{SurfaceResult: in.SurfaceResult, Mitred: in.Mitred}
	if in.ScaleSet {
		if in.ScaleErr != nil {
			return SweepConfig{}, &SweepFaultError{Method: "SectionScale", Err: in.ScaleErr}
		}
		if len(in.Scale) != segments {
			return SweepConfig{}, &SweepFaultError{Method: "SectionScale", Err: fmt.Errorf(
				`%w: SectionScale states %d factors for a path of %d segments`,
				decaderr.ErrDegenerate, len(in.Scale), segments)}
		}
		cfg.Scaled, cfg.Factors = true, slices.Clone(in.Scale)
	}
	if in.TwistSet {
		if in.TwistErr != nil {
			return SweepConfig{}, &SweepFaultError{Method: "Twist", Err: in.TwistErr}
		}
		cfg.Twisted = in.Twist.Mag() != 0
		cfg.Twist = in.Twist
	}
	if cfg.Twisted && (cfg.SurfaceResult || cfg.Mitred || cfg.Scaled) {
		return SweepConfig{}, fmt.Errorf(`%w: nonzero twist does not combine with a sheet, mitred joins, or section scale`, decaderr.ErrUnsupported)
	}
	if cfg.SurfaceResult && (cfg.Mitred || cfg.Scaled) {
		return SweepConfig{}, fmt.Errorf(`%w: a mitred or scaled sweep builds a solid only; SurfaceResult is not implemented for it (agent-docs/sweep-design.md Table SM row SM9)`, decaderr.ErrUnsupported)
	}
	return cfg, nil
}

// LoftConfig is the folded loft input set.
type LoftConfig struct {
	Alignment     []int
	SurfaceResult bool
	// Splits holds the splits of the first and the second section.
	Splits [2][]LoftSplit
}
