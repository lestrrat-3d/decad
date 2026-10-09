// Package featureoption owns the option vocabulary shared by profile-fed features.
package featureoption

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// ExtrudeOption configures Extrude.
type ExtrudeOption interface {
	option.Interface
	extrudeOption()
}

// RevolveOption configures Revolve.
type RevolveOption interface {
	option.Interface
	revolveOption()
}

// SweepOption configures Sweep.
type SweepOption interface {
	option.Interface
	sweepOption()
}

// LoftOption configures Loft.
type LoftOption interface {
	option.Interface
	loftOption()
}

// SurfaceResultOption is accepted by all four profile-fed features.
type SurfaceResultOption interface {
	ExtrudeOption
	RevolveOption
	SweepOption
	LoftOption
}

// CoilOption configures Coil.
type CoilOption interface {
	option.Interface
	coilOption()
}

type extrudeOption struct{ option.Interface }
type sweepOption struct{ option.Interface }
type loftOption struct{ option.Interface }
type coilOption struct{ option.Interface }
type surfaceResultOption struct{ option.Interface }

func (extrudeOption) extrudeOption()       {}
func (sweepOption) sweepOption()           {}
func (loftOption) loftOption()             {}
func (coilOption) coilOption()             {}
func (surfaceResultOption) extrudeOption() {}
func (surfaceResultOption) revolveOption() {}
func (surfaceResultOption) sweepOption()   {}
func (surfaceResultOption) loftOption()    {}

type identTaper struct{}
type identSurfaceResult struct{}
type identSweepTwist struct{}
type identMitredJoins struct{}
type identSectionScale struct{}
type identLoftAlignment struct{}
type identLeftHand struct{}

// WithTaper records the signed extrusion taper angle.
func WithTaper(a units.Value) ExtrudeOption {
	return extrudeOption{option.New(identTaper{}, a)}
}

// WithSurfaceResult omits closing faces from a profile-fed feature.
func WithSurfaceResult() SurfaceResultOption {
	return surfaceResultOption{option.New(identSurfaceResult{}, struct{}{})}
}

// WithSweepTwist records a sweep twist angle.
func WithSweepTwist(angle units.Value) SweepOption {
	return sweepOption{option.New(identSweepTwist{}, angle)}
}

// WithMitredJoins selects a mitred polyline sweep.
func WithMitredJoins() SweepOption {
	return sweepOption{option.New(identMitredJoins{}, struct{}{})}
}

// WithSectionScale records one section scale per path segment.
func WithSectionScale(factors ...units.Value) SweepOption {
	return sweepOption{option.New(identSectionScale{}, append([]units.Value(nil), factors...))}
}

// WithLoftAlignment records one segment offset per paired loop.
func WithLoftAlignment(offsets ...int) LoftOption {
	out := make([]int, len(offsets))
	copy(out, offsets)
	return loftOption{option.New(identLoftAlignment{}, out)}
}

// WithLeftHand reverses a coil's turn sense.
func WithLeftHand() CoilOption {
	return coilOption{option.New(identLeftHand{}, struct{}{})}
}

// IsSurfaceResult reports whether an option names the shared surface result.
// Revolve tests identifiers directly, so this follows that call's semantics.
func IsSurfaceResult(o option.Interface) bool {
	_, ok := o.Ident().(identSurfaceResult)
	return ok
}

// ExtrudeConfig is the folded extrusion option set.
type ExtrudeConfig struct {
	Taper         units.Value
	SurfaceResult bool
}

// DecodeExtrude preserves Extrude's last-taper-wins and repeated-surface rules.
func DecodeExtrude(opts []ExtrudeOption) (ExtrudeConfig, error) {
	cfg := ExtrudeConfig{Taper: units.Degrees(0)}
	for _, o := range opts {
		if o == nil {
			return ExtrudeConfig{}, fmt.Errorf(`%w: a nil option names nothing to apply`, decaderr.ErrDegenerate)
		}
		switch o.Ident().(type) {
		case identTaper:
			v, ok := option.Get[units.Value](o)
			if !ok {
				return ExtrudeConfig{}, fmt.Errorf(`%w: WithTaper carries no angle`, decaderr.ErrDegenerate)
			}
			cfg.Taper = v
		case identSurfaceResult:
			cfg.SurfaceResult = true
		}
	}
	if cfg.Taper.Kind() != units.Angle {
		return ExtrudeConfig{}, fmt.Errorf(`%w: a taper must be an angle, got %s`, decaderr.ErrUnitKind, cfg.Taper.Kind())
	}
	if _, err := cfg.Taper.In(units.Radian); err != nil {
		return ExtrudeConfig{}, fmt.Errorf(`%w: the taper is not representable: %s`, decaderr.ErrNotFinite, err)
	}
	return cfg, nil
}

// SweepConfig is the validated sweep option set.
type SweepConfig struct {
	SurfaceResult bool
	Mitred        bool
	Scaled        bool
	Factors       []float64
}

// DecodeSweep validates sweep options against the path's segment count.
func DecodeSweep(opts []SweepOption, segments int) (SweepConfig, error) {
	var cfg SweepConfig
	haveTwist := false
	var twist sweepOption
	var scale []units.Value
	for _, raw := range opts {
		if raw == nil {
			return SweepConfig{}, fmt.Errorf(`%w: a nil option names nothing to apply`, decaderr.ErrDegenerate)
		}
		if _, ok := raw.(surfaceResultOption); ok {
			cfg.SurfaceResult = true
			continue
		}
		o, ok := raw.(sweepOption)
		if !ok {
			return SweepConfig{}, fmt.Errorf(`%w: the sweep option is not a decad sweep option (%T)`, decaderr.ErrDegenerate, raw)
		}
		switch ident := o.Ident().(type) {
		case identSweepTwist:
			if haveTwist {
				return SweepConfig{}, fmt.Errorf(`%w: WithSweepTwist was passed more than once`, decaderr.ErrDegenerate)
			}
			twist, haveTwist = o, true
		case identMitredJoins:
			if cfg.Mitred {
				return SweepConfig{}, fmt.Errorf(`%w: WithMitredJoins was passed more than once`, decaderr.ErrDegenerate)
			}
			cfg.Mitred = true
		case identSectionScale:
			if cfg.Scaled {
				return SweepConfig{}, fmt.Errorf(`%w: WithSectionScale was passed more than once`, decaderr.ErrDegenerate)
			}
			factors, ok := option.Get[[]units.Value](o)
			if !ok {
				return SweepConfig{}, fmt.Errorf(`%w: WithSectionScale carries no factors`, decaderr.ErrDegenerate)
			}
			scale, cfg.Scaled = factors, true
		default:
			return SweepConfig{}, fmt.Errorf(`%w: unknown sweep option identifier %T`, decaderr.ErrDegenerate, ident)
		}
	}
	if cfg.Scaled {
		factors, err := validateSectionScale(scale, segments)
		if err != nil {
			return SweepConfig{}, err
		}
		cfg.Factors = factors
	}
	if haveTwist {
		angle, ok := option.Get[units.Value](twist)
		if !ok {
			return SweepConfig{}, fmt.Errorf(`%w: WithSweepTwist carries no angle`, decaderr.ErrDegenerate)
		}
		if angle.Kind() != units.Angle {
			return SweepConfig{}, fmt.Errorf(`%w: sweep twist must be an angle, got %s`, decaderr.ErrUnitKind, angle.Kind())
		}
		if _, err := angle.In(units.Radian); err != nil {
			return SweepConfig{}, fmt.Errorf(`%w: the sweep twist is not representable: %s`, decaderr.ErrNotFinite, err)
		}
		if angle.Mag() != 0 {
			return SweepConfig{}, fmt.Errorf(`%w: nonzero sweep twist is not implemented`, decaderr.ErrUnsupported)
		}
	}
	if cfg.SurfaceResult && (cfg.Mitred || cfg.Scaled) {
		return SweepConfig{}, fmt.Errorf(`%w: a mitred or scaled sweep builds a solid only; WithSurfaceResult is not implemented for it (docs/sweep-design.md Table SM row SM9)`, decaderr.ErrUnsupported)
	}
	return cfg, nil
}

func validateSectionScale(raw []units.Value, segments int) ([]float64, error) {
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
	if len(out) != segments {
		return nil, fmt.Errorf(`%w: WithSectionScale states %d factors for a path of %d segments`, decaderr.ErrDegenerate, len(out), segments)
	}
	return out, nil
}

// LoftConfig is the folded loft option set.
type LoftConfig struct {
	Alignment     []int
	SurfaceResult bool
}

// DecodeLoft checks owned options and repeated alignment before the sketch seam.
func DecodeLoft(opts []LoftOption) (LoftConfig, error) {
	var cfg LoftConfig
	haveAlignment := false
	for _, raw := range opts {
		if raw == nil {
			return LoftConfig{}, fmt.Errorf(`%w: a nil option names nothing to apply`, decaderr.ErrDegenerate)
		}
		if _, ok := raw.(surfaceResultOption); ok {
			cfg.SurfaceResult = true
			continue
		}
		o, ok := raw.(loftOption)
		if !ok {
			return LoftConfig{}, fmt.Errorf(`%w: the loft option is not a decad loft option (%T)`, decaderr.ErrDegenerate, raw)
		}
		switch ident := o.Ident().(type) {
		case identLoftAlignment:
			if haveAlignment {
				return LoftConfig{}, fmt.Errorf(`%w: WithLoftAlignment was passed more than once; two alignment payloads name two different correspondences`, decaderr.ErrDegenerate)
			}
			v, ok := option.Get[[]int](o)
			if !ok {
				return LoftConfig{}, fmt.Errorf(`%w: WithLoftAlignment carries no offsets`, decaderr.ErrDegenerate)
			}
			cfg.Alignment = slices.Clone(v)
			haveAlignment = true
		default:
			return LoftConfig{}, fmt.Errorf(`%w: unknown loft option identifier %T`, decaderr.ErrDegenerate, ident)
		}
	}
	return cfg, nil
}

// DecodeCoil checks the owned coil option and its at-most-once rule.
func DecodeCoil(opts []CoilOption) (bool, error) {
	leftHand := false
	for _, raw := range opts {
		if raw == nil {
			return false, fmt.Errorf(`%w: a nil option names nothing to apply`, decaderr.ErrDegenerate)
		}
		o, ok := raw.(coilOption)
		if !ok {
			return false, fmt.Errorf(`%w: the coil option is not a decad coil option (%T)`, decaderr.ErrDegenerate, raw)
		}
		switch ident := o.Ident().(type) {
		case identLeftHand:
			if leftHand {
				return false, fmt.Errorf(`%w: WithLeftHand was passed more than once`, decaderr.ErrDegenerate)
			}
			leftHand = true
		default:
			return false, fmt.Errorf(`%w: unknown coil option identifier %T`, decaderr.ErrDegenerate, ident)
		}
	}
	return leftHand, nil
}
