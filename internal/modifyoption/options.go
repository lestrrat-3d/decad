// Package modifyoption owns the Fillet, Chamfer, and Shell option vocabulary.
package modifyoption

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// FilletOption configures Fillet.
type FilletOption interface {
	option.Interface
	filletOption()
}

// ChamferOption configures Chamfer.
type ChamferOption interface {
	option.Interface
	chamferOption()
}

// FilletChamferOption is accepted by both Fillet and Chamfer.
type FilletChamferOption interface {
	FilletOption
	ChamferOption
}

// ShellOption configures Shell.
type ShellOption interface {
	option.Interface
	shellOption()
}

type filletChamferOption struct{ option.Interface }

func (filletChamferOption) filletOption()  {}
func (filletChamferOption) chamferOption() {}

type chamferOption struct{ option.Interface }

func (chamferOption) chamferOption() {}

type shellOption struct{ option.Interface }

func (shellOption) shellOption() {}

type identTangentChain struct{}
type identAsymmetricChamfer struct{}
type identNoOpenings struct{}
type identShellSense struct{}

const (
	shellInward = iota
	shellOutward
)

func shellSenseName(s int) string {
	switch s {
	case shellInward:
		return "Inward"
	case shellOutward:
		return "Outward"
	default:
		return fmt.Sprintf("ShellSense(%d)", int(s))
	}
}

// WithTangentChain expands selected edges across proven tangent continuations.
func WithTangentChain() FilletChamferOption {
	return filletChamferOption{option.New(identTangentChain{}, struct{}{})}
}

// Asymmetric records a copied reference selector and the second setback.
// The decoder fills OtherMM and OtherDelta after validating Other.
type Asymmetric[Q any] struct {
	Reference  Q
	Foreign    string
	Other      units.Value
	OtherMM    float64
	OtherDelta float64
}

// WithAsymmetricChamfer records the reference and second distance.
func WithAsymmetricChamfer[Q any](a Asymmetric[Q]) ChamferOption {
	return chamferOption{option.New(identAsymmetricChamfer{}, a)}
}

// WithNoOpenings asks Shell to keep every face.
func WithNoOpenings() ShellOption {
	return shellOption{option.New(identNoOpenings{}, struct{}{})}
}

// WithShellSense records the root package's wall sense as its numeric value.
func WithShellSense(s int) ShellOption {
	return shellOption{option.New(identShellSense{}, s)}
}

// FilletConfig is the decoded Fillet option set.
type FilletConfig struct {
	TangentChain bool
}

// ChamferConfig is the decoded Chamfer option set.
type ChamferConfig[Q any] struct {
	TangentChain bool
	Asymmetric   *Asymmetric[Q]
}

// ShellConfig is the decoded Shell option set.
type ShellConfig struct {
	Sense      int
	NoOpenings bool
}

// ErrOptionConflict reports an option list that names no single intent.
func ErrOptionConflict(format string, args ...any) error {
	return fmt.Errorf(`%w: `+format+` (modify-reach SX1)`, append([]any{decaderr.ErrDegenerate}, args...)...)
}

// DecodeFillet validates and folds Fillet options.
func DecodeFillet(opts []FilletOption) (FilletConfig, error) {
	var out FilletConfig
	for _, raw := range opts {
		if raw == nil {
			return FilletConfig{}, fmt.Errorf(`%w: a nil option names nothing to apply`, decaderr.ErrDegenerate)
		}
		// An embedded interface can promote the marker onto a foreign type.
		o, ok := raw.(filletChamferOption)
		if !ok {
			return FilletConfig{}, fmt.Errorf(`%w: the fillet option is not a decad fillet option (%T)`, decaderr.ErrDegenerate, raw)
		}
		switch ident := o.Ident().(type) {
		case identTangentChain:
			out.TangentChain = true
		default:
			return FilletConfig{}, ErrOptionConflict(`unknown fillet option identifier %T`, ident)
		}
	}
	return out, nil
}

// DecodeChamfer validates and folds Chamfer options.
func DecodeChamfer[Q any](opts []ChamferOption) (ChamferConfig[Q], error) {
	var out ChamferConfig[Q]
	for _, raw := range opts {
		if raw == nil {
			return ChamferConfig[Q]{}, fmt.Errorf(`%w: a nil option names nothing to apply`, decaderr.ErrDegenerate)
		}
		var o option.Interface
		switch v := raw.(type) {
		case filletChamferOption:
			o = v
		case chamferOption:
			o = v
		default:
			return ChamferConfig[Q]{}, fmt.Errorf(`%w: the chamfer option is not a decad chamfer option (%T)`, decaderr.ErrDegenerate, raw)
		}
		switch ident := o.Ident().(type) {
		case identTangentChain:
			out.TangentChain = true
		case identAsymmetricChamfer:
			if out.Asymmetric != nil {
				return ChamferConfig[Q]{}, ErrOptionConflict(`WithAsymmetricChamfer is given twice; one chamfer takes one reference and one other distance`)
			}
			a, ok := option.Get[Asymmetric[Q]](o)
			if !ok {
				return ChamferConfig[Q]{}, ErrOptionConflict(`WithAsymmetricChamfer carries no reference and distance`)
			}
			mm, delta, err := extent.MagnitudeInBounded(a.Other, units.Length, units.Millimeter,
				"the asymmetric chamfer's other distance")
			if err != nil {
				return ChamferConfig[Q]{}, err
			}
			if mm == 0 {
				return ChamferConfig[Q]{}, fmt.Errorf(`%w: an asymmetric chamfer's other distance must be positive; a zero setback leaves that face where it is`, decaderr.ErrDegenerate)
			}
			a.OtherMM, a.OtherDelta = mm, delta
			out.Asymmetric = &a
		default:
			return ChamferConfig[Q]{}, ErrOptionConflict(`unknown chamfer option identifier %T`, ident)
		}
	}
	return out, nil
}

// DecodeShell validates and folds Shell options, defaulting to Inward.
func DecodeShell(opts []ShellOption) (ShellConfig, error) {
	out := ShellConfig{Sense: shellInward}
	sensed := false
	for _, raw := range opts {
		if raw == nil {
			return ShellConfig{}, fmt.Errorf(`%w: a nil option names nothing to apply`, decaderr.ErrDegenerate)
		}
		// An embedded interface can promote the marker onto a foreign type.
		o, ok := raw.(shellOption)
		if !ok {
			return ShellConfig{}, fmt.Errorf(`%w: the shell option is not a decad shell option (%T)`, decaderr.ErrDegenerate, raw)
		}
		switch ident := o.Ident().(type) {
		case identShellSense:
			v, ok := option.Get[int](o)
			if !ok {
				return ShellConfig{}, fmt.Errorf(`%w: WithShellSense carries no sense`, decaderr.ErrDegenerate)
			}
			if v != shellInward && v != shellOutward {
				return ShellConfig{}, fmt.Errorf(`%w: unknown shell sense %d`, decaderr.ErrDegenerate, int(v))
			}
			if sensed && v != out.Sense {
				return ShellConfig{}, ErrOptionConflict(`WithShellSense names both %s and %s`,
					shellSenseName(out.Sense), shellSenseName(v))
			}
			out.Sense, sensed = v, true
		case identNoOpenings:
			out.NoOpenings = true
		default:
			return ShellConfig{}, fmt.Errorf(`%w: unknown shell option identifier %T`, decaderr.ErrDegenerate, ident)
		}
	}
	return out, nil
}
