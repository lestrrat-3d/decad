package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/featureoption"
)

// These values keep the public option tiers in decad while the option codec
// validates and folds their payloads in internal/featureoption.
type extrudeOptionValue struct{ featureoption.ExtrudeOption }
type sweepOptionValue struct{ featureoption.SweepOption }
type loftOptionValue struct{ featureoption.LoftOption }
type surfaceResultOptionValue struct {
	featureoption.SurfaceResultOption
}

func (extrudeOptionValue) extrudeOption()       {}
func (sweepOptionValue) sweepOption()           {}
func (loftOptionValue) loftOption()             {}
func (surfaceResultOptionValue) extrudeOption() {}
func (surfaceResultOptionValue) revolveOption() {}
func (surfaceResultOptionValue) sweepOption()   {}
func (surfaceResultOptionValue) loftOption()    {}

func decodeExtrudeOptions(opts []ExtrudeOption) (featureoption.ExtrudeConfig, error) {
	encoded := make([]featureoption.ExtrudeOption, len(opts))
	for i, opt := range opts {
		if opt == nil {
			continue
		}
		switch value := opt.(type) {
		case extrudeOptionValue:
			encoded[i] = value.ExtrudeOption
		case surfaceResultOptionValue:
			encoded[i] = value.SurfaceResultOption
		default:
			return featureoption.ExtrudeConfig{}, fmt.Errorf(`%w: the extrude option is not a decad extrude option (%T)`, ErrDegenerate, opt)
		}
	}
	return featureoption.DecodeExtrude(encoded)
}

func decodeSweepOptions(opts []SweepOption, segments int) (featureoption.SweepConfig, error) {
	encoded := make([]featureoption.SweepOption, len(opts))
	for i, opt := range opts {
		if opt == nil {
			continue
		}
		switch value := opt.(type) {
		case sweepOptionValue:
			encoded[i] = value.SweepOption
		case surfaceResultOptionValue:
			encoded[i] = value.SurfaceResultOption
		default:
			return featureoption.SweepConfig{}, fmt.Errorf(`%w: the sweep option is not a decad sweep option (%T)`, ErrDegenerate, opt)
		}
	}
	return featureoption.DecodeSweep(encoded, segments)
}

func decodeLoftOptions(opts []LoftOption) (featureoption.LoftConfig, error) {
	encoded := make([]featureoption.LoftOption, len(opts))
	for i, opt := range opts {
		if opt == nil {
			continue
		}
		switch value := opt.(type) {
		case loftOptionValue:
			encoded[i] = value.LoftOption
		case surfaceResultOptionValue:
			encoded[i] = value.SurfaceResultOption
		default:
			return featureoption.LoftConfig{}, fmt.Errorf(`%w: the loft option is not a decad loft option (%T)`, ErrDegenerate, opt)
		}
	}
	return featureoption.DecodeLoft(encoded)
}
