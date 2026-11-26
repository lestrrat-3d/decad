// Package modifyoption owns the Fillet, Chamfer and Shell setting values the
// root commands fold their methods into.
package modifyoption

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/units"
)

// The wall senses of a shell, as the root package's ShellSense numbers them.
const (
	ShellInward = iota
	ShellOutward
	ShellCentered
)

// Asymmetric records a copied reference selector and the second setback.
// ValidateAsymmetric fills OtherMM and OtherDelta from Other.
type Asymmetric[Q any] struct {
	Reference  Q
	Foreign    string
	Other      units.Value
	OtherMM    float64
	OtherDelta float64
}

// FaceThickness is the thickness a shell gives the walls whose outward normal is a stated world axis direction.
type FaceThickness struct {
	// Axis is 0, 1 or 2 for X, Y or Z and Sign is +1 or -1 for the direction along it.
	Axis, Sign int
	T          units.Value
}

// FilletConfig is the Fillet setting set.
type FilletConfig struct {
	TangentChain bool
}

// ShellConfig is the Shell setting set.
type ShellConfig struct {
	Sense         int
	NoOpenings    bool
	FaceThickness []FaceThickness
}

// ValidateAsymmetric checks the second setback of an asymmetric chamfer and
// returns the record with its millimetre value and error bound filled in.
func ValidateAsymmetric[Q any](a Asymmetric[Q]) (Asymmetric[Q], error) {
	mm, delta, err := extent.MagnitudeInBounded(a.Other, units.Length, units.Millimeter,
		"the asymmetric chamfer's other distance")
	if err != nil {
		return a, err
	}
	if mm == 0 {
		return a, fmt.Errorf(`%w: an asymmetric chamfer's other distance must be positive; a zero setback leaves that face where it is`, decaderr.ErrDegenerate)
	}
	a.OtherMM, a.OtherDelta = mm, delta
	return a, nil
}

// ValidateShellSense refuses a sense number that names no wall sense.
func ValidateShellSense(s int) error {
	if s != ShellInward && s != ShellOutward && s != ShellCentered {
		return fmt.Errorf(`%w: unknown shell sense %d (SX1)`, decaderr.ErrDegenerate, s)
	}
	return nil
}
