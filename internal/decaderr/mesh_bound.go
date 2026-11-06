package decaderr

import (
	"fmt"

	"github.com/lestrrat-3d/units"
)

// MeshBoundError is Tessellate's refusal of a chord tolerance finer than the
// minimum mesh bound a faceted body holds. Every tolerance at or above Floor
// restates the same held facets. It unwraps to [ErrUnsupported].
type MeshBoundError struct{ requested, floor float64 } // millimetres

// NewMeshBoundError returns the refusal of a requested chord tolerance below
// floor, both in millimetres.
func NewMeshBoundError(requested, floor float64) *MeshBoundError {
	return &MeshBoundError{requested: requested, floor: floor}
}

// Requested returns the refused chord tolerance.
func (e *MeshBoundError) Requested() units.Value { return units.Millimeters(e.requested) }

// Floor returns the minimum mesh bound the body holds.
func (e *MeshBoundError) Floor() units.Value { return units.Millimeters(e.floor) }

// Error reports the refusal and the tolerance that would be accepted.
func (e *MeshBoundError) Error() string {
	requested, floor := e.Requested(), e.Floor()
	return fmt.Sprintf(`%v: requested tolerance %s is below the faceted body's minimum mesh bound %s; retry with a tolerance of at least %s to restate the held mesh`, ErrUnsupported, requested, floor, floor)
}

// Unwrap returns [ErrUnsupported] so errors.Is keeps branching on it.
func (e *MeshBoundError) Unwrap() error { return ErrUnsupported }
