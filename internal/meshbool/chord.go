package meshbool

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ChordFactor sets the evaluator's chord tolerance from a pair's diameter.
const ChordFactor = 2e-5

// ChordOperand holds one operand's inflated bounds and its own tessellation
// reservation. The caller reads these values from the body once per pair.
type ChordOperand struct {
	Lo, Hi r3.Vec
	Floor  float64
}

// PairChordFrom combines two inflated boxes and raises the diameter-derived
// tolerance past both operands' reservations. It refuses a pair without a
// finite positive extent.
func PairChordFrom(a, b ChordOperand) (tol, diameter float64, ok bool) {
	lo := r3.Vec{
		X: math.Min(a.Lo.X, b.Lo.X),
		Y: math.Min(a.Lo.Y, b.Lo.Y),
		Z: math.Min(a.Lo.Z, b.Lo.Z),
	}
	hi := r3.Vec{
		X: math.Max(a.Hi.X, b.Hi.X),
		Y: math.Max(a.Hi.Y, b.Hi.Y),
		Z: math.Max(a.Hi.Z, b.Hi.Z),
	}
	diameter = hi.Sub(lo).Len()
	if diameter <= 0 || proofbound.IsNonFinite(diameter) {
		return 0, 0, false
	}
	return math.Max(diameter*ChordFactor, math.Max(a.Floor, b.Floor)), diameter, true
}
