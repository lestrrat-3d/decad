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

// OverlapChordRatio is how many times the pair's diameter must exceed its
// operands' box overlap before a Cut chords its fresh operands by the overlap
// (OverlapDiameter). A short tool at one end of a long bar passes it; a pair
// whose operands mostly overlap keeps the pair's chord, and so its facet
// counts and the work every later check spends on them.
const OverlapChordRatio = 8

// OverlapDiameter is the diameter of the two inflated boxes' common part, and
// false when they share no volume. A Cut's operands are chorded by it: only
// their common part can meet, so a short tool at one end of a long bar
// chords the pair as finely as a short bar would. The pair's own tolerance,
// and so every held operand's request and the chain-depth gate, still reads
// the whole pair's diameter.
func OverlapDiameter(a, b ChordOperand) (float64, bool) {
	d := r3.Vec{
		X: math.Min(a.Hi.X, b.Hi.X) - math.Max(a.Lo.X, b.Lo.X),
		Y: math.Min(a.Hi.Y, b.Hi.Y) - math.Max(a.Lo.Y, b.Lo.Y),
		Z: math.Min(a.Hi.Z, b.Hi.Z) - math.Max(a.Lo.Z, b.Lo.Z),
	}
	if !(d.X > 0 && d.Y > 0 && d.Z > 0) {
		return 0, false
	}
	return d.Len(), true
}

// OperandChord is the chord tolerance one operand is requested at for a pair
// whose own tolerance is pairTol. An operand that restates a held mesh takes
// the pair's tolerance: its request only has to sit at or above its floor, and
// it chords nothing. Every other operand is chorded at the diameter-derived
// tolerance alone, raised only to its own floor, so a partner that holds a
// coarse fixed mesh (a twisted sweep, a coil, a mitred sweep) does not coarsen
// it (agent-docs/chained-cut-design.md §4). The result never exceeds pairTol.
// diameter is the pair diameter scaled as pairTol is, and floor is the
// operand's own ChordOperand.Floor scaled likewise.
func OperandChord(pairTol, diameter, floor float64, restating bool) float64 {
	if restating {
		return pairTol
	}
	return math.Min(pairTol, math.Max(diameter*ChordFactor, floor))
}
