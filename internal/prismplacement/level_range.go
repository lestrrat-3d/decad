package prismplacement

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// LevelRange is the exact range a sweep level's true value lies in: the
// single rational DenotedLevel proves, or else the held float widened by its
// displacement on both sides (agent-docs/prism-boolean-design.md §3.2). Every
// G5 relation, and every stacked reading that decides a level's order or
// equality, compares these ranges and never the held floats alone.
type LevelRange struct {
	Lo, Hi *big.Rat
}

// LevelRangeOf is the range of the level held with displacement delta and
// denotation d, lifted by the exact shift. ok is false for a non-finite level
// or displacement, or a negative displacement.
func LevelRangeOf(held, delta float64, d LevelDenotation, shift *big.Rat) (LevelRange, bool) {
	if exact := DenotedLevel(held, delta, d); exact != nil {
		exact.Add(exact, shift)
		return LevelRange{Lo: exact, Hi: exact}, true
	}
	h, dr := proofarith.FloatRat(held), proofarith.FloatRat(delta)
	if h == nil || dr == nil || dr.Sign() < 0 {
		return LevelRange{}, false
	}
	h.Add(h, shift)
	return LevelRange{Lo: new(big.Rat).Sub(h, dr), Hi: new(big.Rat).Add(h, dr)}, true
}

// Below reports that every level r allows lies strictly below every level s
// allows.
func (r LevelRange) Below(s LevelRange) bool { return r.Hi.Cmp(s.Lo) < 0 }

// AtOrBelow reports that every level r allows lies at or below every level s
// allows. Two ranges meeting at one end order this way, and two single
// rationals do when they are equal or in order.
func (r LevelRange) AtOrBelow(s LevelRange) bool { return r.Hi.Cmp(s.Lo) <= 0 }

// SameLevel reports that r and s are one and the same rational: both are
// single rationals, and equal. Overlapping ranges prove no equality.
func (r LevelRange) SameLevel(s LevelRange) bool {
	return r.Lo.Cmp(r.Hi) == 0 && s.Lo.Cmp(s.Hi) == 0 && r.Lo.Cmp(s.Lo) == 0
}

// zRanges is the range of each sweep level of op lifted by shift.
func zRanges(op Operand, shift *big.Rat) (LevelRange, LevelRange, bool) {
	r0, ok0 := LevelRangeOf(op.Z0, op.Z0Delta, op.Z0Denoted, shift)
	r1, ok1 := LevelRangeOf(op.Z1, op.Z1Delta, op.Z1Denoted, shift)
	return r0, r1, ok0 && ok1
}

// IntervalRanges is A's two level ranges and B's after G5's exact shift onto
// A's axis.
func IntervalRanges(pa, pb Operand) (a0, a1, b0, b1 LevelRange, ok bool) {
	a0, a1, okA := zRanges(pa, new(big.Rat))
	b0, b1, okB := zRanges(pb, ZShift(pa, pb))
	return a0, a1, b0, b1, okA && okB
}
