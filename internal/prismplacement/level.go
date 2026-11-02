package prismplacement

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/denotation"
)

// LevelDenotation records a validated exact prism level.
type LevelDenotation = denotation.LevelDenotation

// DenoteLevel records an exact level and its held displacement.
func DenoteLevel(exact *big.Rat, held, delta float64) LevelDenotation {
	return denotation.DenoteLevel(exact, held, delta)
}

// DenotedLevel returns a copy of the level denoted by the held value.
func DenotedLevel(held, delta float64, d LevelDenotation) *big.Rat {
	return denotation.DenotedLevel(held, delta, d)
}

// SameDenotedLevel reports whether both records prove the same exact level.
func SameDenotedLevel(aHeld, aDelta float64, a LevelDenotation, bHeld, bDelta float64, b LevelDenotation) bool {
	return denotation.SameDenotedLevel(aHeld, aDelta, a, bHeld, bDelta, b)
}

// DenotedShiftedLevel lifts B's denoted level onto A's axis through G5's exact shift.
func DenotedShiftedLevel(pa, pb Operand, held, delta float64, d LevelDenotation) *big.Rat {
	level := DenotedLevel(held, delta, d)
	if level == nil {
		return nil
	}
	return level.Add(level, ZShift(pa, pb))
}
