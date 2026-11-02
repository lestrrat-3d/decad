package denotation

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// LevelDenotation is the exact rational a held sweep level denotes, recorded
// beside the held float and the displacement bound it was proven against
// (agent-docs/stacked-prism-design.md §2.1). The zero value records nothing.
//
// The held float and bound are part of the record so that a copy which
// rewrites the level without restating its denotation cannot carry a stale
// rational: DenotedLevel answers only for the exact held float and bound the
// denotation was recorded with.
type LevelDenotation struct {
	exact       *big.Rat
	held, delta float64
}

// DenoteLevel records exact as the level that held denotes within delta. It
// returns the zero value when exact is nil, or when held lies farther than
// delta from exact, which disproves the producer's claim.
func DenoteLevel(exact *big.Rat, held, delta float64) LevelDenotation {
	if exact == nil || !(proofarith.RationalFloatError(exact, held) <= delta) {
		return LevelDenotation{}
	}
	return LevelDenotation{exact: new(big.Rat).Set(exact), held: held, delta: delta}
}

// DenotedLevel returns the exact level a held level denotes, or nil when no
// record proves it. A level with zero displacement denotes its own held
// float. A displaced level needs a denotation recorded for that same held
// float and bound. The result is a fresh value the caller may modify.
func DenotedLevel(held, delta float64, d LevelDenotation) *big.Rat {
	if d.exact != nil && d.held == held && d.delta == delta {
		return new(big.Rat).Set(d.exact)
	}
	if delta == 0 {
		return proofarith.FloatRat(held)
	}
	return nil
}

// SameDenotedLevel reports whether both records prove the same exact level.
func SameDenotedLevel(aHeld, aDelta float64, aDenoted LevelDenotation, bHeld, bDelta float64, bDenoted LevelDenotation) bool {
	a := DenotedLevel(aHeld, aDelta, aDenoted)
	if a == nil {
		return false
	}
	b := DenotedLevel(bHeld, bDelta, bDenoted)
	if b == nil {
		return false
	}
	return a.Cmp(b) == 0
}
