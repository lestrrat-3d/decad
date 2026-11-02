package denotation

import (
	"context"
	"math"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
)

// LevelDenotationSnapshot binds the original exact pointer and a copy of
// its value to the held-level and displacement bits at producer Build time.
type LevelDenotationSnapshot struct {
	source, value *big.Rat
	held, delta   float64
}

// LevelDenotationSnapshotCost bounds the rational copy allocated by Snapshot.
func LevelDenotationSnapshotCost(d LevelDenotation) (uint64, bool) {
	if d.exact == nil {
		return 0, true
	}
	n, z := d.exact.Num().BitLen(), d.exact.Denom().BitLen()
	if n > 4096 || z > 4096 {
		return 0, false
	}
	return uint64(unsafe.Sizeof(big.Rat{})) + 4*uint64((n+7)/8+(z+7)/8) + 256, true
}

// SnapshotLevelDenotation captures source identity and exact contents.
// Its caller reserves LevelDenotationSnapshotCost before calling it.
func SnapshotLevelDenotation(d LevelDenotation) LevelDenotationSnapshot {
	s := LevelDenotationSnapshot{source: d.exact, held: d.held, delta: d.delta}
	if d.exact != nil {
		s.value = new(big.Rat).Set(d.exact)
	}
	return s
}

// ReplayMatches refuses a replaced pointer, changed rational, or changed
// held-level bits under the caller's shared replay meter.
func (s LevelDenotationSnapshot) ReplayMatches(ctx context.Context, d LevelDenotation,
	budget proof.ReplayKeyBudget) (bool, error) {
	if budget == nil {
		return false, nil
	}
	if err := budget.SpendWork(ctx, 8); err != nil {
		return false, err
	}
	if d.exact != s.source || math.Float64bits(d.held) != math.Float64bits(s.held) ||
		math.Float64bits(d.delta) != math.Float64bits(s.delta) || (d.exact == nil) != (s.value == nil) {
		return false, nil
	}
	if d.exact == nil {
		return true, nil
	}
	if _, ok := LevelDenotationSnapshotCost(d); !ok {
		return false, nil
	}
	cmp, err := proof.ReplayRatCompare(ctx, d.exact, s.value, budget)
	return err == nil && cmp == 0, err
}
