package prismplacement

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
)

// ReplayLevelIdentity owns a copy of one level's rational value and binds its
// original pointer. Its fields remain private to the placement package.
type ReplayLevelIdentity struct {
	ptr, copy   *big.Rat
	held, delta uint64
}

// CaptureReplayLevelIdentity copies a bounded level before it is published.
func CaptureReplayLevelIdentity(ctx context.Context, d LevelDenotation, b proof.ReplayKeyBudget) (ReplayLevelIdentity, bool, error) {
	var out ReplayLevelIdentity
	if b == nil {
		return out, false, nil
	}
	if err := b.SpendWork(ctx, 32); err != nil {
		return out, false, err
	}
	if math.IsNaN(d.held) || math.IsInf(d.held, 0) || math.IsNaN(d.delta) || math.IsInf(d.delta, 0) {
		return out, false, nil
	}
	out.ptr = d.exact
	out.held = math.Float64bits(d.held)
	out.delta = math.Float64bits(d.delta)
	if d.exact == nil {
		return out, true, nil
	}
	n, q := d.exact.Num().BitLen(), d.exact.Denom().BitLen()
	if n > 4096 || q > 4096 || d.exact.Denom().Sign() <= 0 {
		return ReplayLevelIdentity{}, false, nil
	}
	words := uint64((n+63)/64 + (q+63)/64 + 2)
	if err := b.SpendWork(ctx, 16*words); err != nil {
		return ReplayLevelIdentity{}, false, err
	}
	if err := b.ReserveRecords(ctx, 3); err != nil {
		return ReplayLevelIdentity{}, false, err
	}
	if err := b.ReserveBytes(ctx, 32*words+512); err != nil {
		return ReplayLevelIdentity{}, false, err
	}
	out.copy = new(big.Rat).Set(d.exact)
	return out, true, nil
}

// Matches rechecks the original pointer, value and held float bits.
func (s ReplayLevelIdentity) Matches(ctx context.Context, d LevelDenotation, b proof.ReplayKeyBudget) (bool, error) {
	if b == nil {
		return false, nil
	}
	if err := b.SpendWork(ctx, 32); err != nil {
		return false, err
	}
	if s.ptr != d.exact || s.held != math.Float64bits(d.held) || s.delta != math.Float64bits(d.delta) {
		return false, nil
	}
	if s.ptr == nil {
		return s.copy == nil, nil
	}
	if s.copy == nil || s.ptr.Num().BitLen() > 4096 || s.ptr.Denom().BitLen() > 4096 || s.ptr.Denom().Sign() <= 0 {
		return false, nil
	}
	words := uint64((s.ptr.Num().BitLen()+63)/64 + (s.ptr.Denom().BitLen()+63)/64 + 2)
	if err := b.SpendWork(ctx, 16*words); err != nil {
		return false, err
	}
	return s.ptr.Num().Cmp(s.copy.Num()) == 0 && s.ptr.Denom().Cmp(s.copy.Denom()) == 0, nil
}
