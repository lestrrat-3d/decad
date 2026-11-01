package proofbound

import (
	"context"
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

type replayTrigCacheEntry struct {
	key      [2]uint64
	half     bool
	values   [4]replayTrigInteger
	negative [4]bool
}

// ReplayTrigCache owns fixed-capacity half-turn series readings for one call.
type ReplayTrigCache struct {
	entries []replayTrigCacheEntry
	budget  *ReplayBudget
}

func NewReplayTrigCache(ctx context.Context, capacity int, b *ReplayBudget) (*ReplayTrigCache, error) {
	if b == nil || capacity < 0 || capacity > 65536 {
		return nil, fmt.Errorf("%w: bounded trig cache capacity", decaderr.ErrUnsupported)
	}
	if err := b.ReserveRecords(ctx, uint64(capacity)+1); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(capacity), uint64(unsafe.Sizeof(replayTrigCacheEntry{}))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(ReplayTrigCache{}))); err != nil {
		return nil, err
	}
	return &ReplayTrigCache{entries: make([]replayTrigCacheEntry, 0, capacity), budget: b}, nil
}
func (a *replayTrigArena) gridSeriesCached(grid replayTrigSigned, cache *ReplayTrigCache) (replayTrigSpan, replayTrigSpan, error) {
	if cache == nil || cache.budget != a.budget || grid.negative {
		return a.gridSeries(grid)
	}
	if err := a.charge(1); err != nil {
		return replayTrigSpan{}, replayTrigSpan{}, err
	}
	key := [2]uint64{a.slots[6][0], a.slots[6][1] & ((uint64(1) << 31) - 1)}
	half := a.slots[6][1]&(uint64(1)<<31) != 0
	for i := range cache.entries {
		if err := a.charge(1); err != nil {
			return replayTrigSpan{}, replayTrigSpan{}, err
		}
		entry := &cache.entries[i]
		if entry.key != key {
			continue
		}
		if err := a.charge(4 * replayTrigWords); err != nil {
			return replayTrigSpan{}, replayTrigSpan{}, err
		}
		if half == entry.half {
			for j := range 4 {
				a.slots[10+j] = entry.values[j]
			}
			return replayTrigSpan{replayTrigSigned{10, entry.negative[0]}, replayTrigSigned{11, entry.negative[1]}}, replayTrigSpan{replayTrigSigned{12, entry.negative[2]}, replayTrigSigned{13, entry.negative[3]}}, nil
		}
		a.slots[10], a.slots[11], a.slots[12], a.slots[13] = entry.values[1], entry.values[0], entry.values[3], entry.values[2]
		return replayTrigSpan{replayTrigSigned{10, !entry.negative[1]}, replayTrigSigned{11, !entry.negative[0]}}, replayTrigSpan{replayTrigSigned{12, !entry.negative[3]}, replayTrigSigned{13, !entry.negative[2]}}, nil
	}
	s, c, err := a.gridSeries(grid)
	if err != nil {
		return s, c, err
	}
	if len(cache.entries) == cap(cache.entries) {
		return s, c, nil
	}
	if err = a.charge(4 * replayTrigWords); err != nil {
		return s, c, err
	}
	e := replayTrigCacheEntry{key: key, half: half, negative: [4]bool{s.lo.negative, s.hi.negative, c.lo.negative, c.hi.negative}, values: [4]replayTrigInteger{a.slots[s.lo.slot], a.slots[s.hi.slot], a.slots[c.lo.slot], a.slots[c.hi.slot]}}
	cache.entries = append(cache.entries, e)
	return s, c, nil
}
