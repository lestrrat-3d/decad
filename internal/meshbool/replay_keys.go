package meshbool

import (
	"context"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type replayKeyEntry struct {
	pointers [4]*big.Int
	key      string
	used     bool
}
type ReplayKeys struct {
	budget *proofbound.ReplayBudget
	table  []replayKeyEntry
	count  int
}

// NewReplayKeys creates a call-scoped exact key memo.
func NewReplayKeys(b *proofbound.ReplayBudget) *ReplayKeys { return &ReplayKeys{budget: b} }

// NewReplayKeysBounded reserves the memo before its allocation.
func NewReplayKeysBounded(ctx context.Context, b *proofbound.ReplayBudget) (*ReplayKeys, error) {
	if b == nil {
		return nil, errReplayMissingReplayKeyBudget
	}
	if err := b.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(ReplayKeys{}))); err != nil {
		return nil, err
	}
	return NewReplayKeys(b), nil
}

func replayKeyHash(p [4]*big.Int) uint64 {
	h := uint64(1469598103934665603)
	for _, x := range p {
		if x == nil {
			h *= 1099511628211
			continue
		}
		words := x.Bits()
		h ^= uint64(x.BitLen()) ^ uint64(x.Sign()+1)
		h *= 1099511628211
		if len(words) > 0 {
			h ^= uint64(words[0])
			h *= 1099511628211
			h ^= uint64(words[len(words)-1])
			h *= 1099511628211
		}
	}
	return h
}

func (k *ReplayKeys) Key(ctx context.Context, p proof.Xpt) (string, error) {
	if k == nil || k.budget == nil {
		return "", errReplayExactKeyNeedsReplayBudget
	}
	raw := [4]*big.Int{p.X, p.Y, p.Z, p.W}
	if err := k.budget.SpendWork(ctx, 16); err != nil {
		return "", err
	}
	if len(k.table) > 0 {
		index := int(replayKeyHash(raw) & uint64(len(k.table)-1))
		for k.table[index].used {
			if err := k.budget.SpendWork(ctx, 1); err != nil {
				return "", err
			}
			if k.table[index].pointers == raw {
				return k.table[index].key, nil
			}
			index = (index + 1) & (len(k.table) - 1)
		}
	}
	if err := k.budget.ReserveRecords(ctx, 1); err != nil {
		return "", err
	}
	if len(k.table) == 0 || k.count >= len(k.table)/2 {
		next := max(16, len(k.table)*2)
		if next < len(k.table) {
			return "", errReplayKeyMemoOverflow
		}
		if err := k.budget.ReserveArray(ctx, uint64(next), uint64(unsafe.Sizeof(replayKeyEntry{}))); err != nil {
			return "", err
		}
		if err := k.budget.ReserveRecords(ctx, uint64(next)); err != nil {
			return "", err
		}
		if uint64(len(k.table)) > (^uint64(0)-uint64(next))/16 {
			return "", errReplayKeyRehashWorkOverflow
		}
		if err := k.budget.SpendWork(ctx, uint64(next)+16*uint64(len(k.table))); err != nil {
			return "", err
		}
		table := make([]replayKeyEntry, next)
		for _, entry := range k.table {
			if entry.used {
				i := int(replayKeyHash(entry.pointers) & uint64(next-1))
				for table[i].used {
					if err := k.budget.SpendWork(ctx, 1); err != nil {
						return "", err
					}
					i = (i + 1) & (next - 1)
				}
				table[i] = entry
			}
		}
		k.table = table
	}
	key, err := proof.ReplayExactKey(ctx, p, k.budget)
	if err != nil {
		return "", err
	}
	if err := k.budget.SpendWork(ctx, 16); err != nil {
		return "", err
	}
	index := int(replayKeyHash(raw) & uint64(len(k.table)-1))
	for k.table[index].used {
		if err := k.budget.SpendWork(ctx, 1); err != nil {
			return "", err
		}
		index = (index + 1) & (len(k.table) - 1)
	}
	k.table[index] = replayKeyEntry{pointers: raw, key: key, used: true}
	k.count++
	return key, nil
}
