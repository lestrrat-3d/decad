package topobuild

import (
	"context"
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

type replayRole struct {
	role string
	face *topo.Face
}

// ReplayRoles keeps ordered source roles without allocating a runtime map.
type ReplayRoles struct {
	entries []replayRole
	budget  *proofbound.ReplayBudget
}

// NewReplayRoles records all source roles in traversal order.
func NewReplayRoles(ctx context.Context, b *topo.Body, budget *proofbound.ReplayBudget) (*ReplayRoles, error) {
	if b == nil || budget == nil {
		return nil, fmt.Errorf("%w: missing replay roles", decaderr.ErrUnsupported)
	}
	var count uint64
	for _, l := range b.Lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		if l == nil {
			return nil, fmt.Errorf("%w: nil source lump", decaderr.ErrUnsupported)
		}
		for _, s := range l.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			if s == nil {
				return nil, fmt.Errorf("%w: nil source shell", decaderr.ErrUnsupported)
			}
			for _, f := range s.Faces {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return nil, err
				}
				if f == nil {
					return nil, fmt.Errorf("%w: nil source face", decaderr.ErrUnsupported)
				}
				if uint64(len(f.Origins)) > ^uint64(0)-count {
					return nil, fmt.Errorf("%w: source role count overflow", decaderr.ErrUnsupported)
				}
				count += uint64(len(f.Origins))
			}
		}
	}
	if count > uint64(int(^uint(0)>>1)) {
		return nil, fmt.Errorf("%w: source role count overflow", decaderr.ErrUnsupported)
	}
	if err := budget.ReserveRecords(ctx, count+1); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, count, uint64(unsafe.Sizeof(replayRole{}))); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(ReplayRoles{}))); err != nil {
		return nil, err
	}
	entries := make([]replayRole, 0, int(count))
	for _, l := range b.Lumps {
		for _, s := range l.Shells {
			for _, f := range s.Faces {
				for _, o := range f.Origins {
					if err := budget.SpendWork(ctx, 1); err != nil {
						return nil, err
					}
					if len(o.Role) > 128 {
						return nil, fmt.Errorf("%w: source role too long", decaderr.ErrUnsupported)
					}
					entries = append(entries, replayRole{role: o.Role, face: f})
				}
			}
		}
	}
	return &ReplayRoles{entries: entries, budget: budget}, nil
}

// Lookup returns the last matching origin, preserving ordinary role semantics.
func (r *ReplayRoles) Lookup(ctx context.Context, role string) (*topo.Face, bool, error) {
	if r == nil || r.budget == nil || len(role) > 128 {
		return nil, false, fmt.Errorf("%w: invalid replay role lookup", decaderr.ErrUnsupported)
	}
	for i := len(r.entries) - 1; i >= 0; i-- { //nolint:modernize // Keep the reverse scan and its budgeted visit order explicit.
		if err := r.budget.SpendWork(ctx, uint64(len(role))+1); err != nil {
			return nil, false, err
		}
		if r.entries[i].role == role {
			return r.entries[i].face, true, nil
		}
	}
	return nil, false, nil
}
