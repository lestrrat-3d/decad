package boolshift

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type replayShiftEntry struct {
	piece int
	shift PieceShift
}

// ReplayShifts stores shift proposals in ascending piece order under one meter.
type ReplayShifts struct {
	entries []replayShiftEntry
	limit   int
	budget  *proofbound.ReplayBudget
}

func NewReplayShifts(ctx context.Context, b *proofbound.ReplayBudget) (*ReplayShifts, error) {
	const limit = 1
	if b == nil {
		return nil, ErrInvalidReplayShifts
	}
	if err := b.ReserveRecords(ctx, uint64(limit)); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(limit), uint64(unsafe.Sizeof(replayShiftEntry{}))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(ReplayShifts{}))); err != nil {
		return nil, err
	}
	return &ReplayShifts{entries: make([]replayShiftEntry, 0, limit), limit: limit, budget: b}, nil
}
func (s *ReplayShifts) Get(ctx context.Context, piece int) (PieceShift, bool, error) {
	if err := ctx.Err(); err != nil {
		return PieceShift{}, false, err
	}
	for _, e := range s.entries {
		if err := s.budget.SpendWork(ctx, 1); err != nil {
			return PieceShift{}, false, err
		}
		if e.piece == piece {
			return e.shift, true, nil
		}
		if e.piece > piece {
			break
		}
	}
	return PieceShift{}, false, nil
}
func (s *ReplayShifts) Set(ctx context.Context, shift PieceShift) error {
	const piece = 0
	i := 0
	for i < len(s.entries) && s.entries[i].piece < piece {
		if err := s.budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		i++
	}
	if err := s.budget.SpendWork(ctx, 1); err != nil {
		return err
	}
	if i < len(s.entries) && s.entries[i].piece == piece {
		s.entries[i].shift = shift
		return nil
	}
	if len(s.entries) >= s.limit {
		return ErrReplayShiftCapacity
	}
	if err := s.budget.SpendWork(ctx, uint64(len(s.entries)-i)); err != nil {
		return err
	}
	s.entries = append(s.entries, replayShiftEntry{})
	copy(s.entries[i+1:], s.entries[i:])
	s.entries[i] = replayShiftEntry{piece: piece, shift: shift}
	return nil
}

// Single reports the sole proposal and its binding without spending replay work.
func (s *ReplayShifts) Single(b *proofbound.ReplayBudget) (PieceShift, bool) {
	if s == nil || s.budget != b || len(s.entries) != 1 || s.entries[0].piece != 0 {
		return PieceShift{}, false
	}
	return s.entries[0].shift, true
}

// Len reports the number of proposals.
func (s *ReplayShifts) Len() int {
	if s == nil {
		return 0
	}
	return len(s.entries)
}
