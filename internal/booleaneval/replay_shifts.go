package booleaneval

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type replayShiftEntry struct {
	piece int
	shift pieceShift
}

// ReplayShifts stores shift proposals in ascending piece order under one meter.
type ReplayShifts struct {
	entries []replayShiftEntry
	limit   int
	budget  *proofbound.ReplayBudget
}

func newReplayShifts(ctx context.Context, b *proofbound.ReplayBudget) (*ReplayShifts, error) {
	const limit = 1
	if b == nil {
		return nil, errBrakeEvalInvalidReplayShifts
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
func (s *ReplayShifts) get(ctx context.Context, piece int) (pieceShift, bool, error) {
	if err := ctx.Err(); err != nil {
		return pieceShift{}, false, err
	}
	for _, e := range s.entries {
		if err := s.budget.SpendWork(ctx, 1); err != nil {
			return pieceShift{}, false, err
		}
		if e.piece == piece {
			return e.shift, true, nil
		}
		if e.piece > piece {
			break
		}
	}
	return pieceShift{}, false, nil
}
func (s *ReplayShifts) set(ctx context.Context, shift pieceShift) error {
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
		return errBrakeEvalReplayShiftCapacity
	}
	if err := s.budget.SpendWork(ctx, uint64(len(s.entries)-i)); err != nil {
		return err
	}
	s.entries = append(s.entries, replayShiftEntry{})
	copy(s.entries[i+1:], s.entries[i:])
	s.entries[i] = replayShiftEntry{piece: piece, shift: shift}
	return nil
}
