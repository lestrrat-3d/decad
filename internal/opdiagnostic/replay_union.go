package opdiagnostic

import (
	"cmp"
	"context"
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type ReplayUnionSummaries struct {
	records []Record
	limit   int
	budget  *proofbound.ReplayBudget
}

func NewReplayUnionSummaries(ctx context.Context, b *proofbound.ReplayBudget, limit int) (*ReplayUnionSummaries, error) {
	if b == nil || limit < 0 {
		return nil, fmt.Errorf("%w: invalid warning capacity", decaderr.ErrUnsupported)
	}
	if err := b.ReserveRecords(ctx, uint64(limit)); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(limit), uint64(unsafe.Sizeof(Record{}))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(ReplayUnionSummaries{}))); err != nil {
		return nil, err
	}
	return &ReplayUnionSummaries{records: make([]Record, 0, limit), limit: limit, budget: b}, nil
}
func (s *ReplayUnionSummaries) clone(ctx context.Context, r Record) (Record, error) {
	if r.ChainAllowance != nil {
		if err := s.budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(float64(0)))); err != nil {
			return Record{}, err
		}
		v := *r.ChainAllowance
		r.ChainAllowance = &v
	}
	return r, nil
}
func (s *ReplayUnionSummaries) AddPair(ctx context.Context, records []Record, next int) error {
	for _, r := range records {
		if err := s.budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		if r.Operand == 0 && next > 1 {
			r.Operand = -1
		} else if r.Operand == 1 {
			r.Operand = next
		}
		idx := -1
		for i, old := range s.records {
			if err := s.budget.SpendWork(ctx, 1); err != nil {
				return err
			}
			if old.Operand == r.Operand && old.Code == r.Code {
				idx = i
				break
			}
		}
		if idx < 0 {
			if len(s.records) >= s.limit {
				return fmt.Errorf("%w: warning capacity", decaderr.ErrUnsupported)
			}
			v, err := s.clone(ctx, r)
			if err != nil {
				return err
			}
			s.records = append(s.records, v)
			continue
		}
		old := s.records[idx]
		if r.Code == CodeChainAllowanceApplied {
			if r.TouchedBound > old.TouchedBound {
				v, err := s.clone(ctx, r)
				if err != nil {
					return err
				}
				s.records[idx] = v
			}
			continue
		}
		old.DisplacementBound = proofbound.AbsSumUpper(old.DisplacementBound, r.DisplacementBound)
		old.AllowedReach = proofbound.AbsSumUpper(old.AllowedReach, r.AllowedReach)
		s.records[idx] = old
	}
	return nil
}
func (s *ReplayUnionSummaries) Records(ctx context.Context) ([]Record, error) {
	if len(s.records) == 0 {
		return nil, ctx.Err()
	}
	if err := s.budget.ReserveArray(ctx, uint64(len(s.records)), uint64(unsafe.Sizeof(Record{}))); err != nil {
		return nil, err
	}
	out := make([]Record, len(s.records))
	for i, r := range s.records {
		v, err := s.clone(ctx, r)
		if err != nil {
			return nil, err
		}
		out[i] = v
	}
	less := func(a, b Record) bool {
		if a.Operand != b.Operand {
			if a.Operand < 0 {
				return false
			}
			if b.Operand < 0 {
				return true
			}
			return a.Operand < b.Operand
		}
		if a.Code == CodeChainAllowanceApplied && b.Code != CodeChainAllowanceApplied {
			return true
		}
		if b.Code == CodeChainAllowanceApplied && a.Code != CodeChainAllowanceApplied {
			return false
		}
		return cmp.Compare(warningToken(a.Code), warningToken(b.Code)) < 0
	}
	for i := 1; i < len(out); i++ {
		v := out[i]
		j := i
		for j > 0 {
			if err := s.budget.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			if !less(v, out[j-1]) {
				break
			}
			out[j] = out[j-1]
			j--
		}
		out[j] = v
	}
	return out, nil
}
