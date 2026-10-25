package opdiagnostic

import (
	"cmp"
	"slices"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// UnionSummaries collects accepted pair-step warnings in a left-to-right Union.
// Storage grows with public operands and warning mechanisms, never mesh pieces.
// The zero value is ready to use; discard it if any later pair fails.
type UnionSummaries struct {
	records map[unionLocation]Record
}

type unionLocation struct {
	operand int
	code    Code
}

// AddPair maps the pair's second input to its original zero-based operand index.
// After the first pair, its first input is a transient result: a warning about
// that input has no single original argument and is recorded at the whole result.
func (s *UnionSummaries) AddPair(records []Record, nextOperand int) {
	for _, record := range records {
		switch record.Operand {
		case 0:
			if nextOperand > 1 {
				record.Operand = -1
			}
		case 1:
			record.Operand = nextOperand
		}
		if s.records == nil {
			s.records = make(map[unionLocation]Record)
		}
		key := unionLocation{operand: record.Operand, code: record.Code}
		old, exists := s.records[key]
		if !exists {
			s.records[key] = cloneRecord(record)
			continue
		}
		if record.Code == CodeChainAllowanceApplied {
			// Preserve an actual gate's complete tuple, choosing its largest
			// touched bound. A component-wise merge could invent a gate.
			if record.TouchedBound > old.TouchedBound {
				s.records[key] = cloneRecord(record)
			}
			continue
		}
		// Repeated changes at one location can affect successive images of
		// the same point. Summing their upper bounds covers that composition.
		old.DisplacementBound = proofbound.AbsSumUpper(old.DisplacementBound, record.DisplacementBound)
		old.AllowedReach = proofbound.AbsSumUpper(old.AllowedReach, record.AllowedReach)
		s.records[key] = old
	}
}

// Records returns independent summaries in public argument, stage, then stable
// code-token order. Whole-result warnings follow warnings at named arguments.
func (s *UnionSummaries) Records() []Record {
	if len(s.records) == 0 {
		return nil
	}
	out := make([]Record, 0, len(s.records))
	for _, record := range s.records {
		out = append(out, cloneRecord(record))
	}
	slices.SortFunc(out, func(a, b Record) int {
		if a.Operand != b.Operand {
			if a.Operand < 0 {
				return 1
			}
			if b.Operand < 0 {
				return -1
			}
			return cmp.Compare(a.Operand, b.Operand)
		}
		if a.Code == CodeChainAllowanceApplied && b.Code != CodeChainAllowanceApplied {
			return -1
		}
		if b.Code == CodeChainAllowanceApplied && a.Code != CodeChainAllowanceApplied {
			return 1
		}
		return cmp.Compare(warningToken(a.Code), warningToken(b.Code))
	})
	return out
}

func cloneRecord(record Record) Record {
	if record.ChainAllowance != nil {
		allowance := *record.ChainAllowance
		record.ChainAllowance = &allowance
	}
	return record
}

func warningToken(code Code) string {
	switch code {
	case CodeGapShiftApplied:
		return "gap_shift_applied"
	case CodeChainAllowanceApplied:
		return "chain_allowance_applied"
	case CodeStitchMergeApplied:
		return "stitch_merge_applied"
	case CodeEmbeddingReachApplied:
		return "embedding_reach_applied"
	default:
		return "unknown"
	}
}
