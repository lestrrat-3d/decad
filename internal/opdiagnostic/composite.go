package opdiagnostic

import (
	"cmp"
	"slices"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// CompositeSummaries bounds successful internal-step warnings by mechanism.
// Composite steps use transient bodies, so their summaries name the outer
// public operation and omit operand locations that cannot be traced safely.
type CompositeSummaries struct {
	records map[Code]Record
}

// Add records warnings from one accepted internal step.
func (s *CompositeSummaries) Add(records []Record, publicOperation int) {
	if len(records) == 0 {
		return
	}
	for _, record := range records {
		if record.Code == CodeUnknown {
			continue
		}
		record.PublicOperation = publicOperation
		record.Operand = -1
		if s.records == nil {
			s.records = make(map[Code]Record)
		}
		old, exists := s.records[record.Code]
		if !exists {
			s.records[record.Code] = cloneRecord(record)
			continue
		}
		if record.Code == CodeChainAllowanceApplied {
			if record.TouchedBound > old.TouchedBound {
				s.records[record.Code] = cloneRecord(record)
			}
			continue
		}
		old.DisplacementBound = proofbound.AbsSumUpper(old.DisplacementBound, record.DisplacementBound)
		old.AllowedReach = proofbound.AbsSumUpper(old.AllowedReach, record.AllowedReach)
		old.PublicOperation = publicOperation
		s.records[record.Code] = old
	}
}

// Records returns bounded summaries ordered by stable warning token.
func (s *CompositeSummaries) Records() []Record {
	if len(s.records) == 0 {
		return nil
	}
	out := make([]Record, 0, len(s.records))
	for _, record := range s.records {
		out = append(out, cloneRecord(record))
	}
	slices.SortFunc(out, func(a, b Record) int {
		return cmp.Compare(warningToken(a.Code), warningToken(b.Code))
	})
	return out
}
