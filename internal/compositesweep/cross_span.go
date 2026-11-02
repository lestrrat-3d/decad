package compositesweep

import (
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/tessellation"
)

// CrossSpanContactAudit checks only pairs whose triangles came from
// different spans. Each reduced span already audited its own facet pairs at
// VerifyBoundary; the shared grid introduces only these new pairs.
//
// It audits the pairs (i, j), i < j, of different spans that the box-gap
// skip below keeps, in ascending (i, j) order, and returns the first error
// that order meets. revolvemesh.ContactCandidates lists those pairs without
// reading every one: per facet it returns, ascending and without duplicates,
// every later facet the box-gap test does not separate and every later facet
// sharing a vertex with it, and a pair it leaves out is one that
// BoxGapExceeds separates and that shares no vertex, so the skip below would
// pass it over. The walk over its rows therefore meets the same pairs in the
// same order as a walk over all pairs, and audits each with the same call.
//
// The fixed proofbound.MaxFacetPairTestsPerCall ceiling counts the work this
// audit does: every candidate-sweep visit, and one test for every listed
// cross-span pair, all reserved before the first exact pair predicate runs. A
// margin the sweep cannot order by (negative or NaN) walks every pair, and
// that walk is refused unless the full pair count fits the ceiling.
func CrossSpanContactAudit(budget *proofbound.WorkBudget, data []revolvemesh.RevolveAuditTri,
	triangles [][3]int, spanOf []int, delta float64) error {
	if len(data) != len(triangles) || len(spanOf) != len(triangles) {
		return fmt.Errorf(`%w: the composite sweep's facet audit has inconsistent triangle records`, decaderr.ErrUnsupported)
	}
	margin := proofbound.ProductUpper(2, delta)
	if !(margin >= 0) {
		pairs, ok := proofbound.WallChoose2(uint64(len(triangles)))
		if !ok || pairs > proofbound.MaxFacetPairTestsPerCall {
			if ok {
				return crossSpanWorkLimit(pairs)
			}
			return errCrossSpanWorkLimit
		}
		return crossSpanAllPairs(budget, data, triangles, spanOf, delta, margin)
	}
	rows, err := crossSpanCandidates(budget, data, triangles, spanOf, margin)
	if err != nil {
		return err
	}
	for i, row := range rows {
		for _, listed := range row {
			if err := crossSpanPair(budget, data, triangles, spanOf, i, int(listed), delta, margin); err != nil {
				return err
			}
		}
	}
	return budget.Err()
}

var errCrossSpanWorkLimit = fmt.Errorf(`%w: the composite sweep's facet-pair audit exceeds its fixed work ceiling`,
	decaderr.ErrUnsupported)

func crossSpanWorkLimit(observed uint64) error {
	return proofbound.WithWorkLimit(errCrossSpanWorkLimit, "facet_pairs", observed, proofbound.MaxFacetPairTestsPerCall)
}

// crossSpanCandidates lists revolvemesh.ContactCandidates' rows with every
// same-span pair removed, since crossSpanPair skips those without a test. The
// candidate sweep runs under a meter that refuses once its visits reach the
// ceiling, and the kept cross-span pairs must fit what the sweep left of it.
func crossSpanCandidates(budget *proofbound.WorkBudget, data []revolvemesh.RevolveAuditTri,
	triangles [][3]int, spanOf []int, margin float64) ([][]int32, error) {
	ceiling := uint64(proofbound.MaxFacetPairTestsPerCall)
	var scanned uint64
	scan := &proofbound.WorkBudget{
		StepFn: func() error {
			if scanned >= ceiling {
				return proofbound.ErrWallWorkBudget
			}
			if err := budget.Step(); err != nil {
				return err
			}
			scanned++
			return nil
		},
		ErrFn: budget.Err,
	}
	rows, err := revolvemesh.ContactCandidates(scan, data, triangles, margin)
	if errors.Is(err, proofbound.ErrWallWorkBudget) {
		return nil, crossSpanWorkLimit(ceiling + 1)
	}
	if err != nil {
		return nil, err
	}
	var kept uint64
	for i, row := range rows {
		out := row[:0]
		for _, j := range row {
			if spanOf[i] != spanOf[j] {
				out = append(out, j)
			}
		}
		rows[i] = out
		kept += uint64(len(out))
	}
	if kept > ceiling-scanned {
		return nil, crossSpanWorkLimit(scanned + kept)
	}
	return rows, nil
}

// crossSpanAllPairs walks every pair (i, j), i < j, in ascending order.
func crossSpanAllPairs(budget *proofbound.WorkBudget, data []revolvemesh.RevolveAuditTri,
	triangles [][3]int, spanOf []int, delta, margin float64) error {
	for i := range triangles {
		for j := i + 1; j < len(triangles); j++ {
			if err := crossSpanPair(budget, data, triangles, spanOf, i, j, delta, margin); err != nil {
				return err
			}
		}
	}
	return budget.Err()
}

// crossSpanPair audits one pair of different spans that the box-gap test does
// not separate, or that shares a vertex.
func crossSpanPair(budget *proofbound.WorkBudget, data []revolvemesh.RevolveAuditTri,
	triangles [][3]int, spanOf []int, i, j int, delta, margin float64) error {
	if spanOf[i] == spanOf[j] {
		return nil
	}
	if err := budget.Step(); err != nil {
		return err
	}
	shared, count := tessellation.SharedVertexIndices(triangles[i], triangles[j])
	if count == 0 && revolvemesh.BoxGapExceeds(data[i].Box, data[j].Box, margin) {
		return nil
	}
	return revolvemesh.AuditRevolvePair(data, triangles, shared, count, i, j, delta)
}
