package loftmesh

import (
	"fmt"
	"runtime"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// A batch holds complete triangle rows. One row has at most F-1 partners,
// which the triangle ceiling already bounds; batching never holds the whole
// candidate list or one result byte per candidate in the complete audit.
const loftAuditCandidateBatchSize = 1_000_000

// preflightLoftRows runs before exact preparation or any S7 classification.
// The count from the selected S8 scanner remains authoritative. This second
// bounded scan proves the row-ordered producer fits the same ceiling and
// reports the same candidate count for every triangle.
func preflightLoftRows(budget *proofbound.WorkBudget, tree treeScan, members, starts []int,
	ceiling uint64) error {
	var scanned uint64
	for _, i := range members {
		rowCount := 0
		work, exceeded, err := tree.visitRow(budget, i, ceiling-scanned, func(int) { rowCount++ })
		if err != nil {
			return err
		}
		if exceeded {
			return errLoftAuditCeiling()
		}
		scanned += work
		if rowCount != starts[i+1]-starts[i] {
			return fmt.Errorf("%w: row-ordered loft candidates differ from S8 count", decaderr.ErrUnsupported)
		}
	}
	return budget.Err()
}

// streamLoftPairs repeats the proven row query once, retaining at most one
// sorted row and one bounded batch. Complete rows enter the existing parallel
// classifier in ascending i order, and each row's j partners are sorted.
// Its replay therefore returns the same first failure and work counts as a
// complete lexicographic candidate list.
func streamLoftPairs(budget *proofbound.WorkBudget, tree treeScan, members, starts []int,
	count, ceiling uint64, batchSize int, tris [][3]int, boxes [][2]r3.Vec, data *LoftAuditData,
	shortcuts LoftAuditShortcuts, work *LoftAuditWork, testPair func(i, j int) error) (int, int, error) {
	batch := loftCandidates{
		starts: make([]int, 1, min(len(members)+1, batchSize+1)),
		js:     make([]int32, 0, min(int(count), batchSize)),
	}
	var row []int32
	var scanned uint64
	var emitted uint64
	flush := func() (int, int, error) {
		if len(batch.js) == 0 {
			return -1, -1, nil
		}
		if len(batch.js) >= 100_000 && runtime.GOMAXPROCS(0) > 1 {
			return parallelLoftPairs(budget, tris, boxes, data, shortcuts, batch, work)
		}
		failI, failJ := -1, -1
		err := batch.each(func(i, j int) error {
			err := testPair(i, j)
			if err != nil {
				failI, failJ = i, j
			}
			return err
		})
		return failI, failJ, err
	}
	for _, i := range members {
		row = row[:0]
		rowWork, exceeded, err := tree.visitRow(budget, i, ceiling-scanned,
			func(j int) { row = append(row, int32(j)) })
		if err != nil {
			return -1, -1, err
		}
		if exceeded {
			return -1, -1, errLoftAuditCeiling()
		}
		scanned += rowWork
		if len(row) != starts[i+1]-starts[i] {
			return -1, -1, fmt.Errorf("%w: row-ordered loft candidates differ from S8 count", decaderr.ErrUnsupported)
		}
		slices.Sort(row)
		if len(batch.js) != 0 && len(batch.js)+len(row) > batchSize {
			if failI, failJ, err := flush(); err != nil {
				return failI, failJ, err
			}
			batch.starts = batch.starts[:1]
			batch.js = batch.js[:0]
			batch.baseRow = i
		}
		if len(batch.starts) == 1 {
			batch.baseRow = i
		}
		batch.js = append(batch.js, row...)
		batch.starts = append(batch.starts, len(batch.js))
		emitted += uint64(len(row))
	}
	if emitted != count {
		return -1, -1, fmt.Errorf("%w: row-ordered loft candidates differ from S8 count", decaderr.ErrUnsupported)
	}
	return flush()
}
