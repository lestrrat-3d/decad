package loftmesh

import (
	"errors"
	"fmt"
	"math/bits"
	"runtime"
	"slices"
	"sync"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// A batch holds complete triangle rows. One row has at most F-1 partners,
// which the triangle ceiling already bounds; batching never holds the whole
// candidate list or one result byte per candidate in the complete audit.
const loftAuditCandidateBatchSize = 1_000_000

type loftRowScan interface {
	visitRow(budget *proofbound.WorkBudget, i int, limit uint64,
		fn func(j int)) (uint64, bool, error)
	walkRow(i int, limit uint64, fn func(j int)) (uint64, bool)
	// walkRowAppend is walkRow with the partners appended to out, in the
	// order walkRow passes them to fn, and the same work count and exceed.
	walkRowAppend(i int, limit uint64, out []int32) ([]int32, uint64, bool)
}

const (
	// loftRowParallelMin is the member count from which row walks fan out.
	loftRowParallelMin = 4096
	// loftRowChunkRows bounds the rows one parallel chunk walks.
	loftRowChunkRows = 8192
)

// loftRowResult is one row's budget-free walk.
type loftRowResult struct {
	work     uint64
	exceeded bool
	count    int
	row      []int32 // sorted partners (stream only)
}

// loftRowChunkEnd returns the end of the chunk of consecutive members that
// starts at lo. A chunk ends after loftRowChunkRows rows or once the S8 counts
// of its rows reach loftAuditCandidateBatchSize, whichever comes first, and
// holds at least one row, so a stream never holds much more than one batch of
// partners in rows.
func loftRowChunkEnd(members, starts []int, lo int) int {
	var counts int
	hi := lo
	for hi < len(members) && hi-lo < loftRowChunkRows && (hi == lo || counts < loftAuditCandidateBatchSize) {
		i := members[hi]
		counts += starts[i+1] - starts[i]
		hi++
	}
	return hi
}

// walkLoftRows walks members[lo:hi] concurrently without touching budget
// except to poll it from the calling goroutine. Row k of the chunk walks with
// limit ceiling, the largest limit any row can get, and is sorted when keep
// is set; otherwise only its partners are counted.
func walkLoftRows(budget *proofbound.WorkBudget, tree loftRowScan, members []int, lo, hi int,
	ceiling uint64, keep bool, workers int, out []loftRowResult) error {
	if ts, ok := tree.(treeScan); ok && keep {
		words := (len(ts.boxes) + 63) / 64
		_, err := orderedwork.Each(hi-lo, workers, 64, budget.Err, func(k int) error {
			r := &out[k]
			*r = loftRowResult{row: r.row[:0]}
			r.row, r.work, r.exceeded = ts.walkRowAppend(members[lo+k], ceiling, r.row)
			r.count = len(r.row)
			r.row = sortRowByBitmap(r.row, words)
			return nil
		})
		return err
	}
	if ts, ok := tree.(treeScan); ok && !keep {
		_, err := orderedwork.Each(hi-lo, workers, 64, budget.Err, func(k int) error {
			r := &out[k]
			*r = loftRowResult{row: r.row[:0]}
			i := members[lo+k]
			r.count, r.work, r.exceeded = ts.flat.walkRowCount(ts.boxes[i], i, ceiling)
			return nil
		})
		return err
	}
	_, err := orderedwork.Each(hi-lo, workers, 64, budget.Err, func(k int) error {
		r := &out[k]
		*r = loftRowResult{row: r.row[:0]}
		if !keep {
			r.work, r.exceeded = tree.walkRow(members[lo+k], ceiling, func(int) { r.count++ })
			return nil
		}
		r.row, r.work, r.exceeded = tree.walkRowAppend(members[lo+k], ceiling, r.row)
		r.count = len(r.row)
		slices.Sort(r.row)
		return nil
	})
	return err
}

// rowBitmaps holds zeroed bitmaps for sortRowByBitmap.
var rowBitmaps sync.Pool

// sortRowByBitmap returns row sorted ascending. A row's partners are distinct
// member indices below words*64, so it sets one bit per partner in a pooled
// bitmap and reads the set bits back in order, clearing them. A repeated
// partner, or a row whose index range is far wider than the row is long, is
// sorted with slices.Sort instead, which gives the same order.
func sortRowByBitmap(row []int32, words int) []int32 {
	if len(row) < 2 {
		return row
	}
	lo, hi := row[0], row[0]
	for _, j := range row {
		lo, hi = min(lo, j), max(hi, j)
	}
	if int(hi>>6-lo>>6)+1 > 16*len(row) {
		slices.Sort(row)
		return row
	}
	var set *[]uint64
	if v, ok := rowBitmaps.Get().(*[]uint64); ok && len(*v) >= words {
		set = v
	} else {
		w := make([]uint64, words)
		set = &w
	}
	bitmap := *set
	for k, j := range row {
		bit := uint64(1) << (uint(j) & 63)
		if bitmap[j>>6]&bit != 0 {
			for _, c := range row[:k] {
				bitmap[c>>6] = 0
			}
			rowBitmaps.Put(set)
			slices.Sort(row)
			return row
		}
		bitmap[j>>6] |= bit
	}
	n := 0
	for w := lo >> 6; w <= hi>>6; w++ {
		x := bitmap[w]
		bitmap[w] = 0
		for x != 0 {
			row[n] = w<<6 | int32(bits.TrailingZeros64(x))
			n++
			x &= x - 1
		}
	}
	rowBitmaps.Put(set)
	return row
}

// replayLoftRow steps budget exactly as visitRow(budget, i, limit, ...) does
// for a row whose walk is r.
func replayLoftRow(budget *proofbound.WorkBudget, r *loftRowResult, limit uint64) (uint64, bool, error) {
	if r.exceeded || r.work > limit {
		return replayRowSteps(budget, limit+1, true)
	}
	return replayRowSteps(budget, r.work, false)
}

// preflightLoftRows runs before exact preparation or any S7 classification.
// The count from the selected S8 scanner remains authoritative. This second
// bounded scan proves the row-ordered producer fits the same ceiling and
// reports the same candidate count for every triangle.
// An optional observer sees serial pairs on the caller, or a joined chunk
// after its row counts and budget steps are replayed.
// It receives every candidate of a completed preflight without changing the
// charged steps. Workers retain bounded rows only when an observer needs them.
func preflightLoftRows(budget *proofbound.WorkBudget, tree loftRowScan, members, starts []int,
	ceiling uint64, observer ...loftRowObserver) error {
	return preflightLoftRowsWorkers(budget, tree, members, starts, ceiling,
		orderedwork.EachWorkers(len(members), loftRowParallelMin), observer...)
}

// preflightLoftRowsWorkers is preflightLoftRows with an explicit worker count;
// with workers < 2 the rows walk serially against the budget. Workers never
// charge the budget, so when a poll finds it exhausted the serial walk takes
// over from the first row not yet replayed. It fails at the step a serial run
// fails at, with that step's error, whenever the poll happens to fire.
func preflightLoftRowsWorkers(budget *proofbound.WorkBudget, tree loftRowScan, members, starts []int,
	ceiling uint64, workers int, observer ...loftRowObserver) error {
	var observe loftRowObserver
	if len(observer) != 0 {
		observe = observer[0]
	}
	var scanned uint64
	if workers >= 2 {
		out := make([]loftRowResult, min(loftRowChunkRows, len(members)))
		for lo := 0; lo < len(members); {
			hi := loftRowChunkEnd(members, starts, lo)
			if err := walkLoftRows(budget, tree, members, lo, hi, ceiling, observe != nil, workers, out); err != nil {
				if errors.Is(err, proofbound.ErrWallWorkBudget) {
					return preflightLoftRowsSerial(budget, tree, members[lo:], starts, ceiling, scanned, observer...)
				}
				return err
			}
			for k := range hi - lo {
				i := members[lo+k]
				work, exceeded, err := replayLoftRow(budget, &out[k], ceiling-scanned)
				if err != nil {
					return err
				}
				if exceeded {
					return errLoftAuditScanLimit(scanned, work, ceiling)
				}
				scanned += work
				if out[k].count != starts[i+1]-starts[i] {
					return fmt.Errorf("%w: row-ordered loft candidates differ from S8 count", decaderr.ErrUnsupported)
				}
			}
			if observe != nil {
				if err := observe.ObserveRows(budget, members[lo:hi], out[:hi-lo], workers); err != nil {
					return err
				}
			}
			lo = hi
		}
		return budget.Err()
	}
	return preflightLoftRowsSerial(budget, tree, members, starts, ceiling, scanned, observer...)
}

// preflightLoftRowsSerial walks members one row at a time against the budget,
// given the work scanned by the rows before them.
func preflightLoftRowsSerial(budget *proofbound.WorkBudget, tree loftRowScan, members, starts []int,
	ceiling, scanned uint64, observer ...loftRowObserver) error {
	var observe loftRowObserver
	if len(observer) != 0 {
		observe = observer[0]
	}
	for _, i := range members {
		rowCount := 0
		work, exceeded, err := tree.visitRow(budget, i, ceiling-scanned, func(j int) {
			rowCount++
			if observe != nil {
				observe.Observe(i, j)
			}
		})
		if err != nil {
			return err
		}
		if exceeded {
			return errLoftAuditScanLimit(scanned, work, ceiling)
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
func streamLoftPairs(budget *proofbound.WorkBudget, tree loftRowScan, members, starts []int,
	count, ceiling uint64, batchSize int, tris [][3]int, boxes [][2]r3.Vec, data *LoftAuditData,
	shortcuts LoftAuditShortcuts, work *LoftAuditWork, testPair func(i, j int) error) (int, int, error) {
	return streamLoftPairsWorkers(budget, tree, members, starts, count, ceiling, batchSize, tris, boxes, data,
		shortcuts, work, testPair, orderedwork.EachWorkers(len(members), loftRowParallelMin))
}

// streamLoftPairsWorkers is streamLoftPairs with an explicit worker count;
// with workers < 2 each row walks serially against the budget. Otherwise a
// chunk of rows walks and sorts concurrently, and the caller replays each
// row's budget steps and feeds the batch in member order.
func streamLoftPairsWorkers(budget *proofbound.WorkBudget, tree loftRowScan, members, starts []int,
	count, ceiling uint64, batchSize int, tris [][3]int, boxes [][2]r3.Vec, data *LoftAuditData,
	shortcuts LoftAuditShortcuts, work *LoftAuditWork, testPair func(i, j int) error,
	workers int) (int, int, error) {
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
	var out []loftRowResult
	if workers >= 2 {
		out = make([]loftRowResult, min(loftRowChunkRows, len(members)))
	}
	var chunkLo, chunkHi int
	for m, i := range members {
		if workers >= 2 && m == chunkHi {
			chunkLo, chunkHi = m, loftRowChunkEnd(members, starts, m)
			if err := walkLoftRows(budget, tree, members, chunkLo, chunkHi, ceiling, true, workers,
				out); err != nil {
				// Workers never charge the budget, so a poll that finds it
				// exhausted hands this row and the rest to the serial walk,
				// which fails at its own step with that step's error.
				if !errors.Is(err, proofbound.ErrWallWorkBudget) {
					return -1, -1, err
				}
				workers = 1
			}
		}
		if workers >= 2 {
			r := &out[m-chunkLo]
			rowWork, exceeded, err := replayLoftRow(budget, r, ceiling-scanned)
			if err != nil {
				return -1, -1, err
			}
			if exceeded {
				return -1, -1, errLoftAuditScanLimit(scanned, rowWork, ceiling)
			}
			scanned += rowWork
			row = r.row
			if len(row) != starts[i+1]-starts[i] {
				return -1, -1, fmt.Errorf("%w: row-ordered loft candidates differ from S8 count", decaderr.ErrUnsupported)
			}
		} else {
			row = row[:0]
			rowWork, exceeded, err := tree.visitRow(budget, i, ceiling-scanned,
				func(j int) { row = append(row, int32(j)) })
			if err != nil {
				return -1, -1, err
			}
			if exceeded {
				return -1, -1, errLoftAuditScanLimit(scanned, rowWork, ceiling)
			}
			scanned += rowWork
			if len(row) != starts[i+1]-starts[i] {
				return -1, -1, fmt.Errorf("%w: row-ordered loft candidates differ from S8 count", decaderr.ErrUnsupported)
			}
			slices.Sort(row)
		}
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
