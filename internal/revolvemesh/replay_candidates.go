package revolvemesh

import (
	"cmp"
	"context"
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
)

// These refusals are constructed once so rejected replay does not allocate.
var (
	errReplayCandidates1 = fmt.Errorf("%w: invalid bounded revolve audit", decaderr.ErrUnsupported)
	errReplayCandidates2 = fmt.Errorf("%w: revolve facet audit has no finite margin", decaderr.ErrUnsupported)
	errReplayCandidates3 = fmt.Errorf("%w: revolve vertex index outside bounded audit", decaderr.ErrUnsupported)
)

func revolveContactAuditReplay(ctx context.Context, data []RevolveAuditTri, tris [][3]int,
	delta float64, charged uint64, budget *proofbound.ReplayBudget) (uint64, error) {
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	if budget == nil || len(data) != len(tris) {
		return 0, errReplayCandidates1
	}
	margin := proofbound.ProductUpper(2, delta)
	if !(margin >= 0) || proofbound.IsNonFinite(margin) {
		return 0, errReplayCandidates2
	}
	ceiling := uint64(proofbound.MaxFacetPairTestsPerCall)
	if charged > ceiling {
		return 0, revolveFacetPairWorkLimit()
	}
	remaining := ceiling - charged
	rows, scanned, err := replayContactCandidates(ctx, data, tris, margin, remaining, budget)
	if err != nil {
		return scanned, err
	}
	if err := budget.SpendWork(ctx, 8*uint64(len(rows))); err != nil {
		return scanned, err
	}
	for i, row := range rows {
		for _, jj := range row {
			if err := budget.SpendWork(ctx, 64); err != nil {
				return scanned, err
			}
			j := int(jj)
			shared, sharedCount := tessellation.SharedVertexIndices(tris[i], tris[j])
			if sharedCount != 0 || !BoxGapExceeds(data[i].Box, data[j].Box, margin) {
				if err := auditRevolvePairReplay(ctx, data, tris, shared, sharedCount, i, j, delta, budget); err != nil {
					return scanned, err
				}
			}
		}
	}
	return scanned, nil
}

// replayContactCandidates preserves the original ordered candidate rows and
// returns the number of unique pairs that need contact classification.
// Candidate discovery, sorting, and every growing row are charged separately
// to the replay budget.
func replayContactCandidates(ctx context.Context, data []RevolveAuditTri, tris [][3]int,
	margin float64, remaining uint64, budget *proofbound.ReplayBudget) ([][]int32, uint64, error) {
	n := len(data)
	if uint64(n) > proofbound.MaxMultiwayRecordsPerCall {
		return nil, 0, budget.ReserveRecords(ctx, proofbound.MaxMultiwayRecordsPerCall+1)
	}
	if err := budget.SpendWork(ctx, 64*uint64(n)); err != nil {
		return nil, 0, err
	}
	axis := revolveSweepAxis(data)
	if err := budget.ReserveRecords(ctx, 2*uint64(n)+1); err != nil {
		return nil, 0, err
	}
	for _, size := range []uint64{uint64(unsafe.Sizeof(int32(0))), uint64(unsafe.Sizeof([]int32{}))} {
		if err := budget.ReserveArray(ctx, uint64(n), size); err != nil {
			return nil, 0, err
		}
	}
	order := make([]int32, n)
	rows := make([][]int32, n)
	if err := budget.SpendWork(ctx, 8*uint64(n)); err != nil {
		return nil, 0, err
	}
	for i := range order {
		order[i] = int32(i)
	}
	if err := replayStableSort(ctx, order, func(a, b int32) bool {
		return cmp.Less(boxCoord(data[a].Box[0], axis), boxCoord(data[b].Box[0], axis))
	}, budget); err != nil {
		return nil, 0, err
	}
	if err := budget.ReserveRecords(ctx, 3*uint64(n)+2); err != nil {
		return nil, 0, err
	}
	if err := budget.ReserveArray(ctx, 2, 128); err != nil {
		return nil, 0, err
	}
	if err := budget.ReserveArray(ctx, 3*uint64(n), uint64(unsafe.Sizeof([]int32{}))); err != nil {
		return nil, 0, err
	}
	incident := make([][]int32, 3*n)
	for i, tri := range tris {
		for _, v := range tri {
			if err := budget.SpendWork(ctx, 64); err != nil {
				return nil, 0, err
			}
			if v < 0 || v >= len(incident) {
				return nil, 0, errReplayCandidates3
			}
			row := incident[v]
			if len(row) == 0 {
				if err := budget.ReserveRecords(ctx, 1); err != nil {
					return nil, 0, err
				}
			}
			var err error
			incident[v], err = replayAppend(ctx, row, int32(i), budget)
			if err != nil {
				return nil, 0, err
			}
		}
	}
	scanned, err := replayBoxCandidates(ctx, data, order, rows, margin, remaining, budget)
	if err != nil {
		return nil, scanned, err
	}
	for i, tri := range tris {
		for _, v := range tri {
			for _, j := range incident[v] {
				if err := budget.SpendWork(ctx, 64); err != nil {
					return nil, scanned, err
				}
				if int(j) <= i {
					continue
				}
				var err error
				rows[i], err = replayAppend(ctx, rows[i], j, budget)
				if err != nil {
					return nil, scanned, err
				}
			}
		}
	}
	for i, row := range rows {
		if err := replayStableSort(ctx, row, func(a, b int32) bool { return a < b }, budget); err != nil {
			return nil, scanned, err
		}
		if err := budget.SpendWork(ctx, 8*uint64(len(row))); err != nil {
			return nil, scanned, err
		}
		out := 0
		for _, v := range row {
			if out == 0 || row[out-1] != v {
				row[out] = v
				out++
			}
		}
		rows[i] = row[:out]
	}
	candidates := uint64(revolveRowsLen(rows))
	if candidates > remaining {
		return nil, remaining, revolveFacetPairWorkLimit()
	}
	return rows, candidates, nil
}
