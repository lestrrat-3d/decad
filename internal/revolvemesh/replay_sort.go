package revolvemesh

import (
	"context"
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// replayStableSort charges each comparison and each move before it happens.
func replayStableSort(ctx context.Context, a []int32, less func(int32, int32) bool,
	budget *proofbound.ReplayBudget) error {
	if len(a) < 2 {
		return ctx.Err()
	}
	if err := budget.ReserveRecords(ctx, uint64(len(a))); err != nil {
		return err
	}
	if err := budget.ReserveArray(ctx, uint64(len(a)), uint64(unsafe.Sizeof(int32(0)))); err != nil {
		return err
	}
	tmp := make([]int32, len(a))
	for width := 1; width < len(a); width *= 2 {
		for lo := 0; lo < len(a); lo += 2 * width {
			mid, hi := min(lo+width, len(a)), min(lo+2*width, len(a))
			i, j := lo, mid
			for k := lo; k < hi; k++ {
				if err := budget.SpendWork(ctx, 64); err != nil {
					return err
				}
				if i < mid && (j >= hi || !less(a[j], a[i])) {
					tmp[k] = a[i]
					i++
				} else {
					tmp[k] = a[j]
					j++
				}
			}
		}
		if err := budget.SpendWork(ctx, 8*uint64(len(a))); err != nil {
			return err
		}
		copy(a, tmp)
		if width > len(a)/2 {
			break
		}
	}
	return nil
}

func replayAppend(ctx context.Context, row []int32, value int32, budget *proofbound.ReplayBudget) ([]int32, error) {
	if len(row) == cap(row) {
		if cap(row) > int(^uint(0)>>1)/2 {
			return nil, fmt.Errorf("%w: revolve candidate row too large", decaderr.ErrUnsupported)
		}
		n := max(1, 2*cap(row))
		if err := budget.ReserveRecords(ctx, uint64(n)); err != nil {
			return nil, err
		}
		if err := budget.SpendWork(ctx, 8*uint64(len(row))); err != nil {
			return nil, err
		}
		if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(int32(0)))); err != nil {
			return nil, err
		}
		next := make([]int32, len(row), n)
		copy(next, row)
		row = next
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return nil, err
	}
	return append(row, value), nil
}
