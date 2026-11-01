package triangulation

import (
	"context"
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
)

// EarClipReplay triangulates a bounded polygon under the shared replay meter.
func EarClipReplay(ctx context.Context, pts []Point2, poly []int, b *proofbound.ReplayBudget) ([][3]int, error) {
	if b == nil {
		return nil, decaderr.ErrUnsupported
	}
	if err := b.SpendWork(ctx, 16); err != nil {
		return nil, err
	}
	if len(poly) < 3 || len(poly) > 1<<14 {
		return nil, fmt.Errorf("%w: bounded cap polygon size", decaderr.ErrUnsupported)
	}
	for _, index := range poly {
		if err := b.SpendWork(ctx, 16); err != nil {
			return nil, err
		}
		if index < 0 || index >= len(pts) ||
			proofbound.IsNonFinite(pts[index].U) || proofbound.IsNonFinite(pts[index].V) {
			return nil, fmt.Errorf("%w: invalid cap point", decaderr.ErrUnsupported)
		}
	}
	if err := b.ReserveRecords(ctx, 2*uint64(len(poly))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(len(poly)), uint64(unsafe.Sizeof(int(0)))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(len(poly)), uint64(unsafe.Sizeof([3]int{}))); err != nil {
		return nil, err
	}
	if err := b.SpendWork(ctx, uint64(len(poly))); err != nil {
		return nil, err
	}
	idx := append([]int(nil), poly...)
	tris := make([][3]int, 0, len(idx))
	for len(idx) > 2 {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		n := len(idx)
		clipped := false
		for i := range n {
			if err := b.SpendWork(ctx, 32); err != nil {
				return nil, err
			}
			ia, ib, ic := idx[(i-1+n)%n], idx[i], idx[(i+1)%n]
			if ia < 0 || ib < 0 || ic < 0 || ia >= len(pts) || ib >= len(pts) || ic >= len(pts) {
				return nil, fmt.Errorf("%w: invalid cap point index", decaderr.ErrUnsupported)
			}
			cr := Cross2(pts[ia], pts[ib], pts[ic])
			if cr < 0 {
				continue
			}
			if cr == 0 {
				if !isBridgeStub(ia, ib, ic) {
					continue
				}
			} else {
				blocked := false
				ip, in := (i-1+n)%n, (i+1)%n
				a, c, d := pts[idx[ip]], pts[idx[i]], pts[idx[in]]
				for j := range n {
					// The worst path evaluates a turn and three triangle edges.
					if err := b.SpendWork(ctx, 128); err != nil {
						return nil, err
					}
					if j == ip || j == i || j == in {
						continue
					}
					pi, prev, next := idx[j], idx[(j-1+n)%n], idx[(j+1)%n]
					if pi < 0 || prev < 0 || next < 0 || pi >= len(pts) || prev >= len(pts) || next >= len(pts) {
						return nil, fmt.Errorf("%w: invalid cap blocker index", decaderr.ErrUnsupported)
					}
					q := pts[pi]
					turn := Cross2(pts[prev], q, pts[next])
					if turn > 0 || (turn == 0 && (q == a || q == c || q == d)) {
						continue
					}
					if pointInTri(q, a, c, d) {
						blocked = true
						break
					}
				}
				if blocked {
					continue
				}
				tris = append(tris, [3]int{ia, ib, ic})
			}
			if err := b.SpendWork(ctx, 16*uint64(n-i-1)); err != nil {
				return nil, err
			}
			idx = append(idx[:i], idx[i+1:]...)
			clipped = true
			break
		}
		if !clipped {
			return nil, tessellation.NewExpectedError(fmt.Errorf("%w: the chorded cap boundary self-intersects; tessellate at a finer tolerance", decaderr.ErrDegenerate))
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return tris, nil
}
