package tessellation

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type replayDirectedEdge struct{ from, to int }

func replayDirectedLess(a, b replayDirectedEdge) bool {
	return a.from < b.from || a.from == b.from && a.to < b.to
}

// ReplayRequireClosedMesh checks the same directed-edge condition as
// RequireClosedMesh using a bounded, cancellable in-place heap sort. It is one
// output audit, not a complete boundary or volume verification.
func ReplayRequireClosedMesh(ctx context.Context, triangles [][3]int, budget *proofbound.ReplayBudget) error {
	if budget == nil {
		return decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return err
	}
	if len(triangles) > math.MaxInt/3 {
		return decaderr.ErrUnsupported
	}
	n := len(triangles) * 3
	if err := budget.ReserveRecords(ctx, uint64(n)); err != nil {
		return err
	}
	if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(replayDirectedEdge{}))); err != nil {
		return err
	}
	refuse := func() error {
		if err := budget.SpendWork(ctx, 128); err != nil {
			return err
		}
		if err := budget.ReserveRecords(ctx, 1); err != nil {
			return err
		}
		if err := budget.ReserveBytes(ctx, 512); err != nil {
			return err
		}
		return degenerate(`the chorded boundary could not be triangulated into a watertight mesh`)
	}
	edges := make([]replayDirectedEdge, n)
	for i := range triangles {
		if err := budget.SpendWork(ctx, 16); err != nil {
			return err
		}
		tri := triangles[i]
		for k := range 3 {
			edges[3*i+k] = replayDirectedEdge{tri[k], tri[(k+1)%3]}
		}
	}
	sift := func(root, end int) error {
		for root < end/2 {
			if err := budget.SpendWork(ctx, 32); err != nil {
				return err
			}
			child := root*2 + 1
			if child+1 < end && replayDirectedLess(edges[child], edges[child+1]) {
				child++
			}
			if !replayDirectedLess(edges[root], edges[child]) {
				return nil
			}
			edges[root], edges[child] = edges[child], edges[root]
			root = child
		}
		return nil
	}
	for start := n / 2; start > 0; start-- {
		if err := sift(start-1, n); err != nil {
			return err
		}
	}
	for end := n - 1; end > 0; end-- {
		if err := budget.SpendWork(ctx, 8); err != nil {
			return err
		}
		edges[0], edges[end] = edges[end], edges[0]
		if err := sift(0, end); err != nil {
			return err
		}
	}
	for i := range edges {
		if err := budget.SpendWork(ctx, 16); err != nil {
			return err
		}
		e := edges[i]
		if i > 0 && edges[i-1] == e {
			return refuse()
		}
		reverse := replayDirectedEdge{e.to, e.from}
		lo, hi := 0, n
		for lo < hi {
			if err := budget.SpendWork(ctx, 16); err != nil {
				return err
			}
			mid := lo + (hi-lo)/2
			if replayDirectedLess(edges[mid], reverse) {
				lo = mid + 1
			} else {
				hi = mid
			}
		}
		if lo == n || edges[lo] != reverse {
			return refuse()
		}
	}
	return ctx.Err()
}
