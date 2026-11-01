package tessellation

import (
	"context"
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

const replayAuditLimit = 65536

func replayEdgeLess(a, b [2]int) bool {
	return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1])
}

// RequireClosedMeshReplay checks the ordinary directed-edge invariant using
// two fixed, precharged arrays. Sorting and reverse lookup spend the same
// replay work counter; no map or per-edge allocation is used.
func RequireClosedMeshReplay(ctx context.Context, triangles [][3]int, budget *proofbound.ReplayBudget) error {
	if budget == nil {
		return decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 16); err != nil {
		return err
	}
	if len(triangles) > replayAuditLimit {
		return fmt.Errorf("%w: replay closed-mesh audit exceeds its facet bound", decaderr.ErrUnsupported)
	}
	n := 3 * len(triangles)
	if err := budget.ReserveRecords(ctx, 2*uint64(n)); err != nil {
		return err
	}
	for range 2 {
		if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof([2]int{}))); err != nil {
			return err
		}
	}
	edges, scratch := make([][2]int, n), make([][2]int, n)
	for i, tri := range triangles {
		for k := range 3 {
			if err := budget.SpendWork(ctx, 32); err != nil {
				return err
			}
			edges[3*i+k] = [2]int{tri[k], tri[(k+1)%3]}
		}
	}
	for width := 1; width < n; width *= 2 {
		for lo := 0; lo < n; lo += 2 * width {
			mid, hi := min(lo+width, n), min(lo+2*width, n)
			i, j := lo, mid
			for k := lo; k < hi; k++ {
				if err := budget.SpendWork(ctx, 32); err != nil {
					return err
				}
				if i < mid && (j >= hi || !replayEdgeLess(edges[j], edges[i])) {
					scratch[k] = edges[i]
					i++
				} else {
					scratch[k] = edges[j]
					j++
				}
			}
		}
		edges, scratch = scratch, edges
		if width > n/2 {
			break
		}
	}
	for i, e := range edges {
		if err := budget.SpendWork(ctx, 32); err != nil {
			return err
		}
		if i > 0 && e == edges[i-1] {
			return degenerate(`the chorded boundary could not be triangulated into a watertight mesh`)
		}
		reverse := [2]int{e[1], e[0]}
		lo, hi := 0, len(edges)
		for lo < hi {
			if err := budget.SpendWork(ctx, 32); err != nil {
				return err
			}
			mid := lo + (hi-lo)/2
			if replayEdgeLess(edges[mid], reverse) {
				lo = mid + 1
			} else {
				hi = mid
			}
		}
		if lo == len(edges) || edges[lo] != reverse {
			return degenerate(`the chorded boundary could not be triangulated into a watertight mesh`)
		}
	}
	return ctx.Err()
}

// RequireVertexLinksReplay checks each link as one cycle, with fixed arrays
// indexed by vertex and no allocation inside a vertex walk.
func RequireVertexLinksReplay(ctx context.Context, vertexCount int, triangles [][3]int,
	budget *proofbound.ReplayBudget) error {
	if budget == nil {
		return decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 16); err != nil {
		return err
	}
	if vertexCount < 0 || vertexCount > replayAuditLimit || len(triangles) > replayAuditLimit {
		return fmt.Errorf("%w: replay vertex-link audit exceeds its mesh bound", decaderr.ErrUnsupported)
	}
	v, t := uint64(vertexCount), uint64(len(triangles))
	if err := budget.ReserveRecords(ctx, 6*v+t); err != nil {
		return err
	}
	for range 5 {
		if err := budget.ReserveArray(ctx, v, uint64(unsafe.Sizeof(int(0)))); err != nil {
			return err
		}
	}
	if err := budget.ReserveArray(ctx, v, uint64(unsafe.Sizeof(false))); err != nil {
		return err
	}
	if err := budget.ReserveArray(ctx, t, uint64(unsafe.Sizeof([3]int{}))); err != nil {
		return err
	}
	if err := budget.SpendWork(ctx, 8*v); err != nil {
		return err
	}
	heads, degree := make([]int, vertexCount), make([]int, vertexCount)
	first, second := make([]int, vertexCount), make([]int, vertexCount)
	touched, seen := make([]int, 0, vertexCount), make([]bool, vertexCount)
	next := make([][3]int, len(triangles))
	for i := range heads {
		heads[i] = -1
	}
	for i, tri := range triangles {
		for k, center := range tri {
			if err := budget.SpendWork(ctx, 32); err != nil {
				return err
			}
			if center < 0 || center >= vertexCount {
				return fmt.Errorf("%w: replay mesh has an invalid vertex", decaderr.ErrUnsupported)
			}
			next[i][k], heads[center] = heads[center], 3*i+k
		}
	}
	for center := range vertexCount {
		if heads[center] < 0 {
			continue
		}
		touched = touched[:0]
		add := func(from, to int) {
			if degree[from] == 0 {
				touched = append(touched, from)
			}
			switch degree[from] {
			case 0:
				first[from] = to
			case 1:
				second[from] = to
			}
			degree[from]++
		}
		for corner := heads[center]; corner >= 0; {
			if err := budget.SpendWork(ctx, 32); err != nil {
				return err
			}
			i, k := corner/3, corner%3
			tri := triangles[i]
			a, b := tri[(k+1)%3], tri[(k+2)%3]
			add(a, b)
			add(b, a)
			corner = next[i][k]
		}
		start := vertexCount
		for _, vertex := range touched {
			if err := budget.SpendWork(ctx, 32); err != nil {
				return err
			}
			if degree[vertex] != 2 {
				return fmt.Errorf("%w: the mesh vertex at index %d has a pinched link", decaderr.ErrUnsupported, center)
			}
			start = min(start, vertex)
		}
		if len(touched) == 0 {
			continue
		}
		prev, cur, visited := -1, start, 1
		seen[start] = true
		for {
			if err := budget.SpendWork(ctx, 32); err != nil {
				return err
			}
			nxt := first[cur]
			if nxt == prev {
				nxt = second[cur]
			}
			if nxt == start {
				break
			}
			if seen[nxt] {
				return fmt.Errorf("%w: the mesh vertex at index %d has a pinched link", decaderr.ErrUnsupported, center)
			}
			seen[nxt] = true
			visited++
			prev, cur = cur, nxt
		}
		if visited != len(touched) {
			return fmt.Errorf("%w: the mesh vertex at index %d has more than one link cycle", decaderr.ErrUnsupported, center)
		}
		for _, vertex := range touched {
			if err := budget.SpendWork(ctx, 32); err != nil {
				return err
			}
			degree[vertex], seen[vertex] = 0, false
		}
	}
	return ctx.Err()
}
