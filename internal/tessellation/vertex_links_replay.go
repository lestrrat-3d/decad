package tessellation

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

const replayVertexLinkLimit = 65536

// ReplayRequireVertexLinks checks each link as one cycle, with fixed arrays
// indexed by vertex and no allocation inside a vertex walk.
func ReplayRequireVertexLinks(ctx context.Context, vertexCount int, triangles [][3]int,
	budget *proofbound.ReplayBudget) error {
	if budget == nil || vertexCount < 0 || vertexCount > replayVertexLinkLimit || len(triangles) > replayVertexLinkLimit {
		return decaderr.ErrUnsupported
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
	heads, degree := make([]int, vertexCount), make([]int, vertexCount)
	first, second := make([]int, vertexCount), make([]int, vertexCount)
	touched, seen := make([]int, 0, vertexCount), make([]bool, vertexCount)
	next := make([][3]int, len(triangles))
	for i := range heads {
		if err := budget.SpendWork(ctx, 8); err != nil {
			return err
		}
		heads[i] = -1
	}
	for i := range triangles {
		if err := budget.SpendWork(ctx, 8); err != nil {
			return err
		}
		tri := triangles[i]
		for k := range tri {
			if err := budget.SpendWork(ctx, 32); err != nil {
				return err
			}
			center := tri[k]
			if center < 0 || center >= vertexCount {
				return decaderr.ErrUnsupported
			}
			next[i][k], heads[center] = heads[center], 3*i+k
		}
	}
	for center := range vertexCount {
		if err := budget.SpendWork(ctx, 8); err != nil {
			return err
		}
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
		for i := range touched {
			if err := budget.SpendWork(ctx, 32); err != nil {
				return err
			}
			vertex := touched[i]
			if degree[vertex] != 2 {
				return decaderr.ErrUnsupported
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
				return decaderr.ErrUnsupported
			}
			seen[nxt] = true
			visited++
			prev, cur = cur, nxt
		}
		if visited != len(touched) {
			return decaderr.ErrUnsupported
		}
		for i := range touched {
			if err := budget.SpendWork(ctx, 32); err != nil {
				return err
			}
			vertex := touched[i]
			degree[vertex], seen[vertex] = 0, false
		}
	}
	return ctx.Err()
}
