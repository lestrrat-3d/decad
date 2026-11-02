package booleaneval

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

func piecesOfReplay(ctx context.Context, m *topo.Mesh, budget *proofbound.ReplayBudget) (pieces, error) {
	if m == nil || budget == nil {
		return pieces{}, errBrakeEvalBoundedPiecesNeedAMeshAndBudget
	}
	v, f := uint64(len(m.Vertices)), uint64(len(m.Triangles))
	if v > proofbound.MaxMultiwayRecordsPerCall || f > proofbound.MaxMultiwayRecordsPerCall {
		return pieces{}, budget.ReserveRecords(ctx, proofbound.MaxMultiwayRecordsPerCall+1)
	}
	if err := budget.ReserveRecords(ctx, 3*v+f+1); err != nil {
		return pieces{}, err
	}
	for _, count := range []uint64{v, v, v, f} {
		if err := budget.ReserveArray(ctx, count, uint64(unsafe.Sizeof(int(0)))); err != nil {
			return pieces{}, err
		}
	}
	parent := make([]int, len(m.Vertices))
	for i := range parent {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return pieces{}, err
		}
		parent[i] = i
	}
	find := func(x int) (int, error) {
		for parent[x] != x {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return 0, err
			}
			parent[x] = parent[parent[x]]
			x = parent[x]
		}
		return x, nil
	}
	for _, tri := range m.Triangles {
		for _, vi := range tri[1:] {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return pieces{}, err
			}
			if tri[0] < 0 || tri[0] >= len(parent) || vi < 0 || vi >= len(parent) {
				return pieces{}, errBrakeEvalInvalidBoundedMeshIndex
			}
			a, err := find(tri[0])
			if err != nil {
				return pieces{}, err
			}
			b, err := find(vi)
			if err != nil {
				return pieces{}, err
			}
			if a != b {
				parent[max(a, b)] = min(a, b)
			}
		}
	}
	ids := make([]int, len(m.Vertices))
	for i := range ids {
		ids[i] = -1
	}
	count := 0
	pc := pieces{ofVertex: make([]int, len(m.Vertices)), ofFacet: make([]int, len(m.Triangles))}
	for i := range m.Vertices {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return pieces{}, err
		}
		root, err := find(i)
		if err != nil {
			return pieces{}, err
		}
		c := ids[root]
		if c < 0 {
			c = count
			count++
			ids[root] = c
		}
		pc.ofVertex[i] = c
	}
	for i, tri := range m.Triangles {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return pieces{}, err
		}
		pc.ofFacet[i] = pc.ofVertex[tri[0]]
	}
	pc.count = count
	return pc, nil
}
