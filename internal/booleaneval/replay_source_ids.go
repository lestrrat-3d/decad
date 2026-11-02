package booleaneval

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

func replaySourceIDs(ctx context.Context, b *proofbound.ReplayBudget, source []*topo.Face, ids *topo.FaceBoundTable) ([]int, error) {
	if err := b.ReserveRecords(ctx, uint64(len(source))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(len(source)), uint64(unsafe.Sizeof(int(0)))); err != nil {
		return nil, err
	}
	out := make([]int, len(source))
	for i, f := range source {
		v, ok, err := ids.Lookup(ctx, f, b)
		if err != nil {
			return nil, err
		}
		if !ok {
			return nil, errBrakeEvalAFacetSSourceIsNotAnOperandFace
		}
		out[i] = int(v)
	}
	return out, nil
}

func replayFacePieces(ctx context.Context, b *proofbound.ReplayBudget, m *topo.Mesh, pc pieces) (*topo.FaceBoundTable, error) {
	table, err := topo.NewFaceBoundTable(ctx, len(m.Source), b)
	if err != nil {
		return nil, err
	}
	for i := range m.Triangles {
		if err := b.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		if i >= len(m.Source) || m.Source[i] == nil {
			continue
		}
		f := m.Source[i]
		prev, ok, err := table.Lookup(ctx, f, b)
		if err != nil {
			return nil, err
		}
		value := pc.ofFacet[i]
		if ok && int(prev) != value {
			value = spanningPieces
		}
		if err := table.Set(ctx, f, float64(value), b); err != nil {
			return nil, err
		}
	}
	return table, nil
}
