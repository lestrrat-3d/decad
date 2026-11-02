package stackedrecord

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// ReplayBlindMeshVertices is the bounded vertex stage of a blind stack mesh.
// The returned ring indices use the same ordering as ordinary Tessellate.
type ReplayBlindMeshVertices struct {
	Chords   ReplayBlindChordStructure
	Vertices []r3.Vec
	Store    []float64
	Bottom   [][]int
	Top      [][]int
}

// ReplayBlindMeshVerticesWithSource charges every frame lift and stored
// coordinate bound after source-bound chording and slab clearance.
func ReplayBlindMeshVerticesWithSource(ctx context.Context, body, host *topo.Body,
	requestedChord float64, budget *proofbound.ReplayBudget) (ReplayBlindMeshVertices, bool, error) {
	chords, ok, err := ReplayBlindChordClearanceWithSource(ctx, body, host, requestedChord, budget)
	if err != nil || !ok {
		return ReplayBlindMeshVertices{}, false, err
	}
	sp, ok := body.Payload.(Payload)
	if !ok || len(chords.Columns) != len(chords.Loops) {
		return ReplayBlindMeshVertices{}, false, nil
	}
	var total uint64
	for _, loop := range chords.Loops {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return ReplayBlindMeshVertices{}, false, err
		}
		total += uint64(len(loop.Samples))
		if total > 1<<18 || len(loop.Samples) != len(loop.BoundOf) {
			return ReplayBlindMeshVertices{}, false, decaderr.ErrUnsupported
		}
	}
	if err := budget.ReserveArray(ctx, 2*total, uint64(unsafe.Sizeof(r3.Vec{}))); err != nil {
		return ReplayBlindMeshVertices{}, false, err
	}
	if err := budget.ReserveArray(ctx, 2*total, uint64(unsafe.Sizeof(float64(0)))); err != nil {
		return ReplayBlindMeshVertices{}, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(chords.Loops)), uint64(unsafe.Sizeof([]int{}))); err != nil {
		return ReplayBlindMeshVertices{}, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(chords.Loops)), uint64(unsafe.Sizeof([]int{}))); err != nil {
		return ReplayBlindMeshVertices{}, false, err
	}
	out := ReplayBlindMeshVertices{Chords: chords,
		Vertices: make([]r3.Vec, 0, 2*total), Store: make([]float64, 0, 2*total),
		Bottom: make([][]int, len(chords.Loops)), Top: make([][]int, len(chords.Loops))}
	base := sp.OuterPrism()
	for ci, loop := range chords.Loops {
		col := chords.Columns[ci]
		if col.Start < 0 || col.End < col.Start || col.End >= len(sp.Slabs) {
			return ReplayBlindMeshVertices{}, false, decaderr.ErrUnsupported
		}
		if err := budget.ReserveArray(ctx, uint64(len(loop.Samples)), uint64(unsafe.Sizeof(int(0)))); err != nil {
			return ReplayBlindMeshVertices{}, false, err
		}
		if err := budget.ReserveArray(ctx, uint64(len(loop.Samples)), uint64(unsafe.Sizeof(int(0)))); err != nil {
			return ReplayBlindMeshVertices{}, false, err
		}
		out.Bottom[ci] = make([]int, len(loop.Samples))
		out.Top[ci] = make([]int, len(loop.Samples))
		for j, p := range loop.Samples {
			if err := budget.SpendWork(ctx, 16); err != nil {
				return ReplayBlindMeshVertices{}, false, err
			}
			for side, z := range [...]float64{sp.Slabs[col.Start].Z0, sp.Slabs[col.End].Z1} {
				point, round, err := base.LiftedVertexReplay(ctx, p.U, p.V, z, budget)
				if err != nil {
					return ReplayBlindMeshVertices{}, false, err
				}
				index := len(out.Vertices)
				out.Vertices = append(out.Vertices, point)
				out.Store = append(out.Store, proofbound.AbsSumUpper(
					proofbound.WalkEndBoundAllow(loop.BoundOf[j]), round))
				if side == 0 {
					out.Bottom[ci][j] = index
				} else {
					out.Top[ci][j] = index
				}
			}
		}
	}
	return out, true, nil
}
