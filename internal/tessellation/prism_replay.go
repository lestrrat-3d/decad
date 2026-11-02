package tessellation

import (
	"context"
	"fmt"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// PrismRectangleWallsReplay emits one rectangle without runtime face maps.
func PrismRectangleWallsReplay[F comparable](ctx context.Context, loop PrismLoop[F], axialDelta float64,
	budget *proofbound.ReplayBudget) (PrismTopology[F], error) {
	if err := ctx.Err(); err != nil {
		return PrismTopology[F]{}, err
	}
	if budget == nil || len(loop.Samples) != 4 || len(loop.BoundOf) != 4 || len(loop.FaceOf) != 4 || len(loop.SagOf) != 4 {
		return PrismTopology[F]{}, fmt.Errorf("%w: bounded prism walls require four aligned stations", decaderr.ErrUnsupported)
	}
	if err := budget.SpendWork(ctx, 1+4+4); err != nil {
		return PrismTopology[F]{}, err
	}
	if err := budget.ReserveRecords(ctx, 1+2+2*4+2*4+4+2+2*8*8); err != nil {
		return PrismTopology[F]{}, err
	}
	for _, allocation := range [][2]uint64{
		{4, uint64(unsafe.Sizeof(sectionrecord.Point2{}))}, {4, uint64(unsafe.Sizeof(proofbound.WalkEndBound{}))},
		{4, uint64(unsafe.Sizeof(int(0)))}, {1, uint64(unsafe.Sizeof([]int{}))}, {1, uint64(unsafe.Sizeof(float64(0)))},
		{12, uint64(unsafe.Sizeof([3]int{}))}, {12, uint64(unsafe.Sizeof(*new(F)))}, {6, uint64(unsafe.Sizeof(PrismReplayFaceBound[F]{}))},
	} {
		if err := budget.ReserveArray(ctx, allocation[0], allocation[1]); err != nil {
			return PrismTopology[F]{}, err
		}
	}
	t := PrismTopology[F]{Points: make([]sectionrecord.Point2, 4), Bounds: make([]proofbound.WalkEndBound, 4), LoopIdx: make([][]int, 1), LoopSag: make([]float64, 1), Triangles: make([][3]int, 0, 12), Sources: make([]F, 0, 12), ReplayFaceBounds: make([]PrismReplayFaceBound[F], 0, 6)}
	copy(t.Points, loop.Samples)
	copy(t.Bounds, loop.BoundOf)
	t.LoopIdx[0] = []int{0, 1, 2, 3}
	t.LoopSag[0] = loop.MaxSag
	t.CapTrim = loop.MaxSag
	t.AreaSlack = proofbound.AbsSumUpper(0, loop.WallSlack, loop.CapSlack, loop.CapSlack)
	t.SegmentArea = proofbound.AbsSumUpper(0, loop.SegmentArea)
	t.Walks = loop.Walks
	t.PerimeterUpper = proofbound.AbsSumUpper(0, loop.PerimeterUpper)
	for j := range 4 {
		g1 := (j + 1) % 4
		face := loop.FaceOf[j]
		t.Triangles = append(t.Triangles, [3]int{2 * j, 2 * g1, 2*g1 + 1}, [3]int{2 * j, 2*g1 + 1, 2*j + 1})
		t.Sources = append(t.Sources, face, face)
		if err := t.SetReplayFaceBound(face, math.Max(0, loop.SagOf[j]), axialDelta, true); err != nil {
			return PrismTopology[F]{}, err
		}
	}
	return t, nil
}
