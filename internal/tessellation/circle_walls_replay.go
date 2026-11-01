package tessellation

import (
	"context"
	"fmt"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// PrismCircleWallsReplay emits one single-face loop in ordinary wall order.
func PrismCircleWallsReplay[F comparable](ctx context.Context, loop PrismLoop[F], axial float64,
	budget *proofbound.ReplayBudget) (PrismTopology[F], error) {
	if err := ctx.Err(); err != nil {
		return PrismTopology[F]{}, err
	}
	n := len(loop.Samples)
	if budget == nil || !budget.IsBrakeReplay() || n < 3 || n > freeform.MaxChordsPerWalk ||
		len(loop.BoundOf) != n || len(loop.FaceOf) != n || len(loop.SagOf) != n {
		return PrismTopology[F]{}, fmt.Errorf("%w: bounded circular walls require one aligned loop", decaderr.ErrUnsupported)
	}
	// The wall loop has one face. Reserve three entries, including both caps;
	// the two facet arrays also reserve room for n-2 triangles at each cap.
	if err := budget.SpendWork(ctx, uint64(3*n+1)); err != nil {
		return PrismTopology[F]{}, err
	}
	for i := 1; i < n; i++ {
		if loop.FaceOf[i] != loop.FaceOf[0] {
			return PrismTopology[F]{}, fmt.Errorf("%w: circular wall has multiple source faces", decaderr.ErrUnsupported)
		}
	}
	if err := budget.ReserveRecords(ctx, uint64(3*n+2*(4*n-4)+6)); err != nil {
		return PrismTopology[F]{}, err
	}
	for _, a := range [][2]uint64{
		{uint64(n), uint64(unsafe.Sizeof(sectionrecord.Point2{}))},
		{uint64(n), uint64(unsafe.Sizeof(proofbound.WalkEndBound{}))},
		{uint64(n), uint64(unsafe.Sizeof(int(0)))},
		{1, uint64(unsafe.Sizeof([]int{}))}, {1, uint64(unsafe.Sizeof(float64(0)))},
		{uint64(4*n - 4), uint64(unsafe.Sizeof([3]int{}))},
		{uint64(4*n - 4), uint64(unsafe.Sizeof(*new(F)))},
		{3, uint64(unsafe.Sizeof(PrismReplayFaceBound[F]{}))},
	} {
		if err := budget.ReserveArray(ctx, a[0], a[1]); err != nil {
			return PrismTopology[F]{}, err
		}
	}
	t := PrismTopology[F]{Points: make([]sectionrecord.Point2, n), Bounds: make([]proofbound.WalkEndBound, n),
		LoopIdx: make([][]int, 1), LoopSag: make([]float64, 1), Triangles: make([][3]int, 0, 4*n-4),
		Sources: make([]F, 0, 4*n-4), ReplayFaceBounds: make([]PrismReplayFaceBound[F], 0, 3)}
	copy(t.Points, loop.Samples)
	copy(t.Bounds, loop.BoundOf)
	t.LoopIdx[0] = make([]int, n)
	t.LoopSag[0] = loop.MaxSag
	t.CapTrim = math.Max(0, loop.MaxSag)
	t.AreaSlack = proofbound.AbsSumUpper(0, loop.WallSlack, loop.CapSlack, loop.CapSlack)
	t.SegmentArea = proofbound.AbsSumUpper(0, loop.SegmentArea)
	t.Walks = loop.Walks
	t.PerimeterUpper = proofbound.AbsSumUpper(0, loop.PerimeterUpper)
	for j := range n {
		t.LoopIdx[0][j] = j
		g1 := (j + 1) % n
		face := loop.FaceOf[j]
		t.Triangles = append(t.Triangles, [3]int{2 * j, 2 * g1, 2*g1 + 1}, [3]int{2 * j, 2*g1 + 1, 2*j + 1})
		t.Sources = append(t.Sources, face, face)
		if err := t.SetReplayFaceBound(face, math.Max(0, loop.SagOf[j]), axial, true); err != nil {
			return PrismTopology[F]{}, err
		}
	}
	return t, nil
}
