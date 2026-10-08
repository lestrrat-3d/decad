package tessellation

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// PrismLoop is one recorded boundary loop's already chorded samples and
// proof charges. FaceOf[j] names the wall of sample j's outgoing chord.
type PrismLoop[F comparable] struct {
	Samples                                  []sectionrecord.Point2
	FaceOf                                   []F
	SagOf                                    []float64
	BoundOf                                  []proofbound.WalkEndBound
	MaxSag, WallSlack, CapSlack, SegmentArea float64
	PerimeterUpper                           float64
	Walks                                    int
}

// PrismTopology holds the shared wall and cap indices and their proof inputs.
type PrismTopology[F comparable] struct {
	Points                                          []sectionrecord.Point2
	Bounds                                          []proofbound.WalkEndBound
	LoopIdx                                         [][]int
	LoopSag                                         []float64
	Triangles                                       [][3]int
	Sources                                         []F
	FaceTrim, FaceAxial                             map[F]float64
	AreaSlack, SegmentArea, PerimeterUpper, CapTrim float64
	Walks                                           int
}

// PrismWalls emits two facets per boundary chord and accumulates the analytic
// proof terms in recorded loop order. Every 2D sample owns adjacent bottom
// and top vertex indices, which the cap later shares.
func PrismWalls[F comparable](loops []PrismLoop[F], sheet bool, axialDelta float64) PrismTopology[F] {
	t := PrismTopology[F]{FaceTrim: map[F]float64{}, FaceAxial: map[F]float64{}}
	for _, loop := range loops {
		t.CapTrim = math.Max(t.CapTrim, loop.MaxSag)
		if sheet {
			t.AreaSlack = proofbound.AbsSumUpper(t.AreaSlack, loop.WallSlack)
		} else {
			t.AreaSlack = proofbound.AbsSumUpper(t.AreaSlack, loop.WallSlack, loop.CapSlack, loop.CapSlack)
		}
		t.SegmentArea = proofbound.AbsSumUpper(t.SegmentArea, loop.SegmentArea)
		t.Walks += loop.Walks
		t.PerimeterUpper = proofbound.AbsSumUpper(t.PerimeterUpper, loop.PerimeterUpper)

		base := len(t.Points)
		t.Points = append(t.Points, loop.Samples...)
		t.Bounds = append(t.Bounds, loop.BoundOf...)
		idx := make([]int, len(loop.Samples))
		for j := range loop.Samples {
			idx[j] = base + j
		}
		t.LoopIdx = append(t.LoopIdx, idx)
		t.LoopSag = append(t.LoopSag, loop.MaxSag)
		for j := range loop.Samples {
			g0 := base + j
			g1 := base + (j+1)%len(loop.Samples)
			f := loop.FaceOf[j]
			t.Triangles = append(t.Triangles,
				[3]int{2 * g0, 2 * g1, 2*g1 + 1},
				[3]int{2 * g0, 2*g1 + 1, 2*g0 + 1})
			t.Sources = append(t.Sources, f, f)
			t.FaceTrim[f] = math.Max(t.FaceTrim[f], loop.SagOf[j])
			t.FaceAxial[f] = axialDelta
		}
	}
	return t
}

// PrismCaps first proves cross-loop clearance, then maps one shared 2D
// triangulation onto both ends. A sheet has no caps and keeps the wall loop's
// minimum sample check.
func PrismCaps[F comparable](ctx context.Context, t *PrismTopology[F], sheet bool,
	capStart, capEnd F, z0Delta, z1Delta float64,
	clearance func(context.Context, []sectionrecord.Point2, [][]int, []float64) error,
	triangulate func(context.Context, []sectionrecord.Point2, [][]int) ([][3]int, error)) error {
	if !sheet {
		t.FaceTrim[capStart] = t.CapTrim
		t.FaceTrim[capEnd] = t.CapTrim
		t.FaceAxial[capStart] = z0Delta
		t.FaceAxial[capEnd] = z1Delta
	}
	if err := clearance(ctx, t.Points, t.LoopIdx, t.LoopSag); err != nil {
		return err
	}
	if sheet {
		if len(t.LoopIdx) == 0 || len(t.LoopIdx[0]) < 3 {
			return fmt.Errorf(`%w: a surface-result wall loop needs at least three boundary samples`, decaderr.ErrDegenerate)
		}
		return nil
	}
	capTris, err := triangulate(ctx, t.Points, t.LoopIdx)
	if err != nil {
		return err
	}
	for _, tri := range capTris {
		t.Triangles = append(t.Triangles,
			[3]int{2*tri[0] + 1, 2*tri[1] + 1, 2*tri[2] + 1},
			[3]int{2 * tri[0], 2 * tri[2], 2 * tri[1]})
		t.Sources = append(t.Sources, capEnd, capStart)
	}
	return nil
}
