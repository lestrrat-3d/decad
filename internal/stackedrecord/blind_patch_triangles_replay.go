package stackedrecord

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/triangulation"
)

// ReplayBlindPatchTriangles precharges the complete scalar clearance,
// hole-bridge, ear-clip, and result-copy call graph for one authenticated
// patch's rings. The caller supplies rings from source-bound wall chording.
func ReplayBlindPatchTriangles(ctx context.Context,
	rings []tessellation.StackedRing[*topo.Face], loops []tessellation.StackedPatchLoop,
	reverse bool, budget *proofbound.ReplayBudget) (tessellation.StackedPatchMesh, error) {
	if budget == nil || len(loops) == 0 || len(loops) > 4096 {
		return tessellation.StackedPatchMesh{}, decaderr.ErrUnsupported
	}
	var samples uint64
	for _, loop := range loops {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return tessellation.StackedPatchMesh{}, err
		}
		if loop.Column < 0 || loop.Column >= len(rings) || len(rings[loop.Column].Samples) < 3 {
			return tessellation.StackedPatchMesh{}, decaderr.ErrUnsupported
		}
		ring := rings[loop.Column]
		if loop.Top && len(ring.Top) != len(ring.Samples) ||
			!loop.Top && len(ring.Bottom) != len(ring.Samples) {
			return tessellation.StackedPatchMesh{}, decaderr.ErrUnsupported
		}
		selected := ring.Bottom
		if loop.Top {
			selected = ring.Top
		}
		for i, point := range ring.Samples {
			if err := budget.SpendWork(ctx, 16); err != nil {
				return tessellation.StackedPatchMesh{}, err
			}
			if proofbound.IsNonFinite(point.U) || proofbound.IsNonFinite(point.V) || selected[i] < 0 {
				return tessellation.StackedPatchMesh{}, decaderr.ErrUnsupported
			}
		}
		samples += uint64(len(ring.Samples))
		if samples > 4096 {
			return tessellation.StackedPatchMesh{}, decaderr.ErrUnsupported
		}
	}
	// Each of the h hole splices adds two entries, so every merged polygon
	// has at most m entries. The record and byte reservations cover the
	// bridge copies, point/index lists, and triangle result. Ring gathering
	// and result copying are linear; clearance, bridges, and ear clipping
	// charge their reached scans separately under this same meter.
	m := samples + 2*uint64(len(loops)-1)
	if m > 4096 {
		return tessellation.StackedPatchMesh{}, decaderr.ErrUnsupported
	}
	work := 512*m + 128*uint64(len(loops))
	records := 16*m*m + 64
	bytes := 256*m*m + 8192
	if err := budget.ReserveBundle(ctx, work, records, bytes); err != nil {
		return tessellation.StackedPatchMesh{}, err
	}
	out, err := tessellation.StackedPatchTriangles(ctx, rings, loops, reverse,
		func(ctx context.Context, points []tessellation.SectionPoint, indices [][]int, sags []float64) error {
			// The floor visits every point; each distinct loop pair visits
			// exactly the product of its two chord counts. Reserve before the
			// ordinary clearance routine performs any of those scans.
			if err := budget.SpendWork(ctx, 64*uint64(len(indices))*uint64(len(indices))); err != nil {
				return err
			}
			cost := 64 * uint64(len(points))
			for i := range indices {
				for j := i + 1; j < len(indices); j++ {
					cost += 256*uint64(len(indices[i]))*uint64(len(indices[j])) + 64
				}
			}
			if err := budget.SpendWork(ctx, cost); err != nil {
				return err
			}
			return tessellation.RequireLoopClearance(ctx, points, indices, sags)
		},
		func(ctx context.Context, points []triangulation.Point2, indices [][]int) ([][3]int, error) {
			return triangulation.TriangulateReplay(ctx, points, indices, budget)
		})
	if err != nil {
		return tessellation.StackedPatchMesh{}, err
	}
	if err := ctx.Err(); err != nil {
		return tessellation.StackedPatchMesh{}, err
	}
	return out, nil
}
