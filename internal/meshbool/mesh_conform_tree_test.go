package meshbool

import (
	"errors"
	"fmt"
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// conformHitsBeforeRejectOrder collects and sorts every tree candidate before
// applying the filter and exact predicate, as EdgeInteriorHits originally did.
func conformHitsBeforeRejectOrder(s *ConformScan, budget *proofbound.WorkBudget,
	a, b int, tri [3]int,
) ([]int, error) {
	pa, pb := s.Approx[a], s.Approx[b]
	filter := NewSegFilter(pa, pb, s.Tau2)
	tau := math.Nextafter(math.Sqrt(s.Tau2), math.Inf(1))
	box := [2]r3.Vec{
		{X: math.Nextafter(math.Min(pa.X, pb.X)-tau, math.Inf(-1)),
			Y: math.Nextafter(math.Min(pa.Y, pb.Y)-tau, math.Inf(-1)),
			Z: math.Nextafter(math.Min(pa.Z, pb.Z)-tau, math.Inf(-1))},
		{X: math.Nextafter(math.Max(pa.X, pb.X)+tau, math.Inf(1)),
			Y: math.Nextafter(math.Max(pa.Y, pb.Y)+tau, math.Inf(1)),
			Z: math.Nextafter(math.Max(pa.Z, pb.Z)+tau, math.Inf(1))},
	}
	var candidates []int
	if err := s.points.visitOverlaps(box, budget.Step, func(i int) error {
		if err := budget.Step(); err != nil {
			return err
		}
		candidates = append(candidates, i)
		return nil
	}); err != nil {
		return nil, err
	}
	sort.Slice(candidates, func(i, j int) bool {
		a, b := candidates[i], candidates[j]
		for axis := range 3 {
			if s.pointCells[a][axis] != s.pointCells[b][axis] {
				return s.pointCells[a][axis] < s.pointCells[b][axis]
			}
		}
		return a < b
	})
	var hits []int
	for _, i := range candidates {
		if i != tri[0] && i != tri[1] && i != tri[2] && !filter.TooFar(s.Approx[i]) &&
			OnSegmentInterior3(s.Verts[a], s.Verts[b], s.Verts[i]) {
			hits = append(hits, i)
		}
	}
	return hits, nil
}

func TestConformHitsPreserveOrderAndEveryBudgetStop(t *testing.T) {
	t.Parallel()
	stop := errors.New("the candidate budget stopped")
	for _, offset := range []float64{0, 1e12} {
		point := func(x, y, z float64) proof.Xpt {
			return proof.XptOf(r3.Vec{X: offset + x, Y: offset + y, Z: offset + z})
		}
		verts := []proof.Xpt{point(0, 0, 0), point(10, 10, 10), point(0, 10, 0)}
		for _, x := range []float64{9, 1, 5, 7, 3} {
			verts = append(verts, point(x, x, x))
		}
		for i := range 24 {
			x := float64(i) / 3
			verts = append(verts, point(x, x+0.1, x+0.2))
		}
		verts = append(verts, point(math.Nextafter(5, 6), 5, 5))
		scan, err := NewConformScan(proofbound.NewWorkBudget(t.Context()), verts)
		require.NoError(t, err)
		tri := [3]int{0, 1, 2}
		for _, edge := range [][2]int{{0, 1}, {1, 0}, {0, 2}} {
			run := func(legacy bool, stopAt int) ([]int, int, error) {
				steps := 0
				budget := &proofbound.WorkBudget{
					StepFn: func() error {
						steps++
						if steps == stopAt {
							return stop
						}
						return nil
					},
					ErrFn: t.Context().Err,
				}
				var hits []int
				var err error
				if legacy {
					hits, err = conformHitsBeforeRejectOrder(scan, budget, edge[0], edge[1], tri)
				} else {
					hits, err = scan.EdgeInteriorHits(budget, edge[0], edge[1], tri)
				}
				return hits, steps, err
			}
			_, total, err := run(true, 0)
			require.NoError(t, err)
			require.Positive(t, total)
			for limit := range total + 2 {
				want, wantSteps, wantErr := run(true, limit)
				got, gotSteps, gotErr := run(false, limit)
				require.Equal(t, wantSteps, gotSteps, "offset %g edge %v limit %d", offset, edge, limit)
				if wantErr != nil {
					require.ErrorIs(t, gotErr, stop)
				} else {
					require.NoError(t, gotErr)
				}
				require.Equal(t, want, got, "offset %g edge %v limit %d", offset, edge, limit)
			}
		}
	}
}

func TestConformScanTreeFindsEveryExactEdgeInteriorVertex(t *testing.T) {
	for _, offset := range []float64{0, 1e12} {
		t.Run(fmt.Sprintf("offset_%g", offset), func(t *testing.T) {
			point := func(x, y, z float64) proof.Xpt {
				return proof.XptOf(r3.Vec{X: offset + x, Y: offset + y, Z: offset + z})
			}
			verts := []proof.Xpt{
				point(0, 0, 0),
				point(10, 10, 10),
				point(0, 1, 0),
				point(5, 5, 5),
				point(5, 5, 6),
				point(8, 2, 2),
				point(9, 9, 9),
			}
			scan, err := NewConformScan(proofbound.NewWorkBudget(t.Context()), verts)
			require.NoError(t, err)

			tri := [3]int{0, 1, 2}
			got, err := scan.EdgeInteriorHits(proofbound.NewWorkBudget(t.Context()), 0, 1, tri)
			require.NoError(t, err)
			var want []int
			for i := range verts {
				if i == tri[0] || i == tri[1] || i == tri[2] {
					continue
				}
				if OnSegmentInterior3(verts[0], verts[1], verts[i]) {
					want = append(want, i)
				}
			}
			require.Equal(t, want, got)
		})
	}
}
