package loftmesh

import (
	"errors"
	"math"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// FALSIFICATION LOG for TestGridScanMatchesCellReference. Each leg of the
// column grid in loft_audit_grid.go was broken, this test re-run, and the
// change reverted; each went RED:
//
//   - the column break `eq.zLo > ep.zHi` changed to `>=`: pairs sharing one z
//     cell went missing.
//   - the work term without `+ 1`: work differed.
//   - the column check removed (report from every column): pairs were
//     reported more than once.
//   - the Step loop run `zHi - eb.zLo` times: scanned and Step counts differed.
//   - the sort by column then member order only (no zLo order): the break
//     skipped pairs.

// cellGridReference is the grid that registers every box in every cell its
// extent meets, kept as the reference the column grid is compared with.
type cellGridReference struct {
	grid          gridScan // sizing only: lo, size, n, boxes
	keys          []int64
	cells         [][]int32
	registrations uint64
	work          uint64
}

func newCellGridReference(boxes [][2]r3.Vec, members []int) cellGridReference {
	g, ok := newGridScan(boxes, members, math.MaxUint64)
	if !ok {
		panic("reference fixture must build a grid")
	}
	r := cellGridReference{grid: g}
	byKey := map[int64][]int32{}
	for _, i := range members {
		lo, hi := g.cellRange(boxes[i])
		for x := lo[0]; x <= hi[0]; x++ {
			for y := lo[1]; y <= hi[1]; y++ {
				for z := lo[2]; z <= hi[2]; z++ {
					key := g.key([3]int{x, y, z})
					byKey[key] = append(byKey[key], int32(i))
					r.registrations++
				}
			}
		}
	}
	for key := range byKey {
		r.keys = append(r.keys, key)
	}
	slices.Sort(r.keys)
	r.work = r.registrations
	for _, key := range r.keys {
		cell := byKey[key]
		r.cells = append(r.cells, cell)
		m := uint64(len(cell))
		r.work += m * (m - 1) / 2
	}
	return r
}

func (r cellGridReference) visit(budget *proofbound.WorkBudget, limit uint64, fn func(i, j int)) (uint64, bool, error) {
	g := r.grid
	scanned := r.registrations
	if scanned > limit {
		return scanned, true, nil
	}
	for k, cell := range r.cells {
		key := r.keys[k]
		for a, ia := range cell {
			boxA := g.boxes[ia]
			for _, ib := range cell[a+1:] {
				if err := budget.Step(); err != nil {
					return scanned, false, err
				}
				scanned++
				if scanned > limit {
					return scanned, true, nil
				}
				boxB := g.boxes[ib]
				if !meshbool.BoxesOverlap(boxA, boxB) {
					continue
				}
				var c [3]int
				for axis := range 3 {
					c[axis] = g.index(axis, math.Max(vecAxis(boxA[0], axis), vecAxis(boxB[0], axis)))
				}
				if g.key(c) != key {
					continue
				}
				i, j := int(ia), int(ib)
				if i > j {
					i, j = j, i
				}
				fn(i, j)
			}
		}
	}
	return scanned, false, nil
}

type gridVisitResult struct {
	scanned  uint64
	exceeded bool
	err      error
	steps    int
	pairs    map[[2]int]int
}

func runGridVisit(visit func(*proofbound.WorkBudget, uint64, func(i, j int)) (uint64, bool, error), limit uint64, failAt int) gridVisitResult {
	errStop := errors.New("stop")
	res := gridVisitResult{pairs: map[[2]int]int{}}
	budget := &proofbound.WorkBudget{StepFn: func() error {
		res.steps++
		if failAt > 0 && res.steps == failAt {
			return errStop
		}
		return nil
	}, ErrFn: func() error { return nil }}
	res.scanned, res.exceeded, res.err = visit(budget, limit, func(i, j int) { res.pairs[[2]int{i, j}]++ })
	return res
}

func TestGridScanMatchesCellReference(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x91d, 0x2e7))
	var small [][2]r3.Vec
	for i := range 300 {
		size := 0.05
		if i%50 == 0 {
			size = 1
		}
		lo := r3.NewVec(rng.Float64()*10, rng.Float64()*10, rng.Float64()*10)
		small = append(small, gridBox(lo, lo.Add(r3.NewVec(rng.Float64()*size, rng.Float64()*size, rng.Float64()*size))))
	}
	fixtures := []gridFixture{{"mixed sizes, small", small}}
	for _, fx := range gridFixtures() {
		if fx.name != "mixed sizes" {
			fixtures = append(fixtures, fx)
		}
	}
	for _, fx := range fixtures {
		t.Run(fx.name, func(t *testing.T) {
			all := make([]int, len(fx.boxes))
			for i := range all {
				all[i] = i
			}
			var some []int
			for i := range fx.boxes {
				if i%7 != 3 {
					some = append(some, i)
				}
			}
			reversed := slices.Clone(all)
			slices.Reverse(reversed)
			for _, members := range [][]int{all, some, reversed} {
				g, ok := newGridScan(fx.boxes, members, math.MaxUint64)
				require.True(t, ok)
				ref := newCellGridReference(fx.boxes, members)
				require.Equal(t, ref.registrations, g.registrations)
				require.Equal(t, ref.work, g.work)
				full := runGridVisit(ref.visit, math.MaxUint64, 0)
				require.Equal(t, full, runGridVisit(g.visit, math.MaxUint64, 0))
				require.Equal(t, ref.work, full.scanned)
				units := int(ref.work - ref.registrations)
				for _, limit := range []uint64{0, ref.registrations - 1, ref.registrations, ref.registrations + 1,
					(ref.registrations + ref.work) / 2, ref.work - 1, ref.work} {
					want, got := runGridVisit(ref.visit, limit, 0), runGridVisit(g.visit, limit, 0)
					require.Equal(t, want.scanned, got.scanned, "limit %d", limit)
					require.Equal(t, want.exceeded, got.exceeded, "limit %d", limit)
					require.Equal(t, want.steps, got.steps, "limit %d", limit)
				}
				for _, failAt := range []int{1, units / 2, units} {
					if failAt < 1 {
						continue
					}
					want, got := runGridVisit(ref.visit, math.MaxUint64, failAt), runGridVisit(g.visit, math.MaxUint64, failAt)
					require.Equal(t, want.scanned, got.scanned, "fail at %d", failAt)
					require.Equal(t, want.err, got.err, "fail at %d", failAt)
					require.Equal(t, want.steps, got.steps, "fail at %d", failAt)
				}
			}
		})
	}
}
