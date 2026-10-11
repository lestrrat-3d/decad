package decad

import (
	"fmt"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// The grid indexes the exact float-held boxes of the original facets. A
// rational prism face is queried through its outward-rounded float box. XY
// cells only prune candidates; the caller still checks the full 3D boxes and
// classifies every remaining pair exactly.
type loftCylinderChamferBoxGrid struct {
	step    *big.Rat
	cells   map[loftCylinderChamferGridCell][]int
	seen    []uint32
	epoch   uint32
	work    int
	maxWork int
}

func loftCylinderChamferBoxGridRange(box [2]r3.Vec,
	step *big.Rat) ([2][2]int64, *big.Int, error) {
	var limits [2][2]int64
	for axis := range 2 {
		lo, hi := box[0].X, box[1].X
		if axis == 1 {
			lo, hi = box[0].Y, box[1].Y
		}
		loRat, hiRat := proofarith.FloatRat(lo), proofarith.FloatRat(hi)
		if loRat == nil || hiRat == nil || loRat.Cmp(hiRat) > 0 {
			return limits, nil, fmt.Errorf("%w: cap collar has an invalid spatial box", ErrUnsupported)
		}
		var ok bool
		limits[axis][0], ok = loftCylinderChamferGridIndex(loRat, step)
		if !ok {
			return limits, nil, fmt.Errorf("%w: cap collar grid index overflows", ErrUnsupported)
		}
		limits[axis][1], ok = loftCylinderChamferGridIndex(hiRat, step)
		if !ok {
			return limits, nil, fmt.Errorf("%w: cap collar grid index overflows", ErrUnsupported)
		}
	}
	spanX := new(big.Int).Sub(big.NewInt(limits[0][1]), big.NewInt(limits[0][0]))
	spanY := new(big.Int).Sub(big.NewInt(limits[1][1]), big.NewInt(limits[1][0]))
	spanX.Add(spanX, big.NewInt(1))
	spanY.Add(spanY, big.NewInt(1))
	area := spanX.Mul(spanX, spanY)
	if area.Sign() <= 0 {
		return limits, nil, fmt.Errorf("%w: cap collar grid has an empty range", ErrUnsupported)
	}
	return limits, area, nil
}

func loftCylinderChamferBoxGridOf(boxes [][2]r3.Vec, z0, z1 float64) (*loftCylinderChamferBoxGrid, error) {
	if len(boxes) == 0 || len(boxes) > 32_768 ||
		proofarith.FloatRat(z0) == nil || proofarith.FloatRat(z1) == nil || z0 >= z1 {
		return nil, fmt.Errorf("%w: cap collar exceeds the facet work limit", ErrUnsupported)
	}
	grid := &loftCylinderChamferBoxGrid{
		step: big.NewRat(1, 2), cells: make(map[loftCylinderChamferGridCell][]int),
		seen: make([]uint32, len(boxes)), maxWork: min(96_000_000, 12_288*len(boxes)),
	}
	maxEntries := min(2_000_000, 128*len(boxes))
	entries := 0
	for index, box := range boxes {
		// Every queried collar is strictly between these cap levels, so a
		// held cap facet cannot meet it and needs no XY grid entries.
		if box[1].Z <= z0 || box[0].Z >= z1 {
			continue
		}
		limits, count, err := loftCylinderChamferBoxGridRange(box, grid.step)
		if err != nil {
			return nil, err
		}
		if count.Cmp(big.NewInt(int64(maxEntries-entries))) > 0 {
			return nil, fmt.Errorf("%w: cap collar grid entries exceed %d", ErrUnsupported, maxEntries)
		}
		entries += int(count.Int64())
		for x := limits[0][0]; ; x++ {
			for y := limits[1][0]; ; y++ {
				cell := loftCylinderChamferGridCell{x: x, y: y}
				grid.cells[cell] = append(grid.cells[cell], index)
				if y == limits[1][1] {
					break
				}
			}
			if x == limits[0][1] {
				break
			}
		}
	}
	return grid, nil
}

func (grid *loftCylinderChamferBoxGrid) candidates(box [2]r3.Vec) ([]int, error) {
	limits, count, err := loftCylinderChamferBoxGridRange(box, grid.step)
	if err != nil {
		return nil, err
	}
	if count.Cmp(big.NewInt(int64(grid.maxWork-grid.work))) > 0 {
		return nil, fmt.Errorf("%w: cap collar grid visits exceed %d", ErrUnsupported, grid.maxWork)
	}
	grid.epoch++
	if grid.epoch == 0 {
		clear(grid.seen)
		grid.epoch = 1
	}
	var candidates []int
	for x := limits[0][0]; ; x++ {
		for y := limits[1][0]; ; y++ {
			members := grid.cells[loftCylinderChamferGridCell{x: x, y: y}]
			grid.work += 1 + len(members)
			if grid.work > grid.maxWork {
				return nil, fmt.Errorf("%w: cap collar grid visits exceed %d", ErrUnsupported, grid.maxWork)
			}
			for _, member := range members {
				if grid.seen[member] != grid.epoch {
					grid.seen[member] = grid.epoch
					candidates = append(candidates, member)
				}
			}
			if y == limits[1][1] {
				break
			}
		}
		if x == limits[0][1] {
			break
		}
	}
	return candidates, nil
}
