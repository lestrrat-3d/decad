package loftmesh

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ChamferSplitEdge is one exact edge of the rational cap split ring.
type ChamferSplitEdge struct {
	A, B proofarith.Xpt
}

type chamferGridCell struct{ x, y int64 }

// ChamferEdgeGrid indexes split ring edges by XY cell so a near facet only
// meets the edges whose reach can overlap its cell.
type ChamferEdgeGrid struct {
	step  *big.Rat
	cells map[chamferGridCell][]int
	edges []ChamferSplitEdge
}

// floor(coord/step) is exact, including for negative coordinates. Very large
// indices refuse before they can overflow the bounded grid loops.
func chamferGridIndex(coord, step *big.Rat) (int64, bool) {
	if coord == nil || step == nil || step.Sign() <= 0 {
		return 0, false
	}
	q := new(big.Rat).Quo(coord, step)
	i := new(big.Int).Div(q.Num(), q.Denom())
	return i.Int64(), i.IsInt64()
}

// ChamferEdgeGridOf builds the cell grid of the split ring edges for a reach.
func ChamferEdgeGridOf(edges []ChamferSplitEdge,
	reach *big.Rat) (ChamferEdgeGrid, error) {
	if len(edges) == 0 || reach == nil || reach.Sign() <= 0 || len(edges) > 32_768 {
		return ChamferEdgeGrid{},
			fmt.Errorf("%w: cap split ring exceeds the spatial work limit", decaderr.ErrUnsupported)
	}
	grid := ChamferEdgeGrid{
		step:  new(big.Rat).Set(reach),
		cells: make(map[chamferGridCell][]int), edges: edges,
	}
	// Each source edge may occupy at most 128 grid cells on average. This
	// bounds memory without trusting an unbounded coordinate span.
	maxEntries := min(2_000_000, 128*len(edges))
	entries := 0
	for index, edge := range edges {
		var limits [2][2]int64
		for axis := range 2 {
			a, b := meshbool.RatCoordOf(edge.A, axis), meshbool.RatCoordOf(edge.B, axis)
			if a.Cmp(b) > 0 {
				a, b = b, a
			}
			lo := new(big.Rat).Sub(a, reach)
			hi := new(big.Rat).Add(b, reach)
			var ok bool
			limits[axis][0], ok = chamferGridIndex(lo, reach)
			if !ok {
				return ChamferEdgeGrid{},
					fmt.Errorf("%w: cap split ring grid index overflows", decaderr.ErrUnsupported)
			}
			limits[axis][1], ok = chamferGridIndex(hi, reach)
			if !ok {
				return ChamferEdgeGrid{},
					fmt.Errorf("%w: cap split ring grid index overflows", decaderr.ErrUnsupported)
			}
		}
		spanX := new(big.Int).Sub(big.NewInt(limits[0][1]), big.NewInt(limits[0][0]))
		spanY := new(big.Int).Sub(big.NewInt(limits[1][1]), big.NewInt(limits[1][0]))
		spanX.Add(spanX, big.NewInt(1))
		spanY.Add(spanY, big.NewInt(1))
		count := spanX.Mul(spanX, spanY)
		if count.Sign() <= 0 || count.Cmp(big.NewInt(int64(maxEntries-entries))) > 0 {
			return ChamferEdgeGrid{},
				fmt.Errorf("%w: cap split ring grid entries exceed %d", decaderr.ErrUnsupported, maxEntries)
		}
		entries += int(count.Int64())
		for x := limits[0][0]; ; x++ {
			for y := limits[1][0]; ; y++ {
				cell := chamferGridCell{x: x, y: y}
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

func (grid ChamferEdgeGrid) candidates(tri [3]proofarith.Xpt) []int {
	center := meshbool.XCentroid(tri[0], tri[1], tri[2])
	x, xOK := chamferGridIndex(meshbool.RatCoordOf(center, 0), grid.step)
	y, yOK := chamferGridIndex(meshbool.RatCoordOf(center, 1), grid.step)
	if !xOK || !yOK {
		return nil
	}
	return grid.cells[chamferGridCell{x: x, y: y}]
}

// ChamferNearPieces is the near part of one band facet and its far split edge.
// The near part of a two-level band facet is exactly a triangle or a
// quadrilateral. The far edge is part of the complete rational split ring.
func ChamferNearPieces(tri [3]int, xverts []proofarith.Xpt,
	capLevel *big.Rat, fraction *big.Rat) ([][3]proofarith.Xpt, ChamferSplitEdge, error) {
	var caps, sides []proofarith.Xpt
	for _, index := range tri {
		if index < 0 || index >= len(xverts) {
			return nil, ChamferSplitEdge{},
				fmt.Errorf("%w: cap band has an invalid near vertex", decaderr.ErrUnsupported)
		}
		p := xverts[index]
		if meshbool.RatCoordOf(p, 2).Cmp(capLevel) == 0 {
			caps = append(caps, p)
		} else {
			sides = append(sides, p)
		}
	}
	var near [][3]proofarith.Xpt
	var a, b proofarith.Xpt
	switch {
	case len(caps) == 1 && len(sides) == 2:
		a = proofarith.Xlerp(sides[0], caps[0], new(big.Int).Set(fraction.Num()), new(big.Int).Set(fraction.Denom()))
		b = proofarith.Xlerp(sides[1], caps[0], new(big.Int).Set(fraction.Num()), new(big.Int).Set(fraction.Denom()))
		near = [][3]proofarith.Xpt{{sides[0], sides[1], b}, {sides[0], b, a}}
	case len(caps) == 2 && len(sides) == 1:
		a = proofarith.Xlerp(sides[0], caps[0], new(big.Int).Set(fraction.Num()), new(big.Int).Set(fraction.Denom()))
		b = proofarith.Xlerp(sides[0], caps[1], new(big.Int).Set(fraction.Num()), new(big.Int).Set(fraction.Denom()))
		near = [][3]proofarith.Xpt{{sides[0], a, b}}
	default:
		return nil, ChamferSplitEdge{},
			fmt.Errorf("%w: cap band lacks two axial levels", decaderr.ErrUnsupported)
	}
	pa, pb := a.Vec(), b.Vec()
	if !proofbound.FiniteVec(pa) || !proofbound.FiniteVec(pb) {
		return nil, ChamferSplitEdge{},
			fmt.Errorf("%w: cap band split has nonfinite coordinates", decaderr.ErrUnsupported)
	}
	return near, ChamferSplitEdge{A: a, B: b}, nil
}

// ChamferFarTriangles is the far part of one band facet. The same split leaves
// one or two exact triangles between the ring and cap.
func ChamferFarTriangles(tri [3]int, xverts []proofarith.Xpt,
	capLevel *big.Rat, fraction *big.Rat) ([][3]proofarith.Xpt, error) {
	if fraction == nil || fraction.Sign() <= 0 || fraction.Cmp(big.NewRat(1, 1)) >= 0 {
		return nil, fmt.Errorf("%w: cap band has no interior split level", decaderr.ErrUnsupported)
	}
	var caps, sides []proofarith.Xpt
	for _, index := range tri {
		if index < 0 || index >= len(xverts) {
			return nil, fmt.Errorf("%w: cap band split has an invalid vertex", decaderr.ErrUnsupported)
		}
		p := xverts[index]
		if meshbool.RatCoordOf(p, 2).Cmp(capLevel) == 0 {
			caps = append(caps, p)
		} else {
			sides = append(sides, p)
		}
	}
	if len(caps) == 1 && len(sides) == 2 {
		a := proofarith.Xlerp(sides[0], caps[0], new(big.Int).Set(fraction.Num()), new(big.Int).Set(fraction.Denom()))
		b := proofarith.Xlerp(sides[1], caps[0], new(big.Int).Set(fraction.Num()), new(big.Int).Set(fraction.Denom()))
		return [][3]proofarith.Xpt{{caps[0], a, b}}, nil
	}
	if len(caps) == 2 && len(sides) == 1 {
		a := proofarith.Xlerp(sides[0], caps[0], new(big.Int).Set(fraction.Num()), new(big.Int).Set(fraction.Denom()))
		b := proofarith.Xlerp(sides[0], caps[1], new(big.Int).Set(fraction.Num()), new(big.Int).Set(fraction.Denom()))
		return [][3]proofarith.Xpt{{caps[0], caps[1], b}, {caps[0], b, a}}, nil
	}
	return nil, fmt.Errorf("%w: cap band split lacks two axial levels", decaderr.ErrUnsupported)
}

// Exact point-to-segment distance is convex in the point. If each vertex of
// one near triangle lies within reach of the same far split segment, every
// point in that triangle lies within reach of the far common boundary.
func chamferPointEdgeWithin(p proofarith.Xpt,
	edge ChamferSplitEdge, reach2 *big.Rat) bool {
	d := proofarith.Xsub(edge.B, edge.A)
	den := proofarith.XdotRat(d, d)
	if den.Sign() == 0 {
		return false
	}
	v := proofarith.Xsub(p, edge.A)
	t := new(big.Rat).Quo(proofarith.XdotRat(v, d), den)
	if t.Sign() < 0 {
		t.SetInt64(0)
	} else if t.Cmp(big.NewRat(1, 1)) > 0 {
		t.SetInt64(1)
	}
	nearest := proofarith.Xlerp(edge.A, edge.B, new(big.Int).Set(t.Num()), new(big.Int).Set(t.Denom()))
	gap := proofarith.Xsub(p, nearest)
	return proofarith.XdotRat(gap, gap).Cmp(reach2) < 0
}

// ChamferNearCovered reports whether the near triangle is within reach of the
// split ring, subdividing to the given depth within the step budget.
func ChamferNearCovered(ctx context.Context, tri [3]proofarith.Xpt,
	grid ChamferEdgeGrid, reach2 *big.Rat,
	depth int, steps *int, maxSteps int) bool {
	candidates := grid.candidates(tri)
	*steps += 1 + len(candidates)
	if *steps > maxSteps || ctx.Err() != nil {
		return false
	}
	for _, index := range candidates {
		edge := grid.edges[index]
		if chamferPointEdgeWithin(tri[0], edge, reach2) &&
			chamferPointEdgeWithin(tri[1], edge, reach2) &&
			chamferPointEdgeWithin(tri[2], edge, reach2) {
			return true
		}
	}
	if depth == 0 {
		return false
	}
	longest, length2 := 0, -1.0
	for i := range 3 {
		a, b := tri[i].Vec(), tri[(i+1)%3].Vec()
		dx, dy, dz := a.X-b.X, a.Y-b.Y, a.Z-b.Z
		if v := dx*dx + dy*dy + dz*dz; v > length2 {
			longest, length2 = i, v
		}
	}
	i, j, k := longest, (longest+1)%3, (longest+2)%3
	mid := proofarith.Xlerp(tri[i], tri[j], big.NewInt(1), big.NewInt(2))
	return ChamferNearCovered(ctx, [3]proofarith.Xpt{tri[i], mid, tri[k]},
		grid, reach2, depth-1, steps, maxSteps) &&
		ChamferNearCovered(ctx, [3]proofarith.Xpt{mid, tri[j], tri[k]},
			grid, reach2, depth-1, steps, maxSteps)
}

// ChamferBoxGrid indexes the exact float-held boxes of the original facets. A
// rational prism face is queried through its outward-rounded float box. XY
// cells only prune candidates; the caller still checks the full 3D boxes and
// classifies every remaining pair exactly.
type ChamferBoxGrid struct {
	step    *big.Rat
	cells   map[chamferGridCell][]int
	seen    []uint32
	epoch   uint32
	work    int
	maxWork int
	// log records every query that reaches the work accounting; only a fork
	// sets logging.
	log     []ChamferGridQuery
	logging bool
}

// ChamferGridQuery is one candidates call: its cell-range count (saturated
// at maxWork+1) and the visits it added, or Tripped when the call refused.
type ChamferGridQuery struct {
	Count, Visits int
	Tripped       bool
}

// Fork returns a grid sharing g's cells and step with its own seen/epoch
// scratch and a zero visit count that records each query.
func (grid *ChamferBoxGrid) Fork() *ChamferBoxGrid {
	return &ChamferBoxGrid{
		step: grid.step, cells: grid.cells, seen: make([]uint32, len(grid.seen)),
		maxWork: grid.maxWork, logging: true,
	}
}

// TakeLog returns and clears a fork's query log.
func (grid *ChamferBoxGrid) TakeLog() []ChamferGridQuery {
	log := grid.log
	grid.log = nil
	return log
}

// LimitWork lowers the visit ceiling to maxWork. It never raises it.
func (grid *ChamferBoxGrid) LimitWork(maxWork int) {
	grid.maxWork = min(grid.maxWork, maxWork)
}

// Replay applies one facet's log to g's own count. It reports false, and
// leaves the count unchanged, at the first query that g itself would refuse
// (Tripped, Count > maxWork-work, or work+Visits > maxWork).
func (grid *ChamferBoxGrid) Replay(log []ChamferGridQuery) bool {
	work := grid.work
	for _, q := range log {
		if q.Tripped || q.Count > grid.maxWork-work || work+q.Visits > grid.maxWork {
			return false
		}
		work += q.Visits
	}
	grid.work = work
	return true
}

func chamferBoxGridRange(box [2]r3.Vec,
	step *big.Rat) ([2][2]int64, *big.Int, error) {
	var limits [2][2]int64
	for axis := range 2 {
		lo, hi := box[0].X, box[1].X
		if axis == 1 {
			lo, hi = box[0].Y, box[1].Y
		}
		loRat, hiRat := proofarith.FloatRat(lo), proofarith.FloatRat(hi)
		if loRat == nil || hiRat == nil || loRat.Cmp(hiRat) > 0 {
			return limits, nil, fmt.Errorf("%w: cap collar has an invalid spatial box", decaderr.ErrUnsupported)
		}
		var ok bool
		limits[axis][0], ok = chamferGridIndex(loRat, step)
		if !ok {
			return limits, nil, fmt.Errorf("%w: cap collar grid index overflows", decaderr.ErrUnsupported)
		}
		limits[axis][1], ok = chamferGridIndex(hiRat, step)
		if !ok {
			return limits, nil, fmt.Errorf("%w: cap collar grid index overflows", decaderr.ErrUnsupported)
		}
	}
	spanX := new(big.Int).Sub(big.NewInt(limits[0][1]), big.NewInt(limits[0][0]))
	spanY := new(big.Int).Sub(big.NewInt(limits[1][1]), big.NewInt(limits[1][0]))
	spanX.Add(spanX, big.NewInt(1))
	spanY.Add(spanY, big.NewInt(1))
	area := spanX.Mul(spanX, spanY)
	if area.Sign() <= 0 {
		return limits, nil, fmt.Errorf("%w: cap collar grid has an empty range", decaderr.ErrUnsupported)
	}
	return limits, area, nil
}

// ChamferBoxGridOf builds the XY grid of the original facet boxes that
// intersect the open slab between z0 and z1.
func ChamferBoxGridOf(boxes [][2]r3.Vec, z0, z1 float64) (*ChamferBoxGrid, error) {
	if len(boxes) == 0 || len(boxes) > 32_768 ||
		proofarith.FloatRat(z0) == nil || proofarith.FloatRat(z1) == nil || z0 >= z1 {
		return nil, fmt.Errorf("%w: cap collar exceeds the facet work limit", decaderr.ErrUnsupported)
	}
	grid := &ChamferBoxGrid{
		step: big.NewRat(1, 2), cells: make(map[chamferGridCell][]int),
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
		limits, count, err := chamferBoxGridRange(box, grid.step)
		if err != nil {
			return nil, err
		}
		if count.Cmp(big.NewInt(int64(maxEntries-entries))) > 0 {
			return nil, fmt.Errorf("%w: cap collar grid entries exceed %d", decaderr.ErrUnsupported, maxEntries)
		}
		entries += int(count.Int64())
		for x := limits[0][0]; ; x++ {
			for y := limits[1][0]; ; y++ {
				cell := chamferGridCell{x: x, y: y}
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

func (grid *ChamferBoxGrid) candidates(box [2]r3.Vec) ([]int, error) {
	limits, count, err := chamferBoxGridRange(box, grid.step)
	if err != nil {
		return nil, err
	}
	if count.Cmp(big.NewInt(int64(grid.maxWork-grid.work))) > 0 {
		if grid.logging {
			grid.log = append(grid.log, ChamferGridQuery{Count: grid.maxWork + 1, Tripped: true})
		}
		return nil, fmt.Errorf("%w: cap collar grid visits exceed %d", decaderr.ErrUnsupported, grid.maxWork)
	}
	start := grid.work
	grid.epoch++
	if grid.epoch == 0 {
		clear(grid.seen)
		grid.epoch = 1
	}
	var candidates []int
	for x := limits[0][0]; ; x++ {
		for y := limits[1][0]; ; y++ {
			members := grid.cells[chamferGridCell{x: x, y: y}]
			grid.work += 1 + len(members)
			if grid.work > grid.maxWork {
				if grid.logging {
					grid.log = append(grid.log, ChamferGridQuery{
						Count: int(count.Int64()), Visits: grid.work - start, Tripped: true})
				}
				return nil, fmt.Errorf("%w: cap collar grid visits exceed %d", decaderr.ErrUnsupported, grid.maxWork)
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
	if grid.logging {
		grid.log = append(grid.log, ChamferGridQuery{Count: int(count.Int64()), Visits: grid.work - start})
	}
	return candidates, nil
}
