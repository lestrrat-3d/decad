package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type loftCylinderChamferSplitEdge struct {
	a, b proofarith.Xpt
}

type loftCylinderChamferGridCell struct{ x, y int64 }

type loftCylinderChamferEdgeGrid struct {
	step  *big.Rat
	cells map[loftCylinderChamferGridCell][]int
	edges []loftCylinderChamferSplitEdge
}

// floor(coord/step) is exact, including for negative coordinates. Very large
// indices refuse before they can overflow the bounded grid loops.
func loftCylinderChamferGridIndex(coord, step *big.Rat) (int64, bool) {
	if coord == nil || step == nil || step.Sign() <= 0 {
		return 0, false
	}
	q := new(big.Rat).Quo(coord, step)
	i := new(big.Int).Div(q.Num(), q.Denom())
	return i.Int64(), i.IsInt64()
}

func loftCylinderChamferEdgeGridOf(edges []loftCylinderChamferSplitEdge,
	reach *big.Rat) (loftCylinderChamferEdgeGrid, error) {
	if len(edges) == 0 || reach == nil || reach.Sign() <= 0 || len(edges) > 32_768 {
		return loftCylinderChamferEdgeGrid{},
			fmt.Errorf("%w: cap split ring exceeds the spatial work limit", ErrUnsupported)
	}
	grid := loftCylinderChamferEdgeGrid{
		step:  new(big.Rat).Set(reach),
		cells: make(map[loftCylinderChamferGridCell][]int), edges: edges,
	}
	// Each source edge may occupy at most 128 grid cells on average. This
	// bounds memory without trusting an unbounded coordinate span.
	maxEntries := min(2_000_000, 128*len(edges))
	entries := 0
	for index, edge := range edges {
		var limits [2][2]int64
		for axis := range 2 {
			a, b := meshbool.RatCoordOf(edge.a, axis), meshbool.RatCoordOf(edge.b, axis)
			if a.Cmp(b) > 0 {
				a, b = b, a
			}
			lo := new(big.Rat).Sub(a, reach)
			hi := new(big.Rat).Add(b, reach)
			var ok bool
			limits[axis][0], ok = loftCylinderChamferGridIndex(lo, reach)
			if !ok {
				return loftCylinderChamferEdgeGrid{},
					fmt.Errorf("%w: cap split ring grid index overflows", ErrUnsupported)
			}
			limits[axis][1], ok = loftCylinderChamferGridIndex(hi, reach)
			if !ok {
				return loftCylinderChamferEdgeGrid{},
					fmt.Errorf("%w: cap split ring grid index overflows", ErrUnsupported)
			}
		}
		spanX := new(big.Int).Sub(big.NewInt(limits[0][1]), big.NewInt(limits[0][0]))
		spanY := new(big.Int).Sub(big.NewInt(limits[1][1]), big.NewInt(limits[1][0]))
		spanX.Add(spanX, big.NewInt(1))
		spanY.Add(spanY, big.NewInt(1))
		count := spanX.Mul(spanX, spanY)
		if count.Sign() <= 0 || count.Cmp(big.NewInt(int64(maxEntries-entries))) > 0 {
			return loftCylinderChamferEdgeGrid{},
				fmt.Errorf("%w: cap split ring grid entries exceed %d", ErrUnsupported, maxEntries)
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

func (grid loftCylinderChamferEdgeGrid) candidates(tri [3]proofarith.Xpt) []int {
	center := meshbool.XCentroid(tri[0], tri[1], tri[2])
	x, xOK := loftCylinderChamferGridIndex(meshbool.RatCoordOf(center, 0), grid.step)
	y, yOK := loftCylinderChamferGridIndex(meshbool.RatCoordOf(center, 1), grid.step)
	if !xOK || !yOK {
		return nil
	}
	return grid.cells[loftCylinderChamferGridCell{x: x, y: y}]
}

// The near part of a two-level band facet is exactly a triangle or a
// quadrilateral. The far edge is part of the complete rational split ring.
func loftCylinderChamferNearPieces(tri [3]int, xverts []proofarith.Xpt,
	capLevel *big.Rat, fraction *big.Rat) ([][3]proofarith.Xpt, loftCylinderChamferSplitEdge, error) {
	var caps, sides []proofarith.Xpt
	for _, index := range tri {
		if index < 0 || index >= len(xverts) {
			return nil, loftCylinderChamferSplitEdge{},
				fmt.Errorf("%w: cap band has an invalid near vertex", ErrUnsupported)
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
		a = proofarith.Xlerp(sides[0], caps[0], fraction.Num(), fraction.Denom())
		b = proofarith.Xlerp(sides[1], caps[0], fraction.Num(), fraction.Denom())
		near = [][3]proofarith.Xpt{{sides[0], sides[1], b}, {sides[0], b, a}}
	case len(caps) == 2 && len(sides) == 1:
		a = proofarith.Xlerp(sides[0], caps[0], fraction.Num(), fraction.Denom())
		b = proofarith.Xlerp(sides[0], caps[1], fraction.Num(), fraction.Denom())
		near = [][3]proofarith.Xpt{{sides[0], a, b}}
	default:
		return nil, loftCylinderChamferSplitEdge{},
			fmt.Errorf("%w: cap band lacks two axial levels", ErrUnsupported)
	}
	pa, pb := a.Vec(), b.Vec()
	if !proofbound.FiniteVec(pa) || !proofbound.FiniteVec(pb) {
		return nil, loftCylinderChamferSplitEdge{},
			fmt.Errorf("%w: cap band split has nonfinite coordinates", ErrUnsupported)
	}
	return near, loftCylinderChamferSplitEdge{a: a, b: b}, nil
}

// The same split leaves one or two exact triangles between the ring and cap.
func loftCylinderChamferFarTriangles(tri [3]int, xverts []proofarith.Xpt,
	capLevel *big.Rat, fraction *big.Rat) ([][3]proofarith.Xpt, error) {
	if fraction == nil || fraction.Sign() <= 0 || fraction.Cmp(big.NewRat(1, 1)) >= 0 {
		return nil, fmt.Errorf("%w: cap band has no interior split level", ErrUnsupported)
	}
	var caps, sides []proofarith.Xpt
	for _, index := range tri {
		if index < 0 || index >= len(xverts) {
			return nil, fmt.Errorf("%w: cap band split has an invalid vertex", ErrUnsupported)
		}
		p := xverts[index]
		if meshbool.RatCoordOf(p, 2).Cmp(capLevel) == 0 {
			caps = append(caps, p)
		} else {
			sides = append(sides, p)
		}
	}
	if len(caps) == 1 && len(sides) == 2 {
		a := proofarith.Xlerp(sides[0], caps[0], fraction.Num(), fraction.Denom())
		b := proofarith.Xlerp(sides[1], caps[0], fraction.Num(), fraction.Denom())
		return [][3]proofarith.Xpt{{caps[0], a, b}}, nil
	}
	if len(caps) == 2 && len(sides) == 1 {
		a := proofarith.Xlerp(sides[0], caps[0], fraction.Num(), fraction.Denom())
		b := proofarith.Xlerp(sides[0], caps[1], fraction.Num(), fraction.Denom())
		return [][3]proofarith.Xpt{{caps[0], caps[1], b}, {caps[0], b, a}}, nil
	}
	return nil, fmt.Errorf("%w: cap band split lacks two axial levels", ErrUnsupported)
}

// Exact point-to-segment distance is convex in the point. If each vertex of
// one near triangle lies within reach of the same far split segment, every
// point in that triangle lies within reach of the far common boundary.
func loftCylinderChamferPointEdgeWithin(p proofarith.Xpt,
	edge loftCylinderChamferSplitEdge, reach2 *big.Rat) bool {
	d := proofarith.Xsub(edge.b, edge.a)
	den := proofarith.XdotRat(d, d)
	if den.Sign() == 0 {
		return false
	}
	v := proofarith.Xsub(p, edge.a)
	t := new(big.Rat).Quo(proofarith.XdotRat(v, d), den)
	if t.Sign() < 0 {
		t.SetInt64(0)
	} else if t.Cmp(big.NewRat(1, 1)) > 0 {
		t.SetInt64(1)
	}
	nearest := proofarith.Xlerp(edge.a, edge.b, t.Num(), t.Denom())
	gap := proofarith.Xsub(p, nearest)
	return proofarith.XdotRat(gap, gap).Cmp(reach2) < 0
}

func loftCylinderChamferNearCovered(ctx context.Context, tri [3]proofarith.Xpt,
	grid loftCylinderChamferEdgeGrid, reach2 *big.Rat,
	depth int, steps *int, maxSteps int) bool {
	candidates := grid.candidates(tri)
	*steps += 1 + len(candidates)
	if *steps > maxSteps || ctx.Err() != nil {
		return false
	}
	for _, index := range candidates {
		edge := grid.edges[index]
		if loftCylinderChamferPointEdgeWithin(tri[0], edge, reach2) &&
			loftCylinderChamferPointEdgeWithin(tri[1], edge, reach2) &&
			loftCylinderChamferPointEdgeWithin(tri[2], edge, reach2) {
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
	return loftCylinderChamferNearCovered(ctx, [3]proofarith.Xpt{tri[i], mid, tri[k]},
		grid, reach2, depth-1, steps, maxSteps) &&
		loftCylinderChamferNearCovered(ctx, [3]proofarith.Xpt{mid, tri[j], tri[k]},
			grid, reach2, depth-1, steps, maxSteps)
}

// The source-ring audit supplies a true-result boundary point on every
// split edge. Exact distance to that ring then bounds every band facet.
func loftCylinderChamferNearBandReach(ctx context.Context, mesh *bevelJoinedMesh,
	fp facetedPayload, source loftCylinderChamferSource, ideal []proofarith.Xpt, bands [2][2]int,
	clipBound, publishedBound float64) error {
	reach := math.Nextafter(publishedBound-clipBound, math.Inf(-1))
	if reach <= 0 || proofbound.IsNonFinite(reach) {
		return fmt.Errorf("%w: cap band has no boundary allowance", ErrUnsupported)
	}
	reachRat := proofarith.FloatRat(reach)
	reach2 := new(big.Rat).Mul(reachRat, reachRat)
	fraction := big.NewRat(11, 20)
	for cap, band := range bands {
		capLevel := proofarith.FloatRat(source.z0)
		if cap == 1 {
			capLevel = proofarith.FloatRat(source.z1)
		}
		var near [][3]proofarith.Xpt
		var edges []loftCylinderChamferSplitEdge
		var sourceEdges [][2]proofarith.Xpt
		for i := band[0]; i < band[1]; i++ {
			parts, edge, err := loftCylinderChamferNearPieces(mesh.tris[i], ideal, capLevel, fraction)
			if err != nil {
				return err
			}
			near = append(near, parts...)
			far, err := loftCylinderChamferFarTriangles(mesh.tris[i], ideal, capLevel, fraction)
			if err != nil {
				return err
			}
			near = append(near, far...)
			edges = append(edges, edge)
			sourceEdges = append(sourceEdges, [2]proofarith.Xpt{edge.a, edge.b})
		}
		if _, err := loftCylinderSourceRingInside(ctx, fp, sourceEdges); err != nil {
			return err
		}
		grid, err := loftCylinderChamferEdgeGridOf(edges, reachRat)
		if err != nil {
			return err
		}
		maxSteps := min(8_000_000, 512*len(edges))
		steps := 0
		for i, tri := range near {
			if !loftCylinderChamferNearCovered(ctx, tri, grid, reach2, 10, &steps, maxSteps) {
				var diagnostic [4]float64
				points := [4]proofarith.Xpt{tri[0], tri[1], tri[2],
					meshbool.XCentroid(tri[0], tri[1], tri[2])}
				for k, p := range points {
					diagnostic[k] = math.Inf(1)
					for _, edge := range edges {
						diagnostic[k] = min(diagnostic[k],
							meshbool.SegSegDist3(p.Vec(), p.Vec(), edge.a.Vec(), edge.b.Vec()))
					}
				}
				return fmt.Errorf("%w: near cap band %d facet %d has no certified split-ring reach after %d checks: distances=%v bound=%.17g",
					ErrUnsupported, cap, i, steps, diagnostic, publishedBound)
			}
		}
	}
	return nil
}
