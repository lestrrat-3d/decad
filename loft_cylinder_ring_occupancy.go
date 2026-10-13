package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// loftCylinderRingOccupancy records only the exact checks completed for one
// connected, constant-height cutter ring. It does not certify the cutter's
// cap, collar, or entire band.
type loftCylinderRingOccupancy struct {
	inner, outer, seam, candidates int
	minimumClearance2              *big.Rat
}

type loftCylinderRingWall struct {
	points          [3]loftCylinderRingVec
	box             loftCylinderRingBox
	normal          loftCylinderRingVec
	normal2, plane0 *big.Rat
}

type loftCylinderRingTooth struct {
	loft   loftPayload
	circle loftCylinderSourceCircle
	bound2 *big.Rat
	// At a certified open circle height, root walls are strictly inside the
	// blank and cannot bound an edge wholly outside its radius.
	exterior []loftCylinderRingWall
	// The fitted and tip cells exclude both source connector walls and the
	// hidden root wall. A seam segment checks connectors exactly instead.
	fitted []loftCylinderRingWall
}

func loftCylinderRingValidPoint(p proofarith.Xpt) bool {
	return p.X != nil && p.Y != nil && p.Z != nil && p.W != nil && p.W.Sign() > 0
}

func loftCylinderRingXY(p loftCylinderRingVec) loftCylinderXY {
	return loftCylinderXY{x: p[0], y: p[1]}
}

func loftCylinderRingRadius2(p loftCylinderRingVec) *big.Rat {
	return loftCylinderSourceDot(loftCylinderRingXY(p), loftCylinderRingXY(p))
}

func loftCylinderRingCycle(edges [][2]proofarith.Xpt) (*big.Rat, bool) {
	if len(edges) < 3 || len(edges) > 8192 {
		return nil, false
	}
	adjacent := make(map[string][]string, len(edges))
	seen := make(map[string]struct{}, len(edges))
	var level *big.Rat
	for _, edge := range edges {
		if !loftCylinderRingValidPoint(edge[0]) || !loftCylinderRingValidPoint(edge[1]) {
			return nil, false
		}
		a, b := edge[0].Key(), edge[1].Key()
		if a == b {
			return nil, false
		}
		za := new(big.Rat).SetFrac(edge[0].Z, edge[0].W)
		zb := new(big.Rat).SetFrac(edge[1].Z, edge[1].W)
		if za.Cmp(zb) != 0 || level != nil && za.Cmp(level) != 0 {
			return nil, false
		}
		level = za
		if a > b {
			a, b = b, a
		}
		key := a + "|" + b
		if _, exists := seen[key]; exists {
			return nil, false
		}
		seen[key] = struct{}{}
		adjacent[a] = append(adjacent[a], b)
		adjacent[b] = append(adjacent[b], a)
	}
	if len(adjacent) != len(edges) {
		return nil, false
	}
	var start string
	for key, neighbors := range adjacent {
		if len(neighbors) != 2 {
			return nil, false
		}
		start = key
	}
	visited := map[string]bool{start: true}
	queue := []string{start}
	for len(queue) != 0 {
		key := queue[len(queue)-1]
		queue = queue[:len(queue)-1]
		for _, next := range adjacent[key] {
			if !visited[next] {
				visited[next] = true
				queue = append(queue, next)
			}
		}
	}
	return level, len(visited) == len(adjacent)
}

func loftCylinderRingToothOf(body *Body, radius float64) (loftCylinderRingTooth, error) {
	if body == nil {
		return loftCylinderRingTooth{}, fmt.Errorf("%w: ring tooth source is absent", ErrUnsupported)
	}
	lp, ok := body.payload.(loftPayload)
	if !ok {
		return loftCylinderRingTooth{}, fmt.Errorf("%w: ring tooth has no Loft source", ErrUnsupported)
	}
	circle, err := loftCylinderSourceCircleOf(lp, radius)
	if err != nil {
		return loftCylinderRingTooth{}, err
	}
	bound := lp.proof.FacetDeparture
	if bound < 0 || proofbound.IsNonFinite(bound) {
		return loftCylinderRingTooth{}, fmt.Errorf("%w: ring tooth has no finite wall bound", ErrUnsupported)
	}
	n := len(lp.verts) / 2
	root := -1
	for j, segment := range lp.segment[0] {
		if segment == 5 {
			root = j
			break
		}
	}
	if root < 3 || lp.walls != 2*n {
		return loftCylinderRingTooth{}, fmt.Errorf("%w: ring tooth has no wall cell map", ErrUnsupported)
	}
	result := loftCylinderRingTooth{
		loft: lp, circle: circle,
		bound2: new(big.Rat).Mul(proofarith.FloatRat(bound), proofarith.FloatRat(bound)),
	}
	result.exterior = make([]loftCylinderRingWall, 0, 2*root)
	result.fitted = make([]loftCylinderRingWall, 0, 2*(root-2))
	for cell := range n {
		for half := range 2 {
			indices := lp.tris[2*cell+half]
			tri := [3]r3.Vec{lp.verts[indices[0]], lp.verts[indices[1]], lp.verts[indices[2]]}
			wall := loftCylinderRingWall{box: loftCylinderRingTriangleBox(tri, bound)}
			for k, p := range tri {
				wall.points[k] = loftCylinderRingVertex(p)
			}
			wall.normal = loftCylinderRingCross(loftCylinderRingSub(wall.points[1], wall.points[0]),
				loftCylinderRingSub(wall.points[2], wall.points[0]))
			wall.normal2 = loftCylinderRingDot(wall.normal, wall.normal)
			wall.plane0 = loftCylinderRingDot(wall.points[0], wall.normal)
			if cell < root {
				result.exterior = append(result.exterior, wall)
			}
			if cell > 0 && cell < root-1 {
				result.fitted = append(result.fitted, wall)
			}
		}
	}
	return result, nil
}

// A same-side segment farther than bound from the entire triangle plane
// cannot meet that triangle or its parameter-matched true Loft wall. This
// only rejects pairs; unresolved pairs use the complete exact distance.
func loftCylinderRingPlaneApart(a, b loftCylinderRingVec,
	wall loftCylinderRingWall, bound2 *big.Rat) bool {
	if bound2 == nil || bound2.Sign() <= 0 || wall.normal2 == nil ||
		wall.normal2.Sign() == 0 || wall.plane0 == nil {
		return false
	}
	da := new(big.Rat).Sub(loftCylinderRingDot(a, wall.normal), wall.plane0)
	db := new(big.Rat).Sub(loftCylinderRingDot(b, wall.normal), wall.plane0)
	if da.Sign() == 0 || da.Sign() != db.Sign() {
		return false
	}
	if da.Sign() < 0 {
		da.Neg(da)
		db.Neg(db)
	}
	if db.Cmp(da) < 0 {
		da = db
	}
	return new(big.Rat).Mul(da, da).Cmp(new(big.Rat).Mul(bound2, wall.normal2)) > 0
}

func loftCylinderRingWallClear(ctx context.Context, a, b loftCylinderRingVec,
	box loftCylinderRingBox, tooth loftCylinderRingTooth, walls []loftCylinderRingWall,
	result *loftCylinderRingOccupancy) error {
	for wallIndex, wall := range walls {
		if !loftCylinderRingBoxesMeet(box, wall.box) {
			continue
		}
		result.candidates++
		if result.candidates > 1000000 || result.candidates%256 == 0 && ctx.Err() != nil {
			if err := ctx.Err(); err != nil {
				return err
			}
			return fmt.Errorf("%w: ring wall clearance work limit", ErrUnsupported)
		}
		if loftCylinderRingPlaneApart(a, b, wall, tooth.bound2) {
			continue
		}
		q := loftCylinderRingSegmentTriangle2(a, b, wall.points)
		if q == nil || q.Cmp(tooth.bound2) <= 0 {
			gap2 := math.NaN()
			if q != nil {
				gap2, _ = q.Float64()
			}
			bound2, _ := tooth.bound2.Float64()
			return fmt.Errorf("%w: ring may meet true Loft wall %d (gap²≈%g, bound²≈%g)",
				ErrUnsupported, wallIndex, gap2, bound2)
		}
		if result.minimumClearance2 == nil || q.Cmp(result.minimumClearance2) < 0 {
			result.minimumClearance2 = q
		}
	}
	return nil
}

// loftCylinderSourceRingInside proves that an unordered exact split ring lies
// inside the original blank-and-teeth union at one strict interior height.
// Each edge avoids every true wall; one point inside the exact blank fixes
// occupancy of the connected cycle. Root walls lie inside the blank below
// each tooth's certified collar. Circle-crossing edges use the source circle
// sector and exact connector slices; all other exposed walls use the Loft's
// two-sided, parameter-matched wall displacement. A bore is excluded by its
// exact radius. The caller separately proves its cutter band's full surface,
// both cap planes, and either root collar.
func loftCylinderSourceRingInside(ctx context.Context, fp facetedPayload,
	edges [][2]proofarith.Xpt) (loftCylinderRingOccupancy, error) {
	if !fp.contactAudited || fp.xform != r3.Identity() {
		return loftCylinderRingOccupancy{},
			fmt.Errorf("%w: split ring has no audited source body", ErrUnsupported)
	}
	source, ok := loftCylinderChamferSourceOf(fp)
	if !ok {
		return loftCylinderRingOccupancy{},
			fmt.Errorf("%w: split ring has no retained loft/cylinder source", ErrUnsupported)
	}
	if err := loftCylinderChamferBridgeSources(&source); err != nil {
		return loftCylinderRingOccupancy{}, err
	}
	return loftCylinderSourceRingInsideInputs(ctx, source, edges)
}

func loftCylinderSourceRingMappedPoint(p proofarith.Xpt, level *big.Rat) proofarith.Xpt {
	den := level.Denom()
	return proofarith.Xpt{
		X: new(big.Int).Mul(p.X, den), Y: new(big.Int).Mul(p.Y, den),
		Z: new(big.Int).Mul(p.W, level.Num()), W: new(big.Int).Mul(p.W, den),
	}
}

func loftCylinderSourceRingInsideInputs(ctx context.Context, source loftCylinderChamferSource,
	edges [][2]proofarith.Xpt) (loftCylinderRingOccupancy, error) {
	var result loftCylinderRingOccupancy
	refuse := func(reason string) (loftCylinderRingOccupancy, error) {
		return result, fmt.Errorf("%w: %s", ErrUnsupported, reason)
	}
	if len(source.teeth) == 0 || source.blankRadius <= 0 || source.boreRadius < 0 ||
		source.boreRadius >= source.blankRadius || proofbound.IsNonFinite(source.blankRadius) ||
		proofbound.IsNonFinite(source.boreRadius) ||
		len(source.ringLevelShift) != 0 && len(source.ringLevelShift) != len(source.teeth) || ctx.Err() != nil {
		if err := ctx.Err(); err != nil {
			return result, err
		}
		return refuse("ring lacks a certified blank and tooth source")
	}
	level, valid := loftCylinderRingCycle(edges)
	if !valid || level.Cmp(proofarith.FloatRat(source.z0)) <= 0 ||
		level.Cmp(source.topSourceLevel) >= 0 {
		return refuse("ring is not one strict interior cycle")
	}
	type ringTooth struct {
		loftCylinderRingTooth
		level       *big.Rat
		sectorBoxes [2]loftCylinderSourceXYBox
		sectorValid bool
		sectorReady bool
		connector   [2]loftCylinderSourceTimeSlice
	}
	toothRecords := make([]ringTooth, len(source.teeth))
	for i, body := range source.teeth {
		tooth, err := loftCylinderRingToothOf(body, source.blankRadius)
		if err != nil {
			return result, err
		}
		if tooth.circle.z0.Cmp(proofarith.FloatRat(source.z0)) != 0 ||
			tooth.circle.z1.Cmp(proofarith.FloatRat(source.z1)) != 0 {
			return refuse("ring tooth has different blank cap levels")
		}
		original, ok := source.originalTeeth[i].payload.(loftPayload)
		if !ok {
			return refuse("ring has no authenticated original tooth")
		}
		scale := proofarith.FloatRat(original.xform.Basis().EZ.Z)
		mappedLevel := new(big.Rat).Set(level)
		if len(source.ringLevelShift) != 0 {
			shift := source.ringLevelShift[i]
			if shift == nil || shift.Sign() < 0 {
				return refuse("ring has no finite physical tooth level")
			}
			mappedLevel.Sub(mappedLevel, shift)
		}
		mapped := new(big.Rat).Quo(mappedLevel, scale)
		if mapped.Cmp(tooth.circle.safeBottom) <= 0 || mapped.Cmp(tooth.circle.safeTop) >= 0 {
			return refuse("ring meets a source cap or root collar")
		}
		toothRecords[i] = ringTooth{loftCylinderRingTooth: tooth, level: mapped}
	}
	r2 := new(big.Rat).Mul(proofarith.FloatRat(source.blankRadius), proofarith.FloatRat(source.blankRadius))
	bore2 := new(big.Rat).Mul(proofarith.FloatRat(source.boreRadius), proofarith.FloatRat(source.boreRadius))
	witness := false
	for edgeIndex, edge := range edges {
		if err := ctx.Err(); err != nil {
			return result, err
		}
		a, b := loftCylinderRingPoint(edge[0]), loftCylinderRingPoint(edge[1])
		amin, bmin := loftCylinderRingXY(a), loftCylinderRingXY(b)
		min2 := loftCylinderSourceRadiusMin2(amin, bmin)
		if min2.Cmp(bore2) <= 0 {
			return refuse("ring may reach the centered bore")
		}
		ra, rb := loftCylinderRingRadius2(a), loftCylinderRingRadius2(b)
		if ra.Cmp(r2) < 0 || rb.Cmp(r2) < 0 {
			witness = true
		}
		max2 := ra
		if rb.Cmp(max2) > 0 {
			max2 = rb
		}
		if max2.Cmp(r2) < 0 {
			result.inner++
			continue
		}
		if min2.Cmp(r2) > 0 {
			result.outer++
			for toothIndex, tooth := range toothRecords {
				mappedA, mappedB := a, b
				mappedA[2], mappedB[2] = tooth.level, tooth.level
				if err := loftCylinderRingWallClear(ctx, mappedA, mappedB,
					loftCylinderRingEdgeBox(mappedA, mappedB), tooth.loftCylinderRingTooth,
					tooth.exterior, &result); err != nil {
					return result, fmt.Errorf("ring edge %d tooth %d from %v to %v at proxy z=%s: %w",
						edgeIndex, toothIndex, edge[0].Vec(), edge[1].Vec(), tooth.level.RatString(), err)
				}
			}
			continue
		}
		result.seam++
		sectorCount := 0
		for toothIndex := range toothRecords {
			tooth := &toothRecords[toothIndex]
			if !tooth.sectorReady {
				boxes, sectorErr := tooth.circle.CrossingBox(tooth.level, tooth.level)
				tooth.sectorBoxes, tooth.sectorValid = boxes, sectorErr == nil
				t := new(big.Rat).Quo(new(big.Rat).Sub(tooth.level, tooth.circle.z0),
					new(big.Rat).Sub(tooth.circle.z1, tooth.circle.z0))
				for i, connector := range tooth.circle.connector {
					tooth.connector[i] = connector.timeSlice(t)
				}
				tooth.sectorReady = true
			}
			mappedEdge := [2]proofarith.Xpt{
				loftCylinderSourceRingMappedPoint(edge[0], tooth.level),
				loftCylinderSourceRingMappedPoint(edge[1], tooth.level),
			}
			if tooth.sectorValid && loftCylinderSourceTriangleInSector(tooth.sectorBoxes, tooth.level, tooth.level,
				[3]proofarith.Xpt{mappedEdge[0], mappedEdge[1], mappedEdge[1]}) {
				sectorCount++
			}
			for _, slice := range tooth.connector {
				inner, middle, outer := slice.a, slice.d, slice.f
				if loftCylinderEdgesMeet(amin, bmin, inner, middle) ||
					loftCylinderEdgesMeet(amin, bmin, middle, outer) {
					return refuse("ring may touch a true connector wall")
				}
			}
			mappedA, mappedB := a, b
			mappedA[2], mappedB[2] = tooth.level, tooth.level
			if err := loftCylinderRingWallClear(ctx, mappedA, mappedB,
				loftCylinderRingEdgeBox(mappedA, mappedB), tooth.loftCylinderRingTooth,
				tooth.fitted, &result); err != nil {
				return result, fmt.Errorf("ring seam edge %d tooth %d from %v to %v at proxy z=%s: %w",
					edgeIndex, toothIndex, edge[0].Vec(), edge[1].Vec(), tooth.level.RatString(), err)
			}
		}
		if sectorCount != 1 {
			return refuse("ring circle crossing has no unique occupied tooth sector")
		}
	}
	if !witness {
		return refuse("ring has no strict blank-inside witness")
	}
	return result, nil
}
