package decad

import (
	"context"
	"fmt"
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
	points [3]loftCylinderRingVec
	box    loftCylinderRingBox
}

type loftCylinderRingTooth struct {
	loft   loftPayload
	circle loftCylinderSourceCircle
	bound2 *big.Rat
	walls  []loftCylinderRingWall
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
	result.walls = make([]loftCylinderRingWall, 0, lp.walls)
	result.fitted = make([]loftCylinderRingWall, 0, 2*(root-2))
	for cell := range n {
		for half := range 2 {
			indices := lp.tris[2*cell+half]
			tri := [3]r3.Vec{lp.verts[indices[0]], lp.verts[indices[1]], lp.verts[indices[2]]}
			wall := loftCylinderRingWall{box: loftCylinderRingTriangleBox(tri, bound)}
			for k, p := range tri {
				wall.points[k] = loftCylinderRingVertex(p)
			}
			result.walls = append(result.walls, wall)
			if cell > 0 && cell < root-1 {
				result.fitted = append(result.fitted, wall)
			}
		}
	}
	return result, nil
}

func loftCylinderRingWallClear(ctx context.Context, a, b loftCylinderRingVec,
	box loftCylinderRingBox, tooth loftCylinderRingTooth, walls []loftCylinderRingWall,
	result *loftCylinderRingOccupancy) error {
	for _, wall := range walls {
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
		q := loftCylinderRingSegmentTriangle2(a, b, wall.points)
		if q == nil || q.Cmp(tooth.bound2) <= 0 {
			return fmt.Errorf("%w: ring may meet a true Loft wall", ErrUnsupported)
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
	if record := fp.loftCylinderJoin; record != nil {
		for _, shift := range record.axialBridge {
			if shift > 0 {
				return loftCylinderRingOccupancy{},
					fmt.Errorf("%w: split ring has no source proof across an axial bridge", ErrUnsupported)
			}
		}
		return loftCylinderSourceRingInsideInputs(ctx, record.teeth,
			record.radius, 0, record.z0, record.z1, edges)
	}
	if record := fp.loftCylinderBore; record != nil {
		for _, shift := range record.axialBridge {
			if shift > 0 {
				return loftCylinderRingOccupancy{},
					fmt.Errorf("%w: split ring has no source proof across a bored axial bridge", ErrUnsupported)
			}
		}
		return loftCylinderSourceRingInsideInputs(ctx, record.teeth,
			record.blankRadius, record.boreRadius, record.z0, record.z1, edges)
	}
	return loftCylinderRingOccupancy{},
		fmt.Errorf("%w: split ring has no retained loft/cylinder source", ErrUnsupported)
}

func loftCylinderSourceRingInsideInputs(ctx context.Context, teeth []*Body,
	blankRadius, boreRadius, z0, z1 float64,
	edges [][2]proofarith.Xpt) (loftCylinderRingOccupancy, error) {
	var result loftCylinderRingOccupancy
	refuse := func(reason string) (loftCylinderRingOccupancy, error) {
		return result, fmt.Errorf("%w: %s", ErrUnsupported, reason)
	}
	if len(teeth) == 0 || blankRadius <= 0 || boreRadius < 0 || boreRadius >= blankRadius ||
		proofbound.IsNonFinite(blankRadius) || proofbound.IsNonFinite(boreRadius) || ctx.Err() != nil {
		if err := ctx.Err(); err != nil {
			return result, err
		}
		return refuse("ring lacks a certified blank and tooth source")
	}
	level, valid := loftCylinderRingCycle(edges)
	if !valid || level.Cmp(proofarith.FloatRat(z0)) <= 0 ||
		level.Cmp(proofarith.FloatRat(z1)) >= 0 {
		return refuse("ring is not one strict interior cycle")
	}
	toothRecords := make([]loftCylinderRingTooth, len(teeth))
	for i, body := range teeth {
		tooth, err := loftCylinderRingToothOf(body, blankRadius)
		if err != nil {
			return result, err
		}
		if tooth.circle.z0.Cmp(proofarith.FloatRat(z0)) != 0 ||
			tooth.circle.z1.Cmp(proofarith.FloatRat(z1)) != 0 {
			return refuse("ring tooth has different blank cap levels")
		}
		if level.Cmp(tooth.circle.safeBottom) <= 0 || level.Cmp(tooth.circle.safeTop) >= 0 {
			return refuse("ring meets a source cap or root collar")
		}
		toothRecords[i] = tooth
	}
	r2 := new(big.Rat).Mul(proofarith.FloatRat(blankRadius), proofarith.FloatRat(blankRadius))
	bore2 := new(big.Rat).Mul(proofarith.FloatRat(boreRadius), proofarith.FloatRat(boreRadius))
	witness := false
	for _, edge := range edges {
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
		box := loftCylinderRingEdgeBox(a, b)
		if min2.Cmp(r2) > 0 {
			result.outer++
			for _, tooth := range toothRecords {
				if err := loftCylinderRingWallClear(ctx, a, b, box, tooth, tooth.walls, &result); err != nil {
					return result, err
				}
			}
			continue
		}
		result.seam++
		sectorCount := 0
		for _, tooth := range toothRecords {
			if tooth.circle.ContainsTriangleSector(level, level,
				[3]proofarith.Xpt{edge[0], edge[1], edge[1]}) {
				sectorCount++
			}
			for _, connector := range tooth.circle.connector {
				t := new(big.Rat).Quo(new(big.Rat).Sub(level, tooth.circle.z0),
					new(big.Rat).Sub(tooth.circle.z1, tooth.circle.z0))
				inner := loftCylinderSourceLerp(connector.a[0], connector.a[1], t)
				middle := loftCylinderSourceLerp(connector.d[0], connector.d[1], t)
				outer := loftCylinderSourceLerp(connector.f[0], connector.f[1], t)
				if loftCylinderEdgesMeet(amin, bmin, inner, middle) ||
					loftCylinderEdgesMeet(amin, bmin, middle, outer) {
					return refuse("ring may touch a true connector wall")
				}
			}
			if err := loftCylinderRingWallClear(ctx, a, b, box, tooth, tooth.fitted, &result); err != nil {
				return result, err
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
