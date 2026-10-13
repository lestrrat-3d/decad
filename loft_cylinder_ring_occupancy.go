package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type loftCylinderRingTooth struct {
	loft   loftPayload
	circle loftCylinderSourceCircle
	bound2 *big.Rat
	// At a certified open circle height, root walls are strictly inside the
	// blank and cannot bound an edge wholly outside its radius.
	exterior []loftcylinder.RingWall
	// The fitted and tip cells exclude both source connector walls and the
	// hidden root wall. A seam segment checks connectors exactly instead.
	fitted []loftcylinder.RingWall
}

func loftCylinderRingXY(p loftcylinder.RingVec) loftCylinderXY {
	return loftCylinderXY{x: p[0], y: p[1]}
}

func loftCylinderRingRadius2(p loftcylinder.RingVec) *big.Rat {
	return loftCylinderSourceDot(loftCylinderRingXY(p), loftCylinderRingXY(p))
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
	result.exterior = make([]loftcylinder.RingWall, 0, 2*root)
	result.fitted = make([]loftcylinder.RingWall, 0, 2*(root-2))
	for cell := range n {
		for half := range 2 {
			indices := lp.tris[2*cell+half]
			tri := [3]r3.Vec{lp.verts[indices[0]], lp.verts[indices[1]], lp.verts[indices[2]]}
			wall := loftcylinder.RingWall{Box: loftcylinder.RingTriangleBox(tri, bound)}
			for k, p := range tri {
				wall.Points[k] = loftcylinder.RingVertex(p)
			}
			wall.Normal = loftcylinder.RingCross(loftcylinder.RingSub(wall.Points[1], wall.Points[0]),
				loftcylinder.RingSub(wall.Points[2], wall.Points[0]))
			wall.Normal2 = loftcylinder.RingDot(wall.Normal, wall.Normal)
			wall.Plane0 = loftcylinder.RingDot(wall.Points[0], wall.Normal)
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
	edges [][2]proofarith.Xpt) (loftcylinder.RingOccupancy, error) {
	if !fp.contactAudited || fp.xform != r3.Identity() {
		return loftcylinder.RingOccupancy{},
			fmt.Errorf("%w: split ring has no audited source body", ErrUnsupported)
	}
	source, ok := loftCylinderChamferSourceOf(fp)
	if !ok {
		return loftcylinder.RingOccupancy{},
			fmt.Errorf("%w: split ring has no retained loft/cylinder source", ErrUnsupported)
	}
	if err := loftCylinderChamferBridgeSources(&source); err != nil {
		return loftcylinder.RingOccupancy{}, err
	}
	return loftCylinderSourceRingInsideInputs(ctx, source, edges)
}

func loftCylinderSourceRingInsideInputs(ctx context.Context, source loftCylinderChamferSource,
	edges [][2]proofarith.Xpt) (loftcylinder.RingOccupancy, error) {
	var result loftcylinder.RingOccupancy
	refuse := func(reason string) (loftcylinder.RingOccupancy, error) {
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
	level, valid := loftcylinder.RingCycle(edges)
	if !valid || level.Cmp(proofarith.FloatRat(source.z0)) <= 0 ||
		level.Cmp(source.topSourceLevel) >= 0 {
		return refuse("ring is not one strict interior cycle")
	}
	type ringTooth struct {
		loftCylinderRingTooth
		level *big.Rat
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
		a, b := loftcylinder.RingPoint(edge[0]), loftcylinder.RingPoint(edge[1])
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
			result.Inner++
			continue
		}
		if min2.Cmp(r2) > 0 {
			result.Outer++
			for toothIndex, tooth := range toothRecords {
				mappedA, mappedB := a, b
				mappedA[2], mappedB[2] = tooth.level, tooth.level
				if err := loftcylinder.RingWallClear(ctx, mappedA, mappedB,
					loftcylinder.RingEdgeBox(mappedA, mappedB), tooth.bound2,
					tooth.exterior, &result); err != nil {
					return result, fmt.Errorf("ring edge %d tooth %d from %v to %v at proxy z=%s: %w",
						edgeIndex, toothIndex, edge[0].Vec(), edge[1].Vec(), tooth.level.RatString(), err)
				}
			}
			continue
		}
		result.Seam++
		sectorCount := 0
		for toothIndex, tooth := range toothRecords {
			mappedEdge := [2]proofarith.Xpt{
				loftcylinder.RingMappedPoint(edge[0], tooth.level),
				loftcylinder.RingMappedPoint(edge[1], tooth.level),
			}
			if tooth.circle.ContainsTriangleSector(tooth.level, tooth.level,
				[3]proofarith.Xpt{mappedEdge[0], mappedEdge[1], mappedEdge[1]}) {
				sectorCount++
			}
			for _, connector := range tooth.circle.connector {
				t := new(big.Rat).Quo(new(big.Rat).Sub(tooth.level, tooth.circle.z0),
					new(big.Rat).Sub(tooth.circle.z1, tooth.circle.z0))
				inner := loftCylinderSourceLerp(connector.a[0], connector.a[1], t)
				middle := loftCylinderSourceLerp(connector.d[0], connector.d[1], t)
				outer := loftCylinderSourceLerp(connector.f[0], connector.f[1], t)
				if loftCylinderEdgesMeet(amin, bmin, inner, middle) ||
					loftCylinderEdgesMeet(amin, bmin, middle, outer) {
					return refuse("ring may touch a true connector wall")
				}
			}
			mappedA, mappedB := a, b
			mappedA[2], mappedB[2] = tooth.level, tooth.level
			if err := loftcylinder.RingWallClear(ctx, mappedA, mappedB,
				loftcylinder.RingEdgeBox(mappedA, mappedB), tooth.bound2,
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
