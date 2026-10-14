package loftcylinder

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func ringXY(p RingVec) XY {
	return XY{X: p[0], Y: p[1]}
}

func ringRadius2(p RingVec) *big.Rat {
	return SourceDot(ringXY(p), ringXY(p))
}

// RingToothLevel is one tooth's circle record and walls at its mapped ring height.
type RingToothLevel struct {
	Circle   SourceCircle
	Level    *big.Rat
	Bound2   *big.Rat
	Exterior []RingWall
	Fitted   []RingWall
}

// RingEdgesInside proves that an unordered exact split ring lies
// inside the original blank-and-teeth union at one strict interior height.
// Each edge avoids every true wall; one point inside the exact blank fixes
// occupancy of the connected cycle. Root walls lie inside the blank below
// each tooth's certified collar. Circle-crossing edges use the source circle
// sector and exact connector slices; all other exposed walls use the Loft's
// two-sided, parameter-matched wall displacement. A bore is excluded by its
// exact radius. The caller separately proves its cutter band's full surface,
// both cap planes, and either root collar.
func RingEdgesInside(ctx context.Context, edges [][2]proofarith.Xpt, teeth []RingToothLevel,
	blankRadius, boreRadius float64) (RingOccupancy, error) {
	var result RingOccupancy
	refuse := func(reason string) (RingOccupancy, error) {
		return result, fmt.Errorf("%w: %s", decaderr.ErrUnsupported, reason)
	}
	r2 := proofarith.MulRat(new(big.Rat), proofarith.FloatRat(blankRadius), proofarith.FloatRat(blankRadius))
	bore2 := proofarith.MulRat(new(big.Rat), proofarith.FloatRat(boreRadius), proofarith.FloatRat(boreRadius))
	witness := false
	// levelBoxes holds each tooth's outward-rounded Z interval, filled on the
	// first outer edge. A level is the same at both ends of a mapped edge.
	var levelBoxes [][2]float64
	for edgeIndex, edge := range edges {
		if err := ctx.Err(); err != nil {
			return result, err
		}
		a, b := RingPoint(edge[0]), RingPoint(edge[1])
		amin, bmin := ringXY(a), ringXY(b)
		min2 := sourceRadiusMin2(amin, bmin)
		if min2.Cmp(bore2) <= 0 {
			return refuse("ring may reach the centered bore")
		}
		ra, rb := ringRadius2(a), ringRadius2(b)
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
			if levelBoxes == nil {
				levelBoxes = make([][2]float64, len(teeth))
				for i := range teeth {
					levelBoxes[i] = [2]float64{
						proofbound.RatFloatDown(teeth[i].Level), proofbound.RatFloatUp(teeth[i].Level),
					}
				}
			}
			// Axes 0 and 1 of the mapped edge are those of the edge itself.
			xy := RingEdgeBox(a, b)
			for toothIndex, tooth := range teeth {
				mappedA, mappedB := a, b
				mappedA[2], mappedB[2] = tooth.Level, tooth.Level
				box := xy
				box[2] = levelBoxes[toothIndex]
				if err := RingWallClear(ctx, mappedA, mappedB,
					box, tooth.Bound2,
					tooth.Exterior, &result); err != nil {
					return result, fmt.Errorf("ring edge %d tooth %d from %v to %v at proxy z=%s: %w",
						edgeIndex, toothIndex, edge[0].Vec(), edge[1].Vec(), tooth.Level.RatString(), err)
				}
			}
			continue
		}
		result.Seam++
		sectorCount := 0
		for toothIndex, tooth := range teeth {
			mappedEdge := [2]proofarith.Xpt{
				RingMappedPoint(edge[0], tooth.Level),
				RingMappedPoint(edge[1], tooth.Level),
			}
			if tooth.Circle.ContainsTriangleSector(tooth.Level, tooth.Level,
				[3]proofarith.Xpt{mappedEdge[0], mappedEdge[1], mappedEdge[1]}) {
				sectorCount++
			}
			for _, connector := range tooth.Circle.Connector {
				t := new(big.Rat).Quo(proofarith.SubRat(new(big.Rat), tooth.Level, tooth.Circle.Z0),
					proofarith.SubRat(new(big.Rat), tooth.Circle.Z1, tooth.Circle.Z0))
				inner := sourceLerp(connector.a[0], connector.a[1], t)
				middle := sourceLerp(connector.d[0], connector.d[1], t)
				outer := sourceLerp(connector.f[0], connector.f[1], t)
				if EdgesMeet(amin, bmin, inner, middle) ||
					EdgesMeet(amin, bmin, middle, outer) {
					return refuse("ring may touch a true connector wall")
				}
			}
			mappedA, mappedB := a, b
			mappedA[2], mappedB[2] = tooth.Level, tooth.Level
			if err := RingWallClear(ctx, mappedA, mappedB,
				RingEdgeBox(mappedA, mappedB), tooth.Bound2,
				tooth.Fitted, &result); err != nil {
				return result, fmt.Errorf("ring seam edge %d tooth %d from %v to %v at proxy z=%s: %w",
					edgeIndex, toothIndex, edge[0].Vec(), edge[1].Vec(), tooth.Level.RatString(), err)
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
