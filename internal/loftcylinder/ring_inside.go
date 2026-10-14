package loftcylinder

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

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
	return RingEdgesInsideShared(ctx, edges, teeth, blankRadius, boreRadius, new(RingEdgeRecords))
}

// RingEdgesInsideShared is RingEdgesInside with the per-edge records of
// records, which a second check of the same edges at other tooth levels can
// reuse. records must not be nil.
func RingEdgesInsideShared(ctx context.Context, edges [][2]proofarith.Xpt, teeth []RingToothLevel,
	blankRadius, boreRadius float64, records *RingEdgeRecords) (RingOccupancy, error) {
	var result RingOccupancy
	refuse := func(reason string) (RingOccupancy, error) {
		return result, fmt.Errorf("%w: %s", decaderr.ErrUnsupported, reason)
	}
	r2 := proofarith.MulRat(new(big.Rat), proofarith.FloatRat(blankRadius), proofarith.FloatRat(blankRadius))
	bore2 := proofarith.MulRat(new(big.Rat), proofarith.FloatRat(boreRadius), proofarith.FloatRat(boreRadius))
	witness := false
	// levelBoxes holds each tooth's outward-rounded Z interval, filled on the
	// first outer or seam edge. A level is the same at both ends of a mapped
	// edge.
	var levelBoxes [][2]float64
	fillLevelBoxes := func() {
		if levelBoxes != nil {
			return
		}
		levelBoxes = make([][2]float64, len(teeth))
		for i := range teeth {
			levelBoxes[i] = [2]float64{
				proofbound.RatFloatDown(teeth[i].Level), proofbound.RatFloatUp(teeth[i].Level),
			}
		}
	}
	// seams holds each tooth's level records, each built on its first seam edge.
	var seams []ringSeamLevel
	for edgeIndex, edge := range edges {
		if err := ctx.Err(); err != nil {
			return result, err
		}
		rec := records.recordOf(edgeIndex, edge, bore2, r2)
		amin, bmin := rec.amin, rec.bmin
		if rec.reachesBore {
			return refuse("ring may reach the centered bore")
		}
		if rec.witness {
			witness = true
		}
		switch rec.class {
		case ringEdgeInner:
			result.Inner++
			continue
		case ringEdgeOuter:
			result.Outer++
			fillLevelBoxes()
			// Axes 0 and 1 of the mapped edge are those of the edge itself.
			xy := rec.box()
			for toothIndex, tooth := range teeth {
				mappedA := RingVec{amin.X, amin.Y, tooth.Level}
				mappedB := RingVec{bmin.X, bmin.Y, tooth.Level}
				box := RingBox{xy[0], xy[1], levelBoxes[toothIndex]}
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
		fillLevelBoxes()
		xy := rec.box()
		if seams == nil {
			seams = make([]ringSeamLevel, len(teeth))
		}
		sectorCount := 0
		for toothIndex, tooth := range teeth {
			mappedEdge := [2]proofarith.Xpt{
				RingMappedPoint(edge[0], tooth.Level),
				RingMappedPoint(edge[1], tooth.Level),
			}
			seam := &seams[toothIndex]
			if !seam.built {
				*seam = ringSeamLevelOf(tooth)
			}
			if seam.sectorOK && triangleInSector(seam.sector, tooth.Level, tooth.Level,
				[3]proofarith.Xpt{mappedEdge[0], mappedEdge[1], mappedEdge[1]}) {
				sectorCount++
			}
			for _, slice := range seam.connectors {
				if EdgesMeet(amin, bmin, slice[0], slice[1]) ||
					EdgesMeet(amin, bmin, slice[1], slice[2]) {
					return refuse("ring may touch a true connector wall")
				}
			}
			mappedA := RingVec{amin.X, amin.Y, tooth.Level}
			mappedB := RingVec{bmin.X, bmin.Y, tooth.Level}
			if err := RingWallClear(ctx, mappedA, mappedB,
				RingBox{xy[0], xy[1], levelBoxes[toothIndex]}, tooth.Bound2,
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

// ringSeamLevel is what every seam edge reads of one tooth at its level:
// the source crossing boxes, and the inner, middle and outer points of each
// connector's slice. Each is a function of the tooth's circle and level.
type ringSeamLevel struct {
	built      bool
	sector     [2]XYBox
	sectorOK   bool
	connectors [2][3]XY
}

func ringSeamLevelOf(tooth RingToothLevel) ringSeamLevel {
	seam := ringSeamLevel{built: true}
	boxes, err := tooth.Circle.CrossingBox(tooth.Level, tooth.Level)
	seam.sector, seam.sectorOK = boxes, err == nil
	for i, connector := range tooth.Circle.Connector {
		t := new(big.Rat).Quo(proofarith.SubRat(new(big.Rat), tooth.Level, tooth.Circle.Z0),
			proofarith.SubRat(new(big.Rat), tooth.Circle.Z1, tooth.Circle.Z0))
		seam.connectors[i] = [3]XY{
			sourceLerp(connector.a[0], connector.a[1], t),
			sourceLerp(connector.d[0], connector.d[1], t),
			sourceLerp(connector.f[0], connector.f[1], t),
		}
	}
	return seam
}
