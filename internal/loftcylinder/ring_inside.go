package loftcylinder

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
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

// The source-ring audit supplies a true-result boundary point on every
// split edge. Exact distance to that ring then bounds every band facet.
func chamferNearBandReach(ctx context.Context, mesh *facetedrecord.JoinedMesh,
	fp facetedrecord.Payload, source ChamferSource, ideal []proofarith.Xpt, bands [2][2]int,
	clipBound, publishedBound float64, multipleRings bool, reflectedShift []*big.Rat) error {
	reach := math.Nextafter(publishedBound-clipBound, math.Inf(-1))
	if reach <= 0 || proofbound.IsNonFinite(reach) {
		return fmt.Errorf("%w: cap band has no boundary allowance", decaderr.ErrUnsupported)
	}
	reachRat := proofarith.FloatRat(reach)
	reach2 := new(big.Rat).Mul(reachRat, reachRat)
	fractions := []*big.Rat{big.NewRat(11, 20)}
	if multipleRings {
		fractions = []*big.Rat{big.NewRat(1, 4), big.NewRat(1, 2), big.NewRat(3, 4)}
	}
	teeth := make(chamferRingTeeth)
	for endCap, band := range bands {
		if band[0] == band[1] {
			continue
		}
		capLevel := proofarith.FloatRat(source.Z0)
		if endCap == 1 {
			capLevel = proofarith.FloatRat(source.Z1)
		}
		var near [][3]proofarith.Xpt
		var edges []loftmesh.ChamferSplitEdge
		for _, fraction := range fractions {
			var sourceEdges [][2]proofarith.Xpt
			for i := band[0]; i < band[1]; i++ {
				parts, edge, err := loftmesh.ChamferNearPieces(mesh.Tris[i], ideal, capLevel, fraction)
				if err != nil {
					return err
				}
				if !multipleRings {
					near = append(near, parts...)
					far, err := loftmesh.ChamferFarTriangles(mesh.Tris[i], ideal, capLevel, fraction)
					if err != nil {
						return err
					}
					near = append(near, far...)
				}
				edges = append(edges, edge)
				sourceEdges = append(sourceEdges, [2]proofarith.Xpt{edge.A, edge.B})
			}
			// The direct and the reflected check read the same edges.
			records := new(RingEdgeRecords)
			if _, err := chamferSourceRingInside(ctx, fp, sourceEdges, teeth, records); err != nil {
				return err
			}
			if len(reflectedShift) != 0 {
				shifted := source
				shifted.RingLevelShift = reflectedShift
				if _, err := chamferSourceRingInsideInputs(ctx, shifted, sourceEdges, teeth, records); err != nil {
					return fmt.Errorf("reflected cap split ring: %w", err)
				}
			}
		}
		if multipleRings {
			for i := band[0]; i < band[1]; i++ {
				tri := mesh.Tris[i]
				near = append(near, [3]proofarith.Xpt{ideal[tri[0]], ideal[tri[1]], ideal[tri[2]]})
			}
		}
		grid, err := loftmesh.ChamferEdgeGridOf(edges, reachRat)
		if err != nil {
			return err
		}
		maxSteps := min(8_000_000, 512*len(edges))
		depth := 10
		if multipleRings {
			maxSteps = min(8_000_000, 2_048*len(edges))
			depth = 12
		}
		steps := 0
		for i, tri := range near {
			if !loftmesh.ChamferNearCovered(ctx, tri, grid, reach2, depth, &steps, maxSteps) {
				var diagnostic [4]float64
				points := [4]proofarith.Xpt{tri[0], tri[1], tri[2],
					meshbool.XCentroid(tri[0], tri[1], tri[2])}
				for k, p := range points {
					diagnostic[k] = math.Inf(1)
					for _, edge := range edges {
						diagnostic[k] = min(diagnostic[k],
							meshbool.SegSegDist3(p.Vec(), p.Vec(), edge.A.Vec(), edge.B.Vec()))
					}
				}
				return fmt.Errorf("%w: near cap band %d facet %d has no certified split-ring reach after %d checks: distances=%v bound=%.17g",
					decaderr.ErrUnsupported, endCap, i, steps, diagnostic, publishedBound)
			}
		}
	}
	return nil
}

type chamferRingTooth struct {
	loft   loftrecord.Payload
	circle SourceCircle
	bound2 *big.Rat
	// At a certified open circle height, root walls are strictly inside the
	// blank and cannot bound an edge wholly outside its radius.
	exterior []RingWall
	// The fitted and tip cells exclude both source connector walls and the
	// hidden root wall. A seam segment checks connectors exactly instead.
	fitted []RingWall
}

func chamferRingToothOf(body *topo.Body, radius float64) (chamferRingTooth, error) {
	if body == nil {
		return chamferRingTooth{}, fmt.Errorf("%w: ring tooth source is absent", decaderr.ErrUnsupported)
	}
	lp, ok := body.Payload.(loftrecord.Payload)
	if !ok {
		return chamferRingTooth{}, fmt.Errorf("%w: ring tooth has no Loft source", decaderr.ErrUnsupported)
	}
	circle, err := SourceCircleOf(lp, radius)
	if err != nil {
		return chamferRingTooth{}, err
	}
	bound := lp.Proof.FacetDeparture
	if bound < 0 || proofbound.IsNonFinite(bound) {
		return chamferRingTooth{}, fmt.Errorf("%w: ring tooth has no finite wall bound", decaderr.ErrUnsupported)
	}
	exterior, fitted, ok := RingToothWalls(lp, bound)
	if !ok {
		return chamferRingTooth{}, fmt.Errorf("%w: ring tooth has no wall cell map", decaderr.ErrUnsupported)
	}
	return chamferRingTooth{
		loft: lp, circle: circle,
		bound2:   new(big.Rat).Mul(proofarith.FloatRat(bound), proofarith.FloatRat(bound)),
		exterior: exterior, fitted: fitted,
	}, nil
}

// chamferRingToothKey names the inputs of chamferRingToothOf.
type chamferRingToothKey struct {
	body   *topo.Body
	radius float64
}

// chamferRingTeeth holds the tooth records one near-band proof has
// built. A record is a function of its body's payload and the radius
// alone, and every reader only reads it. Only successfully built records are
// kept, so a call that fails after building some teeth can leave those
// records in the map. The near-band proof returns that error and drops the
// map. A nil map keeps nothing.
type chamferRingTeeth map[chamferRingToothKey]chamferRingTooth

// chamferSourceRingInside proves that an unordered exact split ring lies
// inside the original blank-and-teeth union at one strict interior height.
// Each edge avoids every true wall; one point inside the exact blank fixes
// occupancy of the connected cycle. Root walls lie inside the blank below
// each tooth's certified collar. Circle-crossing edges use the source circle
// sector and exact connector slices; all other exposed walls use the Loft's
// two-sided, parameter-matched wall displacement. A bore is excluded by its
// exact radius. The caller separately proves its cutter band's full surface,
// both cap planes, and either root collar.
func chamferSourceRingInside(ctx context.Context, fp facetedrecord.Payload,
	edges [][2]proofarith.Xpt, teeth chamferRingTeeth,
	records *RingEdgeRecords) (RingOccupancy, error) {
	if !fp.ContactAudited || fp.Xform != r3.Identity() {
		return RingOccupancy{},
			fmt.Errorf("%w: split ring has no audited source body", decaderr.ErrUnsupported)
	}
	source, ok := ChamferSourceOf(fp)
	if !ok {
		return RingOccupancy{},
			fmt.Errorf("%w: split ring has no retained loft/cylinder source", decaderr.ErrUnsupported)
	}
	if err := source.BindBridge(); err != nil {
		return RingOccupancy{}, err
	}
	return chamferSourceRingInsideInputs(ctx, source, edges, teeth, records)
}

func chamferSourceRingInsideInputs(ctx context.Context, source ChamferSource,
	edges [][2]proofarith.Xpt, teeth chamferRingTeeth,
	records *RingEdgeRecords) (RingOccupancy, error) {
	var result RingOccupancy
	refuse := func(reason string) (RingOccupancy, error) {
		return result, fmt.Errorf("%w: %s", decaderr.ErrUnsupported, reason)
	}
	if len(source.Teeth) == 0 || source.BlankRadius <= 0 || source.BoreRadius < 0 ||
		source.BoreRadius >= source.BlankRadius || proofbound.IsNonFinite(source.BlankRadius) ||
		proofbound.IsNonFinite(source.BoreRadius) ||
		len(source.RingLevelShift) != 0 && len(source.RingLevelShift) != len(source.Teeth) || ctx.Err() != nil {
		if err := ctx.Err(); err != nil {
			return result, err
		}
		return refuse("ring lacks a certified blank and tooth source")
	}
	level, valid := RingCycle(edges)
	if !valid || level.Cmp(proofarith.FloatRat(source.Z0)) <= 0 ||
		level.Cmp(source.TopSourceLevel) >= 0 {
		return refuse("ring is not one strict interior cycle")
	}
	toothRecords := make([]RingToothLevel, len(source.Teeth))
	for i, body := range source.Teeth {
		key := chamferRingToothKey{body: body, radius: source.BlankRadius}
		tooth, cached := teeth[key]
		if !cached {
			var err error
			tooth, err = chamferRingToothOf(body, source.BlankRadius)
			if err != nil {
				return result, err
			}
			if teeth != nil {
				teeth[key] = tooth
			}
		}
		if tooth.circle.Z0.Cmp(proofarith.FloatRat(source.Z0)) != 0 ||
			tooth.circle.Z1.Cmp(proofarith.FloatRat(source.Z1)) != 0 {
			return refuse("ring tooth has different blank cap levels")
		}
		original, ok := source.OriginalTeeth[i].Payload.(loftrecord.Payload)
		if !ok {
			return refuse("ring has no authenticated original tooth")
		}
		scale := proofarith.FloatRat(original.Xform.Basis().EZ.Z)
		mappedLevel := new(big.Rat).Set(level)
		if len(source.RingLevelShift) != 0 {
			shift := source.RingLevelShift[i]
			if shift == nil || shift.Sign() < 0 {
				return refuse("ring has no finite physical tooth level")
			}
			mappedLevel.Sub(mappedLevel, shift)
		}
		mapped := new(big.Rat).Quo(mappedLevel, scale)
		if mapped.Cmp(tooth.circle.SafeBottom) <= 0 || mapped.Cmp(tooth.circle.SafeTop) >= 0 {
			return refuse("ring meets a source cap or root collar")
		}
		toothRecords[i] = RingToothLevel{Circle: tooth.circle, Level: mapped,
			Bound2: tooth.bound2, Exterior: tooth.exterior, Fitted: tooth.fitted}
	}
	if records == nil {
		records = new(RingEdgeRecords)
	}
	return RingEdgesInsideShared(ctx, edges, toothRecords, source.BlankRadius, source.BoreRadius, records)
}
