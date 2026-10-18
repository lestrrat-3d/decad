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
	circle loftcylinder.SourceCircle
	bound2 *big.Rat
	// At a certified open circle height, root walls are strictly inside the
	// blank and cannot bound an edge wholly outside its radius.
	exterior []loftcylinder.RingWall
	// The fitted and tip cells exclude both source connector walls and the
	// hidden root wall. A seam segment checks connectors exactly instead.
	fitted []loftcylinder.RingWall
}

func loftCylinderRingToothOf(body *Body, radius float64) (loftCylinderRingTooth, error) {
	if body == nil {
		return loftCylinderRingTooth{}, fmt.Errorf("%w: ring tooth source is absent", ErrUnsupported)
	}
	lp, ok := body.n.Payload.(loftPayload)
	if !ok {
		return loftCylinderRingTooth{}, fmt.Errorf("%w: ring tooth has no Loft source", ErrUnsupported)
	}
	circle, err := loftcylinder.SourceCircleOf(lp, radius)
	if err != nil {
		return loftCylinderRingTooth{}, err
	}
	bound := lp.Proof.FacetDeparture
	if bound < 0 || proofbound.IsNonFinite(bound) {
		return loftCylinderRingTooth{}, fmt.Errorf("%w: ring tooth has no finite wall bound", ErrUnsupported)
	}
	exterior, fitted, ok := loftcylinder.RingToothWalls(lp, bound)
	if !ok {
		return loftCylinderRingTooth{}, fmt.Errorf("%w: ring tooth has no wall cell map", ErrUnsupported)
	}
	return loftCylinderRingTooth{
		loft: lp, circle: circle,
		bound2:   new(big.Rat).Mul(proofarith.FloatRat(bound), proofarith.FloatRat(bound)),
		exterior: exterior, fitted: fitted,
	}, nil
}

// loftCylinderRingToothKey names the inputs of loftCylinderRingToothOf.
type loftCylinderRingToothKey struct {
	body   *Body
	radius float64
}

// loftCylinderRingTeeth holds the tooth records one near-band proof has
// built. A record is a function of its body's payload and the radius
// alone, and every reader only reads it. Only successfully built records are
// kept, so a call that fails after building some teeth can leave those
// records in the map. The near-band proof returns that error and drops the
// map. A nil map keeps nothing.
type loftCylinderRingTeeth map[loftCylinderRingToothKey]loftCylinderRingTooth

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
	edges [][2]proofarith.Xpt, teeth loftCylinderRingTeeth,
	records *loftcylinder.RingEdgeRecords) (loftcylinder.RingOccupancy, error) {
	if !fp.ContactAudited || fp.Xform != r3.Identity() {
		return loftcylinder.RingOccupancy{},
			fmt.Errorf("%w: split ring has no audited source body", ErrUnsupported)
	}
	source, ok := loftCylinderChamferSourceOf(fp)
	if !ok {
		return loftcylinder.RingOccupancy{},
			fmt.Errorf("%w: split ring has no retained loft/cylinder source", ErrUnsupported)
	}
	bridgeBound, topSourceLevel, err := loftcylinder.ChamferBridgeSources(&loftcylinder.ChamferBridgeSourcesInput{
		ProofTeeth: bodyNodes(source.teeth), OriginalTeeth: bodyNodes(source.originalTeeth),
		AxialBridge: source.axialBridge, Z0: source.z0, Z1: source.z1, BridgeBound: source.bridgeBound,
	})
	if err != nil {
		return loftcylinder.RingOccupancy{}, err
	}
	source.bridgeBound, source.topSourceLevel = bridgeBound, topSourceLevel
	return loftCylinderSourceRingInsideInputs(ctx, source, edges, teeth, records)
}

func loftCylinderSourceRingInsideInputs(ctx context.Context, source loftCylinderChamferSource,
	edges [][2]proofarith.Xpt, teeth loftCylinderRingTeeth,
	records *loftcylinder.RingEdgeRecords) (loftcylinder.RingOccupancy, error) {
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
	toothRecords := make([]loftcylinder.RingToothLevel, len(source.teeth))
	for i, body := range source.teeth {
		key := loftCylinderRingToothKey{body: body, radius: source.blankRadius}
		tooth, cached := teeth[key]
		if !cached {
			var err error
			tooth, err = loftCylinderRingToothOf(body, source.blankRadius)
			if err != nil {
				return result, err
			}
			if teeth != nil {
				teeth[key] = tooth
			}
		}
		if tooth.circle.Z0.Cmp(proofarith.FloatRat(source.z0)) != 0 ||
			tooth.circle.Z1.Cmp(proofarith.FloatRat(source.z1)) != 0 {
			return refuse("ring tooth has different blank cap levels")
		}
		original, ok := source.originalTeeth[i].n.Payload.(loftPayload)
		if !ok {
			return refuse("ring has no authenticated original tooth")
		}
		scale := proofarith.FloatRat(original.Xform.Basis().EZ.Z)
		mappedLevel := new(big.Rat).Set(level)
		if len(source.ringLevelShift) != 0 {
			shift := source.ringLevelShift[i]
			if shift == nil || shift.Sign() < 0 {
				return refuse("ring has no finite physical tooth level")
			}
			mappedLevel.Sub(mappedLevel, shift)
		}
		mapped := new(big.Rat).Quo(mappedLevel, scale)
		if mapped.Cmp(tooth.circle.SafeBottom) <= 0 || mapped.Cmp(tooth.circle.SafeTop) >= 0 {
			return refuse("ring meets a source cap or root collar")
		}
		toothRecords[i] = loftcylinder.RingToothLevel{Circle: tooth.circle, Level: mapped,
			Bound2: tooth.bound2, Exterior: tooth.exterior, Fitted: tooth.fitted}
	}
	if records == nil {
		records = new(loftcylinder.RingEdgeRecords)
	}
	return loftcylinder.RingEdgesInsideShared(ctx, edges, toothRecords, source.blankRadius, source.boreRadius, records)
}
