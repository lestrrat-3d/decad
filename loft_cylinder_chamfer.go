package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// loftCylinderChamferSource is the already audited cap and side correspondence
// of an axial loft joined to an exact circular blank. The cap loops are CCW
// when viewed from +Z, including the start cap whose public coedges are CW.
type loftCylinderChamferSource struct {
	blank          *Body
	teeth          []*Body
	originalTeeth  []*Body
	axialBridge    []float64
	bridgeBound    float64
	topSourceLevel *big.Rat
	// Optional physical-height correction for a reflected, axially bridged
	// source tooth. The ring's shared held level stays unchanged.
	ringLevelShift []*big.Rat
	outer, inner   [2][]int
	capGroups      [2]int
	boreGroup      int
	blankRadius    float64
	boreRadius     float64
	boreBound      float64
	innerInset     float64
	outerExcess    float64
	rootCollar     float64
	z0, z1         float64
}

func loftCylinderChamferSourceOf(fp facetedPayload) (loftCylinderChamferSource, bool) {
	if !fp.ContactAudited || fp.Xform != r3.Identity() {
		return loftCylinderChamferSource{}, false
	}
	if r := fp.LoftCylinderBore; r != nil {
		return loftCylinderChamferSource{
			blank: bodyHandle(r.Blank), teeth: bodyHandles(r.ProofTeeth),
			originalTeeth: bodyHandles(r.Teeth), axialBridge: r.AxialBridge,
			outer: r.Outer, inner: r.Inner, capGroups: r.CapGroups,
			boreGroup: r.WallGroup, blankRadius: r.BlankRadius,
			boreRadius: r.BoreRadius, boreBound: r.BoreBound, innerInset: r.InnerInset,
			outerExcess: r.OuterExcess, rootCollar: r.RootCollar,
			z0: r.Z0, z1: r.Z1,
		}, true
	}
	if r := fp.LoftCylinderJoin; r != nil {
		return loftCylinderChamferSource{
			blank: bodyHandle(r.Blank), teeth: bodyHandles(r.ProofTeeth),
			originalTeeth: bodyHandles(r.Teeth), axialBridge: r.AxialBridge,
			outer: r.CapOutlines, capGroups: r.CapGroups,
			boreGroup: -1, blankRadius: r.Radius,
			innerInset: r.InnerInset, outerExcess: r.OuterExcess,
			rootCollar: r.RootCollar, z0: r.Z0, z1: r.Z1,
		}, true
	}
	return loftCylinderChamferSource{}, false
}

// tryLoftCylinderCapChamfer accepts exactly the two full outer cap loops of
// an authenticated, unplaced loft/cylinder join or its centered bore result.
// It never interprets an arbitrary faceted edge as an offset carrier.
func tryLoftCylinderCapChamfer(ctx context.Context, b *Body, fp facetedPayload,
	edges []*Edge, d, dDelta float64, asym *asymmetricChamfer) (*Body, bool, error) {
	source, ok := loftCylinderChamferSourceOf(fp)
	if !ok {
		return nil, false, nil
	}
	if asym != nil {
		return nil, true, fmt.Errorf("%w: asymmetric chamfer is not certified for this loft/cylinder cap", ErrUnsupported)
	}
	if dDelta != 0 || d <= 0 || !proofbound.FiniteVec(r3.Vec{X: d, Y: source.z0, Z: source.z1}) {
		return nil, true, fmt.Errorf("%w: loft/cylinder cap setback has no exact millimeter magnitude", ErrUnsupported)
	}
	startExact := new(big.Rat).Add(proofarith.FloatRat(source.z0), proofarith.FloatRat(d))
	endExact := new(big.Rat).Sub(proofarith.FloatRat(source.z1), proofarith.FloatRat(d))
	if startExact.Cmp(endExact) >= 0 || source.z0+d == source.z0 || source.z1-d == source.z1 {
		return nil, true, fmt.Errorf("%w: loft/cylinder cap bands reach each other or round onto a cap", ErrUnsupported)
	}
	if err := loftCylinderChamferBridgeSources(&source); err != nil {
		return nil, true, err
	}
	if source.topSourceLevel.Cmp(endExact) <= 0 {
		return nil, true, fmt.Errorf("%w: cap chamfer bridge reaches its side level", ErrUnsupported)
	}
	var caps [2]*Face
	for _, face := range b.Faces() {
		for _, origin := range face.n.Origins {
			if origin == refOf(CapStart(b)) {
				if caps[0] != nil {
					return nil, true, fmt.Errorf("%w: loft/cylinder start cap is split", ErrUnsupported)
				}
				caps[0] = face
			}
			if origin == refOf(CapEnd(b)) {
				if caps[1] != nil {
					return nil, true, fmt.Errorf("%w: loft/cylinder end cap is split", ErrUnsupported)
				}
				caps[1] = face
			}
		}
	}
	if caps[0] == nil || caps[1] == nil {
		return nil, true, fmt.Errorf("%w: loft/cylinder result has no current cap faces", ErrUnsupported)
	}
	selected := make(map[*topo.Edge]struct{}, len(edges))
	for _, edge := range edges {
		selected[edgeNode(edge)] = struct{}{}
	}
	if len(selected) != len(edges) {
		return nil, true, fmt.Errorf("%w: loft/cylinder chamfer repeats an edge", ErrUnsupported)
	}
	for cap, face := range caps {
		if len(face.n.Loops) == 0 || !loftcylinder.ChamferCapWalk(
			face.n.Loops[0], fp.Verts, source.outer[cap], cap == 0) {
			return nil, true, fmt.Errorf("%w: loft/cylinder cap %d has no outer loop", ErrUnsupported, cap)
		}
		for _, ce := range face.n.Loops[0].CoEdges {
			edge := ce.Edge
			if _, yes := selected[edge]; !yes {
				return nil, true, fmt.Errorf("%w: loft/cylinder chamfer needs both complete outer cap loops", ErrUnsupported)
			}
			delete(selected, edge)
		}
	}
	if len(selected) != 0 {
		return nil, true, fmt.Errorf("%w: loft/cylinder chamfer selection includes a bore or side edge", ErrUnsupported)
	}
	body, err := buildLoftCylinderCapChamfer(ctx, documentHandle(b.n.Doc), b.n.Doc.NextProducer, fp, source, d)
	return body, true, err
}
