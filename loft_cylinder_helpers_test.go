package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/meshbool"
)

// These helpers give the loft/cylinder join tests the handle-typed call shape
// the tests were written against; each calls the internal builder directly.

func tryLoftCylinderUnionTeeth(ctx context.Context, d *Document, ref producerID,
	blank *Body, teeth []*Body) (*Body, bool, error) {
	node, ok, err := loftcylinder.UnionTeeth(ctx, envOf(documentNode(d), ref), verifyAllMesher{},
		bodyNode(blank), bodyNodes(teeth))
	return bodyHandle(node), ok, err
}

func tryLoftCylinderUnionTeethWithUpperBridge(ctx context.Context, d *Document, ref producerID,
	blank *Body, teeth []*Body, upperBridges []float64) (*Body, bool, error) {
	node, ok, err := loftcylinder.UnionTeethWithUpperBridge(ctx, envOf(documentNode(d), ref), verifyAllMesher{},
		bodyNode(blank), bodyNodes(teeth), upperBridges)
	return bodyHandle(node), ok, err
}

func tryLoftCylinderBoreCut(ctx context.Context, op meshbool.OperationKind, d *Document, ref producerID,
	target, tool *Body) (*Body, bool, error) {
	node, ok, err := loftcylinder.JoinBoreCut(ctx, op, envOf(documentNode(d), ref), verifyAllMesher{},
		bodyNode(target), bodyNode(tool))
	return bodyHandle(node), ok, err
}

func tryPlacedLoftCylinderUnion(ctx context.Context, d *Document, ref producerID,
	a, b *Body) (bool, error) {
	_, ok, err := loftcylinder.PlacedUnion(ctx, envOf(documentNode(d), ref), verifyAllMesher{},
		bodyNode(a), bodyNode(b))
	return ok, err
}

func tryPlacedLoftCylinderBoreCut(ctx context.Context, d *Document, ref producerID,
	target, tool *Body) (bool, error) {
	_, ok, err := loftcylinder.PlacedBoreCut(ctx, envOf(documentNode(d), ref), verifyAllMesher{},
		bodyNode(target), bodyNode(tool))
	return ok, err
}

func loftMirrorCylinderPairs(ctx context.Context, d *Document, ref producerID, blank *Body, pairs []*Body,
	source *loftMirrorJoinRecord, placeWorkers int) (*Body, bool, error) {
	node, ok, err := loftcylinder.MirrorPairsWithWorkers(ctx, envOf(documentNode(d), ref), verifyAllMesher{},
		bodyNode(blank), bodyNodes(pairs), source, placeWorkers)
	return bodyHandle(node), ok, err
}
