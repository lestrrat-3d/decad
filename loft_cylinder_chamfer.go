package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
)

// tryLoftCylinderCapChamfer accepts exactly the two full outer cap loops of
// an authenticated, unplaced loft/cylinder join or its centered bore result,
// and builds the result in internal/loftcylinder. It stays in root because it
// reads the receiver's cap feature refs and maps the result node to its
// handle.
func tryLoftCylinderCapChamfer(ctx context.Context, b *Body, fp facetedPayload,
	edges []*Edge, d, dDelta float64, asym *asymmetricChamfer) (*Body, bool, error) {
	source, ok := loftcylinder.ChamferSourceOf(fp)
	if !ok {
		return nil, false, nil
	}
	if asym != nil {
		return nil, true, fmt.Errorf("%w: asymmetric chamfer is not certified for this loft/cylinder cap", ErrUnsupported)
	}
	if err := source.BindCapSetback(d, dDelta); err != nil {
		return nil, true, err
	}
	if err := loftcylinder.ChamferSelectsOuterLoops(&b.n, refOf(CapStart(b)), refOf(CapEnd(b)),
		edgeNodes(edges), fp.Verts, source.Outer); err != nil {
		return nil, true, err
	}
	body, err := bodyResult(loftcylinder.ChamferBothCaps(ctx, envOf(b.n.Doc, b.n.Doc.NextProducer),
		fp, source, d))
	return body, true, err
}
