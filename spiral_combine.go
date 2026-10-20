package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
)

func spiralCombineBody(ctx context.Context, d *Document, ref producerID,
	blank *Body, tools []*Body) (*Body, error) {
	source, err := spiralsector.CombineInputs(ctx, bodyNode(blank), bodyNodes(tools))
	if err != nil {
		return nil, err
	}
	combined, err := spiralCombinedToolBody(ctx, d, ref, source)
	if err != nil {
		return nil, err
	}
	eval, err := evaluateBoolean(ctx, meshbool.OpUnion, blank, combined)
	if err != nil {
		return nil, asBooleanError(meshbool.OpUnion, err)
	}
	return bodyResult(facetedrecord.BuildWithProof(ctx, envOf(documentNode(d), ref), eval.payload, eval.audit, measurementToInternal(eval.volume), eval.volumeRat))
}

// spiralCombinedToolBody builds the transient, unregistered disjoint union of
// all input tools that spiralsector.CombinedToolPayload assembles, then binds
// each composite face to the live tool face it copies. It stays in root
// because it builds the body through envOf and documentNode on the root
// Document and returns a root Body handle.
func spiralCombinedToolBody(ctx context.Context, d *Document, ref producerID,
	source *spiralCombineSource) (*Body, error) {
	p, err := spiralsector.CombinedToolPayload(ctx, source)
	if err != nil {
		return nil, err
	}
	body, err := bodyResult(facetedrecord.Build(ctx, envOf(documentNode(d), ref), p))
	if err != nil {
		return nil, err
	}
	if err := spiralsector.BindCombinedOwners(bodyNode(body), p.SpiralTools); err != nil {
		return nil, err
	}
	return body, nil
}
