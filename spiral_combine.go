package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
)

func spiralCombineBody(ctx context.Context, d *Document, ref producerID,
	blank *Body, tools []*Body) (*Body, error) {
	source, err := spiralCombineInputs(ctx, blank, tools)
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

// errSpiralCopyNotExact is the entrance refusal for a copied tooth that carries
// no exact rotation about the X axis through the origin. Its text appears in
// no later refusal, so a caller can tell the entrance gate from a failed proof.
var errSpiralCopyNotExact = errors.New("carries no exact X-axis rotation")

// spiralCombineInputs checks the first source's curved join, then consumes
// the neutral copy-lineage and shell-separation proof. Its X-axis gate belongs
// to the legacy inverse-seam proof, not the copy lineage.
func spiralCombineInputs(ctx context.Context, blank *Body, tools []*Body) (*spiralCombineSource, error) {
	if len(tools) < 2 || len(tools) > 31 {
		return nil, fmt.Errorf("%w: this spiral-tooth Union class accepts two to 31 tools", ErrUnsupported)
	}
	first, ok, err := spiralsector.CertifyFirstJoin(ctx, revolveshell.RootCandidates(bodyNode(blank)), bodyNode(tools[0]))
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, fmt.Errorf("%w: target and first tool have no spiral source certificate", ErrUnsupported)
	}
	copies, err := facetedrecord.CertifyCopies(ctx, bodyNodes(tools))
	if err != nil {
		return nil, err
	}
	// The blank is invariant under an exact rotation about X, so the first
	// tooth's source facts hold for the copy. This includes a tangent cap:
	// rotating its plane preserves its exact contact with the heel cone and
	// every strict source gap. The copy proof above checks complete-tooth
	// separation. A float placement matrix is not orthogonal and would scale
	// the blank, so every copy must carry the exact record.
	for i, rec := range copies.Exact[1:] {
		if _, ok := spiralsector.AxisCopyMap(rec); !ok {
			return nil, fmt.Errorf("%w: copied tool %d %w; copy it with WithCopyRotation "+
				"about the X axis through the origin", ErrUnsupported, i+2, errSpiralCopyNotExact)
		}
	}
	for i, move := range copies.Moves[1:] {
		if !spiralsector.NearAxisRotation(move) {
			return nil, fmt.Errorf("%w: tool %d cannot use the legacy axial seam proof", ErrUnsupported, i+2)
		}
	}
	return &spiralCombineSource{First: first, Tools: bodyNodes(tools),
		Moves: copies.Moves, Copies: copies}, nil
}
