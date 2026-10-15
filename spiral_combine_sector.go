package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/spiralsector"
)

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
	first, ok, err := certifySpiralFirstJoin(ctx, blank, tools[0])
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, fmt.Errorf("%w: target and first tool have no spiral source certificate", ErrUnsupported)
	}
	// No proof carries the exact tangency and the one-step gaps of a
	// cone-tangent first tooth to a copy of it.
	if first.Tooth != nil && first.Tooth.Tangent != nil {
		return nil, fmt.Errorf("%w: a cone-tangent first tooth has no patterned-copy proof", ErrUnsupported)
	}
	copies, err := certifyFacetedCopies(ctx, tools)
	if err != nil {
		return nil, err
	}
	// The blank is invariant under an exact rotation about X, so the first
	// tooth's source facts hold for the copy. A float placement matrix is not
	// orthogonal and would scale the blank, so every copy must carry the record.
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
