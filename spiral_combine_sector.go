package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/spiralsector"
)

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
	copies, err := certifyFacetedCopies(ctx, tools)
	if err != nil {
		return nil, err
	}
	for i, move := range copies.moves[1:] {
		if !spiralsector.NearAxisRotation(move) {
			return nil, fmt.Errorf("%w: tool %d cannot use the legacy axial seam proof", ErrUnsupported, i+2)
		}
	}
	return &spiralCombineSource{first: first, tools: tools,
		moves: copies.moves, copies: copies}, nil
}
