package spiralsector

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// errCopyNotExact is the entrance refusal for a copied tooth that carries
// no exact rotation about the X axis through the origin. Its text appears in
// no later refusal, so a caller can tell the entrance gate from a failed proof.
var errCopyNotExact = errors.New("carries no exact X-axis rotation")

// CombineInputs checks the first source's curved join, then consumes
// the neutral copy-lineage and shell-separation proof. Its X-axis gate belongs
// to the legacy inverse-seam proof, not the copy lineage.
func CombineInputs(ctx context.Context, blank *topo.Body, tools []*topo.Body) (*facetedrecord.SpiralCombineSource, error) {
	if len(tools) < 2 || len(tools) > 31 {
		return nil, fmt.Errorf("%w: this spiral-tooth Union class accepts two to 31 tools", decaderr.ErrUnsupported)
	}
	first, ok, err := CertifyFirstJoin(ctx, revolveshell.RootCandidates(blank), tools[0])
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, fmt.Errorf("%w: target and first tool have no spiral source certificate", decaderr.ErrUnsupported)
	}
	copies, err := facetedrecord.CertifyCopies(ctx, tools)
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
		if _, ok := AxisCopyMap(rec); !ok {
			return nil, fmt.Errorf("%w: copied tool %d %w; copy it with WithCopyRotation "+
				"about the X axis through the origin", decaderr.ErrUnsupported, i+2, errCopyNotExact)
		}
	}
	for i, move := range copies.Moves[1:] {
		if !NearAxisRotation(move) {
			return nil, fmt.Errorf("%w: tool %d cannot use the legacy axial seam proof", decaderr.ErrUnsupported, i+2)
		}
	}
	return &facetedrecord.SpiralCombineSource{First: first, Tools: tools,
		Moves: copies.Moves, Copies: copies}, nil
}
