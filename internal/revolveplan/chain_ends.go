package revolveplan

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

type chainFreeEnd struct {
	edge           *topo.Edge
	chain, segment int
	atStart        bool
}

type chainBuiltEnds struct{ start, end *topo.Edge }

// FreeEnd reads the source end authenticated by the body's chain build.
// body and edge must not be nil.
func FreeEnd(ctx context.Context, budget *proofbound.WorkBudget, body *topo.Body, edge *topo.Edge) (int, int, bool, error) {
	miss := func() (int, int, bool, error) {
		return 0, 0, false, fmt.Errorf(`%w: Extend needs a free swept edge at an open meridian end`, decaderr.ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return 0, 0, false, err
	}
	rp, ok := body.Payload.(ChainPayload)
	if !ok {
		return miss()
	}
	for _, end := range rp.freeEnds {
		if err := budget.Step(); err != nil {
			return 0, 0, false, err
		}
		if end.edge != edge {
			continue
		}
		if end.chain < 0 || end.chain >= len(rp.Chains) {
			return miss()
		}
		segments := rp.Chains[end.chain].Segments
		if end.segment < 0 || end.segment >= len(segments) ||
			(end.atStart && end.segment != 0) || (!end.atStart && end.segment != len(segments)-1) ||
			len(edge.Faces) != 1 || edge.Faces[0].Body != body {
			return miss()
		}
		if err := ctx.Err(); err != nil {
			return 0, 0, false, err
		}
		return end.chain, end.segment, end.atStart, nil
	}
	if err := ctx.Err(); err != nil {
		return 0, 0, false, err
	}
	return miss()
}
