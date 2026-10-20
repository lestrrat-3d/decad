package loftcylinder

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// CapChamfer accepts exactly the two full outer cap loops of
// an authenticated, unplaced loft/cylinder join or its centered bore result,
// and builds the result in this package. asymmetric is true when the caller
// asked for unequal setbacks, which this class refuses.
func CapChamfer(ctx context.Context, env topo.Env, b *topo.Body, fp facetedrecord.Payload,
	edges []*topo.Edge, d, dDelta float64, asymmetric bool) (*topo.Body, bool, error) {
	source, ok := ChamferSourceOf(fp)
	if !ok {
		return nil, false, nil
	}
	if asymmetric {
		return nil, true, fmt.Errorf("%w: asymmetric chamfer is not certified for this loft/cylinder cap", decaderr.ErrUnsupported)
	}
	if err := source.BindCapSetback(d, dDelta); err != nil {
		return nil, true, err
	}
	if err := ChamferSelectsOuterLoops(b, provenance.Ref{Producer: b.Origin.Producer, Role: provenance.RoleCapStart}, provenance.Ref{Producer: b.Origin.Producer, Role: provenance.RoleCapEnd},
		edges, fp.Verts, source.Outer); err != nil {
		return nil, true, err
	}
	body, err := ChamferBothCaps(ctx, env, fp, source, d)
	return body, true, err
}
