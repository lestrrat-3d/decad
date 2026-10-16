package facetedrecord

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Build assembles the Faceted body from the payload: exact
// component/void analysis, per-source-face topology, and measurements with
// the composed proven bounds.
func Build(ctx context.Context, env topo.Env, pp Payload) (*topo.Body, error) {
	return BuildWithProof(ctx, env, pp, nil, measurement.Measurement{}, nil)
}

// BuildWithProof reuses a proof only for the evaluator's unchanged
// result mesh. Placement calls Build and proves its moved mesh anew.
func BuildWithProof(ctx context.Context, env topo.Env, pp Payload,
	audit *facetproof.MeshAudit, volume measurement.Measurement, volRat *big.Rat) (*topo.Body, error) {
	in := Mesh{Verts: pp.Verts, VertexBound: pp.VertexBound, Tris: pp.Tris, Src: pp.Src,
		Groups: pp.Groups, AxialSectionJoin: pp.AxialSectionJoin, SpiralTools: pp.SpiralTools != nil,
		VolSymDiff: pp.VolSymDiff, AreaSlack: pp.AreaSlack, DPair: pp.DPair}
	body, faceOf, diameterReading, err := AssembleBody(ctx, env, in,
		audit, volume, volRat)
	if err != nil {
		return nil, err
	}
	pp.Diameter = diameterReading
	pp.FaceOf = faceOf
	body.Payload = pp
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return body, nil
}
