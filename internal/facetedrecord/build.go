package facetedrecord

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/units"
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
	in := meshOfPayload(pp)
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

// BuildWithProofBounded uses the caller's single proof budget for the private
// direct-shell route. A false result refuses this route before publication.
// Existing BuildWithProof callers keep their independent work counters.
func BuildWithProofBounded(ctx context.Context, budget *proofbound.WorkBudget, env topo.Env, pp Payload,
	audit *facetproof.MeshAudit, volume measurement.Measurement, volRat *big.Rat) (*topo.Body, bool, error) {
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	if budget == nil || audit == nil || volRat == nil {
		return nil, false, nil
	}
	if err := budget.Err(); err != nil {
		return nil, false, boundedBuildError(ctx, err)
	}
	body, faceOf, diameterReading, ok, err := AssembleBodyBounded(ctx, budget, env, meshOfPayload(pp),
		audit, volume, volRat)
	if err != nil || !ok {
		return nil, ok, err
	}
	pp.Diameter = diameterReading
	pp.FaceOf = faceOf
	body.Payload = pp
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	if err := budget.Err(); err != nil {
		return nil, false, boundedBuildError(ctx, err)
	}
	return body, true, nil
}

// BuildWithProofReplay keeps the bounded single-shell arithmetic checks while
// admitting up to 65,536 facets under the caller's cumulative replay budget.
func BuildWithProofReplay(ctx context.Context, budget *proofbound.WorkBudget, env topo.Env, pp Payload,
	audit *facetproof.MeshAudit, volume measurement.Measurement, volRat *big.Rat) (*topo.Body, bool, error) {
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	if budget == nil || audit == nil || volRat == nil {
		return nil, false, nil
	}
	body, faceOf, diameter, ok, err := assembleBodyBounded(ctx, budget, env, meshOfPayload(pp), audit, volume, volRat, 65536)
	if err != nil || !ok {
		return nil, ok, err
	}
	pp.Diameter = diameter
	pp.FaceOf = faceOf
	body.Payload = pp
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	return body, true, nil
}

// ChargeAreaSlack adds extra to the AreaSlack of body, a body Build assembled
// from a Payload, and to every area bound AssembleBody composed from it: each
// face's and the body's. AreaSlack enters no other reading, so the result is
// the body Build would assemble from the payload with the larger AreaSlack, up
// to the rounding of the sums: each bound is its old value plus extra, rounded
// up. A zero extra changes nothing; a negative or non-finite one is
// ErrUnsupported.
func ChargeAreaSlack(body *topo.Body, extra float64) error {
	pp, ok := body.Payload.(Payload)
	if !ok {
		return fmt.Errorf("%w: the body holds no faceted payload", decaderr.ErrUnsupported)
	}
	if extra == 0 {
		return nil
	}
	if !(extra > 0) || proofbound.IsNonFinite(extra) {
		return fmt.Errorf("%w: an area slack charge must be finite and positive", decaderr.ErrUnsupported)
	}
	slack := proofbound.AbsSumUpper(pp.AreaSlack, extra)
	bound := proofbound.AbsSumUpper(body.Area.Bound.Base(), extra)
	if proofbound.IsNonFinite(slack) || proofbound.IsNonFinite(bound) {
		return fmt.Errorf("%w: the charged body has no finite area bound", decaderr.ErrUnsupported)
	}
	for _, f := range topobuild.Faces(body) {
		f.AreaBound = proofbound.AbsSumUpper(f.AreaBound, extra)
	}
	body.Area.Bound = units.SquareMillimeters(bound)
	body.Area.Exactness = measurement.ExactnessOf(bound)
	pp.AreaSlack = slack
	body.Payload = pp
	return nil
}

func meshOfPayload(pp Payload) Mesh {
	in := Mesh{Verts: pp.Verts, VertexBound: pp.VertexBound, Tris: pp.Tris, Src: pp.Src,
		Groups: pp.Groups, AxialSectionJoin: pp.AxialSectionJoin,
		VolSymDiff: pp.VolSymDiff, AreaSlack: pp.AreaSlack, DPair: pp.DPair}
	return in
}
