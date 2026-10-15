package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
)

// evalRevolve builds the analytic revolved body from the payload: side
// surfaces of revolution per boundary segment, caps only for a partial
// sweep, shared edges and vertices, and bounded mass measurements
// (docs/evaluator-design.md §6). The payload's segment kinds are line, circle
// and arc; anything else has already been rejected by the mass-property
// integrals it runs first.
func evalRevolve(d *Document, ref producerID, rp revolvePayload) (*Body, error) {
	return bodyResult(revolveplan.Build(context.Background(), envOf(documentNode(d), ref), rp, freeform.NewFreeformWork()))
}
