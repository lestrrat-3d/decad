package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
)

// evalPrism builds the analytic prism body from the payload: side faces per
// boundary segment, two caps, shared edges and vertices, and Exact
// measurements (docs/evaluator-design.md §5). The payload's segment kinds
// are line, circle and arc; anything else has already been rejected by the
// mass-property integrals it runs first.
//
// work is the record's ONE free-form work counter: the preflight this build runs
// and every walkOf under it charge the same ceiling, and a caller that already
// spent part of it on this record passes it in rather than opening a second one.
func evalPrism(d *Document, ref producerID, pp prismPayload, work *freeform.FreeformWork) (*Body, error) {
	return bodyResult(prismrecord.Build(context.Background(), envOf(documentNode(d), ref), pp, work))
}
