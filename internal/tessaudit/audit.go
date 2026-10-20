// Package tessaudit decides which payload classes tessellate with a facet
// contact audit that a lower verification level may decline.
package tessaudit

import (
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// A faceted payload is absent because its embedding is proven when the body is
// created, not when it is restated: the boolean's stitch and a faceted
// placement both check every facet their float rounding moved and refuse a
// held mesh they cannot keep embedded (meshbool.EnforceHeldEmbedding,
// docs/evaluator-design.md §9). Restatement moves no coordinate, so there is
// no audit for a lower level to decline.
func AuditsFacetContact(p topo.Payload) bool {
	if _, ok := p.(breprecord.AllEdgeChamferPayload); ok {
		return true
	}
	if _, ok := p.(patchchain.Payload); ok {
		return true
	}
	_, ok := p.(revolveplan.Payload)
	if ok {
		return true
	}
	if sp, ok := p.(sweeprecord.Payload); ok {
		return len(sp.Spans) != 0 || sp.Arc
	}
	if sp, ok := p.(sweeprecord.ChainPayload); ok {
		return len(sp.Spans) != 0 || sp.Arc
	}
	if _, ok := p.(patchchain.BodyPayload); ok {
		return true
	}
	sp, ok := p.(stitchrecord.Payload)
	return ok && sp.Tris == nil
}
