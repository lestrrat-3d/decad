package pairkernel

import (
	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// PlanarPairAdmits reports whether the exact planar arm serves this pair.
// It exists for the payload classes the analytic kernel has no carrier model
// for — a mitred sweep (agent-docs/sweep-design.md Table DM row DM5), a faceted
// Boolean result (agent-docs/payload-verification-design.md §7) and a coil
// (agent-docs/helix-design.md Table CD row CD5) — each read off its own held
// triangle set with that set's displacement, and it admits a prism or a
// stitched solid only as their partner, read exactly off its own record
// (contact_faceted_pair.go's planarPrismSolid and planarStitchSolid). A coil
// is served only against a prism or a stitched solid: a coil against a
// mitred sweep, a faceted result or another coil stays undecided. A pair of
// prisms or stitched solids keeps the analytic kernel's answer, and every
// other payload keeps its own design's staging: a cup is never tessellated
// for verification (payload verification §1), and a loft, a composite sweep
// and a cap-loop chamfer stay undecided until their own adapters land.
func PlanarPairAdmits(a, b *topo.Body) bool {
	served, coils := false, 0
	for _, body := range []*topo.Body{a, b} {
		switch body.Payload.(type) {
		case sweepmitre.Payload, facetedrecord.Payload:
			served = true
		case coilshell.Payload:
			served = true
			coils++
		case prismrecord.Payload, stitchrecord.Payload:
		default:
			return false
		}
	}
	if coils == 0 {
		return served
	}
	for _, body := range []*topo.Body{a, b} {
		switch body.Payload.(type) {
		case coilshell.Payload, prismrecord.Payload, stitchrecord.Payload:
		default:
			return false
		}
	}
	return coils == 1
}
