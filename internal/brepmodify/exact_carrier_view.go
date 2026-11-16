package brepmodify

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/displacedface"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// exactCarrierViewBlend is a Chamfer of a prism whose section carries a
// displacement (agent-docs/brep-modify-design.md §2.2). The corner rewrite
// refuses such a receiver, since it reads the displaced coordinates, so the
// body is restated as its faceted view like shellViewBlend's: its own
// tessellation, which charges the section
// displacement into every facet bound, with a retained tag on each cap and on
// each side face whose loop has exact carriers (prismrecord.Payload
// CarrierExact). A side face of any other loop keeps no tag: its recorded
// surface lies only within the displacement of the true one, so its facets
// stay in the view as plain obstacles of the strip check. The retained edge
// blend then builds only where every face it rewrites carries an exact tag.
//
// matched is false when b is no displaced prism, when no face would keep a
// side tag, or by shellViewBlend's rule; the caller then keeps its own
// refusal.
func exactCarrierViewBlend(ctx context.Context, env topo.Env, hooks Hooks, b *topo.Body, edges []*topo.Edge,
	refs map[*topo.Edge]*topo.Face, blend viewBlendFunc,
) (*topo.Body, bool, error) {
	pp, ok := b.Payload.(prismrecord.Payload)
	if !ok || pp.SectionDelta == 0 || pp.SurfaceResult || len(edges) == 0 {
		return nil, false, nil
	}
	record, hasRecord, err := prismrecord.CarrierRecordOf(proofbound.NewWorkBudget(ctx), b)
	if err != nil {
		return nil, false, err
	}
	if pp.ExactCarriers == nil && !hasRecord {
		return nil, false, nil
	}
	var recorded *prismrecord.CarrierRecord
	if hasRecord {
		recorded = &record
	}
	untag, kept, err := displacedSideFaces(proofbound.NewWorkBudget(ctx), b, pp, recorded)
	if err != nil || !kept {
		return nil, false, err
	}
	return retainedViewBlend(ctx, env, hooks, b, edges, refs, untag, blend)
}

// displacedSideFaces lists, by index in topobuild.Faces order, every face of
// the displaced prism b whose surface the view must not retain (the shared
// rule of internal/displacedface).
func displacedSideFaces(budget *proofbound.WorkBudget, b *topo.Body, pp prismrecord.Payload, record *prismrecord.CarrierRecord) (map[int]struct{}, bool, error) {
	return displacedface.PrismSideFaces(budget, b, pp, record)
}

// untagged is g with no retained surface: its facets keep their bounds and
// the planarity of the source face, and no reader takes a surface, a plane
// record or a denoted patch from it.
func untagged(g facetedrecord.Group) facetedrecord.Group {
	g.Surface = nil
	g.Retained = false
	g.Denoted = nil
	g.NormalBound = 0
	g.CapRecord = nil
	g.AxialDelta = 0
	g.HasAxialDelta = false
	return g
}
