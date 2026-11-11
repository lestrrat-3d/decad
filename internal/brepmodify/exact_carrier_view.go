package brepmodify

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
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
	if !ok || pp.SectionDelta == 0 || pp.ExactCarriers == nil || pp.SurfaceResult || len(edges) == 0 {
		return nil, false, nil
	}
	untag, kept, err := displacedSideFaces(proofbound.NewWorkBudget(ctx), b, pp)
	if err != nil || !kept {
		return nil, false, err
	}
	return retainedViewBlend(ctx, env, hooks, b, edges, refs, untag, blend)
}

// displacedSideFaces lists, by index in topobuild.Faces order, every face of
// the displaced prism b whose surface the view must not retain: each face
// that is neither a cap nor a side face of a loop with exact carriers, a face
// whose roles name no single loop included. kept reports whether some side
// face keeps its tag.
func displacedSideFaces(budget *proofbound.WorkBudget, b *topo.Body, pp prismrecord.Payload) (map[int]struct{}, bool, error) {
	exact := map[int]bool{}
	untag := map[int]struct{}{}
	kept := false
	for fi, f := range topobuild.Faces(b) {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		li, side, isCap := faceLoop(f)
		if isCap {
			continue
		}
		if !side {
			untag[fi] = struct{}{}
			continue
		}
		isExact, seen := exact[li]
		if !seen {
			var err error
			if isExact, err = pp.CarrierExact(budget, li); err != nil {
				return nil, false, err
			}
			exact[li] = isExact
		}
		if !isExact {
			untag[fi] = struct{}{}
			continue
		}
		kept = true
	}
	return untag, kept, nil
}

// faceLoop reads a prism face's roles: cap when every role is a cap's, or
// the loop index li when every role is a side role side(li,j) of that one
// loop. Any other face reads as neither.
func faceLoop(f *topo.Face) (int, bool, bool) {
	if len(f.Origins) == 0 {
		return 0, false, false
	}
	caps, loop := 0, -1
	for _, o := range f.Origins {
		if o.Role == provenance.RoleCapStart || o.Role == provenance.RoleCapEnd {
			caps++
			continue
		}
		var li, j int
		if n, err := fmt.Sscanf(o.Role, "side(%d,%d)", &li, &j); err != nil || n != 2 ||
			o.Role != fmt.Sprintf("side(%d,%d)", li, j) || li < 0 || (loop >= 0 && li != loop) {
			return 0, false, false
		}
		loop = li
	}
	switch {
	case caps == len(f.Origins):
		return 0, false, true
	case caps == 0:
		return loop, true, false
	default:
		return 0, false, false
	}
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
