package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// Route E (brep-modify §5) authenticates public edge and reference-face
// handles here. breprecord reads and rewrites the selected BRep record.
type brepEdgeBlend = breprecord.EdgeBlend
type brepEdgeEnd = breprecord.EdgeEnd
type brepRestated = breprecord.EdgeRestated
type brepMaterialWedgeMismatchError = breprecord.MaterialWedgeMismatchError

type brepEdgeRoute struct {
	*breprecord.EdgeRoute
	call brepModifyRequest
}

func newBrepEdgeRoute(bp brepPayload, topo *brepTopology,
	call brepModifyRequest, budget *proofbound.WorkBudget) *brepEdgeRoute {
	return &brepEdgeRoute{EdgeRoute: breprecord.NewEdgeRoute(bp, topo,
		call.op, fmt.Sprint(call.sel), selectedEdgesContext(call.edges), call.blend.kind,
		budget), call: call}
}

// brepBlendEdges commits the route E record after its edge and face gates.
func brepBlendEdges(ctx context.Context, d *Document, bp brepPayload, call brepModifyRequest) (*Body, error) {
	r, blends, err := prepareBrepEdgeBlends(ctx, bp, call, false, nil)
	if err != nil {
		return nil, err
	}
	for _, blend := range blends {
		if err := r.RequireBlendMaterialWedge(blend); err != nil {
			return nil, err
		}
	}
	out, err := r.Rewrite(blends)
	if err != nil {
		return nil, err
	}

	// Stage 6: the closure. The body build pairs every edge of the rewritten
	// record again; an edge that does not pair refuses there.
	body, err := bodyResult(breprecord.Build(ctx, envOf(documentNode(d), d.nextProducerID()), out))
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, ctxErr
		}
		return nil, fmt.Errorf(`%w; the %s's rewritten brep record (brep-modify §5.3 step 7); selector %s matched [%s]`,
			err, call.blend.kind, call.sel, selectedEdgesContext(call.edges))
	}
	if body.n.Volume.Value.Base() <= 0 {
		return nil, fmt.Errorf(`%w: the %s's rewritten brep record encloses no volume (brep-modify §5.3 step 7); selector %s matched [%s]`,
			ErrDegenerate, call.blend.kind, call.sel, selectedEdgesContext(call.edges))
	}
	return body, nil
}

// prepareBrepEdgeBlends runs route E's gates in Table SB order. It also
// allows shared vertices where the later partial-route builder closes them.
func prepareBrepEdgeBlends(ctx context.Context, bp brepPayload, call brepModifyRequest, shared bool,
	terminals map[[3]float64]bool) (*brepEdgeRoute, []*brepEdgeBlend, error) {
	btopo, err := breprecord.TopologyContext(ctx, bp)
	if err != nil {
		return nil, nil, err
	}
	r := newBrepEdgeRoute(bp, btopo, call, proofbound.NewWorkBudget(ctx))
	r.Terminals = terminals

	// Stage 2c: each edge is a straight line along one reference axis (EB1),
	// and no two share a vertex (EB7).
	blends := make([]*brepEdgeBlend, len(call.edges))
	for ei, e := range call.edges {
		eb, err := r.admitEdge(ei, e)
		if err != nil {
			return nil, nil, err
		}
		blends[ei] = eb
	}
	if !shared {
		vertexOf := map[[3]float64]*brepEdgeBlend{}
		for _, eb := range blends {
			for _, v := range eb.V {
				if other, ok := vertexOf[v]; ok {
					return nil, nil, r.Refuse(eb, "SB5", fmt.Sprintf(`it shares the vertex at (%s) with %s; a vertex blend is not supported`,
						r.Render(v), other.Context))
				}
				vertexOf[v] = eb
			}
		}
	}

	// Stage 3: the edge topology, gate by gate over every edge.
	incident := r.Incidence()
	for _, eb := range blends {
		if err := r.RequireThreeEdges(eb, incident); err != nil {
			return nil, nil, err
		}
	}
	// The restatement passes (§5.2) collect every wall the call needs as a
	// plane, end faces first (SB7), then rim-adjacent walls (SB8); a wall one
	// edge needs as an end face and another beside it is one plane for both.
	restated := map[int]brepRestated{}
	for _, eb := range blends {
		if err := r.FindEndFaces(eb, incident, restated); err != nil {
			return nil, nil, err
		}
	}
	for _, eb := range blends {
		if err := r.RestateRims(eb, restated); err != nil {
			return nil, nil, err
		}
	}
	if len(restated) > 0 {
		oldView := r.BP.RefView()
		next, err := r.WithRestated(ctx, restated)
		if err != nil {
			return nil, nil, err
		}
		r = &brepEdgeRoute{EdgeRoute: next, call: call}
		// The restated record keeps every face index and every edge, but
		// numbers its uses afresh. Restating face zero can also change the
		// reference frame and reverse the edge's axis order, so preserve each
		// end face by its physical vertex when matching the edge again.
		newView := r.BP.RefView()
		for ei, eb := range blends {
			again, err := r.admitEdge(eb.Ordinal, call.edges[eb.Ordinal])
			if err != nil {
				return nil, nil, err
			}
			oldFirst := oldView.Point(eb.V[0][0], eb.V[0][1], eb.V[0][2])
			newFirst := newView.Point(again.V[0][0], again.V[0][1], again.V[0][2])
			newLast := newView.Point(again.V[1][0], again.V[1][1], again.V[1][2])
			switch {
			case oldFirst.Sub(newFirst).Len() <= 1e-6:
				again.End = eb.End
			case oldFirst.Sub(newLast).Len() <= 1e-6:
				again.End = [2]brepEdgeEnd{eb.End[1], eb.End[0]}
			default:
				return nil, nil, r.Refuse(again, "SB8", `restating its walls moved an end vertex`)
			}
			blends[ei] = again
		}
	}
	for _, eb := range blends {
		if err := r.ClassifySides(eb); err != nil {
			return nil, nil, err
		}
	}
	for _, eb := range blends {
		if err := r.RequireNaturalFaces(eb); err != nil {
			return nil, nil, err
		}
	}
	for _, eb := range blends {
		if err := r.LocateCorners(eb); err != nil {
			return nil, nil, err
		}
	}

	// Stage 4: the construction's own gates per edge — S4 and S5 in G0, then
	// the recomputation in G1 and SB9.
	for _, eb := range blends {
		if err := r.computeBlends(eb); err != nil {
			return nil, nil, err
		}
	}
	return r, blends, nil
}

// admitEdge authenticates one public edge against the record at EB1/SB4.
func (r *brepEdgeRoute) admitEdge(ordinal int, e *Edge) (*brepEdgeBlend, error) {
	eb := &brepEdgeBlend{Ordinal: ordinal, Context: selectedEdgeContext(ordinal, e)}
	switch e.n.Curve.(type) {
	case surfacegeom.Line3:
	case surfacegeom.Circle3, surfacegeom.Arc3:
		return nil, r.Refuse(eb, "SB4", `the edge is a circle or an arc (a hole rim or a boss root), and a cone or torus band is not a face this record holds`)
	default:
		return nil, r.Refuse(eb, "SB4", fmt.Sprintf(`the edge is a %T, not a straight line`, publicCurve(e.n.Curve)))
	}
	if e.n.Start == nil || e.n.End == nil {
		return nil, r.Refuse(eb, "SB4", `the edge has no vertices`)
	}
	refView := r.BP.RefView()
	const tol = 1e-6
	found := -1
	for ei, pair := range r.Topo.Edges {
		if err := survey2d.WallBudgetStep(r.Budget); err != nil {
			return nil, err
		}
		owner := r.Topo.Uses[pair[0]]
		if owner.Key.Circular {
			continue
		}
		p := refView.Point(owner.DirFrom[0], owner.DirFrom[1], owner.DirFrom[2])
		q := refView.Point(owner.DirTo[0], owner.DirTo[1], owner.DirTo[2])
		if !matchEndpoints(e.n.Start.Position, e.n.End.Position, p, q, tol) {
			continue
		}
		if found >= 0 {
			return nil, r.Refuse(eb, "SB6", `the edge matches more than one edge of the receiver's face record`)
		}
		found = ei
	}
	if found < 0 {
		return nil, r.Refuse(eb, "SB6", `the edge matches no whole edge of the receiver's face record`)
	}
	eb.Pair = r.Topo.Edges[found]
	owner := r.Topo.Uses[eb.Pair[0]]
	a, b := owner.DirFrom, owner.DirTo
	axis := -1
	for i := range 3 {
		if a[i] == b[i] {
			continue
		}
		if axis >= 0 {
			return nil, r.Refuse(eb, "SB4", `the edge is an oblique line, not one along a reference axis`)
		}
		axis = i
	}
	if axis < 0 {
		return nil, r.Refuse(eb, "SB4", `the edge has no length`)
	}
	if a[axis] > b[axis] {
		a, b = b, a
	}
	eb.Axis, eb.V = axis, [2][3]float64{a, b}
	for k, v := range eb.V {
		eb.Terminal[k] = r.Terminals == nil || r.Terminals[v]
	}
	eb.Adj = [2]int{owner.Face, r.Topo.Uses[eb.Pair[1]].Face}
	return eb, nil
}

// refuseAsymmetricReference reports an SX16 reference-face mismatch.
func (r *brepEdgeRoute) refuseAsymmetricReference(eb *brepEdgeBlend, reason string) error {
	return fmt.Errorf(`%w: %s (modify-reach SX16); selector %s, %s`, ErrUnsupported,
		reason, r.call.sel, eb.Context)
}

// asymmetricReferenceSide maps the resolved public face to one of the two
// record faces beside the edge. Restatement can change wall(i) to face(i),
// but it keeps i, so the record index is the shared identity across it.
func (r *brepEdgeRoute) asymmetricReferenceSide(eb *brepEdgeBlend) (int, error) {
	ref := r.call.asym.refs[r.call.edges[eb.Ordinal]]
	if ref == nil {
		return -1, r.refuseAsymmetricReference(eb, `the asymmetric reference has no face for this edge`)
	}
	selected := -1
	for j, index := range eb.Adj {
		if !r.call.asym.matchesRecordFace(ref, index) {
			continue
		}
		if selected >= 0 && selected != j {
			return -1, r.refuseAsymmetricReference(eb, `the asymmetric reference names both adjacent record faces`)
		}
		selected = j
	}
	if selected < 0 {
		return -1, r.refuseAsymmetricReference(eb, `the asymmetric reference has no adjacent record face identity`)
	}
	return selected, nil
}

// computeBlends is brep-modify §5.3 steps 1 and 2: the corner blend in G0
// (S4, S5), then the same blend recomputed in G1 (S4, S5), whose centre and
// two feet, mapped into G0's frame, must equal G0's bit for bit (SB9).
func (r *brepEdgeRoute) computeBlends(eb *brepEdgeBlend) error {
	referenceSide := -1
	if r.call.asym != nil {
		var err error
		referenceSide, err = r.asymmetricReferenceSide(eb)
		if err != nil {
			return err
		}
	}
	for k := range eb.End {
		if !eb.Terminal[k] {
			continue
		}
		if err := survey2d.WallBudgetStep(r.Budget); err != nil {
			return err
		}
		end := eb.End[k]
		loops, err := r.CornerLoops(end.Face)
		if err != nil {
			return err
		}
		var cb *cornerBlend
		if referenceSide < 0 {
			cb, err = r.call.blend.corner(loops[end.Loop], end.Loop, end.Corner, r.call.edges[eb.Ordinal])
		} else {
			dA, dB := r.call.asym.other, r.call.asym.d
			if end.Arriving == referenceSide {
				dA, dB = dB, dA
			}
			cb, err = computeChamfer(loops[end.Loop], end.Corner, dA, dB)
		}
		if err != nil {
			return fmt.Errorf(`%w; selector %s, %s at brep face %s's corner (%s)`, err, r.call.sel,
				eb.Context, r.BP.Faces[end.Face].Role, r.Render(eb.V[k]))
		}
		eb.Blend[k] = cb
	}
	if !eb.Terminal[0] || !eb.Terminal[1] {
		return nil
	}
	g0, g1 := eb.End[0].Face, eb.End[1].Face
	m, ok := brepgeom.NewMap2(r.Topo.Embeds[g1], r.Topo.Embeds[g0])
	if !ok {
		return r.Refuse(eb, "SB9", `the two end faces do not lie across one axis`)
	}
	for j := range 2 {
		f0, _ := breprecord.FootOf(eb.End[0], eb.Blend[0], j)
		f1, _ := breprecord.FootOf(eb.End[1], eb.Blend[1], j)
		u, v := m.Point(f1.U, f1.V)
		if u != f0.U || v != f0.V {
			return r.Refuse(eb, "SB9", fmt.Sprintf(`the blend's foot on brep face %s computed in the two end faces disagrees: (%s, %s) against (%s, %s) in brep face %s's frame`,
				r.BP.Faces[eb.Adj[j]].Role, renderCoord(f0.U), renderCoord(f0.V), renderCoord(u), renderCoord(v), r.BP.Faces[g0].Role))
		}
	}
	mapped, err := brepgeom.MapSegment(m, eb.Blend[1].Connector)
	if err != nil {
		return r.Refuse(eb, "SB9", err.Error())
	}
	c0, arc0 := eb.Blend[0].Connector.(arcSeg)
	c1, arc1 := mapped.(arcSeg)
	if arc0 != arc1 || (arc0 && c0.Center != c1.Center) {
		return r.Refuse(eb, "SB9", `the blend's centre computed in the two end faces disagrees`)
	}
	return nil
}
