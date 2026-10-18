package decad

import (
	"context"
	"errors"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the fillet of docs/modify-design.md: Fillet rounds the
// convex or concave lateral edges of a straight prism (Table R row R1). The
// reduction is §2's — a lateral edge is a CORNER of the recorded 2D section,
// and rounding it is a rewrite of that section into a tangent arc of radius r,
// fed back through evalPrism so mass properties, tessellation and the surveys
// come for free (§10, Table D). The section rewrite is decad's OWN synthesized
// geometry (§5), so decad owns its validity with exact closed-form tests — the
// §5 audit (S8 orientation, S6 self-consuming trim, S7 crossing/contact, S9
// nesting) —
// never a residual. The blend surface is a Cylinder (§6); its wall carries the
// side(i,j) role AND a second fillet(i,j) role naming the same (loop, segment)
// of the rewritten record (Table B, B1).
//
// Fillet rounds lateral edges — line/line, line/arc and arc/arc corners,
// convex and concave — with B1's roles and atomic commit (modify §13). A
// selection of complete cap loops is route L's fillet arm over the prism's
// face view (docs/loop-fillet-design.md RF3). Single straight cap edges take
// route E; complete loops with independent straight edges take route V
// (docs/vertex-blend-design.md). A revolve
// receiver takes the same corner rewrite on its meridian (revolve_blend.go,
// docs/modify-reach-design.md §7), and a brep or stacked boolean result takes
// routes P and E of docs/brep-modify-design.md (brep_modify.go); any other
// receiver is S3 (ErrUnsupported). A zero-twist, one-line solid Sweep reads
// its stored exact prism reduction for lateral-edge fillets. Other Sweep
// paths are S3, and its cap-edge selections are S1.
// A one-sided positive circular draft also admits its one complete far cap
// rim through the bounded meridian reduction in draft_circle_fillet.go.

// FilletOption configures Fillet. WithTangentChain is the one option a
// fillet takes (docs/modify-reach-design.md §2).
type FilletOption interface{ filletOption() }

// filletTol is the closed-form degeneracy tolerance for the section rewrite:
// two directions parallel within it are a smooth or cusped corner (S4), and an
// offset intersection is rejected below it (S5). The section is decad's own
// exact geometry, so this only absorbs float noise, never an admission on a
// residual.
const filletTol = sectionaudit.Tolerance

// Fillet rounds selected section corners of a straight prism, its admitted
// one-line Sweep reduction, or a matching-section axial loft with a tangent
// arc of radius r, returning the new body and retiring b
// (docs/modify-design.md §6, core §8). A nil b is ErrDegenerate. sel is
// resolved against the live b; a query matching nothing is loud (ErrNoMatch / ErrCardinality,
// S16). r is a length magnitude, gated like every other (S15); a zero r is
// S13. One straight cap edge of a prism or one-line solid Sweep takes route E;
// other unsupported cap selections are S1 (ErrUnsupported). A rewritten
// section faces the §5 audit before anything is built.
//
// WithTangentChain expands the edges sel resolves to across every edge that
// continues them with proven G1 continuity before any receiver gate runs
// (docs/modify-reach-design.md §5); sel's cardinality assertion applies to
// the seeds it names. A continuation that branches or that this evaluator
// cannot decide is ErrUnsupported (SX2), and so is a faceted boolean result
// (SX9), refused before expansion.
//
// On a revolve, Fillet rounds swept meridian junctions instead
// (docs/modify-reach-design.md §7): a latitude Circle3 of a full turn or a
// junction Arc3 of a partial one, each the sweep of one corner of the
// recorded meridian. The meridian gets the same tangent arc, faces the same
// audit, and is re-gated against the axis — a new axis contact is
// ErrDegenerate, an arc centred across the axis ErrUnsupported — before the
// revolve is rebuilt over the receiver's own axis, sweep and placement. The
// blend is a Torus, or a Sphere when its centre lies on the axis. One
// quarter-turn rectangular angular-cap radial rim has a bounded faceted
// route; other cap edges and edges on the axis are SX5 (ErrUnsupported).
// An axial loft whose two section records match also accepts line/arc corners
// on its hole-free section. It rewrites both records together and rebuilds the
// loft. Unchanged free-form segments retain their recorded curves; a new
// analytic piece must be proven clear of each non-adjacent free-form segment.
//
// An analytic boolean result (a brep or stacked body) that reads as a prism
// along a reference axis is filleted as that prism
// (docs/brep-modify-design.md route P): its lateral edges are the prism's, and
// the result is a prism. Any other selection of straight edges along one axis
// of such a body's face record takes route E (§5): a swept straight wall
// the edge ends on or runs along as a rim is first restated as the planar
// rectangle it sweeps (§5.2), both end faces of each edge take the tangent
// arc at their corner, the two faces beside it are trimmed to the arc's
// feet, and one cylindrical wall carrying a fillet(k) role is added; the
// result is a brep body whose record pairs every edge again before it is
// built. A curved edge, two selected edges sharing a vertex, an edge whose
// end face is a curved face or an earlier blend, a straight wall the route
// needs as a plane that is oblique, split or carries a displaced level, end
// faces whose arcs disagree, and a body whose faces carry a section
// displacement (SB1) are ErrUnsupported (Table SB).
// P8's exact front top edge uses a radius-1 quarter-cylinder cutter before
// route E's SB7 curved-third-face gate (modify-general §4.3c).
// Any other selection of such a body's edges is route L's
// (docs/modify-general-design.md §4): complete
// loops of planar faces are filleted as a pipe band at radius r
// (docs/loop-fillet-design.md) — a quarter cylinder along each line, a torus
// around each arc and at each reflex corner, cylinders meeting along an
// Ellipse3 at a line-line miter, and a FilletMiter3 at a regular sharp
// corner involving an arc — the walls beside each loop trimmed to its
// side level and the face holding it to its offset contour; the result is a
// brep body whose patches carry filletLoop(f,l,p). Selected straight edges
// on part of one planar loop form a partial band; a swept common wall can be
// restated as that planar loop. Loops sharing an edge are SL1; complete loops
// mixed with independent straight edges take route V. A neighbour route L
// cannot trim SL2, and an offset miter that folds or cannot be bounded is
// SF1 (both ErrUnsupported).
// A prism's complete cap loops take the same arm through its face view and
// return a brep body (RF3). A single cap edge on a strictly convex straight
// prism uses a bounded faceted cutter when that edge or its terminal walls
// are oblique; the cutter and receiver commit atomically. A faceted Union of
// a rooted oblique circular boss and rectangular plate also accepts its one
// complete root junction (docs/oblique-boss-fillet-design.md). Any other
// receiver outside those routes, loft §17 and draft §10.4 is S3
// (ErrUnsupported).
//
// A faceted Union of two rectangular prisms also admits one untouched outer
// vertical edge when the other prism is certified clear of the full removed
// radius band (docs/modify-reach-design.md §10.1).
// A loop-filleted body publishes its volume, centroid, area, box and
// through-all extent, and passes Verify; it meshes with an occupied-volume
// proof, exports to STEP, takes part in a mesh boolean, is surveyed for
// undercuts and concave radii, and is placed, mirrored and patterned like any
// brep body (docs/loop-fillet-design.md Table DF).
func Fillet(ctx context.Context, b *Body, sel EdgeSelector, r units.Value, opts ...FilletOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a fillet`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	// Stage 1 pre-gates (§4): a live receiver (S17), a valid magnitude (S15),
	// a non-zero one (S13), then a selector that matches (S16).
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	// Table X (docs/surface-design.md §11): refused ahead of the selector gate
	// so the answer does not depend on what the selector matched.
	if err := refuseSheetOperand(b, "Fillet"); err != nil {
		return nil, err
	}
	o, err := decodeFilletOptions(opts)
	if err != nil {
		return nil, err
	}
	rmm, rDelta, err := extent.MagnitudeInBounded(r, units.Length, units.Millimeter, "the fillet radius")
	if err != nil {
		return nil, err
	}
	if rmm == 0 {
		return nil, fmt.Errorf(`%w: a zero-radius fillet is the body the caller already holds`, ErrDegenerate)
	}
	// decad owns the selector vocabulary, so only the built-in query can be
	// resolved and recorded. Reject foreign implementations before invoking
	// their callback, and treat a typed nil query like an untyped nil.
	q, ok := sel.(*EdgeQuery)
	switch {
	case sel == nil:
		return nil, errNilSelector
	case !ok:
		return nil, fmt.Errorf(`%w: the fillet's edge selector is not a decad edge query (%T)`, ErrDegenerate, sel)
	case q == nil:
		return nil, errNilSelector
	}
	edges, err := q.SelectEdges(b)
	if err != nil {
		return nil, err
	}
	// Reach stage 2 (docs/modify-reach-design.md §4/§5): the seed query's
	// cardinality has been enforced above; the tangent chain expands it.
	if o.TangentChain {
		if edges, err = expandTangentChain(ctx, b, sel, edges); err != nil {
			return nil, err
		}
	}

	// SX10: a capBlendPayload receiver is staged before the generic "not a
	// prism" refusal, so the more specific reason leads.
	if err := capblend.RequireNotReceiver(b.n.Payload, "fillets"); err != nil {
		return nil, err
	}
	if out, matched, err := tryDraftCircleRimFillet(ctx, b, edges, rmm, rDelta); matched || err != nil {
		return out, err
	}
	if err := draftrecord.RequireNotReceiver(b.n.Payload, "fillets"); err != nil {
		return nil, err
	}
	if bp, ok := b.n.Payload.(brepPayload); ok {
		out, matched, err := tryRoundedBrepCapEdgeFillet(ctx, b, bp, edges, rmm, rDelta)
		if err != nil {
			return nil, err
		}
		if matched {
			return out, nil
		}
	}
	blend := revolveBlendOp{
		kind: "fillet",
		corner: func(loop cornerLoop, _, ci int, _ *Edge) (*cornerBlend, error) {
			return computeFillet(loop, ci, rmm)
		},
	}
	// The exact P8 front top edge takes a bounded cutter before the brep
	// receiver's SB7 curved-third-face refusal (modify-general §4.3c).
	// Other brep and stacked receivers take docs/brep-modify-design.md §2.
	// A loop's fillet radius is both of route L's setbacks
	// (docs/loop-fillet-design.md §4).
	loopCall := brepModifyRequest{
		op:     "fillets",
		admits: func(pp prismPayload, _ prismCaps) error { return requireLateralEdges(ctx, pp, edges) },
		sel:    sel, edges: edges, blend: &blend,
		loop:     &capSetback{Dc: rmm, DcDelta: rDelta, Ds: rmm, DsDelta: rDelta},
		loopKind: breprecord.BandFillet,
	}
	route, err := modifyBrepReceiver(ctx, b, loopCall)
	if err != nil {
		return nil, err
	}
	if route.body != nil {
		return commitModifyResult(ctx, b, route.body)
	}

	// Reach RX2 (docs/modify-reach-design.md §7): a quarter-turn rectangular
	// revolve also admits its one outer angular-cap radial rim.
	if rp, ok := b.n.Payload.(revolvePayload); ok {
		if out, matched, err := tryRevolveCapRimFillet(ctx, b, rp, edges, rmm, rDelta); matched || err != nil {
			return out, err
		}
		return b.blendRevolveJunctions(ctx, sel, edges, rp, blend)
	}
	if lp, ok := b.n.Payload.(loftPayload); ok {
		return b.filletStraightLoft(ctx, sel, edges, lp, rmm)
	}
	if fp, ok := b.n.Payload.(facetedPayload); ok && fp.ObliqueBoss != nil {
		return b.filletObliqueBoss(ctx, edges, fp.ObliqueBoss, rmm, rDelta)
	}
	if fp, ok := b.n.Payload.(facetedPayload); ok && fp.PrismUnionFillet != nil {
		// A retained source pair alone does not authenticate this selection.
		// Keep the ordinary S3 refusal and its complete edge context on a miss.
		if out, matched, err := b.filletPrismUnion(ctx, edges, fp.PrismUnionFillet, rmm, rDelta); matched || err != nil {
			return out, err
		}
	}
	if fp, ok := b.n.Payload.(facetedPayload); ok && fp.PrismPatternJoin != nil {
		out, err := prismPatternFilletBody(ctx, b, edges, rmm, rDelta)
		if err != nil {
			return nil, err
		}
		return commitModifyResult(ctx, b, out)
	}

	// Stage 2 (§4): the receiver's payload class (S3), then every selected
	// edge is a lateral edge mapped to a section corner (S1).
	pp, ok := b.n.Payload.(prismPayload)
	// A one-span straight solid Sweep builds through prismrecord.Build and keeps
	// that exact reduction beside its path. Its original edges can be matched
	// against the reduction's corners without replacing the public selector.
	sweepReduction := false
	if reduction, admitted := sweeprecord.StraightPrismReduction(b.n.Payload); admitted {
		pp, ok = reduction, true
		sweepReduction = true
	}
	if route.prism != nil {
		pp, ok = *route.prism, true
	}
	if !ok {
		return nil, fmt.Errorf(`%w: this evaluator fillets a straight prism, its one-line Sweep reduction, a revolve, or a matching-section axial loft only; selector %s matched [%s]`,
			ErrUnsupported, sel, selectedEdgesContext(edges))
	}
	if err := pp.RequireExactSection("fillets"); err != nil {
		return nil, err
	}

	budget := proofbound.NewWorkBudget(ctx)
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, pp.Profile)
	if err != nil {
		return nil, err
	}
	// Which corners each loop gets a fillet at, and the fillet's blend data.
	blendAt := make([]map[int]*cornerBlend, len(loops))
	for i := range blendAt {
		blendAt[i] = map[int]*cornerBlend{}
	}
	matched := make([]matchedCorner, 0, len(edges))
	for ei, e := range edges {
		li, ci, found, err := matchCornerBudget(budget, pp, loops, e)
		if err != nil {
			return nil, err
		}
		if !found {
			if sweepReduction {
				if len(edges) != 1 {
					return nil, fmt.Errorf(`%w: a straight Sweep fillets one cap edge at a time; selector %s matched [%s]`,
						ErrUnsupported, sel, selectedEdgesContext(edges))
				}
				if !straightSweepCapEdge(e) {
					return nil, fmt.Errorf(`%w: %s is not a cap edge of the straight Sweep`,
						ErrUnsupported, selectedEdgeContext(ei, e))
				}
				// The one-line Sweep was built from pp. Route E matches this
				// published edge to that exact prism's face view, then checks the
				// material wedge before rewriting either end face.
				bp, err := breprecord.OfPrism(pp)
				if err != nil {
					return nil, err
				}
				if err := breprecord.RequireExactSection(bp, loopCall.op); err != nil {
					return nil, err
				}
				body, err := brepBlendEdges(ctx, d, bp, loopCall)
				if err != nil {
					return nil, err
				}
				return commitModifyResult(ctx, b, body)
			}
			// RF3 (docs/loop-fillet-design.md §3): a cap edge sends the
			// prism through its brep face view. Route E, L or V then reads the
			// whole selection (docs/vertex-blend-design.md §2).
			if original, ok := b.n.Payload.(prismPayload); ok {
				if out, recognized, err := tryObliqueCapEdgeFillet(ctx, b, original, edges, rmm, rDelta); recognized {
					return out, err
				}
			}
			body, err := prismFaceViewBlend(ctx, d, pp, loopCall)
			if err != nil {
				return nil, err
			}
			return commitModifyResult(ctx, b, body)
		}
		matched = append(matched, newMatchedCorner(ei, e, li, ci, loops[li]))
		blendAt[li][ci] = nil // marked; the blend is computed in stage 3
	}

	// Stage 3 (§4): the construction's own gates, per corner — S4 (a corner
	// exists) then S5 (a blend of that radius exists).
	for _, corner := range matched {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		cb, err := computeFillet(loops[corner.loop], corner.corner, rmm)
		if err != nil {
			return nil, fmt.Errorf(`%w; selector %s, %s`, err, sel, corner)
		}
		blendAt[corner.loop][corner.corner] = cb
	}

	// The rewritten section, and the fillet arcs' (loop, segment) indices.
	profile, filletArcs, err := offset2d.RewriteProfileBudget(budget, pp.Profile, loops, blendAt)
	if err != nil {
		return nil, err
	}

	// Stage 4 (§4/§5): the audit of the rewritten profile — S8, S6, S7, S9.
	if err := offset2d.AuditRewriteBudget(budget, pp.Profile, profile, loops, blendAt); err != nil {
		if errors.Is(err, context.Canceled) {
			return nil, context.Canceled
		}
		if errors.Is(err, context.DeadlineExceeded) {
			return nil, context.DeadlineExceeded
		}
		return nil, wrapModifyAuditError(sel, matched, err)
	}

	// Build through evalPrism (§2): same frame, interval and placement, only
	// the section changed.
	ref := d.nextProducerID()
	// The blend descriptors ride on the payload so a re-evaluation (a copy or a
	// placement) re-mints its own fillet(i,j) roles; evalPrism applies them.
	// The rewritten section is a NEW record no preflight has seen, so the build
	// opens its one counter here (docs/spline-design.md §5.2).
	// A section rewrite is a change of the SECTION: both sweep levels come
	// through unchanged, and so does each one's own axial displacement.
	body, err := bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), prismrecord.Payload{
		Profile:   profile,
		Frame:     pp.Frame,
		Z0:        pp.Z0,
		Z1:        pp.Z1,
		Z0Delta:   pp.Z0Delta,
		Z1Delta:   pp.Z1Delta,
		Xform:     pp.Xform,
		BlendSegs: filletArcs,
		BlendKind: "fillet",
	}, freeform.NewFreeformWork()))
	if err != nil {
		return nil, err
	}
	// Keep the consumed input aligned with document liveness at the commit edge.
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body, b)
	return body, nil
}

// straightSweepCapEdge checks the public role pair before the stored prism's
// face view matches the selected geometry. A one-line Sweep renames only its
// swept walls to side(0,loop,segment); its two cap roles are unchanged.
func straightSweepCapEdge(edge *Edge) bool {
	facing := edge.Faces()
	if len(facing) != 2 {
		return false
	}
	capRole, wallRole := false, false
	for _, face := range facing {
		for _, origin := range face.n.Origins {
			switch {
			case origin.Role == provenance.RoleCapStart || origin.Role == provenance.RoleCapEnd:
				capRole = true
			case strings.HasPrefix(origin.Role, "side(0,"):
				wallRole = true
			}
		}
	}
	return capRole && wallRole
}

// cornerLoop is survey2d.CornerLoop: one boundary loop resolved into its
// coalesced corner walk.
type cornerLoop = survey2d.CornerLoop

// matchedCorner retains one selector result beside the section coordinate it
// resolved to. The selected-edge ordinal follows SelectEdges' stable result
// order; the loop/corner coordinate and plane-local point identify the same
// junction in the recorded section.
type matchedCorner struct {
	edgeOrdinal int
	edge        *Edge
	loop        int
	corner      int
	point       planePoint
}

func newMatchedCorner(edgeOrdinal int, edge *Edge, loop, corner int, cl cornerLoop) matchedCorner {
	w := cl.Walks[corner]
	return matchedCorner{
		edgeOrdinal: edgeOrdinal,
		edge:        edge,
		loop:        loop,
		corner:      corner,
		point:       planePoint{U: w.StartU, V: w.StartV},
	}
}

func (m matchedCorner) String() string {
	return fmt.Sprintf(`%s maps to loop %d corner %d at (u, v) = (%s, %s)`,
		selectedEdgeContext(m.edgeOrdinal, m.edge), m.loop, m.corner,
		renderCoord(m.point.U), renderCoord(m.point.V))
}

// selectedEdgeContext renders one selected edge as its result ordinal plus the
// geometry that identifies it.
//
// Closure is decided by shared vertex identity, never by coordinates: both
// builders that mint a closed edge intern one *Vertex for its two ends
// (internal/facetedrecord/topology.go's vertexOf memoizes by mesh-vertex index; extrude.go
// assigns a full circle's edge the same start and end vertex), so the test is
// the pointer comparison edge.Start() == edge.End(). A Circle3 additionally
// reports the centre and radius no endpoint pair can carry, and falls under
// that same closed case; every other closed curve — including a
// boolean-built body's FacetedCurve rim — renders "closed through (p)"
// instead of repeating the one point as if it were two. Two DISTINCT
// vertices that merely sit at one position are a genuinely collapsed edge,
// not a closed one, and still render "from (p) to (p)" — that is the
// zero-length edge they are. Coordinates go through renderVec, so
// value-equal geometry renders identically.
func selectedEdgeContext(ordinal int, edge *Edge) string {
	if edge == nil || edge.n.Start == nil || edge.n.End == nil {
		return fmt.Sprintf(`selected edge[%d]`, ordinal)
	}
	if c, ok := edge.n.Curve.(surfacegeom.Circle3); ok {
		return fmt.Sprintf(`selected edge[%d] closed circle through (%s), centre (%s), radius %s`,
			ordinal, renderVec(edge.n.Start.Position), renderVec(c.Center), c.Radius)
	}
	if c, ok := edge.n.Curve.(surfacegeom.Ellipse3); ok {
		return fmt.Sprintf(`selected edge[%d] ellipse arc from (%s) to (%s), centre (%s), semi-axes %s and %s`,
			ordinal, renderVec(edge.n.Start.Position), renderVec(edge.n.End.Position), renderVec(c.Center),
			c.SemiMajor, c.SemiMinor)
	}
	if edge.n.Start == edge.n.End {
		return fmt.Sprintf(`selected edge[%d] closed through (%s)`, ordinal, renderVec(edge.n.Start.Position))
	}
	return fmt.Sprintf(`selected edge[%d] from (%s) to (%s)`, ordinal,
		renderVec(edge.n.Start.Position), renderVec(edge.n.End.Position))
}

func selectedEdgesContext(edges []*Edge) string {
	contexts := make([]string, len(edges))
	for i, edge := range edges {
		contexts[i] = selectedEdgeContext(i, edge)
	}
	return strings.Join(contexts, `; `)
}

// wrapModifyAuditError attaches the matched-entity context to an audit failure.
// The audit's own error — sentinel and reason — leads, and the selector plus the
// per-corner mappings follow it, so a refusal over many selected edges still
// reads its cause first. The wrapped sentinel is unchanged, so errors.Is keeps
// branching on it.
func wrapModifyAuditError(sel EdgeSelector, matched []matchedCorner, err error) error {
	coordinates := make([]string, len(matched))
	for i, corner := range matched {
		coordinates[i] = corner.String()
	}
	err = renderAuditCoordinates(err)
	return fmt.Errorf(`%w; selector %s matched [%s]`, err, sel, strings.Join(coordinates, `; `))
}

// matchCornerBudget maps a selected edge to a (loop, corner) of the section: a
// lateral edge is the vertical junction at a corner, so its two vertices are
// the corner point lifted to the two caps. A cap edge matches nothing (its
// endpoints share a cap plane), which is exactly S1's honest reading of the
// class.
func matchCornerBudget(budget *proofbound.WorkBudget, pp prismPayload, loops []cornerLoop, e *Edge) (int, int, bool, error) {
	if _, ok := e.n.Curve.(surfacegeom.Line3); !ok {
		return 0, 0, false, nil
	}
	if e.n.Start == nil || e.n.End == nil {
		return 0, 0, false, nil
	}
	const tol = 1e-6
	for li, loop := range loops {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, 0, false, err
		}
		n := len(loop.Walks)
		if n == 1 && loop.Walks[0].Closed {
			continue
		}
		for ci, w := range loop.Walks {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return 0, 0, false, err
			}
			j0 := pp.Point(w.StartU, w.StartV, pp.Z0)
			j1 := pp.Point(w.StartU, w.StartV, pp.Z1)
			if matchEndpoints(e.n.Start.Position, e.n.End.Position, j0, j1, tol) {
				return li, ci, true, nil
			}
		}
	}
	return 0, 0, false, nil
}

// matchEndpoints reports whether {a, b} equals {p, q} as an unordered pair
// within tol.
func matchEndpoints(a, b, p, q r3.Vec, tol float64) bool {
	near := func(u, v r3.Vec) bool { return u.Sub(v).Len() <= tol }
	return (near(a, p) && near(b, q)) || (near(a, q) && near(b, p))
}

// cornerBlend is the shared fillet and chamfer section rewrite.
type cornerBlend = offset2d.Blend

// computeFillet builds one section corner blend.
func computeFillet(loop cornerLoop, ci int, r float64) (*cornerBlend, error) {
	return offset2d.Fillet(loop.Walks, ci, r, filletTol)
}
