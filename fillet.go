package decad

import (
	"context"
	"errors"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the fillet of docs/modify-design.md: Body.Fillet rounds the
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
// cap-edge selector is S1 (ErrUnsupported, the vertex blend §6). A revolve
// receiver takes the same corner rewrite on its meridian (revolve_blend.go,
// docs/modify-reach-design.md §7), and a brep or stacked boolean result takes
// routes P and E of docs/brep-modify-design.md (brep_modify.go); any other
// receiver is S3 (ErrUnsupported).

// FilletOption configures Fillet. WithTangentChain is the one option a
// fillet takes (docs/modify-reach-design.md §2).
type FilletOption interface {
	option.Interface
	filletOption()
}

// filletTol is the closed-form degeneracy tolerance for the section rewrite:
// two directions parallel within it are a smooth or cusped corner (S4), and an
// offset intersection is rejected below it (S5). The section is decad's own
// exact geometry, so this only absorbs float noise, never an admission on a
// residual.
const filletTol = sectionaudit.Tolerance

// Fillet rounds the selected lateral edges of a straight prism with a tangent
// arc of radius r, returning the new body and retiring the receiver
// (docs/modify-design.md §6, core §8). sel is resolved against the live
// receiver; a query matching nothing is loud (ErrNoMatch / ErrCardinality,
// S16). r is a length magnitude, gated like every other (S15); a zero r is
// S13. A selected edge that is not a lateral edge — a cap edge — is S1
// (ErrUnsupported). The rewritten section faces the §5 audit before anything
// is built, so no unproven body is ever made.
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
// blend is a Torus, or a Sphere when its centre lies on the axis. Any other
// revolve edge — a cap edge, an edge on the axis — is SX5 (ErrUnsupported).
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
// displacement (SB1) are ErrUnsupported (Table SB). Any other receiver that
// is neither a prism nor a revolve is S3 (ErrUnsupported).
func (b *Body) Fillet(ctx context.Context, sel EdgeSelector, r units.Value, opts ...FilletOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a fillet`, ErrDegenerate)
	}
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := b.doc
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
	rmm, err := sectionrecord.MagnitudeIn(r, units.Length, units.Millimeter, "the fillet radius")
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
	if err := requireNotCapBlendReceiver(b.payload, "fillets"); err != nil {
		return nil, err
	}
	blend := revolveBlendOp{
		kind: "fillet",
		corner: func(loop cornerLoop, _, ci int, _ *Edge) (*cornerBlend, error) {
			return computeFillet(loop, ci, rmm)
		},
	}
	// A brep or stacked receiver takes the brep route
	// (docs/brep-modify-design.md §2), ahead of the generic refusal.
	route, err := modifyBrepReceiver(ctx, b, brepModifyRequest{
		op:     "fillets",
		admits: func(pp prismPayload, _ prismCaps) error { return requireLateralEdges(ctx, pp, edges) },
		sel:    sel, edges: edges, blend: &blend,
	})
	if err != nil {
		return nil, err
	}
	if route.body != nil {
		return commitModifyResult(ctx, b, route.body)
	}

	// Reach RX2 (docs/modify-reach-design.md §7): a revolve receiver rounds its
	// swept meridian junctions through the same corner rewrite.
	if rp, ok := b.payload.(revolvePayload); ok {
		return b.blendRevolveJunctions(ctx, sel, edges, rp, blend)
	}

	// Stage 2 (§4): the receiver's payload class (S3), then every selected
	// edge is a lateral edge mapped to a section corner (S1).
	pp, ok := b.payload.(prismPayload)
	if route.prism != nil {
		pp, ok = *route.prism, true
	}
	if !ok {
		return nil, fmt.Errorf(`%w: this evaluator fillets a straight prism or a revolve only; selector %s matched [%s]`,
			ErrUnsupported, sel, selectedEdgesContext(edges))
	}
	if err := requireExactSection(pp, "fillets"); err != nil {
		return nil, err
	}

	budget := proofbound.NewWorkBudget(ctx)
	loops, err := prismCornerLoopsBudget(budget, pp)
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
			return nil, fmt.Errorf(`%w: a fillet of a cap edge is the vertex-blend problem, not yet supported; selector %s, %s`,
				ErrUnsupported, sel, selectedEdgeContext(ei, e))
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
	profile, filletArcs, err := rewriteProfileBudget(budget, pp.profile, loops, blendAt)
	if err != nil {
		return nil, err
	}

	// Stage 4 (§4/§5): the audit of the rewritten profile — S8, S6, S7, S9.
	if err := auditRewriteBudget(budget, pp.profile, profile, loops, blendAt); err != nil {
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
	body, err := evalPrismContext(ctx, d, ref, prismPayload{
		profile:   profile,
		frame:     pp.frame,
		z0:        pp.z0,
		z1:        pp.z1,
		z0Delta:   pp.z0Delta,
		z1Delta:   pp.z1Delta,
		xform:     pp.xform,
		blendSegs: filletArcs,
		blendKind: "fillet",
	}, freeform.NewFreeformWork())
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

// cornerLoop is one boundary loop resolved into its coalesced corner walk, the
// same decomposition buildLoopSides and the surveys use: a lateral edge is a
// junction between two consecutive walks.
type cornerLoop struct {
	walks []survey2d.SideWalk
}

// matchedCorner retains one selector result beside the section coordinate it
// resolved to. The selected-edge ordinal follows SelectEdges' stable result
// order; the loop/corner coordinate and plane-local point identify the same
// junction in the recorded section.
type matchedCorner struct {
	edgeOrdinal int
	edge        *Edge
	loop        int
	corner      int
	point       Point2
}

func newMatchedCorner(edgeOrdinal int, edge *Edge, loop, corner int, cl cornerLoop) matchedCorner {
	w := cl.walks[corner]
	return matchedCorner{
		edgeOrdinal: edgeOrdinal,
		edge:        edge,
		loop:        loop,
		corner:      corner,
		point:       Point2{U: w.StartU, V: w.StartV},
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
// (boolean_body.go's vertexOf memoizes by mesh-vertex index; extrude.go
// assigns a full circle's edge the same start and end vertex), so the test is
// the pointer comparison edge.start == edge.end. A Circle3 additionally
// reports the centre and radius no endpoint pair can carry, and falls under
// that same closed case; every other closed curve — including a
// boolean-built body's FacetedCurve rim — renders "closed through (p)"
// instead of repeating the one point as if it were two. Two DISTINCT
// vertices that merely sit at one position are a genuinely collapsed edge,
// not a closed one, and still render "from (p) to (p)" — that is the
// zero-length edge they are. Coordinates go through renderVec, so
// value-equal geometry renders identically.
func selectedEdgeContext(ordinal int, edge *Edge) string {
	if edge == nil || edge.start == nil || edge.end == nil {
		return fmt.Sprintf(`selected edge[%d]`, ordinal)
	}
	if c, ok := edge.curve.(Circle3); ok {
		return fmt.Sprintf(`selected edge[%d] closed circle through (%s), centre (%s), radius %s`,
			ordinal, renderVec(edge.start.position), renderVec(c.Center), c.Radius)
	}
	if edge.start == edge.end {
		return fmt.Sprintf(`selected edge[%d] closed through (%s)`, ordinal, renderVec(edge.start.position))
	}
	return fmt.Sprintf(`selected edge[%d] from (%s) to (%s)`, ordinal,
		renderVec(edge.start.position), renderVec(edge.end.position))
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

// requireExactSection refuses a modify receiver whose payload carries a section
// displacement (docs/prism-boolean-design.md §7). Every modify op is a rewrite
// of the receiver's recorded section (modify §2), and a rewrite of a record that
// is only within a displacement of the section it denotes has no proven
// displacement of its own — an offset miter or a blend centre amplifies it by
// the corner geometry, which nothing here bounds. The body exists and this
// evaluator cannot build it, so the sentinel is ErrUnsupported (modify §1's
// existence test). Zero for every payload a caller draws, so this changes
// nothing for a drawn, placed or already-modified prism.
func requireExactSection(pp prismPayload, op string) error {
	if pp.sectionDelta == 0 {
		return nil
	}
	return fmt.Errorf(
		`%w: this evaluator %s a receiver whose recorded section is the section it denotes only; this body's section carries a proven displacement of %g mm from the one its construction denotes`,
		ErrUnsupported, op, pp.sectionDelta,
	)
}

func prismCornerLoopsBudget(budget *proofbound.WorkBudget, pp prismPayload) ([]cornerLoop, error) {
	return profileCornerLoopsBudget(budget, pp.profile)
}

// profileCornerLoopsBudget resolves every loop of a recorded section into its
// coalesced corner walk in the section's own plane-local coordinates. A prism
// reads its section through it, and a revolve its meridian
// (revolve_blend.go): the corner rewrite is the same 2D construction for both.
func profileCornerLoopsBudget(budget *proofbound.WorkBudget, profile ProfileRecord) ([]cornerLoop, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return nil, err
	}
	// One free-form counter for this whole record walk: no moments preflight ran
	// on the section this reads, so the ceiling starts here and covers every
	// segment of every loop below.
	work := freeform.NewFreeformWork()
	var out []cornerLoop
	for _, loop := range append([]LoopRecord{profile.Outer}, profile.Holes...) {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		raw := make([]survey2d.SideWalk, len(loop.Segments))
		for i, seg := range loop.Segments {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return nil, err
			}
			w, err := walkOf(seg, work)
			if err != nil {
				return nil, err
			}
			if err := boundarywalk.RequireAnalyticWalk(w, "a modify corner rewrite"); err != nil {
				return nil, err
			}
			raw[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
		}
		walks, err := coalesceWalksBudget(raw, budget)
		if err != nil {
			return nil, err
		}
		out = append(out, cornerLoop{walks: walks})
	}
	return out, nil
}

// matchCornerBudget maps a selected edge to a (loop, corner) of the section: a
// lateral edge is the vertical junction at a corner, so its two vertices are
// the corner point lifted to the two caps. A cap edge matches nothing (its
// endpoints share a cap plane), which is exactly S1's honest reading of the
// class.
func matchCornerBudget(budget *proofbound.WorkBudget, pp prismPayload, loops []cornerLoop, e *Edge) (int, int, bool, error) {
	if _, ok := e.curve.(Line3); !ok {
		return 0, 0, false, nil
	}
	if e.start == nil || e.end == nil {
		return 0, 0, false, nil
	}
	const tol = 1e-6
	for li, loop := range loops {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, 0, false, err
		}
		n := len(loop.walks)
		if n == 1 && loop.walks[0].Closed {
			continue
		}
		for ci, w := range loop.walks {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return 0, 0, false, err
			}
			j0 := pp.point(w.StartU, w.StartV, pp.z0)
			j1 := pp.point(w.StartU, w.StartV, pp.z1)
			if matchEndpoints(e.start.position, e.end.position, j0, j1, tol) {
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
	return offset2d.Fillet(loop.walks, ci, r, filletTol)
}

// rewriteProfile applies every corner's blend to the section, returning the new
// ProfileRecord and, per loop, the segment indices that are blend connectors
// (their faces carry the second fillet(i,j) / chamfer(i,j) role, Table B).
func rewriteProfile(orig ProfileRecord, loops []cornerLoop, blendAt []map[int]*cornerBlend) (ProfileRecord, []map[int]struct{}) {
	profile, blendSegs, _ := rewriteProfileBudget(nil, orig, loops, blendAt)
	return profile, blendSegs
}

func rewriteProfileBudget(budget *proofbound.WorkBudget, orig ProfileRecord, loops []cornerLoop, blendAt []map[int]*cornerBlend) (ProfileRecord, []map[int]struct{}, error) {
	origLoops := append([]LoopRecord{orig.Outer}, orig.Holes...)
	newLoops := make([]LoopRecord, len(origLoops))
	blendSegs := make([]map[int]struct{}, len(origLoops))
	for li := range origLoops {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return ProfileRecord{}, nil, err
		}
		blendSegs[li] = map[int]struct{}{}
		if len(blendAt[li]) == 0 {
			newLoops[li] = cloneLoopRecord(origLoops[li])
			continue
		}
		segs, connectors, err := rewriteLoop(budget, loops[li], blendAt[li])
		if err != nil {
			return ProfileRecord{}, nil, err
		}
		newLoops[li] = LoopRecord{Segments: segs}
		blendSegs[li] = connectors
	}
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return ProfileRecord{}, nil, err
	}
	return ProfileRecord{Outer: newLoops[0], Holes: newLoops[1:]}, blendSegs, nil
}

// rewriteLoop applies the section blend to one coalesced loop.
func rewriteLoop(budget *proofbound.WorkBudget, loop cornerLoop, blends map[int]*cornerBlend) ([]CurveSegment, map[int]struct{}, error) {
	return offset2d.RewriteLoop(budget, loop.walks, blends)
}

// walkSegment re-emits a coalesced walk trimmed to its two endpoints.
func walkSegment(w survey2d.SideWalk, sU, sV, eU, eV float64) CurveSegment {
	return offset2d.OriginalSegment(w, sU, sV, eU, eV)
}

// arcSegment records an arc in its walk sense.
func arcSegment(center, start, end Point2, ccw bool) CurveSegment {
	return offset2d.ArcSegment(center, start, end, ccw)
}

// addBlendRoles gives every blend wall its second kind(i,j) role (Table B): the
// wall built from a blend connector already carries side(i,j); the second role —
// "fillet" for Fillet, "chamfer" for Chamfer — names the same (loop, segment) of
// the rewritten record.
func addBlendRoles(ctx context.Context, body *Body, ref producerID, blendSegs []map[int]struct{}, kind string) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	for li, segs := range blendSegs {
		if err := ctx.Err(); err != nil {
			return err
		}
		for sj := range segs {
			if err := ctx.Err(); err != nil {
				return err
			}
			side := fmt.Sprintf("side(%d,%d)", li, sj)
			blend := fmt.Sprintf("%s(%d,%d)", kind, li, sj)
			for _, f := range body.Faces() {
				if err := ctx.Err(); err != nil {
					return err
				}
				for _, o := range f.origins {
					if err := ctx.Err(); err != nil {
						return err
					}
					if o.producer == ref && o.Role == side {
						f.origins = append(f.origins, FeatureRef{producer: ref, Role: blend})
						break
					}
				}
			}
		}
	}
	return nil
}
