package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/extent"

	"github.com/lestrrat-3d/decad/internal/offset2d"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/units"
)

// This file is the fillet of agent-docs/modify-design.md: Fillet rounds the
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
// face view (agent-docs/loop-fillet-design.md RF3). Single straight cap edges take
// route E; complete loops with independent straight edges take route V
// (agent-docs/vertex-blend-design.md). A revolve
// receiver takes the same corner rewrite on its meridian (revolve_blend.go,
// agent-docs/modify-reach-design.md §7), and a brep or stacked boolean result takes
// routes P and E of agent-docs/brep-modify-design.md (brep_modify.go); any other
// receiver is S3 (ErrUnsupported). A zero-twist, one-line solid Sweep reads
// its stored exact prism reduction for lateral-edge fillets. Other Sweep
// paths are S3, and its cap-edge selections are S1.
// A one-sided positive circular draft also admits its one complete far cap
// rim through the bounded meridian reduction in draft_circle.go.

// FilletOption configures Fillet. WithTangentChain is the one option a
// fillet takes (agent-docs/modify-reach-design.md §2).
type FilletOption interface{ filletOption() }

// Fillet rounds selected section corners of a straight prism, its admitted
// one-line Sweep reduction, or a matching-section axial loft with a tangent
// arc of radius r, returning the new body and retiring b
// (agent-docs/modify-design.md §6, core §8). A nil b is ErrDegenerate. sel is
// resolved against the live b; a query matching nothing is loud (ErrNoMatch / ErrCardinality,
// S16). r is a length magnitude, gated like every other (S15); a zero r is
// S13. One straight cap edge of a prism or one-line solid Sweep takes route E;
// other unsupported cap selections are S1 (ErrUnsupported). A rewritten
// section faces the §5 audit before anything is built.
//
// WithTangentChain expands the edges sel resolves to across every edge that
// continues them with proven G1 continuity before any receiver gate runs
// (agent-docs/modify-reach-design.md §5); sel's cardinality assertion applies to
// the seeds it names. A continuation that branches or that this evaluator
// cannot decide is ErrUnsupported (SX2), and so is a faceted boolean result
// (SX9), refused before expansion.
//
// On a revolve, Fillet rounds swept meridian junctions instead
// (agent-docs/modify-reach-design.md §7): a latitude Circle3 of a full turn or a
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
// (agent-docs/brep-modify-design.md route P): its lateral edges are the prism's, and
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
// (agent-docs/modify-general-design.md §4): complete
// loops of planar faces are filleted as a pipe band at radius r
// (agent-docs/loop-fillet-design.md) — a quarter cylinder along each line, a torus
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
// complete root junction (agent-docs/oblique-boss-fillet-design.md). Any other
// receiver outside those routes, loft §17 and draft §10.4 is S3
// (ErrUnsupported).
//
// A faceted Union of two rectangular prisms also admits one untouched outer
// vertical edge when the other prism is certified clear of the full removed
// radius band (agent-docs/modify-reach-design.md §10.1).
// A loop-filleted body publishes its volume, centroid, area, box and
// through-all extent, and passes Verify; it meshes with an occupied-volume
// proof, exports to STEP, takes part in a mesh boolean, is surveyed for
// undercuts and concave radii, and is placed, mirrored and patterned like any
// brep body (agent-docs/loop-fillet-design.md Table DF).
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
	// Table X (agent-docs/surface-design.md §11): refused ahead of the selector gate
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
	// Reach stage 2 (agent-docs/modify-reach-design.md §4/§5): the seed query's
	// cardinality has been enforced above; the tangent chain expands it.
	if o.TangentChain {
		if edges, err = expandTangentChain(ctx, b, sel, edges); err != nil {
			return nil, err
		}
	}

	res, err := brepmodify.Fillet(ctx, envOf(b.n.Doc, b.n.Doc.NextProducer), &b.n, brepmodify.FilletCall{
		Sel: fmt.Sprint(sel), Edges: edgeNodes(edges), R: rmm, RDelta: rDelta, Hooks: modifyHooks(),
	})
	if err != nil {
		return nil, err
	}
	return commitResult(ctx, b, res)
}

// cornerLoop is survey2d.CornerLoop: one boundary loop resolved into its
// coalesced corner walk.
type cornerLoop = survey2d.CornerLoop

// cornerBlend is the shared fillet and chamfer section rewrite.
type cornerBlend = offset2d.Blend
