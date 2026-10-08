package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the chamfer of docs/modify-design.md §7: Body.Chamfer bevels the
// convex or concave lateral edges of a straight prism (Table R, R1). It is the
// fillet's sibling under the §2 reduction — a lateral edge is a CORNER of the
// recorded 2D section, and bevelling it is a rewrite of that section, fed back
// through evalPrism so mass properties, tessellation and the surveys come for
// free (§10, Table D). Chamfer's ONE new piece of geometry is the corner
// rewrite: the corner is cut off by a straight CHORD between the two setback
// feet (a LineSeg), never a tangent arc, so the bevel wall is a PLANE (§7). It
// then shares everything downstream with the fillet — the §5 audit
// (auditRewriteBudget, S8/S6/S7/S9), the profile rewrite (rewriteProfile), evalPrism,
// and the blend-role machinery (addBlendRoles) — via the common cornerBlend.
//
// The setback d is measured along the adjacent boundary curve, d from the
// corner along EACH walk; WithAsymmetricChamfer instead sets back d along the
// walk whose wall is the edge's reference face and its other distance along
// the other walk (docs/modify-reach-design.md §6). There is NO S5 gate: a
// chord exists between any two distinct feet, so the fillet's no-blend-centre
// refusal has no chamfer case (Table B, B1). A revolve receiver takes the same chord on its meridian
// (revolve_blend.go, docs/modify-reach-design.md §7), and a brep or stacked
// boolean result takes routes P and E of docs/brep-modify-design.md
// (brep_modify.go); any other non-prism receiver is S3 (ErrUnsupported).
//
// Chamfer also takes docs/modify-reach-design.md §8.3's second receiver class,
// which the fillet does not: a selection covering every geometric edge of one
// or more COMPLETE prism cap loops builds a cap-loop chamfer through
// capblend.go instead of this file's corner rewrite, at an equal setback or,
// under WithAsymmetricChamfer, at the two setbacks each cap's reference face
// picks (§8.3.1). A cap-edge selection that is not one or more complete loops,
// or one mixing cap edges with lateral ones, is SX4 (ErrUnsupported).

// ChamferOption configures Chamfer: WithTangentChain and
// WithAsymmetricChamfer (docs/modify-reach-design.md §2).
type ChamferOption interface {
	option.Interface
	chamferOption()
}

// Chamfer bevels the selected lateral edges of a straight prism with a straight
// chord set back a distance d along each adjacent wall, returning the new body
// and retiring the receiver (docs/modify-design.md §7, core §8). sel is resolved
// against the live receiver; a query matching nothing is loud (ErrNoMatch /
// ErrCardinality, S16). d is a length magnitude, gated like every other (S15); a
// zero d is S13. The rewritten section faces the §5 audit before anything is
// built, so no unproven body is ever made and an over-large setback is refused
// (S6), never clipped.
//
// On a revolve, Chamfer bevels swept meridian junctions instead
// (docs/modify-reach-design.md §7): the meridian corner gets the same chord,
// faces the same audit and the revolve's axis gates, and the revolve is rebuilt
// over the receiver's own axis, sweep and placement. The bevel is a Cone, a
// Cylinder where the chord runs parallel to the axis, or a Plane where it runs
// perpendicular. Any other revolve edge — a cap edge, an edge on the axis — is
// SX5 (ErrUnsupported).
//
// A receiver that is none of a prism, a revolve, or a brep or stacked
// boolean result is S3 (ErrUnsupported).
//
// WithTangentChain expands the edges sel resolves to as Fillet's does
// (docs/modify-reach-design.md §5). WithAsymmetricChamfer sets each edge back
// d across its reference face and the option's other distance across the
// other adjacent face (§6), on a prism's lateral edges and a revolve's
// junctions alike; each setback faces the S6 audit on its own walk. On a
// complete cap loop the reference picks per cap (§8.3.1): the cap face takes d
// across the cap and the other distance down the side walls, and a side wall
// takes d down the side and the other distance across the cap. A reference
// that names no adjacent face of a chamfered edge, or both, or a face beside
// no chamfered edge, is ErrCardinality (SX3). An asymmetric chamfer of a brep
// or stacked boolean result is ErrUnsupported (SX16).
//
// A selection of CAP edges is the cap-loop chamfer of
// docs/modify-reach-design.md §8.3: sel covering every geometric edge of one
// or more complete prism cap loops bevels each of those loops with a band of
// analytic patches, and the result is a cap-blend body whose volume, area,
// bounds, undercut and minimum-radius readings all answer. A cap-edge
// selection that is not one or more complete loops, and one that mixes cap
// edges with lateral ones, are both SX4 (ErrUnsupported). A setback across the
// cap that empties the cap contour is SX6 (ErrDegenerate), and bands whose
// setbacks down the side reach the far end of the sweep are SX7
// (ErrUnsupported). A chamfer whose setback is so small beside the geometry it
// displaces that the displacement rounds away is SX13 (ErrUnsupported), rather
// than a band with the taper gone: a circular wall so large beside the setback
// across the cap that the offset's radial change rounds back onto its own
// radius, or a sweep so tall beside the setback down the side that the band's
// side level rounds back onto its cap level. A further modify op on the
// result is SX10 (ErrUnsupported), and a clearance pair its bounding boxes do
// not already decide reads Suspect (Table DX row DX6).
//
// An analytic boolean result (a brep or stacked body) that reads as a prism
// along a reference axis is chamfered as that prism
// (docs/brep-modify-design.md route P): lateral edges and complete cap loops
// alike, so a cross-drilled hole's mouth takes a countersink. Any other
// selection of straight edges along one axis of such a body's face record
// takes route E (§5): a swept straight wall the edge ends on or runs along
// as a rim is first restated as the planar rectangle it sweeps (§5.2), both
// end faces of each edge take the chord at their corner, the two faces beside
// it are trimmed to the chord's feet, and one planar wall carrying a
// chamfer(k) role is added; the result is a brep body whose record pairs
// every edge again before it is built. A curved edge, two selected edges
// sharing a vertex, an edge whose end face is a curved face or an earlier
// blend, a straight wall the route needs as a plane that is oblique, split
// or carries a displaced level, end faces whose chords disagree, and a body
// whose faces carry a section displacement (SB1) are ErrUnsupported (Table
// SB).
func (b *Body) Chamfer(ctx context.Context, sel EdgeSelector, d units.Value, opts ...ChamferOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a chamfer`, ErrDegenerate)
	}
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	doc := b.doc
	// Stage 1 pre-gates (§4): a live receiver (S17), a valid magnitude (S15),
	// a non-zero one (S13), then a selector that matches (S16).
	if err := doc.requireLive(b); err != nil {
		return nil, err
	}
	// Table X (docs/surface-design.md §11): refused ahead of the selector gate
	// so the answer does not depend on what the selector matched.
	if err := refuseSheetOperand(b, "Chamfer"); err != nil {
		return nil, err
	}
	o, err := decodeChamferOptions(opts)
	if err != nil {
		return nil, err
	}
	dmm, dDelta, err := magnitudeInBounded(d, units.Length, units.Millimeter, "the chamfer setback")
	if err != nil {
		return nil, err
	}
	if dmm == 0 {
		return nil, fmt.Errorf(`%w: a zero-distance chamfer is the body the caller already holds`, ErrDegenerate)
	}
	// decad owns the selector vocabulary, so only the built-in query can be
	// resolved and recorded. Reject foreign implementations before invoking
	// their callback, and treat a typed nil query like an untyped nil.
	q, ok := sel.(*EdgeQuery)
	switch {
	case sel == nil:
		return nil, errNilSelector
	case !ok:
		return nil, fmt.Errorf(`%w: the chamfer's edge selector is not a decad edge query (%T)`, ErrDegenerate, sel)
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
	// Reach stage 3 (§6, SX3): the asymmetric reference resolves against the
	// receiver, after expansion, to one adjacent face per chamfered edge.
	var asym *asymmetricChamfer
	if o.Asymmetric != nil {
		refs, err := resolveAsymmetricReference(b, o.Asymmetric, edges)
		if err != nil {
			return nil, err
		}
		asym = &asymmetricChamfer{body: b, refs: refs, d: dmm, other: o.Asymmetric.otherMM,
			dDelta: dDelta, otherDelta: o.Asymmetric.otherDelta}
	}

	// SX10: a capBlendPayload receiver is staged before the generic
	// "not a prism" refusal, so the more specific reason leads.
	if err := requireNotCapBlendReceiver(b.payload, "chamfers"); err != nil {
		return nil, err
	}
	// SX16: docs/brep-modify-design.md states no asymmetric setback for either
	// brep route, so the option is refused on every brep or stacked receiver.
	if asym != nil {
		switch b.payload.(type) {
		case brepPayload, stackedPrismPayload:
			return nil, fmt.Errorf(`%w: this evaluator builds an asymmetric chamfer on a prism or a revolve only; a brep or stacked receiver takes the equal setback (modify-reach SX16)`, ErrUnsupported)
		}
	}
	blend := revolveBlendOp{
		kind: "chamfer",
		corner: func(loop cornerLoop, li, ci int, e *Edge) (*cornerBlend, error) {
			if asym == nil {
				return computeChamfer(loop, ci, dmm, dmm)
			}
			dA, dB, err := asym.setbacks(loop, li, ci, e)
			if err != nil {
				return nil, err
			}
			return computeChamfer(loop, ci, dA, dB)
		},
	}
	// A brep or stacked receiver takes the brep route
	// (docs/brep-modify-design.md §2), ahead of the generic refusal.
	route, err := modifyBrepReceiver(ctx, b, brepModifyRequest{op: "chamfers",
		admits: func(pp prismPayload, caps prismCaps) error {
			_, _, _, err := classifyChamferSelection(ctx, pp, caps, sel, edges)
			return err
		},
		sel: sel, edges: edges, blend: &blend})
	if err != nil {
		return nil, err
	}
	if route.body != nil {
		return commitModifyResult(ctx, b, route.body)
	}

	// Stage 2 (§4): the receiver's payload class (S3), then every selected
	// edge is a lateral edge mapped to a section corner (S1) OR — reach RX1's
	// second class — every geometric edge of one or more complete prism cap
	// loops (SX4 otherwise; docs/modify-reach-design.md §4/§8.3).
	// Reach RX2 (docs/modify-reach-design.md §7): a revolve receiver bevels its
	// swept meridian junctions through the same corner rewrite.
	if rp, ok := b.payload.(revolvePayload); ok {
		return b.blendRevolveJunctions(ctx, sel, edges, rp, blend)
	}
	pp, ok := b.payload.(prismPayload)
	caps := prismCapsOf(b)
	if route.prism != nil {
		pp, ok, caps = *route.prism, true, route.caps
	}
	if !ok {
		return nil, fmt.Errorf(`%w: this evaluator chamfers a straight prism or a revolve only; selector %s matched [%s]`,
			ErrUnsupported, sel, selectedEdgesContext(edges))
	}
	if err := requireExactSection(pp, "chamfers"); err != nil {
		return nil, err
	}

	startLoops, endLoops, lateral, err := classifyChamferSelection(ctx, pp, caps, sel, edges)
	if err != nil {
		return nil, err
	}
	if !lateral {
		// An equal chamfer sets both caps back d across the cap and d down the
		// side; a two-distance one takes each cap's pair from its reference
		// face (docs/modify-reach-design.md §8.3.1).
		start := capSetback{dc: dmm, dcDelta: dDelta, ds: dmm, dsDelta: dDelta}
		end := start
		if asym != nil {
			if start, end, err = asym.capSetbacks(caps, edges, startLoops, endLoops); err != nil {
				return nil, err
			}
		}
		ref := doc.nextProducerID()
		body, err := buildCapBlend(ctx, doc, ref, pp, start, end, startLoops, endLoops)
		if err != nil {
			return nil, err
		}
		if err := doc.requireLive(b); err != nil {
			return nil, err
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		doc.commit(body, b)
		return body, nil
	}

	budget := proofbound.NewWorkBudget(ctx)
	loops, err := prismCornerLoopsBudget(budget, pp)
	if err != nil {
		return nil, err
	}
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
			return nil, fmt.Errorf(`%w: a chamfer of a cap edge is the vertex-blend problem, not yet supported; selector %s, %s`,
				ErrUnsupported, sel, selectedEdgeContext(ei, e))
		}
		matched = append(matched, newMatchedCorner(ei, e, li, ci, loops[li]))
		blendAt[li][ci] = nil // marked; the bevel is computed in stage 3
	}

	// Stage 3 (§4): the construction's own gate, per corner — S4 (a corner
	// exists). There is no S5: a chord exists between any two distinct feet.
	for _, corner := range matched {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		cb, err := blend.corner(loops[corner.loop], corner.loop, corner.corner, corner.edge)
		if err != nil {
			return nil, fmt.Errorf(`%w; selector %s, %s`, err, sel, corner)
		}
		blendAt[corner.loop][corner.corner] = cb
	}

	// The rewritten section, and the bevel chords' (loop, segment) indices.
	profile, chamferSegs, err := rewriteProfileBudget(budget, pp.profile, loops, blendAt)
	if err != nil {
		return nil, err
	}

	// Stage 4 (§4/§5): the same audit the fillet runs — S8, S6 (an over-large
	// setback that reaches or passes a walk's far end), S7, S9.
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
	ref := doc.nextProducerID()
	// The blend descriptors ride on the payload so a re-evaluation (a copy or a
	// placement) re-mints its own chamfer(i,j) roles; evalPrism applies them.
	// The rewritten section is a NEW record no preflight has seen, so the build
	// opens its one counter here (docs/spline-design.md §5.2).
	// A section rewrite is a change of the SECTION: both sweep levels come
	// through unchanged, and so does each one's own axial displacement.
	body, err := evalPrismContext(ctx, doc, ref, prismPayload{
		profile:   profile,
		frame:     pp.frame,
		z0:        pp.z0,
		z1:        pp.z1,
		z0Delta:   pp.z0Delta,
		z1Delta:   pp.z1Delta,
		xform:     pp.xform,
		blendSegs: chamferSegs,
		blendKind: "chamfer",
	}, freeform.NewFreeformWork())
	if err != nil {
		return nil, err
	}
	// Keep the consumed input aligned with document liveness at the commit edge.
	if err := doc.requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	doc.commit(body, b)
	return body, nil
}

// computeChamfer builds one section corner blend.
func computeChamfer(loop cornerLoop, ci int, dA, dB float64) (*cornerBlend, error) {
	return offset2d.Chamfer(loop.walks, ci, dA, dB, filletTol)
}
