package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/extent"

	"github.com/lestrrat-3d/units"
)

// Chamfer turns selected sharp edges into flat bevels. A setback is how far
// the bevel starts from the original edge; a cap is an end face, while a
// lateral edge runs along the side. The code picks a construction route for
// the body's shape, checks the result, then replaces the old body in its document.

// This file is the chamfer of agent-docs/modify-design.md §7: Chamfer bevels the
// convex or concave lateral edges of a straight prism (Table R, R1). It is the
// fillet's sibling under the §2 reduction — a lateral edge is a CORNER of the
// recorded 2D section, and bevelling it is a rewrite of that section, fed back
// through evalPrism so mass properties, tessellation and the surveys come for
// free (§10, Table D). Chamfer's ONE new piece of geometry is the corner
// rewrite: the corner is cut off by a straight CHORD between the two setback
// feet (a LineSeg), never a tangent arc, so the bevel wall is a PLANE (§7). It
// then shares everything downstream with the fillet — the §5 audit
// (offset2d.AuditRewriteBudget, S8/S6/S7/S9), the profile rewrite (rewriteProfile), evalPrism,
// and the blend-role machinery (topobuild.AddBlendRoles) — via the common cornerBlend.
//
// The setback d is measured along the adjacent boundary curve, d from the
// corner along EACH walk; WithAsymmetricChamfer instead sets back d along the
// walk whose wall is the edge's reference face and its other distance along
// the other walk (agent-docs/modify-reach-design.md §6). There is NO S5 gate: a
// chord exists between any two distinct feet, so the fillet's no-blend-centre
// refusal has no chamfer case (Table B, B1). A revolve receiver takes the same chord on its meridian
// (revolve_blend.go, agent-docs/modify-reach-design.md §7), and a brep or stacked
// boolean result takes routes P and E of agent-docs/brep-modify-design.md and route
// L of agent-docs/modify-general-design.md (brep_modify.go); any other non-prism
// receiver is S3 (ErrUnsupported), except a one-span straight solid Sweep's
// selected lateral edges or one straight cap edge, the bounded tapered
// full-circle far-rim case in agent-docs/draft-design.md §10.5, and the
// source-authenticated perpendicular cylindrical bore mouth in
// agent-docs/cross-cylinder-bore-chamfer-design.md.
//
// Chamfer also takes agent-docs/modify-reach-design.md §8.3's second receiver class,
// which the fillet does not: a selection covering every geometric edge of one
// or more COMPLETE prism cap loops builds a cap-loop chamfer through
// capblend.go instead of this file's corner rewrite, at an equal setback or,
// under WithAsymmetricChamfer, at the two setbacks each cap's reference face
// picks (§8.3.1). A single straight cap edge takes route E through the
// prism's face view or the bounded cutter when its terminal walls are
// oblique. Other partial cap selections and mixed cap/lateral selections
// are SX4 (ErrUnsupported).

// ChamferOption configures Chamfer: WithTangentChain and
// WithAsymmetricChamfer (agent-docs/modify-reach-design.md §2).
type ChamferOption interface{ chamferOption() }

// Chamfer bevels the selected lateral edges of a straight prism with a straight
// chord set back a distance d along each adjacent wall, returning the new body
// and retiring b (agent-docs/modify-design.md §7, core §8). A nil b is ErrDegenerate.
// sel is resolved against the live b; a query matching nothing is loud (ErrNoMatch /
// ErrCardinality, S16). d is a length magnitude, gated like every other (S15); a
// zero d is S13. The rewritten section faces the §5 audit before anything is
// built, so no unproven body is ever made and an over-large setback is refused
// (S6), never clipped.
//
// On a revolve, Chamfer bevels swept meridian junctions instead
// (agent-docs/modify-reach-design.md §7): the meridian corner gets the same chord,
// faces the same audit and the revolve's axis gates, and the revolve is rebuilt
// over the receiver's own axis, sweep and placement. The bevel is a Cone, a
// Cylinder where the chord runs parallel to the axis, or a Plane where it runs
// perpendicular. One source-authenticated quarter-turn rectangular radial
// cap rim takes the bounded faceted route in agent-docs/modify-reach-design.md §7.2.
// Other cap edges and edges on the axis are SX5 (ErrUnsupported).
//
// A one-sided positive-taper circular extrude also accepts one complete
// far-cap rim at an equal setback on its disk and cone (draft §10.5).
//
// A one-span straight solid Sweep takes selected lateral edges through its
// stored exact prism reduction. One straight cap edge takes that reduction's
// prism face view through route E. Other cap selections and asymmetric
// reference roles remain unsupported; other Sweep kinds are S3 (ErrUnsupported).
// A receiver outside the admitted classes is S3 (ErrUnsupported).
//
// WithTangentChain expands the edges sel resolves to as Fillet's does
// (agent-docs/modify-reach-design.md §5). WithAsymmetricChamfer sets each edge back
// d across its reference face and the option's other distance across the
// other adjacent face (§6), on a prism's lateral edges and a revolve's
// junctions alike; each setback faces the S6 audit on its own walk. On a
// complete cap loop the reference picks per cap (§8.3.1): the cap face takes d
// across the cap and the other distance down the side walls, and a side wall
// takes d down the side and the other distance across the cap. A reference
// that names no adjacent face of a chamfered edge, or both, or a face beside
// no chamfered edge, is ErrCardinality (SX3). An asymmetric chamfer of an
// independent straight brep edge takes route E, and a complete loop of a
// planar brep face takes route L. Stacked receivers use those same routes
// through their face view; selections outside them remain SX16.
//
// A selection covering one straight prism cap edge uses route E or the
// bounded cutter for an admitted oblique edge. A selection
// covering every geometric edge of one or more complete prism cap loops uses
// agent-docs/modify-reach-design.md §8.3's cap-loop chamfer. It bevels each loop
// with a band of analytic patches, and the result is a cap-blend body whose
// volume, area, bounds, undercut and minimum-radius readings all answer.
// A selection covering the whole outer loop of one or both caps of a straight
// prism whose section holds fit splines and lines, with no corner past 10°,
// bevels it at an equal setback by cutting the prism with a lofted cutter
// (agent-docs/modify-reach-design.md §8.5). Decad cannot offset a fit spline
// exactly, so the cutter's offset curves are fit splines through points of the
// exact offset, and the call refuses an offset that departs from it by more than
// 0.002 of the setback. The result is a mesh Boolean's: faceted, with the Cut's
// own bounds, and it matches the ideal chamfer only within that ratio. A
// setback that reaches the outline's radius of curvature is ErrDegenerate.
//
// All twelve straight edges of an exact axis-aligned box take
// agent-docs/modify-general-design.md §4.3b's simultaneous chamfer, with or
// without a cross bore. Other partial cap selections and mixed cap/lateral
// selections are SX4 (ErrUnsupported). A setback across the
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
// (agent-docs/brep-modify-design.md route P): lateral edges and complete cap loops
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
// whose faces carry a section displacement (SB1), outside brep-modify §4.4's
// lateral blend of a class-B crossing prism, are ErrUnsupported (Table SB).
//
// A selection of one or more complete loops of planar faces of such a body
// takes route L (agent-docs/modify-general-design.md §4): each loop's face takes
// the loop offset dc into its material, every face beside the loop is trimmed
// ds along the face's normal, and a band of Plane and Cone patches joins the
// two, removing a wedge where the walls beside the loop descend into the body
// (a hole mouth, a plate's top loop) and filling the concave corner where
// they rise off it (a boss root). The result is a brep body whose volume,
// area, centroid and box answer, which Verify, placement and a further modify
// op read; it tessellates, exports to STEP (analytic where every patch is a
// plane, faceted where a cone is), and its undercut and concave-radius
// surveys read the patches. It is a boolean operand where each band is a whole
// turn, line-line miters or exact tangent joins; a band with a cone at a
// reflex corner is export-only and ErrUnsupported as an operand. Several convex edges of a faceted solid every face of
// which squares to a world axis, which share corners and so have no route of their own, are the solid less the wedge
// of each edge, or with concave edges the solid plus it (agent-docs/orthogonal-boolean-design.md B17, B18), a faceted result
// with a bound. A clearance
// pair its boxes do not decide reads Suspect. A selection that is part of a loop, mixes loops with lone edges or
// holds two loops sharing an edge is SL1, a face beside a loop that is curved,
// oblique, split or on both sides of the loop's face is SL2, and a band
// reaching a far face end is SX7 (each ErrUnsupported); a setback that
// empties a loop's offset is SX6 (ErrDegenerate).
func Chamfer(ctx context.Context, b *Body, sel EdgeSelector, d units.Value,
	opts ...ChamferOption) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationChamfer, retErr) }()
	// Geometry work can take time, so callers need a context that can cancel it.
	// The body needs a document because a successful chamfer replaces that body there.
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a chamfer`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	doc := documentHandle(b.n.Doc)
	// Stage 1 pre-gates (§4): a live receiver (S17), a valid magnitude (S15),
	// a non-zero one (S13), then a selector that matches (S16).
	if err := doc.requireLive(b); err != nil {
		return nil, err
	}
	// Table X (agent-docs/surface-design.md §11): refused ahead of the selector gate
	// so the answer does not depend on what the selector matched.
	if err := refuseSheetOperand(b, "Chamfer"); err != nil {
		return nil, err
	}
	o, err := decodeChamferOptions(opts)
	if err != nil {
		return nil, err
	}
	// Convert the requested length to millimeters. Keep the conversion's error
	// bound too, so later geometry checks do not assume the number is exact.
	dmm, dDelta, err := extent.MagnitudeInBounded(d, units.Length, units.Millimeter, "the chamfer setback")
	if err != nil {
		return nil, err
	}
	if dmm == 0 {
		return nil, fmt.Errorf(`%w: a zero-distance chamfer is the body the caller already holds`, ErrDegenerate)
	}
	// decad owns the selector vocabulary, so only the built-in query can be
	// resolved and recorded. Reject foreign implementations before invoking
	// their callback. An interface can hold a typed nil query even when the
	// interface itself is not nil, so the switch checks both forms.
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
	// Reach stage 2 (agent-docs/modify-reach-design.md §4/§5): the seed query's
	// cardinality has been enforced above; the tangent chain expands it. Expand
	// first so every added edge can also receive an asymmetric reference face.
	if o.TangentChain {
		if edges, err = expandTangentChain(ctx, b, sel, edges); err != nil {
			return nil, err
		}
	}
	// Reach stage 3 (§6, SX3): the asymmetric reference resolves against the
	// receiver, after expansion, to one adjacent face per chamfered edge.
	var asym *brepmodify.Asymmetric
	if o.Asymmetric != nil {
		refs, err := resolveAsymmetricReference(b, o.Asymmetric, edges)
		if err != nil {
			return nil, err
		}
		asym = &brepmodify.Asymmetric{Body: &b.n, Refs: refs, D: dmm, Other: o.Asymmetric.OtherMM,
			DDelta: dDelta, OtherDelta: o.Asymmetric.OtherDelta}
	}

	res, err := brepmodify.Chamfer(ctx, envOf(b.n.Doc, b.n.Doc.NextProducer), &b.n, brepmodify.ChamferCall{
		Sel: fmt.Sprint(sel), Edges: edgeNodes(edges), D: dmm, DDelta: dDelta, Asym: asym, Hooks: modifyHooks(),
	})
	if err != nil {
		// Convex edges of an axis-square solid that share a corner have no route of their own: the solid less the wedge of
		// each edge is their chamfer (agent-docs/orthogonal-boolean-design.md B17).
		if errors.Is(err, ErrUnsupported) && asym == nil && !o.TangentChain {
			if out, handled, werr := chamferByWedges(ctx, doc, b, edges, dmm); werr != nil {
				return nil, werr
			} else if handled {
				return out, nil
			}
		}
		return nil, err
	}
	return commitResult(ctx, OperationChamfer, b, res)
}
