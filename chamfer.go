package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

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
// The setback d is measured along the adjacent boundary curve, d from the corner
// along EACH walk (equal setback: the whole of v1; an asymmetric chamfer is an
// option that has not shipped, §7). There is NO S5 gate: a chord exists between
// any two distinct feet, so the fillet's no-blend-centre refusal has no chamfer
// case (Table B, B1). A cap-edge selector is S1 (ErrUnsupported, the vertex
// blend §6); a non-prism receiver is S3 (ErrUnsupported).

// ChamferOption configures Chamfer. No options are currently supported: a
// chamfer Step's Opts is nil (§7), and the option group exists so an
// asymmetric-chamfer option can be added without changing the signature.
type ChamferOption interface {
	option.Interface
	chamferOption()
}

// Chamfer bevels the selected lateral edges of a straight prism with a straight
// chord set back a distance d along each adjacent wall, returning the new body
// and retiring the receiver (docs/modify-design.md §7, core §8). sel is resolved
// against the live receiver; a query matching nothing is loud (ErrNoMatch /
// ErrCardinality, S16). d is a length magnitude, gated like every other (S15); a
// zero d is S13. A selected edge that is not a lateral edge — a cap edge — is S1
// (ErrUnsupported), and a receiver whose payload is not a prism is S3
// (ErrUnsupported). The rewritten section faces the §5 audit before anything is
// built, so no unproven body is ever made and an over-large setback is refused
// (S6), never clipped.
func (b *Body) Chamfer(sel EdgeSelector, d units.Value, opts ...ChamferOption) (*Body, error) {
	return b.ChamferContext(context.Background(), sel, d, opts...)
}

func (b *Body) ChamferContext(ctx context.Context, sel EdgeSelector, d units.Value, opts ...ChamferOption) (*Body, error) {
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	doc := b.doc
	// Stage 1 pre-gates (§4): a live receiver (S17), a valid magnitude (S15),
	// a non-zero one (S13), then a selector that matches (S16).
	if err := doc.requireLive(b); err != nil {
		return nil, err
	}
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
	}
	dmm, err := magnitudeIn(d, units.Length, units.Millimeter, "the chamfer setback")
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

	// Stage 2 (§4): the receiver's payload class (S3), then every selected
	// edge is a lateral edge mapped to a section corner (S1).
	pp, ok := b.payload.(prismPayload)
	if !ok {
		return nil, fmt.Errorf(`selector %s matched [%s]: %w: this evaluator chamfers a straight prism only`,
			sel, selectedEdgesContext(edges), ErrUnsupported)
	}

	budget := newWorkBudget(ctx)
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
			return nil, fmt.Errorf(`selector %s, %s: %w: a chamfer of a cap edge is the vertex-blend problem, not yet supported`,
				sel, selectedEdgeContext(ei, e), ErrUnsupported)
		}
		matched = append(matched, newMatchedCorner(ei, e, li, ci, loops[li]))
		blendAt[li][ci] = nil // marked; the bevel is computed in stage 3
	}

	// Stage 3 (§4): the construction's own gate, per corner — S4 (a corner
	// exists). There is no S5: a chord exists between any two distinct feet.
	for _, corner := range matched {
		if err := budget.step(); err != nil {
			return nil, err
		}
		cb, err := computeChamfer(loops[corner.loop], corner.corner, dmm)
		if err != nil {
			return nil, fmt.Errorf(`selector %s, %s: %w`, sel, corner, err)
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
	step := Step{
		Op:        OpChamfer,
		Inputs:    []StepRef{b.originStep()},
		Selectors: cloneSelectors([]Selector{q}),
		Values:    []units.Value{d},
	}
	ref := doc.nextStepRef()
	// The blend descriptors ride on the payload so a re-evaluation (a copy or a
	// placement) re-mints its own chamfer(i,j) roles; evalPrism applies them.
	// The rewritten section is a NEW record no preflight has seen, so the build
	// opens its one counter here (docs/spline-design.md §5.2).
	body, err := evalPrismContext(ctx, doc, ref, prismPayload{
		profile:   profile,
		frame:     pp.frame,
		z0:        pp.z0,
		z1:        pp.z1,
		xform:     pp.xform,
		blendSegs: chamferSegs,
		blendKind: "chamfer",
	}, newFreeformWork())
	if err != nil {
		return nil, err
	}
	// Keep the consumed input aligned with recipe liveness at the commit edge.
	if err := doc.requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	doc.commit(step, body, b)
	return body, nil
}

// computeChamfer builds a corner's bevel from its two walks (§7): S4 rejects a
// smooth or cusped corner, then each walk is set back an equal arc length d from
// the corner — d back along the arriving walk to foot fA, d forward along the
// leaving walk to foot fB — and the two feet are joined by a chord. The bevel is
// a plane, so the connector is a LineSeg (fA→fB), which continues the loop's own
// walk sense; there is no offset carrier and no S5, because a chord exists
// between any two distinct feet. IsConvex is not an input: a convex corner's
// chord cuts material away, a concave corner's fills it in, and both are the
// same construction (§7). An over-large setback is left to the §5 S6 audit,
// never clipped here.
func computeChamfer(loop cornerLoop, ci int, d float64) (*cornerBlend, error) {
	n := len(loop.walks)
	arrive := loop.walks[(ci+n-1)%n] // walk A, arriving at the corner
	leave := loop.walks[ci]          // walk B, leaving the corner

	ax, ay, la := normalize2(arrive.tanOutU, arrive.tanOutV)
	bx, by, lb := normalize2(leave.tanInU, leave.tanInV)
	if la == 0 || lb == 0 {
		return nil, fmt.Errorf(`%w: a corner walk has no direction`, ErrDegenerate)
	}
	// S4: a smooth (tangent) or cusped (anti-tangent) corner is no corner.
	if math.Abs(ax*by-ay*bx) <= filletTol {
		return nil, fmt.Errorf(`%w: the two walls meet smoothly — there is no corner to bevel`, ErrDegenerate)
	}

	fA := setbackFoot(arrive, d, true) // d back from the arriving walk's end
	fB := setbackFoot(leave, d, false) // d forward from the leaving walk's start

	return &cornerBlend{
		fA:        fA,
		fB:        fB,
		cutbackA:  d,
		cutbackB:  d,
		connector: LineSeg{Start: fA, End: fB, TStart: 0, TEnd: 1},
	}, nil
}

// setbackFoot is the point an arc length d from a corner along a walk, the
// setback §7 measures along the boundary curve. When back is true the corner is
// the walk's END and the step runs backwards along it (the arriving walk); else
// the corner is the walk's START and the step runs forwards (the leaving walk). A
// line steps by d along its unit travel tangent; an arc steps by the angle d/R
// in the walk's own turn sense — CCW (th1 > th0) increases the bearing, CW
// decreases it — so the foot lands on the arc itself, exactly.
func setbackFoot(w sideWalk, d float64, back bool) Point2 {
	if !w.isCircular() {
		if back {
			ux, uy, _ := normalize2(w.tanOutU, w.tanOutV)
			return Point2{U: w.endU - d*ux, V: w.endV - d*uy}
		}
		ux, uy, _ := normalize2(w.tanInU, w.tanInV)
		return Point2{U: w.startU + d*ux, V: w.startV + d*uy}
	}
	dtheta := d / w.radius
	sign := 1.0 // a CCW walk (th1 > th0) increases the bearing along travel
	if w.th1 < w.th0 {
		sign = -1.0
	}
	th := w.th0 + sign*dtheta // forward from the start
	if back {
		th = w.th1 - sign*dtheta // backward from the end
	}
	return Point2{U: w.cU + w.radius*math.Cos(th), V: w.cV + w.radius*math.Sin(th)}
}
