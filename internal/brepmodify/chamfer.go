package brepmodify

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/modifyselect"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/retainedblend"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ChamferCall is what Chamfer hands the node-level chamfer once it has
// resolved the options, the selector, the tangent chain and the asymmetric
// reference: the selector's text for refusals, the selected edges, the
// setback in millimetres with its conversion bound, and the asymmetric
// reference.
type ChamferCall struct {
	Sel       string
	Edges     []*topo.Edge
	D, DDelta float64
	Asym      *Asymmetric
	Hooks     Hooks
}

// Chamfer builds the chamfer of b from the receiver gates on. The result is
// not yet committed.
func Chamfer(ctx context.Context, env topo.Env, b *topo.Body, call ChamferCall) (Result, error) {
	sel, edges, dmm, dDelta, asym, hooks := call.Sel, call.Edges, call.D, call.DDelta, call.Asym, call.Hooks
	// SX10: a capBlendPayload receiver is staged before the generic
	// "not a prism" refusal, so the more specific reason leads.
	if err := capblend.RequireNotReceiver(b.Payload, "chamfers"); err != nil {
		return Result{}, err
	}
	// A tapered circular body's far rim has its own builder. It only supports
	// equal setbacks, and a matched rim stops us from trying a generic route.
	if asym == nil {
		node, matched, err := DraftCircleRimChamfer(ctx, env, b, edges, dmm, dDelta)
		if matched || err != nil {
			if err != nil {
				return Result{}, err
			}
			return Result{Body: node}, nil
		}
	}
	if err := draftrecord.RequireNotReceiver(b.Payload, "chamfers"); err != nil {
		return Result{}, err
	}
	// A smooth fit-spline cap rim cannot take the exact cap-loop band, so it takes a cutter.
	if res, matched, err := FreeformRimChamfer(ctx, env, hooks, b, edges, dmm, dDelta, asym); matched || err != nil {
		return res, err
	}
	// Several body routes need the same flat cut at a section corner.
	// For unequal setbacks, the chosen reference face decides which nearby
	// section walk receives each distance.
	blend := BlendOp{
		Kind: "chamfer",
		Corner: func(loop survey2d.CornerLoop, li, ci int, e *topo.Edge) (*offset2d.Blend, error) {
			if asym == nil {
				return computeChamfer(loop, ci, dmm, dmm)
			}
			dA, dB, err := asym.Setbacks(loop, li, ci, e)
			if err != nil {
				return nil, err
			}
			return computeChamfer(loop, ci, dA, dB)
		},
	}
	// Give the boundary-representation (brep) builder the selected edges and
	// setback rules together. This lets it reuse the same request when the
	// receiver is a brep or a stack of earlier operations (§2).
	loopCall := Request{Op: "chamfers",
		Admits: func(pp prismrecord.Payload, caps PrismCaps) error {
			_, _, _, err := ClassifyChamferSelection(ctx, pp, caps, sel, edges)
			return err
		},
		Sel: sel, Edges: edges, Blend: &blend, Asym: asym, CurveName: hooks.CurveName,
		Loop: &capblend.Setback{Dc: dmm, DcDelta: dDelta, Ds: dmm, DsDelta: dDelta}, LoopKind: breprecord.BandChamfer}
	if bp, ok := b.Payload.(breprecord.Payload); ok {
		// Some selections need a whole-body chamfer instead of the usual edge route.
		node, matched, err := breprecord.TryAllEdgeChamfer(ctx, env, b, bp, edges, dmm, dDelta, asym != nil)
		if matched || err != nil {
			return Result{Body: node}, err
		}
	}
	// All twelve edges of a bare box take the same whole-body chamfer.
	if node, matched, err := breprecord.TryBareBoxAllEdgeChamfer(ctx, env, b, edges, dmm, dDelta, asym != nil); matched || err != nil {
		return Result{Body: node}, err
	}
	route, err := Receiver(ctx, env, b, loopCall)
	if err != nil {
		return Result{}, err
	}
	if route.Body != nil {
		return Result{Body: route.Body}, nil
	}
	// These recorded shapes have specialized constructions. A recognized shape
	// returns its result or its specific refusal instead of falling through to
	// a route that does not understand that shape.
	if lp, ok := b.Payload.(loftrecord.Payload); ok {
		body, recognized, err := LoftCapChamfer(ctx, env, b, lp, edges, dmm, dDelta, asym)
		if recognized || err != nil {
			return Result{Body: body}, err
		}
	}
	if fp, ok := b.Payload.(facetedrecord.Payload); ok {
		if fp.PrismUnionFillet != nil && asym == nil {
			if body, recognized, err := PrismUnionChamfer(ctx, env, hooks, b, edges, fp.PrismUnionFillet, dmm, dDelta); recognized || err != nil {
				return Result{Body: body}, err
			}
		}
		node, recognized, err := crossbore.MouthChamfer(ctx, env, b, fp, edges, dmm, dDelta, asym != nil)
		if recognized || err != nil {
			return Result{Body: node}, err
		}
		node, recognized, err = loftcylinder.MirrorCapChamfer(ctx, env, b, fp, edges, dmm, dDelta, asym != nil)
		if recognized || err != nil {
			return Result{Body: node}, err
		}
		node, recognized, err = loftcylinder.CapChamfer(ctx, env, b, fp, edges, dmm, dDelta, asym != nil)
		if recognized || err != nil {
			return Result{Body: node}, err
		}
		var ref *retainedblend.Reference
		if asym != nil {
			ref = &retainedblend.Reference{Refs: asym.Refs, Other: asym.Other, OtherDelta: asym.OtherDelta}
		}
		node, recognized, err = retainedblend.Chamfer(ctx, env, b, fp, edges, dmm, dDelta, ref)
		if recognized || err != nil {
			return Result{Body: node}, err
		}
	}

	// Stage 2 (§4): the receiver's payload class (S3), then every selected
	// edge is a lateral edge mapped to a section corner (S1) OR — reach RX1's
	// second class — every geometric edge of one or more complete prism cap
	// loops (SX4 otherwise; agent-docs/modify-reach-design.md §4/§8.3).
	// Reach RX2 (agent-docs/modify-reach-design.md §7): a revolve receiver bevels its
	// swept meridian junctions through the same corner rewrite. One certified
	// quarter-turn radial cap rim takes the source-backed §7.2 route first.
	if rp, ok := b.Payload.(revolveplan.Payload); ok {
		if out, matched, err := RevolveCapRimChamfer(ctx, env, b, rp, edges, dmm, dDelta, asym); matched || err != nil {
			return Result{Body: out}, err
		}
		out, err := BlendRevolveJunctions(ctx, env, sel, edges, rp, blend)
		return Result{Body: out}, err
	}
	pp, ok := b.Payload.(prismrecord.Payload)
	caps := CapsOf(b)
	// A brep route can supply an exact prism view even when the original body
	// is stored differently. Use that view for the remaining prism cases.
	if route.Prism != nil {
		pp, ok, caps = *route.Prism, true, route.Caps
	}
	// The selector has already resolved on the published Sweep. Only the
	// certified one-line solid reads through its stored exact prism.
	sweepStraight := false
	if reduction, admitted := sweeprecord.StraightPrismReduction(b.Payload); admitted {
		pp, ok, sweepStraight = reduction, true, true
	}
	if !ok {
		return Result{}, fmt.Errorf(`%w: this evaluator chamfers a straight prism, its one-line Sweep reduction, or a revolve only; selector %s matched [%s]`,
			decaderr.ErrUnsupported, sel, modifyselect.EdgesContext(edges))
	}
	if sweepStraight && asym != nil {
		return Result{}, fmt.Errorf(`%w: an asymmetric chamfer of a Sweep wall has no mapped reference-face role`, decaderr.ErrUnsupported)
	}
	// The corner rewrite needs the original 2D outline, not an approximate
	// outline recovered from a rendered or sampled shape.
	if err := pp.RequireExactSection("chamfers"); err != nil {
		return Result{}, err
	}
	// A single straight cap edge takes the bounded cutter or route E's corner
	// rewrite in the exact prism face view, including a straight Sweep's
	// stored reduction. Complete loops keep the cap-band route.
	if straightPrismCapEdge(caps, edges) {
		if original, ok := b.Payload.(prismrecord.Payload); ok {
			if out, recognized, err := ObliqueCapEdgeChamfer(ctx, env, hooks, b, original, edges, dmm, dDelta, asym); recognized {
				return out, err
			}
		}
		node, err := PrismFaceViewBlend(ctx, env, pp, loopCall)
		if err != nil {
			return Result{}, err
		}
		return Result{Body: node}, nil
	}

	// Side edges become corners in the 2D outline. A complete ring of cap edges
	// instead needs a bevel band around the end face; mixed selections cannot
	// use either construction.
	startLoops, endLoops, lateral, err := ClassifyChamferSelection(ctx, pp, caps, sel, edges)
	if err != nil {
		return Result{}, err
	}
	if sweepStraight && !lateral {
		return Result{}, fmt.Errorf(`%w: this straight Sweep chamfers only lateral edges; selector %s`, decaderr.ErrUnsupported, sel)
	}
	if !lateral {
		// An equal chamfer sets both caps back d across the cap and d down the
		// side; a two-distance one takes each cap's pair from its reference
		// face (agent-docs/modify-reach-design.md §8.3.1).
		// Dc measures across the end face; Ds measures down its side wall.
		start := capblend.Setback{Dc: dmm, DcDelta: dDelta, Ds: dmm, DsDelta: dDelta}
		end := start
		if asym != nil {
			if start, end, err = asym.CapSetbacks(caps, edges, startLoops, endLoops); err != nil {
				return Result{}, err
			}
		}
		node, err := capblend.BuildChecked(ctx, env, pp, start, end,
			startLoops, endLoops, nil)
		if err != nil {
			return Result{}, err
		}
		return Result{Body: node}, nil
	}

	budget := proofbound.NewWorkBudget(ctx)
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, pp.Profile)
	if err != nil {
		return Result{}, err
	}
	// Keep one lookup table per outline loop. First map each selected 3D edge
	// to a 2D corner; a nil value marks the spot until its bevel is computed.
	// The separate matched list keeps the original edge for useful errors.
	blendAt := make([]map[int]*offset2d.Blend, len(loops))
	for i := range blendAt {
		blendAt[i] = map[int]*offset2d.Blend{}
	}
	matched := make([]modifyselect.MatchedCorner, 0, len(edges))
	for ei, e := range edges {
		li, ci, found, err := modifyselect.MatchCorner(budget, pp, loops, e)
		if err != nil {
			return Result{}, err
		}
		if !found {
			return Result{}, fmt.Errorf(`%w: this cap-edge chamfer selection is unsupported; selector %s, %s`,
				decaderr.ErrUnsupported, sel, modifyselect.EdgeContext(ei, e))
		}
		matched = append(matched, modifyselect.NewMatchedCorner(ei, e, li, ci, loops[li]))
		blendAt[li][ci] = nil // marked; the bevel is computed in stage 3
	}

	// Stage 3 (§4): the construction's own gate, per corner — S4 (a corner
	// exists). There is no S5: a chord exists between any two distinct feet.
	for _, corner := range matched {
		// Large selections can visit many corners; check the shared work budget
		// on each one so cancellation can stop the operation promptly.
		if err := budget.Step(); err != nil {
			return Result{}, err
		}
		cb, err := blend.Corner(loops[corner.Loop], corner.Loop, corner.Corner, corner.Edge)
		if err != nil {
			return Result{}, fmt.Errorf(`%w; selector %s, %s`, err, sel, corner)
		}
		blendAt[corner.Loop][corner.Corner] = cb
	}

	// Replace marked corners with straight chords. Also remember which new
	// segments are bevels so later views can label their faces correctly.
	profile, chamferSegs, err := offset2d.RewriteProfileBudget(budget, pp.Profile, loops, blendAt)
	if err != nil {
		return Result{}, err
	}

	// Stage 4 (§4/§5): the same audit the fillet runs — S8, S6 (an over-large
	// setback that reaches or passes a walk's far end), S7, S9.
	// The audit checks the whole new outline before a body is built. Preserve
	// cancellation errors directly; add edge details only to geometry errors.
	if err := offset2d.AuditRewriteBudget(budget, pp.Profile, profile, loops, blendAt); err != nil {
		if errors.Is(err, context.Canceled) {
			return Result{}, context.Canceled
		}
		if errors.Is(err, context.DeadlineExceeded) {
			return Result{}, context.DeadlineExceeded
		}
		return Result{}, modifyselect.WrapAuditError(sel, matched, err)
	}

	// Build through evalPrism (§2): same frame, interval and placement, only
	// the section changed.
	// The blend descriptors ride on the payload so a re-evaluation (a copy or a
	// placement) re-mints its own chamfer(i,j) roles; evalPrism applies them.
	// The rewritten section is a NEW record no preflight has seen, so the build
	// opens its one counter here (agent-docs/spline-design.md §5.2).
	// A section rewrite is a change of the SECTION: both sweep levels come
	// through unchanged, and so does each one's own axial displacement.
	body, err := prismrecord.Build(ctx, env, prismrecord.Payload{
		Profile:   profile,
		Frame:     pp.Frame,
		Z0:        pp.Z0,
		Z1:        pp.Z1,
		Z0Delta:   pp.Z0Delta,
		Z1Delta:   pp.Z1Delta,
		Xform:     pp.Xform,
		BlendSegs: chamferSegs,
		BlendKind: "chamfer",
	}, freeform.NewFreeformWork())
	if err != nil {
		return Result{}, err
	}
	return Result{Body: body}, nil
}

// computeChamfer builds one section corner blend.
func computeChamfer(loop survey2d.CornerLoop, ci int, dA, dB float64) (*offset2d.Blend, error) {
	// This shared numeric tolerance handles nearly smooth or tiny corners; it
	// does not change the two setback distances requested by the caller.
	return offset2d.Chamfer(loop.Walks, ci, dA, dB, sectionaudit.Tolerance)
}

func straightPrismCapEdge(caps PrismCaps, edges []*topo.Edge) bool {
	// This special route handles one straight edge on an end face. A curved
	// edge or a selection of several edges needs another construction.
	if len(edges) != 1 {
		return false
	}
	if _, straight := edges[0].Curve.(surfacegeom.Line3); !straight {
		return false
	}
	// Find the cap by face membership, since an edge's adjacent faces need
	// not appear in a particular order.
	for _, face := range edges[0].Faces {
		if face == caps.Start || face == caps.End {
			return true
		}
	}
	return false
}
