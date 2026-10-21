package brepmodify

import (
	"context"
	"errors"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/modifyselect"
	"github.com/lestrrat-3d/decad/internal/obliqueboss"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismpattern"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/retainedblend"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// FilletCall is what Fillet hands the node-level fillet once it has resolved
// the options, the selector and the tangent chain: the selector's text for
// refusals, the selected edges, and the radius in millimetres with its
// conversion bound.
type FilletCall struct {
	Sel       string
	Edges     []*topo.Edge
	R, RDelta float64
	Hooks     Hooks
}

// Fillet builds the fillet of b from the receiver gates on. The result is not
// yet committed.
func Fillet(ctx context.Context, env topo.Env, b *topo.Body, call FilletCall) (Result, error) {
	sel, edges, rmm, rDelta, hooks := call.Sel, call.Edges, call.R, call.RDelta, call.Hooks
	// SX10: a capBlendPayload receiver is staged before the generic "not a
	// prism" refusal, so the more specific reason leads.
	if err := capblend.RequireNotReceiver(b.Payload, "fillets"); err != nil {
		return Result{}, err
	}
	if node, matched, err := DraftCircleRimFillet(ctx, env, b, edges, rmm, rDelta); matched || err != nil {
		return Result{Body: node}, err
	}
	if err := draftrecord.RequireNotReceiver(b.Payload, "fillets"); err != nil {
		return Result{}, err
	}
	if bp, ok := b.Payload.(breprecord.Payload); ok {
		out, matched, err := RoundedBrepCapEdgeFillet(ctx, env, hooks, b, bp, edges, rmm, rDelta)
		if err != nil {
			return Result{}, err
		}
		if matched {
			return out, nil
		}
	}
	blend := BlendOp{
		Kind: "fillet",
		Corner: func(loop survey2d.CornerLoop, _, ci int, _ *topo.Edge) (*offset2d.Blend, error) {
			return offset2d.Fillet(loop.Walks, ci, rmm, sectionaudit.Tolerance)
		},
	}
	// The exact P8 front top edge takes a bounded cutter before the brep
	// receiver's SB7 curved-third-face refusal (modify-general §4.3c).
	// Other brep and stacked receivers take agent-docs/brep-modify-design.md §2.
	// A loop's fillet radius is both of route L's setbacks
	// (agent-docs/loop-fillet-design.md §4).
	loopCall := Request{
		Op: "fillets",
		Admits: func(pp prismrecord.Payload, _ PrismCaps) error {
			return modifyselect.RequireLateral(ctx, pp, edges)
		},
		Sel: sel, Edges: edges, Blend: &blend, CurveName: hooks.CurveName,
		Loop:     &capblend.Setback{Dc: rmm, DcDelta: rDelta, Ds: rmm, DsDelta: rDelta},
		LoopKind: breprecord.BandFillet,
	}
	route, err := Receiver(ctx, env, b, loopCall)
	if err != nil {
		return Result{}, err
	}
	if route.Body != nil {
		return Result{Body: route.Body}, nil
	}

	// Reach RX2 (agent-docs/modify-reach-design.md §7): a quarter-turn rectangular
	// revolve also admits its one outer angular-cap radial rim.
	if rp, ok := b.Payload.(revolveplan.Payload); ok {
		if out, matched, err := RevolveCapRimFillet(ctx, env, b, rp, edges, rmm, rDelta); matched || err != nil {
			return Result{Body: out}, err
		}
		out, err := BlendRevolveJunctions(ctx, env, sel, edges, rp, blend)
		return Result{Body: out}, err
	}
	if lp, ok := b.Payload.(loftrecord.Payload); ok {
		node, err := loftrecord.FilletStraight(ctx, env, sel,
			edges, lp, rmm)
		if err != nil {
			return Result{}, err
		}
		return Result{Body: node}, nil
	}
	if fp, ok := b.Payload.(facetedrecord.Payload); ok && fp.ObliqueBoss != nil {
		node, err := obliqueboss.FilletBoss(ctx, env, edges,
			fp.ObliqueBoss, rmm, rDelta)
		if err != nil {
			return Result{}, err
		}
		return Result{Body: node}, nil
	}
	if fp, ok := b.Payload.(facetedrecord.Payload); ok && fp.PrismUnionFillet != nil {
		// A retained source pair alone does not authenticate this selection.
		// Keep the ordinary S3 refusal and its complete edge context on a miss.
		if out, matched, err := PrismUnionFillet(ctx, env, hooks, b, edges, fp.PrismUnionFillet, rmm, rDelta); matched || err != nil {
			return Result{Body: out}, err
		}
	}
	if fp, ok := b.Payload.(facetedrecord.Payload); ok && fp.PrismPatternJoin != nil {
		out, err := prismpattern.FilletBody(ctx, env, hooks.Verified,
			b, edges, rmm, rDelta)
		if err != nil {
			return Result{}, err
		}
		return Result{Body: out}, nil
	}

	if fp, ok := b.Payload.(facetedrecord.Payload); ok {
		// A closed circular rim between two retained faces rebuilds the held
		// mesh near it (agent-docs/retained-edge-blend-design.md). A selection it does
		// not recognize keeps the ordinary S3 refusal and its edge context.
		if out, recognized, err := retainedblend.Fillet(ctx, env, b, fp, edges, rmm, rDelta); recognized || err != nil {
			return Result{Body: out}, err
		}
	}

	// Stage 2 (§4): the receiver's payload class (S3), then every selected
	// edge is a lateral edge mapped to a section corner (S1).
	pp, ok := b.Payload.(prismrecord.Payload)
	// A one-span straight solid Sweep builds through prismrecord.Build and keeps
	// that exact reduction beside its path. Its original edges can be matched
	// against the reduction's corners without replacing the public selector.
	sweepReduction := false
	if reduction, admitted := sweeprecord.StraightPrismReduction(b.Payload); admitted {
		pp, ok = reduction, true
		sweepReduction = true
	}
	if route.Prism != nil {
		pp, ok = *route.Prism, true
	}
	if !ok {
		return Result{}, fmt.Errorf(`%w: this evaluator fillets a straight prism, its one-line Sweep reduction, a revolve, or a matching-section axial loft only; selector %s matched [%s]`,
			decaderr.ErrUnsupported, sel, modifyselect.EdgesContext(edges))
	}
	if err := pp.RequireExactSection("fillets"); err != nil {
		return Result{}, err
	}

	budget := proofbound.NewWorkBudget(ctx)
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, pp.Profile)
	if err != nil {
		return Result{}, err
	}
	// Which corners each loop gets a fillet at, and the fillet's blend data.
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
			if sweepReduction {
				if len(edges) != 1 {
					return Result{}, fmt.Errorf(`%w: a straight Sweep fillets one cap edge at a time; selector %s matched [%s]`,
						decaderr.ErrUnsupported, sel, modifyselect.EdgesContext(edges))
				}
				if !straightSweepCapEdge(e) {
					return Result{}, fmt.Errorf(`%w: %s is not a cap edge of the straight Sweep`,
						decaderr.ErrUnsupported, modifyselect.EdgeContext(ei, e))
				}
				// The one-line Sweep was built from pp. Route E matches this
				// published edge to that exact prism's face view, then checks the
				// material wedge before rewriting either end face.
				bp, err := breprecord.OfPrism(pp)
				if err != nil {
					return Result{}, err
				}
				if err := breprecord.RequireExactSection(bp, loopCall.Op); err != nil {
					return Result{}, err
				}
				node, err := BlendEdges(ctx, env, bp, loopCall)
				if err != nil {
					return Result{}, err
				}
				return Result{Body: node}, nil
			}
			// RF3 (agent-docs/loop-fillet-design.md §3): a cap edge sends the
			// prism through its brep face view. Route E, L or V then reads the
			// whole selection (agent-docs/vertex-blend-design.md §2).
			if original, ok := b.Payload.(prismrecord.Payload); ok {
				if out, recognized, err := ObliqueCapEdgeFillet(ctx, env, hooks, b, original, edges, rmm, rDelta); recognized {
					return out, err
				}
			}
			node, err := PrismFaceViewBlend(ctx, env, pp, loopCall)
			if err != nil {
				return Result{}, err
			}
			return Result{Body: node}, nil
		}
		matched = append(matched, modifyselect.NewMatchedCorner(ei, e, li, ci, loops[li]))
		blendAt[li][ci] = nil // marked; the blend is computed in stage 3
	}

	// Stage 3 (§4): the construction's own gates, per corner — S4 (a corner
	// exists) then S5 (a blend of that radius exists).
	for _, corner := range matched {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return Result{}, err
		}
		cb, err := offset2d.Fillet(loops[corner.Loop].Walks, corner.Corner, rmm, sectionaudit.Tolerance)
		if err != nil {
			return Result{}, fmt.Errorf(`%w; selector %s, %s`, err, sel, corner)
		}
		blendAt[corner.Loop][corner.Corner] = cb
	}

	// The rewritten section, and the fillet arcs' (loop, segment) indices.
	profile, filletArcs, err := offset2d.RewriteProfileBudget(budget, pp.Profile, loops, blendAt)
	if err != nil {
		return Result{}, err
	}

	// Stage 4 (§4/§5): the audit of the rewritten profile — S8, S6, S7, S9.
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
	// placement) re-mints its own fillet(i,j) roles; evalPrism applies them.
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
		BlendSegs: filletArcs,
		BlendKind: "fillet",
	}, freeform.NewFreeformWork())
	if err != nil {
		return Result{}, err
	}
	return Result{Body: body}, nil
}

// straightSweepCapEdge checks the public role pair before the stored prism's
// face view matches the selected geometry. A one-line Sweep renames only its
// swept walls to side(0,loop,segment); its two cap roles are unchanged.
func straightSweepCapEdge(edge *topo.Edge) bool {
	facing := edge.Faces
	if len(facing) != 2 {
		return false
	}
	capRole, wallRole := false, false
	for _, face := range facing {
		for _, origin := range face.Origins {
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
