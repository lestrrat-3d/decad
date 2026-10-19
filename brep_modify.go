package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedshell"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file is the receiver dispatch of docs/brep-modify-design.md
// ("brep-modify §N" below): which record Fillet, Chamfer and Shell hand to the
// brep route (§2, Table RB), the gates every brep or stacked receiver passes
// before any route runs (Table SB's SB1 and SB2), and the order of the
// routes: route P (brep_modify.go), then, for a Fillet or Chamfer,
// route E (brep_modify_edge.go) or route L
// (docs/modify-general-design.md §4, brep_modify_loop.go), which
// brepLoopRoute picks.

// brepModifyRequest is what one modify op hands the brep route
// (brep-modify §2): the verb its refusals name ("fillets", "chamfers" or
// "shells"), whether the op is Shell, and admits, the op's own prism
// classification of its selection (§4.2). admits returns nil when the
// selection classifies against the prism pp with caps as its two cap faces,
// and the op's own refusal otherwise. sel, edges and blend are a Fillet's or
// Chamfer's selector, its resolved edges and its per-corner construction
// (computeFillet or computeChamfer, bound to the op's magnitude), which route
// E (§5) reads; a Shell leaves them nil. shellCall is a Shell's removed faces,
// sense and thickness, which route S (brep_shell.go) reads. loop is a
// Chamfer's setbacks, or a Fillet's radius stated as both setbacks, which
// route L bands each complete loop with (docs/modify-general-design.md §4,
// docs/loop-fillet-design.md §4), and loopKind the band's kind; a Shell
// leaves them unset.
type brepModifyRequest struct {
	op        string
	shell     bool
	admits    func(pp prismPayload, caps prismCaps) error
	sel       EdgeSelector
	edges     []*Edge
	blend     *revolveBlendOp
	asym      *asymmetricChamfer
	shellCall brepShellCall
	loop      *capSetback
	loopKind  brepBandKind
}

// brepRoute is what the brep route hands back to the op. It is empty for a
// receiver the brep route does not take, and the op continues on its own
// path. Route P sets prism, the recognised prism (never stored; the op's
// receiver for the rest of the call), and caps, its two cap faces on the
// receiver body: the op continues on its prism path with them. Route E and
// route S set body, the result they built, which the op commits
// (commitModifyResult).
type brepRoute struct {
	prism *prismPayload
	caps  prismCaps
	body  *Body
}

// modifyBrepReceiver is the brep route for one modify op (brep-modify §2).
// It returns an empty route when the receiver is neither a brepPayload nor a
// stackedPrismPayload. Otherwise it runs gate stage 2a (§6) in order — the
// stacked receiver's face view (SB2), then the whole-record displacement
// rule (SB1) — and then route P (§4): the first reference axis along which
// the record reads as a prism the op's classification admits. When none
// admits, a Shell takes route S (docs/modify-general-design.md §3), which
// builds the result or refuses, and a Fillet or Chamfer takes route E (§5) or
// route L (docs/modify-general-design.md §4), as brepLoopRoute picks, which
// builds the result or refuses with a Table SB or Table SL row.
func modifyBrepReceiver(ctx context.Context, b *Body, req brepModifyRequest) (brepRoute, error) {
	bp, sourceRoles, ok, err := brepModifyRecord(ctx, b.n.Payload, req.op)
	if err != nil || !ok {
		return brepRoute{}, err
	}
	if err := breprecord.RequireExactSection(bp, req.op); err != nil {
		return brepRoute{}, err
	}
	if req.asym != nil {
		req.asym.recordRoles = sourceRoles
		// Routes E and L map public reference faces to their record faces.
		// Route P has no reference-face mapping for this option.
		body, err := brepLoopRoute(ctx, documentHandle(b.n.Doc), bp, req)
		if err != nil {
			return brepRoute{}, err
		}
		return brepRoute{body: body}, nil
	}
	route, refusal, err := brepPrismRoute(ctx, b, bp, req)
	switch {
	case err != nil:
		return brepRoute{}, err
	case route.prism != nil:
		return route, nil
	case req.shell:
		body, err := brepShellThroughCut(ctx, b, bp, req, refusal)
		if err != nil {
			return brepRoute{}, err
		}
		return brepRoute{body: body}, nil
	default:
		body, err := brepLoopRoute(ctx, documentHandle(b.n.Doc), bp, req)
		if err != nil {
			return brepRoute{}, err
		}
		return brepRoute{body: body}, nil
	}
}

// brepShellThroughCut is route S's arm (docs/modify-general-design.md §3): the
// shell of a brep or stacked receiver that reads as no prism whose caps are
// the removed faces. refusal is route P's classification refusal where some
// axis read as a prism (brep-modify SB3), and nil where none did (SB10);
// shellThroughCut names it when the record reads as no through-cut record
// either (SG3), unless one of the bounded stacked routes matches first.
func brepShellThroughCut(ctx context.Context, b *Body, bp brepPayload, req brepModifyRequest, refusal error) (*Body, error) {
	if sp, ok := b.n.Payload.(stackedPrismPayload); ok {
		n, matched, err := stackedshell.Build(ctx, envOf(b.n.Doc, b.n.Doc.NextProducer), sp, bp,
			stackedShellCall(b, req.shellCall))
		if matched || err != nil {
			return bodyResult(n, err)
		}
	}
	return shellThroughCut(ctx, b, bp, req.shellCall, refusal)
}

// commitModifyResult commits a modify op's result in place of its receiver,
// after the receiver's liveness and the context are read once more at the
// commit edge (modify §13's atomic commit).
func commitModifyResult(ctx context.Context, b, body *Body) (*Body, error) {
	if err := documentHandle(b.n.Doc).requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	documentHandle(b.n.Doc).commit(body, b)
	return body, nil
}

// brepModifyRecord is brep-modify §2's dispatch table: a brepPayload is
// handed over as itself, and a stackedPrismPayload as its face view
// (breprecord.OfStacked). The second result is false for every other payload, which
// the brep route does not take. A stacked receiver with no face view (a
// prism group, or a record its own audit refuses) is SB2: breprecord.OfStacked's
// error, naming the op.
func brepModifyRecord(ctx context.Context, payload topo.Payload, op string) (brepPayload, []string, bool, error) {
	switch p := payload.(type) {
	case brepPayload:
		if p.BossShell != nil || p.PocketShell != nil {
			return brepPayload{}, nil, true, fmt.Errorf(`%w: this evaluator does not modify a stacked shell's rounded cavity`, ErrUnsupported)
		}
		return p, nil, true, nil
	case stackedPrismPayload:
		bp, sourceRoles, err := breprecord.OfStackedWithRoles(ctx, p)
		if err == nil {
			return bp, sourceRoles, true, nil
		}
		if ctxErr := ctx.Err(); ctxErr != nil {
			return brepPayload{}, nil, true, ctxErr
		}
		return brepPayload{}, nil, true, fmt.Errorf(`%w; this evaluator %s a stacked receiver through its face view only, and this one has none (brep-modify SB2)`, err, op)
	default:
		return brepPayload{}, nil, false, nil
	}
}

// This file is route P of docs/brep-modify-design.md ("brep-modify §N"): a
// brep or stacked receiver that reads as a prism along a reference axis
// (§4.1, P1–P5) is handed to the op's own prism path as that prism (§4.2).
// Every test below is an exact comparison of recorded floats and records,
// moved between face frames by signed permutations; nothing is sampled,
// solved or admitted on a residual.

// prismCaps names the two cap faces a modify op classifies a prism selection
// against (brep-modify §4.2): start at the prism's z0 level, end at its z1
// level. startLoop and endLoop map a cap face's loop index to the section
// loop it bounds (0 the outer, 1+i hole i); nil reads as the identity. A nil
// face names no cap, and a selection on it classifies as no cap edge.
type prismCaps struct {
	start, end         *Face
	startLoop, endLoop []int
}

// prismCapsOf names a prism body's own capStart and capEnd faces.
func prismCapsOf(b *Body) prismCaps {
	roles := facesByRole(b)
	return prismCaps{start: roles[provenance.RoleCapStart], end: roles[provenance.RoleCapEnd]}
}

// sectionLoop maps one cap face's loop index to its section loop index.
func (c prismCaps) sectionLoop(start bool, li int) int {
	loops := c.endLoop
	if start {
		loops = c.startLoop
	}
	if loops == nil {
		return li
	}
	return loops[li]
}

// brepPrismRoute is brep-modify §4.2: axes 0, 1, 2 in order, the first whose
// prism reading (§4.1) the op's own classification admits is taken. The
// returned route carries that prism and its caps. When no axis admits, the
// route is empty and refusal is the first axis's classification refusal, or
// nil when no axis reads as a prism at all. err is a context error, or the
// refusal of a Shell of a record carrying route L chamfer bands.
//
// A record carrying route L bands (docs/modify-general-design.md §4) reads as
// no prism: a prism reading would read its faces alone and drop the band
// patches the body also holds, so a Fillet or Chamfer goes on to route E or
// route L over the record. A Shell of it refuses, since its erosion meets the
// band patches, which no through-cut record holds (modify-general SG3).
func brepPrismRoute(ctx context.Context, b *Body, bp brepPayload, req brepModifyRequest) (brepRoute, error, error) {
	if len(bp.LoopBands) > 0 {
		if req.shell {
			return brepRoute{}, nil, fmt.Errorf(`%w: this evaluator shells no brep body carrying route L loop bands; their patches are oblique planes, cones, cylinders and tori no through-cut record holds (modify-general SG3)`, ErrUnsupported)
		}
		return brepRoute{}, nil, nil
	}
	embeds, err := breprecord.Embeds(bp.Faces)
	if err != nil {
		// A record the build took has embeds; one that has none reads as no
		// prism, and the caller's own refusal follows.
		return brepRoute{}, nil, nil //nolint:nilerr // no embeds is no prism reading
	}
	var refusal error
	for k := range 3 {
		read, ok, err := recognisePrism(ctx, bp, embeds, k)
		if err != nil {
			return brepRoute{}, nil, err
		}
		if !ok {
			continue
		}
		caps := read.caps(b, bp)
		admitErr := req.admits(read.pp, caps)
		if admitErr == nil {
			pp := read.pp
			return brepRoute{prism: &pp, caps: caps}, nil, nil
		}
		if ctxErr := ctx.Err(); ctxErr != nil {
			return brepRoute{}, nil, ctxErr
		}
		if errors.Is(admitErr, context.Canceled) || errors.Is(admitErr, context.DeadlineExceeded) {
			return brepRoute{}, nil, admitErr
		}
		if refusal == nil {
			refusal = admitErr
		}
	}
	return brepRoute{}, refusal, nil
}

// brepPrismRead is one prism reading of a brep record along a reference axis
// (brep-modify §4.1): the recognised prism, the indices of the record's
// bottom and top faces, and which section loop each of bottom's region loops
// is (top's region is the section, loop for loop).
type brepPrismRead struct {
	pp          prismPayload
	bottom, top int
	bottomLoop  []int
}

// caps names the reading's cap faces on the receiver body: bottom is the
// prism's start cap and top its end cap. A stacked receiver's faces carry the
// stacked body's roles, not its face view's, so it names no cap face.
func (r brepPrismRead) caps(b *Body, bp brepPayload) prismCaps {
	if _, ok := b.n.Payload.(brepPayload); !ok {
		return prismCaps{}
	}
	roles := facesByRole(b)
	return prismCaps{
		start:     roles[bp.Faces[r.bottom].Role],
		end:       roles[bp.Faces[r.top].Role],
		startLoop: r.bottomLoop,
	}
}

func prismFaceRecord(f brepFace) brepgeom.PrismRectFace {
	return brepgeom.PrismRectFace{
		Frame: f.Frame, Region: f.Region, Wall: f.Wall, Z0: f.Z0, Z1: f.Z1,
		Z0Delta: f.Z0Delta, Z1Delta: f.Z1Delta, Split0: len(f.Side0) != 0,
		Split1: len(f.Side1) != 0, Outward: f.Outward, Role: f.Role,
	}
}

func prismFaceRecords(bp brepPayload) []brepgeom.PrismRectFace {
	faces := make([]brepgeom.PrismRectFace, len(bp.Faces))
	for i, f := range bp.Faces {
		faces[i] = prismFaceRecord(f)
	}
	return faces
}

// recognisePrism reads the record as a prism along reference axis k, or
// reports false (brep-modify §4.1, P1–P5): brepgeom.ReadPrismCaps for P1, P3 and P4,
// then brepgeom.ClassifyPrismWalls for P2 and P5. The error is a context error only.
func recognisePrism(ctx context.Context, bp brepPayload, embeds []brepEmbed, k int) (brepPrismRead, bool, error) {
	faces := prismFaceRecords(bp)
	caps, ok := brepgeom.ReadPrismCaps(faces, embeds, k)
	if !ok {
		return brepPrismRead{}, false, nil
	}
	walls, ok, err := brepgeom.ClassifyPrismWalls(ctx, faces, embeds, k, caps)
	if err != nil || !ok {
		return brepPrismRead{}, false, err
	}
	return brepPrismRead{
		pp: prismrecord.Payload{
			Profile: caps.Section, Frame: caps.Frame, Xform: bp.Xform,
			Z0: caps.Zlo, Z1: caps.Zhi,
			Z0Delta: max(bp.Faces[caps.Bottom].Z0Delta, walls.ZloDelta),
			Z1Delta: max(bp.Faces[caps.Top].Z0Delta, walls.ZhiDelta),
		},
		bottom: caps.Bottom, top: caps.Top, bottomLoop: caps.BottomLoop,
	}, true, nil
}

// requireLateralEdges is Fillet's route P classification (brep-modify §4.2):
// every selected edge is a lateral edge of pp, the junction at one corner of
// its section (matchCornerBudget). Any other edge is modify S1's class.
func requireLateralEdges(ctx context.Context, pp prismPayload, edges []*Edge) error {
	budget := proofbound.NewWorkBudget(ctx)
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, pp.Profile)
	if err != nil {
		return err
	}
	for ei, e := range edges {
		_, _, found, err := matchCornerBudget(budget, pp, loops, e)
		if err != nil {
			return err
		}
		if !found {
			return fmt.Errorf(`%w: %s is not a lateral edge of the prism`, ErrUnsupported, selectedEdgeContext(ei, e))
		}
	}
	return nil
}
