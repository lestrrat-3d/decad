package brepmodify

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/stackedshell"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/throughshell"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// This file is the receiver dispatch of agent-docs/brep-modify-design.md
// ("brep-modify §N" below): which record Fillet, Chamfer and Shell hand to the
// brep route (§2, Table RB), the gates every brep or stacked receiver passes
// before any route runs (Table SB's SB1 and SB2), and the order of the
// routes: route P (prism_route.go), then, for a Fillet or Chamfer,
// route E (edge_route.go) or route L
// (agent-docs/modify-general-design.md §4, loop_route.go), which
// brepLoopRoute picks.

// Request is what one modify op hands the brep route
// (brep-modify §2): the verb its refusals name ("fillets", "chamfers" or
// "shells"), whether the op is Shell, and admits, the op's own prism
// classification of its selection (§4.2). admits returns nil when the
// selection classifies against the prism pp with caps as its two cap faces,
// and the op's own refusal otherwise. sel, edges and blend are a Fillet's or
// Chamfer's selector, its resolved edges and its per-corner construction
// (computeFillet or computeChamfer, bound to the op's magnitude), which route
// E (§5) reads; a Shell leaves them nil. shellCall is a Shell's removed faces,
// sense and thickness, which route S (internal/throughshell) reads. loop is a
// Chamfer's setbacks, or a Fillet's radius stated as both setbacks, which
// route L bands each complete loop with (agent-docs/modify-general-design.md §4,
// agent-docs/loop-fillet-design.md §4), and loopKind the band's kind; a Shell
// leaves them unset.
type Request struct {
	Op        string
	Shell     bool
	Admits    func(pp prismrecord.Payload, caps PrismCaps) error
	Sel       string
	Edges     []*topo.Edge
	Blend     *BlendOp
	Asym      *Asymmetric
	ShellCall throughshell.Call
	Loop      *capblend.Setback
	LoopKind  breprecord.BandKind
	// CurveName names a curve in the public API's vocabulary, for the
	// refusal that prints it.
	CurveName func(surfacegeom.Curve) string
}

// BlendOp names one modify op's part in the shared rewrite: the role kind its
// blend wall carries ("fillet" or "chamfer", also the noun its refusals use),
// and the per-corner construction, bound to the op's magnitudes. Corner reads
// corner ci of loop, whose index in the recorded section is li, and the
// selected edge e that mapped to it; an asymmetric chamfer reads li and e to
// assign its two setbacks (agent-docs/modify-reach-design.md §6), and every other
// construction ignores them.
type BlendOp struct {
	Kind   string
	Corner func(loop survey2d.CornerLoop, li, ci int, e *topo.Edge) (*offset2d.Blend, error)
}

// Route is what the brep route hands back to the op. It is empty for a
// receiver the brep route does not take, and the op continues on its own
// path. Route P sets prism, the recognised prism (never stored; the op's
// receiver for the rest of the call), and caps, its two cap faces on the
// receiver body: the op continues on its prism path with them. Route E and
// route S set body, the result they built, which the op commits
// (commitModifyResult).
type Route struct {
	Prism *prismrecord.Payload
	Caps  PrismCaps
	Body  *topo.Body
}

// Receiver is the brep route for one modify op (brep-modify §2).
// It returns an empty route when the receiver is neither a breprecord.Payload nor a
// stackedrecord.Payload. Otherwise it runs gate stage 2a (§6) in order — the
// stacked receiver's face view (SB2), then the whole-record displacement
// rule (SB1) — and then route P (§4): the first reference axis along which
// the record reads as a prism the op's classification admits. When none
// admits, a Shell takes route S (agent-docs/modify-general-design.md §3), which
// builds the result or refuses, and a Fillet or Chamfer takes route E (§5) or
// route L (agent-docs/modify-general-design.md §4), as brepLoopRoute picks, which
// builds the result or refuses with a Table SB or Table SL row.
func Receiver(ctx context.Context, env topo.Env, b *topo.Body, req Request) (Route, error) {
	bp, sourceRoles, ok, err := brepModifyRecord(ctx, b.Payload, req.Op)
	if err != nil || !ok {
		return Route{}, err
	}
	if err := breprecord.RequireExactSection(bp, req.Op); err != nil {
		return Route{}, err
	}
	if req.Asym != nil {
		req.Asym.RecordRoles = sourceRoles
		// Routes E and L map public reference faces to their record faces.
		// Route P has no reference-face mapping for this option.
		body, err := brepLoopRoute(ctx, env, bp, req)
		if err != nil {
			return Route{}, err
		}
		return Route{Body: body}, nil
	}
	route, refusal, err := prismRoute(ctx, b, bp, req)
	switch {
	case err != nil:
		return Route{}, err
	case route.Prism != nil:
		return route, nil
	case req.Shell:
		body, err := shellThroughCut(ctx, env, b, bp, req, refusal)
		if err != nil {
			return Route{}, err
		}
		return Route{Body: body}, nil
	default:
		body, err := brepLoopRoute(ctx, env, bp, req)
		if err != nil {
			return Route{}, err
		}
		return Route{Body: body}, nil
	}
}

// shellThroughCut is route S's arm (agent-docs/modify-general-design.md §3): the
// shell of a brep or stacked receiver that reads as no prism whose caps are
// the removed faces. refusal is route P's classification refusal where some
// axis read as a prism (brep-modify SB3), and nil where none did (SB10);
// throughshell.ShellCut names it when the record reads as no through-cut record
// either (SG3), unless one of the bounded stacked routes matches first.
func shellThroughCut(ctx context.Context, env topo.Env, b *topo.Body, bp breprecord.Payload, req Request, refusal error) (*topo.Body, error) {
	if sp, ok := b.Payload.(stackedrecord.Payload); ok {
		n, matched, err := stackedshell.Build(ctx, env, sp, bp, stackedShellCall(b, req.ShellCall))
		if matched || err != nil {
			return n, err
		}
	}
	return throughshell.ShellCut(ctx, env, b, bp, req.ShellCall, refusal)
}

// brepModifyRecord is brep-modify §2's dispatch table: a breprecord.Payload is
// handed over as itself, and a stackedrecord.Payload as its face view
// (breprecord.OfStacked). The second result is false for every other payload, which
// the brep route does not take. A stacked receiver with no face view (a
// prism group, or a record its own audit refuses) is SB2: breprecord.OfStacked's
// error, naming the op.
func brepModifyRecord(ctx context.Context, payload topo.Payload, op string) (breprecord.Payload, []string, bool, error) {
	switch p := payload.(type) {
	case breprecord.Payload:
		if p.BossShell != nil || p.PocketShell != nil {
			return breprecord.Payload{}, nil, true, fmt.Errorf(`%w: this evaluator does not modify a stacked shell's rounded cavity`, decaderr.ErrUnsupported)
		}
		return p, nil, true, nil
	case stackedrecord.Payload:
		bp, sourceRoles, err := breprecord.OfStackedWithRoles(ctx, p)
		if err == nil {
			return bp, sourceRoles, true, nil
		}
		if ctxErr := ctx.Err(); ctxErr != nil {
			return breprecord.Payload{}, nil, true, ctxErr
		}
		return breprecord.Payload{}, nil, true, fmt.Errorf(`%w; this evaluator %s a stacked receiver through its face view only, and this one has none (brep-modify SB2)`, err, op)
	default:
		return breprecord.Payload{}, nil, false, nil
	}
}

// This file is route P of agent-docs/brep-modify-design.md ("brep-modify §N"): a
// brep or stacked receiver that reads as a prism along a reference axis
// (§4.1, P1–P5) is handed to the op's own prism path as that prism (§4.2).
// Every test below is an exact comparison of recorded floats and records,
// moved between face frames by signed permutations; nothing is sampled,
// solved or admitted on a residual.

// PrismCaps names the two cap faces a modify op classifies a prism selection
// against (brep-modify §4.2): start at the prism's z0 level, end at its z1
// level. startLoop and endLoop map a cap face's loop index to the section
// loop it bounds (0 the outer, 1+i hole i); nil reads as the identity. A nil
// face names no cap, and a selection on it classifies as no cap edge.
type PrismCaps struct {
	Start, End         *topo.Face
	StartLoop, EndLoop []int
}

// CapsOf names a prism body's own capStart and capEnd faces.
func CapsOf(b *topo.Body) PrismCaps {
	roles := topobuild.FacesByRole(b)
	return PrismCaps{Start: roles[provenance.RoleCapStart], End: roles[provenance.RoleCapEnd]}
}

// SectionLoop maps one cap face's loop index to its section loop index.
func (c PrismCaps) SectionLoop(start bool, li int) int {
	loops := c.EndLoop
	if start {
		loops = c.StartLoop
	}
	if loops == nil {
		return li
	}
	return loops[li]
}

// prismRoute is brep-modify §4.2: axes 0, 1, 2 in order, the first whose
// prism reading (§4.1) the op's own classification admits is taken. The
// returned route carries that prism and its caps. When no axis admits, the
// route is empty and refusal is the first axis's classification refusal, or
// nil when no axis reads as a prism at all. err is a context error, or the
// refusal of a Shell of a record carrying route L chamfer bands.
//
// A record carrying route L bands (agent-docs/modify-general-design.md §4) reads as
// no prism: a prism reading would read its faces alone and drop the band
// patches the body also holds, so a Fillet or Chamfer goes on to route E or
// route L over the record. A Shell of it refuses, since its erosion meets the
// band patches, which no through-cut record holds (modify-general SG3).
func prismRoute(ctx context.Context, b *topo.Body, bp breprecord.Payload, req Request) (Route, error, error) {
	if len(bp.LoopBands) > 0 {
		if req.Shell {
			return Route{}, nil, fmt.Errorf(`%w: this evaluator shells no brep body carrying route L loop bands; their patches are oblique planes, cones, cylinders and tori no through-cut record holds (modify-general SG3)`, decaderr.ErrUnsupported)
		}
		return Route{}, nil, nil
	}
	embeds, err := breprecord.Embeds(bp.Faces)
	if err != nil {
		// A record the build took has embeds; one that has none reads as no
		// prism, and the caller's own refusal follows.
		return Route{}, nil, nil //nolint:nilerr // no embeds is no prism reading
	}
	var refusal error
	for k := range 3 {
		read, ok, err := recognisePrism(ctx, bp, embeds, k)
		if err != nil {
			return Route{}, nil, err
		}
		if !ok {
			continue
		}
		caps := read.caps(b, bp)
		admitErr := req.Admits(read.pp, caps)
		if admitErr == nil {
			pp := read.pp
			return Route{Prism: &pp, Caps: caps}, nil, nil
		}
		if ctxErr := ctx.Err(); ctxErr != nil {
			return Route{}, nil, ctxErr
		}
		if errors.Is(admitErr, context.Canceled) || errors.Is(admitErr, context.DeadlineExceeded) {
			return Route{}, nil, admitErr
		}
		if refusal == nil {
			refusal = admitErr
		}
	}
	return Route{}, refusal, nil
}

// closedShellPrismReading is route P for a closed shell
// (agent-docs/modify-general-design.md §3.2): the first reference axis along
// which the brep record reads as a prism (brep-modify §4.1). A closed shell
// removes no face, so there is no cap classification to admit: every reading
// denotes the same solid, and the first one is taken. A record carrying a
// stacked shell's rounded cavity or route L bands reads as no prism. The
// reading must pass SB1's whole-record displacement rule before it is used.
func closedShellPrismReading(ctx context.Context, b *topo.Body, bp breprecord.Payload) (prismrecord.Payload, bool, error) {
	if bp.BossShell != nil || bp.PocketShell != nil || len(bp.LoopBands) > 0 {
		return prismrecord.Payload{}, false, nil
	}
	route, _, err := prismRoute(ctx, b, bp, Request{Op: "shells", Shell: true,
		Admits: func(prismrecord.Payload, PrismCaps) error { return nil }})
	if err != nil || route.Prism == nil {
		return prismrecord.Payload{}, false, err
	}
	if err := breprecord.RequireExactSection(bp, "shells"); err != nil {
		return prismrecord.Payload{}, false, err
	}
	return *route.Prism, true, nil
}

// brepPrismRead is one prism reading of a brep record along a reference axis
// (brep-modify §4.1): the recognised prism, the indices of the record's
// bottom and top faces, and which section loop each of bottom's region loops
// is (top's region is the section, loop for loop).
type brepPrismRead struct {
	pp          prismrecord.Payload
	bottom, top int
	bottomLoop  []int
}

// caps names the reading's cap faces on the receiver body: bottom is the
// prism's start cap and top its end cap. A stacked receiver's faces carry the
// stacked body's roles, not its face view's, so it names no cap face.
func (r brepPrismRead) caps(b *topo.Body, bp breprecord.Payload) PrismCaps {
	if _, ok := b.Payload.(breprecord.Payload); !ok {
		return PrismCaps{}
	}
	roles := topobuild.FacesByRole(b)
	return PrismCaps{
		Start:     roles[bp.Faces[r.bottom].Role],
		End:       roles[bp.Faces[r.top].Role],
		StartLoop: r.bottomLoop,
	}
}

// recognisePrism reads the record as a prism along reference axis k, or
// reports false (brep-modify §4.1, P1–P5): brepgeom.ReadPrismCaps for P1, P3 and P4,
// then brepgeom.ClassifyPrismWalls for P2 and P5. The error is a context error only.
func recognisePrism(ctx context.Context, bp breprecord.Payload, embeds []brepgeom.Embed, k int) (brepPrismRead, bool, error) {
	faces := bp.PrismFaceRecords()
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

// stackedShellCall is call as the plain data internal/stackedshell reads: it
// matches the removed face against source's cap-end and cap-start faces.
func stackedShellCall(source *topo.Body, call throughshell.Call) stackedshell.Call {
	sc := stackedshell.Call{Inward: call.Inward, Removed: len(call.Removed), T: call.Tmm, TDelta: call.TDelta}
	if len(call.Removed) == 1 {
		roles := topobuild.FacesByRole(source)
		sc.CapEnd = call.Removed[0] == roles[provenance.RoleCapEnd]
		sc.CapStart = call.Removed[0] == roles[provenance.RoleCapStart]
	}
	return sc
}
